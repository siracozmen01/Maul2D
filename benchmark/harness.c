// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Dual-backend harness, reference side. Runs the scene set on Box2D v3 and
// emits behavioral metrics. The Maul2D backend plugs into the same scene
// functions once it exists; until then this establishes the reference bands.

#include "maul2d/maul2d.h"

#include "box2d/box2d.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define MAX_BODIES    512
#define SETTLE_WINDOW 60
#define MAX_STEPS     2000

typedef struct SceneResult
{
    const char* name;
    int32_t bodyCount;
    int32_t settleStep; // first step where every dynamic body sleeps (-1 = n/a)
    uint64_t endHash;   // transform hash at settle (settle scenes) or at the horizon
    double jitter;      // settle scenes: max drift in the window after sleep
    double maxEndSpeed; // horizon scenes: fastest body at the end (explosion detector)
    double stepMsMedian;
    double stepMsP99;
} SceneResult;

typedef struct SceneWorld
{
    b2WorldId world;
    b2BodyId bodies[MAX_BODIES];
    int32_t bodyCount;
} SceneWorld;

static double NowMs(void)
{
    struct timespec ts;
    timespec_get(&ts, TIME_UTC);
    return 1000.0 * (double)ts.tv_sec + (double)ts.tv_nsec / 1.0e6;
}

static int CompareDouble(const void* a, const void* b)
{
    double da = *(const double*)a;
    double db = *(const double*)b;
    return da < db ? -1 : (da > db ? 1 : 0);
}

// --- Scenes ----------------------------------------------------------------

static SceneWorld ScenePyramid(void)
{
    SceneWorld s = {0};
    b2WorldDef worldDef = b2DefaultWorldDef();
    s.world = b2CreateWorld(&worldDef);

    b2BodyDef groundDef = b2DefaultBodyDef();
    groundDef.position = (b2Vec2){0.0f, -1.0f};
    b2BodyId ground = b2CreateBody(s.world, &groundDef);
    b2ShapeDef groundShape = b2DefaultShapeDef();
    b2Polygon groundBox = b2MakeBox(50.0f, 1.0f);
    b2CreatePolygonShape(ground, &groundShape, &groundBox);

    // 15-row box pyramid: the classic stacking stability scene.
    b2Polygon box = b2MakeBox(0.5f, 0.5f);
    b2ShapeDef shapeDef = b2DefaultShapeDef();
    shapeDef.density = 1.0f;
    shapeDef.material.friction = 0.6f;

    int32_t rows = 15;
    for (int32_t i = 0; i < rows; ++i)
    {
        for (int32_t j = i; j < rows; ++j)
        {
            b2BodyDef bd = b2DefaultBodyDef();
            bd.type = b2_dynamicBody;
            bd.position = (b2Vec2){(float)j - 0.5f * (float)(rows + i), 0.55f + 1.05f * (float)i};
            b2BodyId body = b2CreateBody(s.world, &bd);
            b2CreatePolygonShape(body, &shapeDef, &box);
            s.bodies[s.bodyCount++] = body;
        }
    }
    return s;
}

static SceneWorld SceneHingeChain(void)
{
    SceneWorld s = {0};
    b2WorldDef worldDef = b2DefaultWorldDef();
    s.world = b2CreateWorld(&worldDef);

    b2BodyDef groundDef = b2DefaultBodyDef();
    groundDef.position = (b2Vec2){0.0f, 20.0f};
    b2BodyId anchor = b2CreateBody(s.world, &groundDef);

    // 30-link revolute chain swinging under gravity: joint stress + sleep.
    b2Polygon link = b2MakeBox(0.5f, 0.125f);
    b2ShapeDef shapeDef = b2DefaultShapeDef();
    shapeDef.density = 20.0f;
    shapeDef.material.friction = 0.2f;

    b2BodyId prev = anchor;
    for (int32_t i = 0; i < 30; ++i)
    {
        b2BodyDef bd = b2DefaultBodyDef();
        bd.type = b2_dynamicBody;
        bd.position = (b2Vec2){0.5f + 1.0f * (float)i, 20.0f};
        b2BodyId body = b2CreateBody(s.world, &bd);
        b2CreatePolygonShape(body, &shapeDef, &link);

        b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
        jd.bodyIdA = prev;
        jd.bodyIdB = body;
        b2Vec2 pivot = (b2Vec2){1.0f * (float)i, 20.0f};
        jd.localAnchorA = b2Body_GetLocalPoint(prev, pivot);
        jd.localAnchorB = b2Body_GetLocalPoint(body, pivot);
        b2CreateRevoluteJoint(s.world, &jd);

        s.bodies[s.bodyCount++] = body;
        prev = body;
    }
    return s;
}

// --- Runner ------------------------------------------------------------------

static uint64_t HashTransforms(const SceneWorld* s)
{
    uint64_t h = M2_HASH_INIT;
    for (int32_t i = 0; i < s->bodyCount; ++i)
    {
        b2Transform t = b2Body_GetTransform(s->bodies[i]);
        h = m2Hash64(h, &t, sizeof(t));
    }
    return h;
}

static int AllAsleep(const SceneWorld* s)
{
    for (int32_t i = 0; i < s->bodyCount; ++i)
    {
        if (b2Body_IsAwake(s->bodies[i]))
        {
            return 0;
        }
    }
    return 1;
}

static SceneResult RunScene(const char* name, SceneWorld (*create)(void), int requireSettle,
                            int32_t horizon)
{
    SceneWorld s = create();
    SceneResult result = {0};
    result.name = name;
    result.bodyCount = s.bodyCount;
    result.settleStep = -1;

    static double stepTimes[MAX_STEPS];
    b2Vec2 settledPos[MAX_BODIES];
    double jitter = 0.0;
    int32_t settleAge = -1;

    int32_t step = 0;
    for (; step < MAX_STEPS; ++step)
    {
        double t0 = NowMs();
        b2World_Step(s.world, 1.0f / 60.0f, 4);
        stepTimes[step] = NowMs() - t0;

        if (requireSettle && result.settleStep < 0 && AllAsleep(&s))
        {
            result.settleStep = step;
            result.endHash = HashTransforms(&s);
            for (int32_t i = 0; i < s.bodyCount; ++i)
            {
                settledPos[i] = b2Body_GetPosition(s.bodies[i]);
            }
            settleAge = 0;
        }
        else if (settleAge >= 0 && settleAge < SETTLE_WINDOW)
        {
            for (int32_t i = 0; i < s.bodyCount; ++i)
            {
                b2Vec2 p = b2Body_GetPosition(s.bodies[i]);
                double dx = (double)p.x - (double)settledPos[i].x;
                double dy = (double)p.y - (double)settledPos[i].y;
                double d2 = dx * dx + dy * dy;
                if (d2 > jitter * jitter)
                {
                    jitter = sqrt(d2);
                }
            }
            settleAge += 1;
        }
        if (requireSettle && settleAge >= SETTLE_WINDOW)
        {
            break;
        }
        if (!requireSettle && step + 1 >= horizon)
        {
            break;
        }
    }

    step += 1;
    if (!requireSettle)
    {
        result.endHash = HashTransforms(&s);
        for (int32_t i = 0; i < s.bodyCount; ++i)
        {
            b2Vec2 vel = b2Body_GetLinearVelocity(s.bodies[i]);
            double speed = sqrt((double)vel.x * (double)vel.x + (double)vel.y * (double)vel.y);
            if (speed > result.maxEndSpeed)
            {
                result.maxEndSpeed = speed;
            }
        }
    }

    qsort(stepTimes, (size_t)step, sizeof(double), CompareDouble);
    result.stepMsMedian = stepTimes[step / 2];
    result.stepMsP99 = stepTimes[(int32_t)((double)step * 0.99)];
    result.jitter = jitter;

    b2DestroyWorld(s.world);
    return result;
}

// --- Maul2D side: the same scenes through the m2 API -----------------------

typedef struct MaulScene
{
    m2WorldId world;
    m2BodyId bodies[MAX_BODIES];
    int32_t bodyCount;
} MaulScene;

static MaulScene MaulPyramid(void)
{
    MaulScene s = {0};
    m2WorldDef def = m2DefaultWorldDef();
    s.world = m2CreateWorld(&def);

    m2BodyDef groundDef = m2DefaultBodyDef();
    groundDef.position = (m2Pos2){0.0, -1.0};
    m2BodyId ground = m2CreateBody(s.world, &groundDef);
    m2ShapeDef groundShape = m2DefaultShapeDef();
    m2Polygon groundBox = m2MakeBox(50.0f, 1.0f);
    m2CreatePolygonShape(ground, &groundShape, &groundBox);

    m2Polygon box = m2MakeBox(0.5f, 0.5f);
    m2ShapeDef shapeDef = m2DefaultShapeDef();
    shapeDef.density = 1.0f;
    shapeDef.friction = 0.6f;
    int32_t rows = 15;
    for (int32_t i = 0; i < rows; ++i)
    {
        for (int32_t j = i; j < rows; ++j)
        {
            m2BodyDef bd = m2DefaultBodyDef();
            bd.type = m2_dynamicBody;
            bd.position = (m2Pos2){(double)j - 0.5 * (double)(rows + i), 0.55 + 1.05 * (double)i};
            m2BodyId body = m2CreateBody(s.world, &bd);
            m2CreatePolygonShape(body, &shapeDef, &box);
            s.bodies[s.bodyCount++] = body;
        }
    }
    return s;
}

static MaulScene MaulHingeChain(void)
{
    MaulScene s = {0};
    m2WorldDef def = m2DefaultWorldDef();
    s.world = m2CreateWorld(&def);

    m2BodyDef anchorDef = m2DefaultBodyDef();
    anchorDef.position = (m2Pos2){0.0, 20.0};
    m2BodyId prev = m2CreateBody(s.world, &anchorDef);

    m2Polygon link = m2MakeBox(0.5f, 0.125f);
    m2ShapeDef shapeDef = m2DefaultShapeDef();
    shapeDef.density = 20.0f;
    shapeDef.friction = 0.2f;
    for (int32_t i = 0; i < 30; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.position = (m2Pos2){0.5 + 1.0 * (double)i, 20.0};
        m2BodyId body = m2CreateBody(s.world, &bd);
        m2CreatePolygonShape(body, &shapeDef, &link);

        m2RevoluteJointDef jd = m2DefaultRevoluteJointDef();
        jd.bodyIdA = prev;
        jd.bodyIdB = body;
        jd.localAnchorA = i == 0 ? (m2Vec2){0.0f, 0.0f} : (m2Vec2){0.5f, 0.0f};
        jd.localAnchorB = (m2Vec2){-0.5f, 0.0f};
        m2CreateRevoluteJoint(s.world, &jd);

        s.bodies[s.bodyCount++] = body;
        prev = body;
    }
    return s;
}

static SceneResult RunMaulScene(const char* name, MaulScene (*create)(void), int requireSettle,
                                int32_t horizon)
{
    MaulScene s = create();
    SceneResult result = {0};
    result.name = name;
    result.bodyCount = s.bodyCount;
    result.settleStep = -1;

    static double stepTimes[MAX_STEPS];
    m2Pos2 settledPos[MAX_BODIES];
    double jitter = 0.0;
    int32_t settleAge = -1;

    int32_t step = 0;
    for (; step < MAX_STEPS; ++step)
    {
        double t0 = NowMs();
        m2World_Step(s.world, 1.0f / 60.0f, 4);
        stepTimes[step] = NowMs() - t0;

        if (requireSettle && result.settleStep < 0)
        {
            bool allAsleep = true;
            for (int32_t i = 0; i < s.bodyCount; ++i)
            {
                allAsleep = allAsleep && !m2Body_IsAwake(s.bodies[i]);
            }
            if (allAsleep)
            {
                result.settleStep = step;
                result.endHash = m2World_Hash(s.world);
                for (int32_t i = 0; i < s.bodyCount; ++i)
                {
                    settledPos[i] = m2Body_GetPosition(s.bodies[i]);
                }
                settleAge = 0;
            }
        }
        else if (settleAge >= 0 && settleAge < SETTLE_WINDOW)
        {
            for (int32_t i = 0; i < s.bodyCount; ++i)
            {
                m2Pos2 p = m2Body_GetPosition(s.bodies[i]);
                double dx = p.x - settledPos[i].x;
                double dy = p.y - settledPos[i].y;
                double d2 = dx * dx + dy * dy;
                if (d2 > jitter * jitter)
                {
                    jitter = sqrt(d2);
                }
            }
            settleAge += 1;
        }
        if (requireSettle && settleAge >= SETTLE_WINDOW)
        {
            break;
        }
        if (!requireSettle && step + 1 >= horizon)
        {
            break;
        }
    }

    step += 1;
    if (!requireSettle)
    {
        result.endHash = m2World_Hash(s.world);
        for (int32_t i = 0; i < s.bodyCount; ++i)
        {
            m2Vec2 vel = m2Body_GetLinearVelocity(s.bodies[i]);
            double speed = sqrt((double)vel.x * (double)vel.x + (double)vel.y * (double)vel.y);
            if (speed > result.maxEndSpeed)
            {
                result.maxEndSpeed = speed;
            }
        }
    }

    qsort(stepTimes, (size_t)step, sizeof(double), CompareDouble);
    result.stepMsMedian = stepTimes[step / 2];
    result.stepMsP99 = stepTimes[(int32_t)((double)step * 0.99)];
    result.jitter = jitter;

    m2DestroyWorld(s.world);
    return result;
}

int main(void)
{
    b2Version v = b2GetVersion();
    printf("harness backends=box2d-%d.%d.%d,maul2d-%d.%d.%d\n", v.major, v.minor, v.revision,
           M2_VERSION_MAJOR, M2_VERSION_MINOR, M2_VERSION_PATCH);

    SceneResult results[4];
    // Pyramid must sleep; the chain is a fixed-horizon stability scene
    // (an undamped pendulum chain legitimately swings for minutes).
    results[0] = RunScene("b2:pyramid15", ScenePyramid, 1, MAX_STEPS);
    results[1] = RunScene("b2:hinge_chain30", SceneHingeChain, 0, 1200);
    results[2] = RunMaulScene("m2:pyramid15", MaulPyramid, 1, MAX_STEPS);
    results[3] = RunMaulScene("m2:hinge_chain30", MaulHingeChain, 0, 1200);

    printf("scene,bodies,settle_step,end_hash,jitter,max_end_speed,step_ms_median,step_ms_p99\n");
    int failures = 0;
    for (int i = 0; i < 4; ++i)
    {
        SceneResult* r = &results[i];
        printf("%s,%d,%d,%016llx,%.6e,%.6e,%.3f,%.3f\n", r->name, r->bodyCount, r->settleStep,
               (unsigned long long)r->endHash, r->jitter, r->maxEndSpeed, r->stepMsMedian,
               r->stepMsP99);
        int settleFailed = (i % 2) == 0 && r->settleStep < 0;
        int exploded = r->maxEndSpeed > 50.0;
        if (settleFailed || exploded)
        {
            printf("FAIL: %s %s\n", r->name, settleFailed ? "never settled" : "exploded");
            failures += 1;
        }
    }
    // The two-sided referee (risk R4): Maul must land inside behavior
    // bands anchored to the reference. Loose v1 bands; the point is a
    // tripwire, not a beauty contest.
    {
        SceneResult* b2p = &results[0];
        SceneResult* m2p = &results[2];
        if (m2p->settleStep < 0 || m2p->settleStep > 3 * b2p->settleStep + 240)
        {
            printf("BAND FAIL: maul pyramid settle %d vs b2 %d\n", m2p->settleStep,
                   b2p->settleStep);
            failures += 1;
        }
        if (m2p->jitter > 1.0e-3)
        {
            printf("BAND FAIL: maul pyramid jitter %.3e\n", m2p->jitter);
            failures += 1;
        }
        SceneResult* m2c = &results[3];
        if (m2c->maxEndSpeed > 50.0)
        {
            printf("BAND FAIL: maul chain exploded (%.2f m/s)\n", m2c->maxEndSpeed);
            failures += 1;
        }
    }
    return failures > 0 ? 1 : 0;
}
