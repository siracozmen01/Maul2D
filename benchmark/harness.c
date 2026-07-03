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

// --- Perf-tracking scenes (timings reported, never banded) --------------
// pyramid30: 465 bodies of pure contact pressure. jointfarm: eighty
// running machines. Behavior bands stay loose - no explosions, stay
// calm - because these rows exist to watch step-time medians drift
// across commits, not to referee physics.

static SceneWorld ScenePyramid30(void)
{
    SceneWorld s = {0};
    b2WorldDef def = b2DefaultWorldDef();
    s.world = b2CreateWorld(&def);
    b2BodyDef gd = b2DefaultBodyDef();
    gd.position = (b2Vec2){0.0f, -1.0f};
    b2BodyId ground = b2CreateBody(s.world, &gd);
    b2ShapeDef gs = b2DefaultShapeDef();
    b2Polygon slab = b2MakeBox(80.0f, 1.0f);
    b2CreatePolygonShape(ground, &gs, &slab);
    b2Polygon box = b2MakeBox(0.5f, 0.5f);
    b2ShapeDef sd = b2DefaultShapeDef();
    sd.material.friction = 0.6f;
    int32_t rows = 30;
    for (int32_t i = 0; i < rows; ++i)
    {
        for (int32_t j = i; j < rows; ++j)
        {
            b2BodyDef bd = b2DefaultBodyDef();
            bd.type = b2_dynamicBody;
            bd.position = (b2Vec2){(float)j - 0.5f * (float)(rows + i), 0.55f + 1.05f * (float)i};
            b2BodyId body = b2CreateBody(s.world, &bd);
            b2CreatePolygonShape(body, &sd, &box);
            s.bodies[s.bodyCount++] = body;
        }
    }
    return s;
}

static MaulScene MaulPyramid30(void)
{
    MaulScene s = {0};
    m2WorldDef def = m2DefaultWorldDef();
    s.world = m2CreateWorld(&def);
    m2BodyDef gd = m2DefaultBodyDef();
    gd.position = (m2Pos2){0.0, -1.0};
    m2BodyId ground = m2CreateBody(s.world, &gd);
    m2ShapeDef gs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(80.0f, 1.0f);
    m2CreatePolygonShape(ground, &gs, &slab);
    m2Polygon box = m2MakeBox(0.5f, 0.5f);
    m2ShapeDef sd = m2DefaultShapeDef();
    sd.friction = 0.6f;
    int32_t rows = 30;
    for (int32_t i = 0; i < rows; ++i)
    {
        for (int32_t j = i; j < rows; ++j)
        {
            m2BodyDef bd = m2DefaultBodyDef();
            bd.type = m2_dynamicBody;
            bd.position = (m2Pos2){(double)j - 0.5 * (double)(rows + i), 0.55 + 1.05 * (double)i};
            m2BodyId body = m2CreateBody(s.world, &bd);
            m2CreatePolygonShape(body, &sd, &box);
            s.bodies[s.bodyCount++] = body;
        }
    }
    return s;
}

static SceneWorld SceneJointFarm(void)
{
    SceneWorld s = {0};
    b2WorldDef def = b2DefaultWorldDef();
    s.world = b2CreateWorld(&def);
    for (int32_t i = 0; i < 40; ++i)
    {
        float x = 4.0f * (float)i;
        b2BodyDef td = b2DefaultBodyDef();
        td.position = (b2Vec2){x, 5.0f};
        b2BodyId tower = b2CreateBody(s.world, &td);
        b2BodyDef wd = b2DefaultBodyDef();
        wd.type = b2_dynamicBody;
        wd.position = (b2Vec2){x, 5.0f};
        b2BodyId blade = b2CreateBody(s.world, &wd);
        b2ShapeDef ws = b2DefaultShapeDef();
        b2Polygon bar = b2MakeBox(1.0f, 0.08f);
        b2CreatePolygonShape(blade, &ws, &bar);
        b2RevoluteJointDef mill = b2DefaultRevoluteJointDef();
        mill.bodyIdA = tower;
        mill.bodyIdB = blade;
        mill.enableMotor = true;
        mill.motorSpeed = 2.5f;
        mill.maxMotorTorque = 30.0f;
        b2CreateRevoluteJoint(s.world, &mill);
        s.bodies[s.bodyCount++] = blade;

        b2BodyDef pd = b2DefaultBodyDef();
        pd.position = (b2Vec2){x + 2.0f, 8.0f};
        b2BodyId pivot = b2CreateBody(s.world, &pd);
        b2BodyDef rd = b2DefaultBodyDef();
        rd.type = b2_dynamicBody;
        rd.position = (b2Vec2){x + 2.6f, 8.0f};
        b2BodyId rod = b2CreateBody(s.world, &rd);
        b2Polygon rodBox = b2MakeBox(0.6f, 0.05f);
        b2CreatePolygonShape(rod, &ws, &rodBox);
        b2RevoluteJointDef swing = b2DefaultRevoluteJointDef();
        swing.bodyIdA = pivot;
        swing.bodyIdB = rod;
        swing.localAnchorB = (b2Vec2){-0.6f, 0.0f};
        swing.enableLimit = true;
        swing.lowerAngle = -0.6f;
        swing.upperAngle = 0.6f;
        b2CreateRevoluteJoint(s.world, &swing);
        s.bodies[s.bodyCount++] = rod;
    }
    return s;
}

static MaulScene MaulJointFarm(void)
{
    MaulScene s = {0};
    m2WorldDef def = m2DefaultWorldDef();
    s.world = m2CreateWorld(&def);
    for (int32_t i = 0; i < 40; ++i)
    {
        double x = 4.0 * (double)i;
        m2BodyDef td = m2DefaultBodyDef();
        td.position = (m2Pos2){x, 5.0};
        m2BodyId tower = m2CreateBody(s.world, &td);
        m2BodyDef wd = m2DefaultBodyDef();
        wd.type = m2_dynamicBody;
        wd.position = (m2Pos2){x, 5.0};
        m2BodyId blade = m2CreateBody(s.world, &wd);
        m2ShapeDef ws = m2DefaultShapeDef();
        m2Polygon bar = m2MakeBox(1.0f, 0.08f);
        m2CreatePolygonShape(blade, &ws, &bar);
        m2RevoluteJointDef mill = m2DefaultRevoluteJointDef();
        mill.bodyIdA = tower;
        mill.bodyIdB = blade;
        mill.enableMotor = true;
        mill.motorSpeed = 2.5f;
        mill.maxMotorTorque = 30.0f;
        m2CreateRevoluteJoint(s.world, &mill);
        s.bodies[s.bodyCount++] = blade;

        m2BodyDef pd = m2DefaultBodyDef();
        pd.position = (m2Pos2){x + 2.0, 8.0};
        m2BodyId pivot = m2CreateBody(s.world, &pd);
        m2BodyDef rd = m2DefaultBodyDef();
        rd.type = m2_dynamicBody;
        rd.position = (m2Pos2){x + 2.6, 8.0};
        m2BodyId rod = m2CreateBody(s.world, &rd);
        m2Polygon rodBox = m2MakeBox(0.6f, 0.05f);
        m2CreatePolygonShape(rod, &ws, &rodBox);
        m2RevoluteJointDef swing = m2DefaultRevoluteJointDef();
        swing.bodyIdA = pivot;
        swing.bodyIdB = rod;
        swing.localAnchorB = (m2Vec2){-0.6f, 0.0f};
        swing.enableLimit = true;
        swing.lowerAngle = -0.6f;
        swing.upperAngle = 0.6f;
        m2CreateRevoluteJoint(s.world, &swing);
        s.bodies[s.bodyCount++] = rod;
    }
    return s;
}

// --- Two-sided behavior trials -----------------------------------------
// Each trial builds the same scene in both engines and returns one
// scalar; the bands below referee the pair. Ramps are polygons with
// pre-rotated vertices so neither engine needs spawn-rotation support.

static void RampVertices(float halfLength, float halfThickness, float angle, float* xs, float* ys)
{
    float c = cosf(angle);
    float sn = sinf(angle);
    float cx[4] = {-halfLength, halfLength, halfLength, -halfLength};
    float cy[4] = {-halfThickness, -halfThickness, halfThickness, halfThickness};
    for (int i = 0; i < 4; ++i)
    {
        xs[i] = c * cx[i] - sn * cy[i];
        ys[i] = sn * cx[i] + c * cy[i];
    }
}

static double BounceTrialB2(void)
{
    b2WorldDef def = b2DefaultWorldDef();
    b2WorldId world = b2CreateWorld(&def);
    b2BodyDef gd = b2DefaultBodyDef();
    gd.position = (b2Vec2){0.0f, -0.5f};
    b2BodyId ground = b2CreateBody(world, &gd);
    b2ShapeDef gs = b2DefaultShapeDef();
    b2Polygon slab = b2MakeBox(20.0f, 0.5f);
    b2CreatePolygonShape(ground, &gs, &slab);

    b2BodyDef bd = b2DefaultBodyDef();
    bd.type = b2_dynamicBody;
    bd.position = (b2Vec2){0.0f, 4.0f};
    b2BodyId ball = b2CreateBody(world, &bd);
    b2ShapeDef sd = b2DefaultShapeDef();
    sd.material.restitution = 0.8f;
    b2Circle circle = {{0.0f, 0.0f}, 0.25f};
    b2CreateCircleShape(ball, &sd, &circle);

    double peak = 0.0;
    for (int i = 0; i < 240; ++i)
    {
        b2World_Step(world, 1.0f / 60.0f, 4);
        if (i > 60)
        {
            double y = b2Body_GetPosition(ball).y;
            peak = y > peak ? y : peak;
        }
    }
    b2DestroyWorld(world);
    return peak;
}

static double BounceTrialM2(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    m2WorldId world = m2CreateWorld(&def);
    m2BodyDef gd = m2DefaultBodyDef();
    gd.position = (m2Pos2){0.0, -0.5};
    m2BodyId ground = m2CreateBody(world, &gd);
    m2ShapeDef gs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(20.0f, 0.5f);
    m2CreatePolygonShape(ground, &gs, &slab);

    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 4.0};
    m2BodyId ball = m2CreateBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    sd.restitution = 0.8f;
    m2Circle circle = {{0.0f, 0.0f}, 0.25f};
    m2CreateCircleShape(ball, &sd, &circle);

    double peak = 0.0;
    for (int i = 0; i < 240; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        if (i > 60)
        {
            double y = m2Body_GetPosition(ball).y;
            peak = y > peak ? y : peak;
        }
    }
    m2DestroyWorld(world);
    return peak;
}

static double RampTrialB2(float mu)
{
    b2WorldDef def = b2DefaultWorldDef();
    b2WorldId world = b2CreateWorld(&def);
    float xs[4];
    float ys[4];
    RampVertices(6.0f, 0.3f, -0.35f, xs, ys); // ~20 degrees down toward +x
    b2Vec2 points[4];
    for (int i = 0; i < 4; ++i)
    {
        points[i] = (b2Vec2){xs[i], ys[i]};
    }
    b2Hull hull = b2ComputeHull(points, 4);
    b2Polygon ramp = b2MakePolygon(&hull, 0.0f);
    b2BodyDef gd = b2DefaultBodyDef();
    b2BodyId ground = b2CreateBody(world, &gd);
    b2ShapeDef gs = b2DefaultShapeDef();
    gs.material.friction = mu;
    b2CreatePolygonShape(ground, &gs, &ramp);

    b2BodyDef bd = b2DefaultBodyDef();
    bd.type = b2_dynamicBody;
    // Sit on the upper part of the slope.
    bd.position = (b2Vec2){-3.0f * cosf(0.35f), 3.0f * sinf(0.35f) + 0.75f};
    b2BodyId box = b2CreateBody(world, &bd);
    b2ShapeDef sd = b2DefaultShapeDef();
    sd.material.friction = mu;
    b2Polygon unit = b2MakeBox(0.3f, 0.3f);
    b2CreatePolygonShape(box, &sd, &unit);

    double x0 = b2Body_GetPosition(box).x;
    for (int i = 0; i < 300; ++i)
    {
        b2World_Step(world, 1.0f / 60.0f, 4);
    }
    double travel = b2Body_GetPosition(box).x - x0;
    b2DestroyWorld(world);
    return travel;
}

static double RampTrialM2(float mu)
{
    m2WorldDef def = m2DefaultWorldDef();
    m2WorldId world = m2CreateWorld(&def);
    float xs[4];
    float ys[4];
    RampVertices(6.0f, 0.3f, -0.35f, xs, ys);
    m2Vec2 points[4];
    for (int i = 0; i < 4; ++i)
    {
        points[i] = (m2Vec2){xs[i], ys[i]};
    }
    m2Polygon ramp = m2MakePolygon(points, 4, 0.0f);
    m2BodyDef gd = m2DefaultBodyDef();
    m2BodyId ground = m2CreateBody(world, &gd);
    m2ShapeDef gs = m2DefaultShapeDef();
    gs.friction = mu;
    m2CreatePolygonShape(ground, &gs, &ramp);

    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){-3.0 * cos(0.35), 3.0 * sin(0.35) + 0.75};
    m2BodyId box = m2CreateBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    sd.friction = mu;
    m2Polygon unit = m2MakeBox(0.3f, 0.3f);
    m2CreatePolygonShape(box, &sd, &unit);

    double x0 = m2Body_GetPosition(box).x;
    for (int i = 0; i < 300; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    double travel = m2Body_GetPosition(box).x - x0;
    m2DestroyWorld(world);
    return travel;
}

static double CarTrialB2(void)
{
    b2WorldDef def = b2DefaultWorldDef();
    b2WorldId world = b2CreateWorld(&def);
    b2BodyDef gd = b2DefaultBodyDef();
    gd.position = (b2Vec2){0.0f, -0.5f};
    b2BodyId road = b2CreateBody(world, &gd);
    b2ShapeDef gs = b2DefaultShapeDef();
    gs.material.friction = 0.9f;
    b2Polygon slab = b2MakeBox(60.0f, 0.5f);
    b2CreatePolygonShape(road, &gs, &slab);

    b2BodyDef cd = b2DefaultBodyDef();
    cd.type = b2_dynamicBody;
    cd.position = (b2Vec2){0.0f, 1.0f};
    b2BodyId chassis = b2CreateBody(world, &cd);
    b2ShapeDef cs = b2DefaultShapeDef();
    b2Polygon deck = b2MakeBox(0.9f, 0.15f);
    b2CreatePolygonShape(chassis, &cs, &deck);

    for (int i = 0; i < 2; ++i)
    {
        float x = i == 0 ? -0.6f : 0.6f;
        b2BodyDef wd = b2DefaultBodyDef();
        wd.type = b2_dynamicBody;
        wd.position = (b2Vec2){x, 0.55f};
        b2BodyId wheel = b2CreateBody(world, &wd);
        b2ShapeDef ws = b2DefaultShapeDef();
        ws.material.friction = 0.9f;
        b2Circle tire = {{0.0f, 0.0f}, 0.3f};
        b2CreateCircleShape(wheel, &ws, &tire);
        b2WheelJointDef ride = b2DefaultWheelJointDef();
        ride.bodyIdA = chassis;
        ride.bodyIdB = wheel;
        ride.localAnchorA = (b2Vec2){x, -0.45f};
        ride.localAxisA = (b2Vec2){0.0f, 1.0f};
        ride.enableSpring = true;
        ride.hertz = 4.0f;
        ride.dampingRatio = 0.7f;
        ride.enableLimit = true;
        ride.lowerTranslation = -0.2f;
        ride.upperTranslation = 0.2f;
        ride.enableMotor = true;
        ride.motorSpeed = -12.0f;
        ride.maxMotorTorque = 20.0f;
        b2CreateWheelJoint(world, &ride);
    }

    for (int i = 0; i < 360; ++i)
    {
        b2World_Step(world, 1.0f / 60.0f, 4);
    }
    double travel = b2Body_GetPosition(chassis).x;
    b2Rot q = b2Body_GetRotation(chassis);
    b2DestroyWorld(world);
    return q.c > 0.9f ? travel : -1000.0; // flipped car disqualifies
}

static double CarTrialM2(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    m2WorldId world = m2CreateWorld(&def);
    m2BodyDef gd = m2DefaultBodyDef();
    gd.position = (m2Pos2){0.0, -0.5};
    m2BodyId road = m2CreateBody(world, &gd);
    m2ShapeDef gs = m2DefaultShapeDef();
    gs.friction = 0.9f;
    m2Polygon slab = m2MakeBox(60.0f, 0.5f);
    m2CreatePolygonShape(road, &gs, &slab);

    m2BodyDef cd = m2DefaultBodyDef();
    cd.type = m2_dynamicBody;
    cd.position = (m2Pos2){0.0, 1.0};
    m2BodyId chassis = m2CreateBody(world, &cd);
    m2ShapeDef cs = m2DefaultShapeDef();
    m2Polygon deck = m2MakeBox(0.9f, 0.15f);
    m2CreatePolygonShape(chassis, &cs, &deck);

    for (int i = 0; i < 2; ++i)
    {
        float x = i == 0 ? -0.6f : 0.6f;
        m2BodyDef wd = m2DefaultBodyDef();
        wd.type = m2_dynamicBody;
        wd.position = (m2Pos2){x, 0.55};
        m2BodyId wheel = m2CreateBody(world, &wd);
        m2ShapeDef ws = m2DefaultShapeDef();
        ws.friction = 0.9f;
        m2Circle tire = {{0.0f, 0.0f}, 0.3f};
        m2CreateCircleShape(wheel, &ws, &tire);
        m2WheelJointDef ride = m2DefaultWheelJointDef();
        ride.bodyIdA = chassis;
        ride.bodyIdB = wheel;
        ride.localAnchorA = (m2Vec2){x, -0.45f};
        ride.localAxisA = (m2Vec2){0.0f, 1.0f};
        ride.enableSpring = true;
        ride.hertz = 4.0f;
        ride.dampingRatio = 0.7f;
        ride.enableLimit = true;
        ride.lowerTranslation = -0.2f;
        ride.upperTranslation = 0.2f;
        ride.enableMotor = true;
        ride.motorSpeed = -12.0f;
        ride.maxMotorTorque = 20.0f;
        m2CreateWheelJoint(world, &ride);
    }

    for (int i = 0; i < 360; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    double travel = m2Body_GetPosition(chassis).x;
    m2Rot q = m2Body_GetRotation(chassis);
    m2DestroyWorld(world);
    return q.c > 0.9f ? travel : -1000.0;
}

static int RunTrials(void)
{
    int failures = 0;

    double b2Peak = BounceTrialB2();
    double m2Peak = BounceTrialM2();
    printf("trial,bounce_peak,b2=%.3f,m2=%.3f\n", b2Peak, m2Peak);
    if (m2Peak < 1.6 || m2Peak > 3.2 || m2Peak < b2Peak - 0.5 || m2Peak > b2Peak + 0.5)
    {
        printf("BAND FAIL: bounce peak m2=%.3f vs b2=%.3f\n", m2Peak, b2Peak);
        failures += 1;
    }

    double b2Stick = RampTrialB2(0.9f);
    double m2Stick = RampTrialM2(0.9f);
    double b2Slide = RampTrialB2(0.05f);
    double m2Slide = RampTrialM2(0.05f);
    printf("trial,ramp_stick,b2=%.3f,m2=%.3f\n", b2Stick, m2Stick);
    printf("trial,ramp_slide,b2=%.3f,m2=%.3f\n", b2Slide, m2Slide);
    if (m2Stick < -0.15 || m2Stick > 0.15)
    {
        printf("BAND FAIL: high-friction box slid %.3f\n", m2Stick);
        failures += 1;
    }
    if (m2Slide < 1.0 || (b2Slide > 1.0 && (m2Slide < 0.5 * b2Slide || m2Slide > 2.0 * b2Slide)))
    {
        printf("BAND FAIL: low-friction slide m2=%.3f vs b2=%.3f\n", m2Slide, b2Slide);
        failures += 1;
    }

    double b2Car = CarTrialB2();
    double m2Car = CarTrialM2();
    printf("trial,car_travel,b2=%.3f,m2=%.3f\n", b2Car, m2Car);
    if (m2Car < 3.0 || b2Car < 3.0)
    {
        printf("BAND FAIL: a car failed to drive (b2=%.3f m2=%.3f)\n", b2Car, m2Car);
        failures += 1;
    }
    else if (m2Car < 0.5 * b2Car || m2Car > 2.0 * b2Car)
    {
        printf("BAND FAIL: car travel m2=%.3f vs b2=%.3f\n", m2Car, b2Car);
        failures += 1;
    }

    return failures;
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
    SceneResult perf[4];
    perf[0] = RunScene("b2:pyramid30", ScenePyramid30, 0, 600);
    perf[1] = RunScene("b2:jointfarm80", SceneJointFarm, 0, 300);
    perf[2] = RunMaulScene("m2:pyramid30", MaulPyramid30, 0, 600);
    perf[3] = RunMaulScene("m2:jointfarm80", MaulJointFarm, 0, 300);
    for (int i = 0; i < 4; ++i)
    {
        SceneResult* r = &perf[i];
        printf("%s,%d,%d,%016llx,%e,%e,%.3f,%.3f\n", r->name, r->bodyCount, r->settleStep,
               (unsigned long long)r->endHash, r->jitter, r->maxEndSpeed, r->stepMsMedian,
               r->stepMsP99);
    }
    // Loose sanity only: perf scenes must stay calm, never explode.
    if (perf[2].maxEndSpeed > 1.0 || perf[3].maxEndSpeed > 20.0)
    {
        printf("BAND FAIL: perf scene lost its calm (pyr30 %.3f farm %.3f)\n", perf[2].maxEndSpeed,
               perf[3].maxEndSpeed);
        failures += 1;
    }

    failures += RunTrials();

    return failures > 0 ? 1 : 0;
}
