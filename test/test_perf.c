// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The perf patrol: four engine-only scenes timed with the world's
// own profile clock, printed machine-readable for the weekly trend
// job. Not a gate on absolute numbers (shared runners are noisy);
// the trend job compares against a rolling baseline with a loose
// disaster threshold, so an accidental quadratic slip screams and
// runner weather does not.

#include "maul2d/maul2d.h"

#include <stdio.h>

static float StepAvgMs(m2WorldId world, int32_t warm, int32_t timed)
{
    for (int32_t i = 0; i < warm; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    float total = 0.0f;
    for (int32_t i = 0; i < timed; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        total += m2World_GetProfile(world).stepMs;
    }
    return total / (float)timed;
}

static void ScenePyramid30(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    m2WorldId world = m2CreateWorld(&def);
    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){0.0, -0.5};
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(40.0f, 0.5f);
    m2CreatePolygonShape(m2CreateBody(world, &fd), &fs, &slab);
    m2Polygon crate = m2MakeBox(0.25f, 0.25f);
    for (int32_t row = 0; row < 30; ++row)
    {
        for (int32_t col = 0; col <= row; ++col)
        {
            m2BodyDef bd = m2DefaultBodyDef();
            bd.type = m2_dynamicBody;
            bd.position = (m2Pos2){(double)col * 0.52 - (double)row * 0.26,
                                   15.2 - (double)row * 0.5};
            m2ShapeDef sd = m2DefaultShapeDef();
            m2CreatePolygonShape(m2CreateBody(world, &bd), &sd, &crate);
        }
    }
    printf("M2_PERF pyramid30 %.4f\n", (double)StepAvgMs(world, 60, 240));
    m2DestroyWorld(world);
}

static void SceneSleepingCity(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    m2WorldId world = m2CreateWorld(&def);
    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){0.0, -0.5};
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(40.0f, 0.5f);
    m2CreatePolygonShape(m2CreateBody(world, &fd), &fs, &slab);
    m2Polygon crate = m2MakeBox(0.25f, 0.25f);
    for (int32_t row = 0; row < 15; ++row)
    {
        for (int32_t col = 0; col <= row; ++col)
        {
            m2BodyDef bd = m2DefaultBodyDef();
            bd.type = m2_dynamicBody;
            bd.position = (m2Pos2){(double)col * 0.52 - (double)row * 0.26,
                                   7.7 - (double)row * 0.5};
            m2ShapeDef sd = m2DefaultShapeDef();
            m2CreatePolygonShape(m2CreateBody(world, &bd), &sd, &crate);
        }
    }
    printf("M2_PERF sleepingcity %.4f\n", (double)StepAvgMs(world, 600, 240));
    m2DestroyWorld(world);
}

static void SceneJointFarm(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    m2WorldId world = m2CreateWorld(&def);
    m2Circle disc = {{0.0f, 0.0f}, 0.3f};
    for (int32_t i = 0; i < 40; ++i)
    {
        m2BodyDef pd = m2DefaultBodyDef();
        pd.position = (m2Pos2){(double)i * 1.5, 3.0};
        m2BodyId post = m2CreateBody(world, &pd);
        m2BodyDef wd = m2DefaultBodyDef();
        wd.type = m2_dynamicBody;
        wd.position = (m2Pos2){(double)i * 1.5, 3.0};
        m2BodyId wheel = m2CreateBody(world, &wd);
        m2ShapeDef sd = m2DefaultShapeDef();
        m2CreateCircleShape(wheel, &sd, &disc);
        m2RevoluteJointDef rj = m2DefaultRevoluteJointDef();
        rj.bodyIdA = post;
        rj.bodyIdB = wheel;
        rj.enableMotor = true;
        rj.motorSpeed = 2.0f;
        rj.maxMotorTorque = 30.0f;
        m2CreateRevoluteJoint(world, &rj);
    }
    printf("M2_PERF jointfarm40 %.4f\n", (double)StepAvgMs(world, 60, 240));
    m2DestroyWorld(world);
}

static void SceneWater1500(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.particleCapacity = 2048;
    m2WorldId world = m2CreateWorld(&def);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){0.0, -0.2};
    m2Polygon slab = m2MakeBox(3.2f, 0.2f);
    m2CreatePolygonShape(m2CreateBody(world, &fd), &sd, &slab);
    m2Polygon wall = m2MakeBox(0.2f, 3.0f);
    m2BodyDef ld = m2DefaultBodyDef();
    ld.position = (m2Pos2){-3.2, 2.8};
    m2CreatePolygonShape(m2CreateBody(world, &ld), &sd, &wall);
    m2BodyDef rd = m2DefaultBodyDef();
    rd.position = (m2Pos2){3.2, 2.8};
    m2CreatePolygonShape(m2CreateBody(world, &rd), &sd, &wall);
    m2Polygon pool = m2MakeBox(2.8f, 1.0f);
    m2World_FillPolygonWithParticles(world, &pool, (m2Pos2){0.0, 1.1}, (m2Vec2){0.0f, 0.0f}, 0);
    printf("M2_PERF water %.4f\n", (double)StepAvgMs(world, 120, 200));
    m2DestroyWorld(world);
}

int main(void)
{
    ScenePyramid30();
    SceneSleepingCity();
    SceneJointFarm();
    SceneWater1500();
    printf("all checks passed\n");
    return 0;
}
