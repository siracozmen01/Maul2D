// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A car in forty lines: chassis, two sprung motorized wheels, a road.
// Run it and watch the telemetry - the same bits on every machine.

#include "maul2d/maul2d.h"

#include <stdio.h>

int main(void)
{
    m2WorldDef worldDef = m2DefaultWorldDef();
    m2WorldId world = m2CreateWorld(&worldDef);

    m2BodyDef roadDef = m2DefaultBodyDef();
    roadDef.position = (m2Pos2){0.0, -0.5};
    m2BodyId road = m2CreateBody(world, &roadDef);
    m2ShapeDef roadShape = m2DefaultShapeDef();
    roadShape.friction = 0.9f;
    m2Polygon slab = m2MakeBox(120.0f, 0.5f);
    m2CreatePolygonShape(road, &roadShape, &slab);

    m2BodyDef chassisDef = m2DefaultBodyDef();
    chassisDef.type = m2_dynamicBody;
    chassisDef.position = (m2Pos2){0.0, 1.0};
    m2BodyId chassis = m2CreateBody(world, &chassisDef);
    m2ShapeDef chassisShape = m2DefaultShapeDef();
    m2Polygon deck = m2MakeBox(0.9f, 0.15f);
    m2CreatePolygonShape(chassis, &chassisShape, &deck);

    for (int i = 0; i < 2; ++i)
    {
        float x = i == 0 ? -0.6f : 0.6f;
        m2BodyDef wheelDef = m2DefaultBodyDef();
        wheelDef.type = m2_dynamicBody;
        wheelDef.position = (m2Pos2){x, 0.55};
        m2BodyId wheel = m2CreateBody(world, &wheelDef);
        m2ShapeDef tireShape = m2DefaultShapeDef();
        tireShape.friction = 0.9f;
        m2Circle tire = {{0.0f, 0.0f}, 0.3f};
        m2CreateCircleShape(wheel, &tireShape, &tire);

        m2WheelJointDef ride = m2DefaultWheelJointDef();
        ride.bodyIdA = chassis;
        ride.bodyIdB = wheel;
        ride.localAnchorA = (m2Vec2){x, -0.45f};
        ride.localAxisA = (m2Vec2){0.0f, 1.0f};
        ride.hertz = 4.0f;
        ride.dampingRatio = 0.7f;
        ride.enableLimit = true;
        ride.lowerTranslation = -0.2f;
        ride.upperTranslation = 0.2f;
        ride.enableMotor = true;
        ride.motorSpeed = -14.0f; // rad/s; clockwise spin drives +x
        ride.maxMotorTorque = 22.0f;
        m2CreateWheelJoint(world, &ride);
    }

    printf("time   position   speed\n");
    for (int step = 1; step <= 300; ++step)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        if (step % 30 == 0)
        {
            m2Pos2 p = m2Body_GetPosition(chassis);
            m2Vec2 v = m2Body_GetLinearVelocity(chassis);
            printf("%4.1fs  %7.3f m  %5.2f m/s\n", (double)step / 60.0, p.x, (double)v.x);
        }
    }

    printf("world hash after five seconds: %016llx\n", (unsigned long long)m2World_Hash(world));
    printf("(that number is identical on every platform - that is the point)\n");
    m2DestroyWorld(world);
    return 0;
}
