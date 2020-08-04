#pragma once

#include <Zumo32U4.h>
#include "Constants.h"
#include "SmartProximitySensor.h"

// extern all the variables so that they are in main.cpp
extern Zumo32U4LineSensors lineSensors;
extern Zumo32U4Encoders encoders;
extern unsigned int lineSensorValues[3];

extern bool lastStopAtEdge;

extern void changeStateToPausing();
extern void changeStateToWaiting();
extern void changeStateToTurningToCenter();
extern void changeStateToDriving();
extern void changeStateToPushing();
extern void changeStateToBacking();
extern void changeStateToScanning();
void changeStateToAnalyzingBorder();

// If we have driven far enough to get into the center, then start
// scanning.  You can point the robot at the center if you want it to go there,
// or you can point it at a nearby part of the border.  This behavior is
// dangerous because we might be really close to the edge, so we only do it
// if our last stopping point was at an edge.
inline bool toCenterSub()
{
    int16_t counts = encoders.getCountsLeft() + encoders.getCountsRight();

    if (lastStopAtEdge && counts > (int16_t)edgeToCenterEncoderTicks * 2)
    {
        changeStateToScanning();
    }
    return false;
}

// Check for the white border.
inline bool lineSensorSub(Direction &dir)
{
    lineSensors.read(lineSensorValues);
    if (lineSensorValues[0] < lineSensorThreshold)
    {
        dir = DirectionRight;
        changeStateToAnalyzingBorder();
        return true;
    }
    if (lineSensorValues[2] < lineSensorThreshold)
    {
        dir = DirectionRight;
        changeStateToAnalyzingBorder();
        return true;
    }
    return false;
}

// Read the proximity sensors to sense the opponent.
inline bool proximitySub()
{
    sense();
    if (objectSeen)
    {
        changeStateToPushing();
        return true;
    }
    return false;
}