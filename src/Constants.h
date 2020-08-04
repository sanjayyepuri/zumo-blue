#pragma once
#include <Wire.h>

enum Direction
{
  DirectionLeft,
  DirectionRight,
};

// When the reading on a line sensor goes below this value, we
// consider that line sensor to have detected the white border at
// the edge of the ring.  This value might need to be tuned for
// different lighting conditions, surfaces, etc.
const uint16_t lineSensorThreshold = 1000;

// The speed that the robot uses when backing up.
const uint16_t reverseSpeed = 400;

// The speed that the robot uses when turning.
const uint16_t turnSpeedHigh = 400;
const uint16_t turnSpeedLow = 400;

// The speed that the robot usually uses when moving forward.
const uint16_t forwardSpeed = 400;

// The speed we drive when analyzing the white border.
const uint16_t analyzeSpeed = 100;

// The speed used when turning towards the center.
const uint16_t turnCenterSpeed = 400;

// The speed that the robot drives when it thinks it is pushing or
// about to push an opponent.
const uint16_t rammingSpeed = 400;

// The speed used on the non-dominant wheel to turn during ramming.
const uint16_t rammingSpeedLow = 200;

// The minimum amount of time to spend scanning for nearby
// opponents, in milliseconds.
const uint16_t scanTimeMin = 200;

// The maximum amount of time to spend scanning for nearby
// opponents, in milliseconds.
// const uint16_t scanTimeMax = 2100;

// The maximum number of degrees to turn while scanning for the opponent.
const uint16_t scanDegreesMax = 360 * 2 + 90;

// The amount of time to wait between detecting a button press
// and actually starting to move, in milliseconds.  Typical robot
// sumo rules require 5 seconds of waiting.
const uint16_t waitTime = 5000;

// If the robot has been driving forward for this amount of time,
// in milliseconds, without reaching a border, the robot decides
// that it must be pushing on another robot and this is a
// stalemate, so it increases its motor speed.
const uint16_t stalemateTime = 1500;

// The number of encoder ticks to travel when we want to go from the
// edge to the center.
const uint16_t edgeToCenterEncoderTicks = 2180;

// The number of encoder ticks to travel when backing away from the
// edge.
const uint16_t reverseEncoderTicks = 400;

// The number of encoder ticks of distance separating the middle and
// side line sensors.
const uint16_t sensorDistance = 440;