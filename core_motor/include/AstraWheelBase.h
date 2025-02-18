/**
 * @file AstraWheelBase.h
 * @author David Sharpe (ds0196@uah.edu)
 * @brief
 *
 */
#pragma once

#include "AstraMotors.h"
#ifdef TESTBED
#    include "project/TESTBED.h"
#else
#    include "project/CORE.h"
#endif

class AstraWheelBase {
    AstraMotors* motorList[4];  // Left front, left rear, right front, right rear

   public:
    AstraWheelBase(AstraMotors* setMotorList[4]);

    void setDuty(float duty);
    void setDuty(float dutyLeft, float dutyRight);
    void setSpeed(float speed);
    void setSpeed(float speedLeft, float speedRight);

    void goForwards(float duty);
    void goBackwards(float duty);
    void turnCW(float duty);
    void turnCCW(float duty);
    void stop();
    void setBrake(bool enable);

    void driveMeters(float meters);
    float getDriveSpeed();
};
