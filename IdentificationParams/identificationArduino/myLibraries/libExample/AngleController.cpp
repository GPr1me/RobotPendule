/*
Projet S3 GRO
Class to control an angle PID
@author Marc-Olivier Thibault
@version 1.0 29/07/2019
*/

#include "AngleController.h"

void AngleController::run()
{
    if (enable_)
    {
        unsigned long initTW = millis();
        if (getAngle() < goal_ && getAngle() > -goal_ && enable_)
        {
            unsigned long tWave = millis() - initTW;
            double scmd = 0.4 * sin(4.8 * tWave / 1000.0);
            commandFunc_(scmd);
        }
        else
        {
            atGoal_ = true;
            enable_ = false;
            if(atGoalFunc != nullptr)
            {
                atGoalFunc();
            }
        }
    }
}