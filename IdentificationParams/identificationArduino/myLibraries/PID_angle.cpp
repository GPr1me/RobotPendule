/*
Projet S3 GRO
Class to control an angle PID
@author Marc-Olivier Thibault
@version 1.0 29/07/2019
*/

#include "PID_angle.h"

void PID_angle::run()
{
    if (enable_)
    {
        unsigned long initTW = millis();
        if (angleCommand_() < goal_ && angleCommand_() > -goal_ && enable_)
        {
            // AMPLITUDE TO BE SET ACCORDING TO THE OBJECTIVE (MINIMISE ENERGY OR TIME)
            unsigned long tWave = millis() - initTW;
            double scmd = 0.4 * sin(4.8 * tWave / 1000.0);
            commandFunc_(scmd);
        }
        else
        {
            atGoal_ = true;
            enable_ = false;
            if(atGoalFunc_ != nullptr)
            {
                atGoalFunc_();
            }
        }
    }
}