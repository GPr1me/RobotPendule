/*
Projet S3 GRO
Class to control pendulum mouvement
@author Marc-Olivier Thibault
@version 1.0 29/07/2019
*/

#include "AngleController.h"

AngleController::AngleController()
{
    pid_ = new PID_angle();
    enabled_ = false;
}

AngleController::AngleController(PID_angle* pid)
{
    pid_ = pid;
    disable(); // Make sure the pid isn't currently in use
}

AngleController::~AngleController()
{
    if(pid_)
    {
        delete pid_;
    }
}

void AngleController::setup(double (*angleCommand)(),
                            double (*measurementFunc)(),
                            void (*commandFunc)(double),
                            void(*atGoalFunc)())
{
    pid_->setGains(0.008, 0, 0.0002);
    pid_->setMeasurementFunc(measurementFunc);
    pid_->setCommandFunc(commandFunc);
    pid_->setAtGoalFunc(atGoalFunc);
    pid_->setEpsilon(1); //TODO: valeur par defaut en ce moment. Effet a verifier
    pid_->setPeriod(20);
}

void AngleController::enable()
{
    pid_->enable();
    enabled_ = true;
}

void AngleController::disable()
{
    pid_->disable();
    enabled_ = false;
}

void AngleController::run()
{
    pid_->run();
}