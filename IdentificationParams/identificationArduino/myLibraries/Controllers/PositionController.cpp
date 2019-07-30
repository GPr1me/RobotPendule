/*
Projet S3 GRO
Class to control linear mouvement
@author Marc-Olivier Thibault
@version 1.0 29/07/2019
*/

#include "PositionController.h"

PositionController::PositionController()
{
    pid_ = new PID();
    enabled_ = false;
}

PositionController::PositionController(PID* pid)
{
    pid_ = pid;
    disable(); // make sure it is not currently in use
}

PositionController::~PositionController()
{
    if (pid_)
    {
        delete pid_;
    }
}

void PositionController::setup(double (*measurementFunc)(),
                               void (*commandFunc)(double),
                               void (*atGoalFunc)())
{
    pid_->setGains(5, 0.02, 0); // CAN BE SET WITH QT?
    pid_->setMeasurementFunc(measurementFunc);
    pid_->setCommandFunc(commandFunc);
    pid_->setAtGoalFunc(atGoalFunc);
    pid_->setEpsilon(0.01); //TODO: valeur par defaut en ce moment. Effet a verifier
    pid_->setPeriod(100);
}

void PositionController::enable()
{
    pid_->enable();
    enabled_ = true;
}

void PositionController::disable()
{
    pid_->disable();
    enabled_ = false;
}

void PositionController::run()
{
    pid_->run();
}