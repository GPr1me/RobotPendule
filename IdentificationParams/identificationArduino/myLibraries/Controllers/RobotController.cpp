/*
Projet S3 GRO
Class to control the project robot mouvements
@author Marc-Olivier Thibault
@version 1.0 30/07/2019
*/

#include "RobotController.h"

RobotController::RobotController()
{
    positionController_ = new PositionController();
    angleController_ = new AngleController();
    runCompleted_ = false;
}
RobotController::RobotController(PositionController *positionController,
                                 AngleController *angleController)
{
    positionController_ = positionController;
    angleController_ = angleController;

    // Make sure nothing is currently running
    positionController_->disable();
    angleController_->disable();

    runCompleted_ = false;
}

RobotController::~RobotController()
{
    if (positionController_)
    {
        delete positionController_;
    }
    if (angleController_)
    {
        delete angleController_;
    }
}

<<<<<<< HEAD
=======
void RobotController::setupActions(int count, ...)
{
    va_list args;
    va_start(args, count);
    for(int i = 0; i < count; i++)
    {
        actions[i] = va_arg(args, Callback);
    }
    va_end(args);

    // ********************************
    // * In case variable args breaks *
    // ********************************
    // actions[0] = firstPositionning;
    // actions[1] = oscillationIncrease;
    // actions[2] = stepOverObstacle;
    // actions[3] = oscillationDecrease;
    // actions[4] = lastPositionning;
    // actions[5] = returnToDefaultPosition;
}

>>>>>>> 87478811666ae4b80fd535dfdc413febb1efcb68
void RobotController::setupPOS(double (*measurementFunc)(),
                               void (*commandFunc)(double),
                               void (*atGoalFunc)())
{
    positionController_->setup(measurementFunc, commandFunc, atGoalFunc);
}

void RobotController::setupANGLE(double (*angleCommand)(),
                                 double (*measurementFunc)(),
                                 void (*commandFunc)(double),
                                 void (*atGoalFunc)())
{
    angleController_->setup(angleCommand, measurementFunc, commandFunc, atGoalFunc);
}

String RobotController::ToString()
{
    if (getActiveController())
    {
        return getActiveController()->ToString();
    }
    return "Aucun contrôleur actif";
}

Controller *RobotController::getActiveController()
{
    if (positionController_->isEnabled)
    {
        return positionController_;
    }
    else if (angleController_->isEnabled)
    {
        return angleController_;
    }
    else
    {
        return nullptr;
    }
}

void RobotController::changeStatus(Status status)
{
}

void RobotController::run()
{
    switch (status_)
    {
    case firstPositionning:
        if (positionController_->isAtGoal())
        {
            changeStatus(oscillationIncrease);
        }
        break;

    case oscillationIncrease:
        if (angleController_->isAtGoal())
        {
            changeStatus(stepOverObstacle);
        }
        break;

    case stepOverObstacle:
        if (positionController_->isAtGoal())
        {
            changeStatus(oscillationDecrease);
        }
        break;

    case oscillationDecrease:
        if (angleController_->isAtGoal())
        {
            changeStatus(lastPositionning);
        }
        break;

    case lastPositionning:
        if (positionController_->isAtGoal())
        {
            changeStatus(returnToDefaultPosition);
        }
        break;

    case returnToDefaultPosition:
        if (positionController_->isAtGoal())
        {
            changeStatus(firstPositionning);
        }
    }
}