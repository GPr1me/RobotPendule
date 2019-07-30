/*
Projet S3 GRO
Class to control the project robot controller
@author Marc-Olivier Thibault
@version 1.0 30/07/2019
*/

#ifndef ROBOTCONTROLLER_H
#define ROBOTCONTROLLER_H

// #include <Arduino.h>
#include <LibS3GRO.h>
#include <Controllers/Controller.h>
#include <Controllers/AngleController.h>
#include <Controllers/PositionController.h>
#include <stdarg.h>

typedef void (*Callback)();

enum Status
{
    firstPositionning,
    oscillationIncrease,
    stepOverObstacle,
    oscillationDecrease,
    lastPositionning,
    returnToDefaultPosition
};

const int NUMBER_OF_STEPS = 6;

class RobotController : public Controller
{
public:
    RobotController(void (*firstPositionning)(),
                    void (*oscillationIncrease)(),
                    void (*stepOverObstacle)(),
                    void (*oscillationDecrease)(),
                    void (*lastPositionning)(),
                    void (*returnToDefaultPosition)());

    RobotController(PositionController *positionController,
                    AngleController *angleController,
                    void (*firstPositionning)(),
                    void (*oscillationIncrease)(),
                    void (*stepOverObstacle)(),
                    void (*oscillationDecrease)(),
                    void (*lastPositionning)(),
                    void (*returnToDefaultPosition)());

    ~RobotController();

    void setupPOS(double (*measurementFunc)(),
                  void (*commandFunc)(double),
                  void (*atGoalFunc)());

    void setupANGLE(double (*angleCommand)(),
                    double (*measurementFunc)(),
                    void (*commandFunc)(double),
                    void (*atGoalFunc)());

    // Not yet useful
    // PositionController* getPositionController(){ return positionController_; };
    // AngleController* getAngleController(){ return angleController_; };

    Controller *getActiveController();

    Status currentStatus() { return status_; };
    void changeStatus();

    void run();

    String ToString();

private:

    void setupActions(int count, ...);

    PositionController *positionController_ = nullptr;
    AngleController *angleController_ = nullptr;

    bool runCompleted_;
    Status status_;

    Callback actions[6];
};
#endif //ROBOTCONTROLLER_H