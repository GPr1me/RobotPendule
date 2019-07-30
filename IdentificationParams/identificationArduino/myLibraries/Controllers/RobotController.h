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

enum Status
{
    firstPositionning,
    oscillationIncrease,
    stepOverObstacle,
    oscillationDecrease,
    lastPositionning,
    returnToDefaultPosition
};

class RobotController: public Controller
{
public:
    RobotController();
    RobotController(PositionController *positionController,
                    AngleController *angleController);

    ~RobotController();

    void setupPOS(double (*measurementFunc)(),
                  void (*commandFunc)(double),
                  void (*atGoalFunc)());
    void setupANGLE(double (*angleCommand)(),
                    double (*measurementFunc)(),
                    void (*commandFunc)(double),
                    void (*atGoalFunc)());

    Controller* getActiveController();

    Status currentStatus(){ return status_; };
    void changeStatus(Status status);

    void run();

    String ToString();

private:
    PositionController *positionController_ = nullptr;
    AngleController *angleController_ = nullptr;

    bool runCompleted_;
    Status status_;
};
#endif //ROBOTCONTROLLER_H