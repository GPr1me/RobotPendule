/*
Projet S3 GRO
Class to control an angle PID
@author Marc-Olivier Thibault
@version 1.0 29/07/2019
*/

#ifndef POSITIONCONTROLLER_H
#define POSITIONCONTROLLER_H

#include <Arduino.h>
#include <LibS3GRO.h>

class PositionController : public Controller
{
public:
    PositionController();
    PositionController(PID* pid);

    ~PositionController();

    void run();
    void setup(double (*measurementFunc)(),
               void (*commandFunc)(double),
               void(*atGoalFunc)());

    void enable();
    void disable();
    bool isEnabled() { return enabled_; };
    
    bool isAtGoal(){ return pid_->isAtGoal(); };

    String ToString(){ return "Position"; };

private:
    PID *pid_ = nullptr;
    bool enabled_;
};
#endif //POSITIONCONTROLLER_H