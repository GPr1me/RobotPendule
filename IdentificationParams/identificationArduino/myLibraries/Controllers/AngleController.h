/*
Projet S3 GRO
Class to control an pendulum mouvement
@author Marc-Olivier Thibault
@version 1.0 29/07/2019
*/

#ifndef ANGLECONTROLLER_H
#define ANGLECONTROLLER_H

#include <Arduino.h>
#include <PID_angle.h>

class AngleController : public Controller
{
public:
    AngleController();
    AngleController(PID_angle* pid);

    ~AngleController();

    void run();
    void setup(double (*angleCommand)(),
               double (*measurementFunc)(),
               void (*commandFunc)(double),
               void(*atGoalFunc)());

    void enable();
    void disable();

    bool isEnabled(){ return enabled_; };
    bool isAtGoal(){ return pid_->isAtGoal(); };

    String ToString(){ return "Angle"; };

private:
    PID_angle* pid_ = nullptr;
    bool enabled_;
};
#endif //ANGLECONTROLLER_H