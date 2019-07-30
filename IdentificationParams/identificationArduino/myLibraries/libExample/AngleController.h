/*
Projet S3 GRO
Class to control an angle PID
@author Marc-Olivier Thibault
@version 1.0 29/07/2019
*/

#ifndef ANGLECONTROLLER_H
#define ANGLECONTROLLER_H

#include <Arduino.h>
#include <LibS3GRO.h>

class AngleController : public PID
{
    public:
    void run();
    double getAngle();

    void setAngleCommand(double(*f)()){ angleCommand_ = f; };
    void setAtGoalFunc(void (*f)()){ atGoalFunc_ = f; };
    void setCommandFunc(void (*f)(double)){ commandFunc_ = f; };

    void setGoal(double goal){ goal_ = goal; };
    bool isAtGoal(){ return atGoal_; };

    private:
    double (*angleCommand_)() = nullptr;
    void (*atGoalFunc_)() = nullptr;
    void (*commandFunc_)(double scmd) = nullptr;

    bool enable_;
    bool goal_;
    bool atGoal_;
};
#endif