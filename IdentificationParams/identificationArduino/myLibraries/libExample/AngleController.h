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

class AngleController : PID
{
    public:
    void run();
    double getAngle();

    void setAngleCommand(double(*f)()){ angleCommand_ = f; };

    private:
    double (*angleCommand_)() = nullptr;
};
#endif