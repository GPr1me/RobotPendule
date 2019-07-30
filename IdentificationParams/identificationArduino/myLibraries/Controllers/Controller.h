/*
Projet S3 GRO
Virtual Class from wich controllers inherit
@author Marc-Olivier Thibault
@version 1.0 30/07/2019
*/

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Arduino.h>

class Controller
{
public:
    virtual void run() = 0;
    virtual bool isAtGoal();
    String ToString() { return "contrôleur: "; };
};
#endif //CONTROLLER_H