#include <deplacement.h>

Deplacement::Deplacement(){}
Deplacement::Deplacement(double goal, PID* pid_pos, ArduinoX* ax){
    this->pid_pos = pid_pos;
    (this->pid_pos)->setGoal(goal);
    this->ax = ax;
    //(this->pid_pos)->setMeasurementFunc(pulseToMeters());
}
Deplacement::~Deplacement(){    
}

void Deplacement::run(){
    pid_pos->run();
}

void Deplacement::setGoal(double goal){
    pid_pos->setGoal(goal); 
}

void Deplacement::setMotorSpeed(){
    if(distance < goal - 3200){
        ax->setMotorPWM(1, 1.0);
    }
    else if(distance < goal){
        ax->setMotorPWM(1, 0.8);
    }
}
//void Deplacement::myProtectedFunction();
    
