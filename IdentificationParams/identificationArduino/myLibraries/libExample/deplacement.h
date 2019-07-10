#ifndef deplacement_H_
#define deplacement_H_

#include <ArduinoX/ArduinoX.h>
#include <PID/PID.h>
//#include <function>

//class to reach a distance in a short amount of time
class Deplacement
{
    public:
    Deplacement();
    Deplacement(double goal, PID* pid_pos, ArduinoX* ax);
    ~Deplacement();

    void run();
    void setGoal(double goal);
    double getDistance();
    
    //protected:
    //void myProtectedFunction();
    
    private:
    void setMotorSpeed();
    double pulseToMeters();
    double distance = 0;
    double goal = 0;
    ArduinoX* ax;
    PID* pid_pos;
};
#endif // LibExample_H_