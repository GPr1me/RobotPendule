/* 
 * GRO 302 - Conception d'un robot mobile
 * Code de démarrage
 * Auteurs: Jean-Samuel Lauzon     
 * date: 1 mai 2019
*/

/*------------------------------ Librairies ---------------------------------*/
#include <LibS3GRO.h>
#include <ArduinoJson.h>
#include <Controllers/RobotController.h>
#include <math.h>

/*------------------------------ Constantes ---------------------------------*/

#define BAUD 115200        // Frequence de transmission serielle
#define UPDATE_PERIODE 100 // Periode (ms) d'envoie d'etat general

#define MAGPIN 32 // Port numerique pour electroaimant
#define POTPIN A5 // Port analogique pour le potentiometre

#define PASPARTOUR 64     // Nombre de pas par tour du moteur
#define RAPPORTVITESSE 50 // Rapport de vitesse du moteur

/*---------------------------- variables globales ---------------------------*/

ArduinoX AX_;               // objet arduinoX
MegaServo servo_;           // objet servomoteur
VexQuadEncoder vexEncoder_; // objet encodeur vex
IMU9DOF imu_;               // objet imu
// PID pid_;                           // objet PID
RobotController *controller;

volatile bool shouldSend_  = false; // drapeau prêt à envoyer un message
volatile bool shouldRead_  = false; // drapeau prêt à lire un message
volatile bool shouldPulse_ = false; // drapeau pour effectuer un pulse
volatile bool isInPulse_   = false; // drapeau pour effectuer un pulse

SoftTimer timerSendMsg_;            // chronometre d'envoie de messages
SoftTimer timerPulse_;              // chronometre pour la duree d'un pulse

uint16_t pulseTime_ = 0;            // temps dun pulse en ms
float pulsePWM_     = 0;            // Amplitude de la tension au moteur [-1,1]

float Axyz[3];                      // tableau pour accelerometre
float Gxyz[3];                      // tableau pour giroscope
float Mxyz[3];                      // tableau pour magnetometre

//identification des moteurs
enum engines{
  REAR, FRONT
};

namespace {
  int POTMIN = 90;
  int POTMAX = 1023;
  int POTAVG = 559;
  double ANGULAR_RANGE = 197.0;      // °
  double pot_angle;                 // °

  double hauteur_obstacle;          // cm
  double longueur_pendule = 0.548;  // m
  double angle_goal;                // °

  double prev_p = 0;
  double cur_p;
  double cur_v;
  double cur_T;
  double lastT = 0;
  double prev_Ti;
  double prev_phi;
  double speed_phi;
  float  inter_time;
  double dist_;

  double power_ax;
  double energy_ax;
  double pot_ratio = (POTMAX - POTMIN) / ANGULAR_RANGE;

  double tcmd;
  double acmd;

  unsigned long tWave;
  bool wFlag;
  bool firstRun;
  bool endRun;

  int countA;
}

// volatile bool shouldSend_ = false;  // drapeau prêt à envoyer un message
// volatile bool shouldRead_ = false;  // drapeau prêt à lire un message
// volatile bool shouldPulse_ = false; // drapeau pour effectuer un pulse
// volatile bool isInPulse_ = false;   // drapeau pour effectuer un pulse

// SoftTimer timerSendMsg_; // chronometre d'envoie de messages
// SoftTimer timerPulse_;   // chronometre pour la duree d'un pulse

// uint16_t pulseTime_ = 0; // temps dun pulse en ms
// float pulsePWM_ = 0;     // Amplitude de la tension au moteur [-1,1]

// float Axyz[3]; // tableau pour accelerometre
// float Gxyz[3]; // tableau pour giroscope
// float Mxyz[3]; // tableau pour magnetometre

/*------------------------- Prototypes de fonctions -------------------------*/

void timerCallback();
void startPulse();
void endPulse();
void sendMsg(); 
void readMsg();
void serialEvent();

//fonction pour oscillations
void reachAngle(double angle);

// Fonctions pour le PID
double computePIDPos();
double computePIDAng();
void PIDcommand(double cmd);
void PIDgoalReached();

double pulseToMeters(); //fonction pour passer de pulse en m
void commandPos(double cmd); //fonction pour la commande position
double pulseToMeters();
void commandPos(double cmd);
double getVel();
double getAngle();
void goalReachedAngle(); //gestion pour maintenir l'angle pendant une distane donnee
void PIDcommandAngle(double cmd);
double getAngleSpeed();
void computeAngleGoal();
void computePowerEnergy();

/*---------------------------- fonctions "Main" -----------------------------*/

void setup()
{
  Serial.begin(BAUD);     // initialisation de la communication serielle
  AX_.init();             // initialisation de la carte ArduinoX
  imu_.init();            // initialisation de la centrale inertielle
  vexEncoder_.init(2, 3); // initialisation de l'encodeur VEX
  // attache de l'interruption pour encodeur vex
  attachInterrupt(vexEncoder_.getPinInt(), [] { vexEncoder_.isr(); }, FALLING);

  // Chronometre envoie message
  timerSendMsg_.setDelay(UPDATE_PERIODE);
  timerSendMsg_.setCallback(timerCallback);
  timerSendMsg_.enable();

  // Chronometre duration pulse
  timerPulse_.setCallback(endPulse);

  // Init controller
  controller = new RobotController();

  controller->setupPOS(computePIDPos, PIDcommand, PIDgoalReached);
  controller->setupANGLE(getAngle, computePIDAng, PIDcommandAngle, goalReachedAngle);
}

/* Boucle principale (infinie)*/
void loop()
{

  if (shouldRead_)
  {
    readMsg();
  }
  if (shouldSend_)
  {
    sendMsg();
  }
  if (shouldPulse_)
  {
    startPulse();
  }

  // mise a jour des chronometres
  timerSendMsg_.update();
  timerPulse_.update();

  // mise à jour du PID
  controller->run();
}

/*---------------------------Definition de fonctions ------------------------*/

void serialEvent() { shouldRead_ = true; }

void timerCallback() { shouldSend_ = true; }

void startPulse()
{
  /* Demarrage d'un pulse */
  timerPulse_.setDelay(pulseTime_);
  timerPulse_.enable();
  timerPulse_.setRepetition(1);
  AX_.setMotorPWM(0, pulsePWM_);
  AX_.setMotorPWM(1, pulsePWM_);
  shouldPulse_ = false;
  isInPulse_ = true;
}

void endPulse()
{
  /* Rappel du chronometre */
  AX_.setMotorPWM(0, 0);
  AX_.setMotorPWM(1, 0);
  timerPulse_.disable();
  isInPulse_ = false;
}

void sendMsg()
{
  /* Envoit du message Json sur le port seriel */
  StaticJsonDocument<500> doc;
  // Elements du message

  doc["time"] = millis();
  doc["potVex"] = analogRead(POTPIN);
  doc["encVex"] = vexEncoder_.getCount();
  doc["goal"] = controller->getActiveController->getGoal();
  doc["motorPos"] = PIDmeasurement();
  doc["voltage"] = AX_.getVoltage();
  doc["current"] = AX_.getCurrent();
  doc["pulsePWM"] = pulsePWM_;
  doc["pulseTime"] = pulseTime_;
  doc["inPulse"] = isInPulse_;
  doc["accelX"] = imu_.getAccelX();
  doc["accelY"] = imu_.getAccelY();
  doc["accelZ"] = imu_.getAccelZ();
  doc["gyroX"] = imu_.getGyroX();
  doc["gyroY"] = imu_.getGyroY();
  doc["gyroZ"] = imu_.getGyroZ();
  doc["isGoal"] = controller->getActiveController->isAtGoal();

  // Serialisation
  serializeJson(doc, Serial);
  // Envoit
  Serial.println();
  shouldSend_ = false;
}

void readMsg()
{
  // Lecture du message Json
  StaticJsonDocument<500> doc;
  JsonVariant parse_msg;

  // Lecture sur le port Seriel
  DeserializationError error = deserializeJson(doc, Serial);
  shouldRead_ = false;

  // Si erreur dans le message
  if (error)
  {
    Serial.print("deserialize() failed: ");
    Serial.println(error.c_str());
    return;
  }

  // Analyse des éléments du message message
  parse_msg = doc["pulsePWM"];
  if (!parse_msg.isNull())
  {
    pulsePWM_ = doc["pulsePWM"].as<float>();
  }

  parse_msg = doc["pulseTime"];
  if (!parse_msg.isNull())
  {
    pulseTime_ = doc["pulseTime"].as<float>();
  }

  parse_msg = doc["pulse"];
  if (!parse_msg.isNull())
  {
    shouldPulse_ = doc["pulse"];
  }
}

// Fonctions pour le PID
double PIDmeasurement()
{
  // To do
  return 0;
}
void PIDcommand(double cmd)
{
  // To do
}
void PIDgoalReached()
{
  // To do
}

void angleCommandFunc(double scmd)
{
  AX_.setMotorPWM(0, scmd);
  AX_.setMotorPWM(1, scmd);
}

// mesure la distance parcourue
double computePIDPos()
{
  return AX_.readEncoder(0) / float(PASPARTOUR * RAPPORTVITESSE) * 2 * PI * 0.05;
}

//mesure l'angle du pendule
double computePIDAng(){
  double angle = getAngle();
  if(angle < 2 && angle >= 0){
    double sp = getAngleSpeed();
    if(sp > -40 && sp < 40){
      return angle + sp*0.1;
    }
    else{
      return angle;
    }  
  }
  else if(angle > -2 && angle < 0){
    double sp = getAngleSpeed();
    if(sp > -40 && sp < 40){
      return angle * 2 + sp*0.1;
    }
    else{
      return angle;
    }
  }
  return getAngle()+getAngleSpeed()*0;
}

// //calcul de la vitesse angulaire
// double getAngleSpeed(){
//   double cur_phi = getAngle();
//   double cur_Ti = millis();
//   speed_phi = (cur_phi - prev_phi)/((cur_Ti - prev_Ti)/1000.0);
//   prev_Ti = cur_Ti;
//   prev_phi = cur_phi;
//   return speed_phi;
// }

//fonction pour mesurer l'angle actuel
double getAngle(){
  // Lecture de tension recentree
  int pot_read = analogRead(POTPIN);
  pot_read -= POTAVG;

  // Conversion tension a angle
  pot_angle = pot_read / -pot_ratio;
  
  return pot_angle;  
}