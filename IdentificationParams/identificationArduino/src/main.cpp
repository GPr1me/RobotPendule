/* 
 * GRO 302 - Conception d'un robot mobile
 * Code de démarrage
 * Auteurs: Jean-Samuel Lauzon     
 * date: 1 mai 2019
*/

/*------------------------------ Librairies ---------------------------------*/
#include <LibS3GRO.h>
#include <ArduinoJson.h>
#include <libExample.h> // Vos propres librairies
#include <math.h>
/*------------------------------ Constantes ---------------------------------*/

#define BAUD            115200      // Frequence de transmission serielle
#define UPDATE_PERIODE  100         // Periode (ms) d'envoie d'etat general

#define MAGPIN          8          // Port numerique pour electroaimant
#define POTPIN          A5         // Port analogique pour le potentiometre

#define PASPARTOUR      64          // Nombre de pas par tour du moteur
#define RAPPORTVITESSE  50          // Rapport de vitesse du moteur

/*---------------------------- variables globales ---------------------------*/

ArduinoX AX_;                       // objet arduinoX
MegaServo servo_;                   // objet servomoteur
VexQuadEncoder vexEncoder_;         // objet encodeur vex
IMU9DOF imu_;                       // objet imu

//declaration des objets PID
PID pid_;                           // objet PID
PID pid_pos;
PID pid_ang;  //TODO

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
  float ANGULAR_RANGE = 197.0;      // °
  double pot_angle;                 // °

  double hauteur_obstacle;          // cm
  double longueur_pendule = 0.548;  // m
  double angle_goal;                // °

  double prev_p = 0;
  double cur_p;
  double cur_v;
  double cur_T;
  double lastT = 0;
  float inter_time;

  double power_ax;
  double energy_ax;

  double tcmd;
}

/*------------------------- Prototypes de fonctions -------------------------*/

void timerCallback();
void startPulse();
void endPulse();
void sendMsg(); 
void readMsg();
void serialEvent();

// Fonctions pour le PID
double computePIDPos();
void PIDcommand(double cmd);
void PIDgoalReached();

double pulseToMeters(); //fonction pour passer de pulse en m
void commandPos(double cmd); //fonction pour la commande position
double pulseToMeters();
void commandPos(double cmd);
double getVel();
double getAngle();

/*---------------------------- fonctions "Main" -----------------------------*/
//timer pour test comportement du electroaimant
unsigned long timer;

void setup() {
  Serial.begin(BAUD);               // initialisation de la communication serielle
  AX_.init();                       // initialisation de la carte ArduinoX 
  imu_.init();                      // initialisation de la centrale inertielle
  vexEncoder_.init(2,3);            // initialisation de l'encodeur VEX
  // attache de l'interruption pour encodeur vex
  attachInterrupt(vexEncoder_.getPinInt(), []{vexEncoder_.isr();}, FALLING);
  
  // Chronometre envoie message
  timerSendMsg_.setDelay(UPDATE_PERIODE);
  timerSendMsg_.setCallback(timerCallback);
  timerSendMsg_.enable();

  // Chronometre duration pulse
  timerPulse_.setCallback(endPulse);
  
  // Initialisation du PID
  /* Sample initialisation
  pid_.setGains(5, 0.01 , 0);
    // Attache des fonctions de retour
  pid_.setMeasurementFunc(computePIDPos);
  pid_.setCommandFunc(PIDcommand);
  pid_.setAtGoalFunc(PIDgoalReached);
  pid_.setEpsilon(0.001); //TODO
  
  pid_.setPeriod(1000/5);
  */

 //PID pour la position
  pid_pos.setGains(5, 0.02 , 0); //gains actuels proviennent de la simulation (valeurs a verifier) 
    // Attache des fonctions de retour
  pid_pos.setMeasurementFunc(computePIDPos);
  pid_pos.setCommandFunc(PIDcommand);
  pid_pos.setAtGoalFunc(PIDgoalReached);
  pid_pos.setEpsilon(0.005); //TODO: valeur par defaut en ce moment. Effet a verifier
  pid_pos.setPeriod(100); //1000 / 10: le pid est ajuste 10 fois par seconde (valeur peut etre changee) 

  //pour test sans qt
  pid_pos.setGoal(0.9); //valeur en distance a atteindre
  pid_pos.enable();

  //PID pour oscillations
  pid_ang.setGains(0.2, 0.01 , 0); //gains actuels proviennent de la simulation (valeurs a verifier) 
    // Attache des fonctions de retour
  pid_ang.setMeasurementFunc(computePIDAng);
  pid_ang.setCommandFunc(PIDcommand);
  pid_ang.setAtGoalFunc(PIDgoalReached);
  pid_ang.setEpsilon(0.001); //TODO: valeur par defaut en ce moment. Effet a verifier
  pid_ang.setPeriod(100);

  AX_.resetEncoder(1);

  timer = millis();

  //code pour activer le potentiometre et le electroaimant
  pinMode(MAGPIN, OUTPUT);
  pinMode(POTPIN, INPUT);

}

/* Boucle principale (infinie)*/
void loop() {

  //code test pour faire avancer le robot
  /*AX_.setMotorPWM(REAR, 1);
  AX_.setMotorPWM(FRONT, -1);*/
  //test pour voir Vmax selon mesures
  //Serial.println(getVel());
  
  //code test pour activer le electroaimant pendant 10 secondes
  if(10000 > (millis() - timer) ){
    digitalWrite(MAGPIN, 1);
  }
  else{
    digitalWrite(MAGPIN, 0);
  }
  
   
  if(shouldRead_){
    readMsg();
  }
  if(shouldSend_){
    sendMsg();
  }
  if(shouldPulse_){
    startPulse();
  }

  // mise a jour des chronometres
  timerSendMsg_.update();
  timerPulse_.update();
  
  // mise à jour du PID
  pid_pos.run();
  
}

/*---------------------------Definition de fonctions ------------------------*/

void serialEvent(){shouldRead_ = true;}

void timerCallback(){shouldSend_ = true;}

void startPulse(){
  /* Demarrage d'un pulse */
  timerPulse_.setDelay(pulseTime_);
  timerPulse_.enable();
  timerPulse_.setRepetition(1);
  AX_.setMotorPWM(0, pulsePWM_);
  AX_.setMotorPWM(1, pulsePWM_);
  shouldPulse_ = false;
  isInPulse_ = true;
}

void endPulse(){
  /* Rappel du chronometre */
  AX_.setMotorPWM(0, 0);
  AX_.setMotorPWM(1, 1);
  timerPulse_.disable();
  isInPulse_ = false;
}

void sendMsg(){
  /* Envoit du message Json sur le port seriel */
  StaticJsonDocument<500> doc;
  // Elements du message

  doc["cmd"] = tcmd;
  doc["time"] = millis();
  doc["potVex"] = pot_angle;
  doc["hauteurObstacle"] = hauteur_obstacle;

  //doc["encVex"] = vexEncoder_.getCount();
  doc["goal"]      = pid_pos.getGoal();
  doc["motorPos"]  = computePIDPos();
  doc["power"]     = power_ax;
  doc["energy"]    = energy_ax;
  doc["pulsePWM"]  = pulsePWM_;
  doc["pulseTime"] = pulseTime_;
  doc["inPulse"]   = isInPulse_;
  //doc["accelX"]    = imu_.getAccelX();
  //doc["accelY"]    = imu_.getAccelY();
  //doc["accelZ"]    = imu_.getAccelZ();
  //doc["gyroX"]     = imu_.getGyroX();
  //doc["gyroY"]     = imu_.getGyroY();
  //doc["gyroZ"]     = imu_.getGyroZ();
  doc["isGoal"]   = pid_pos.isAtGoal();

  // Serialisation
  serializeJson(doc, Serial);
  // Envoit
  Serial.println();
  shouldSend_ = false;
}

void readMsg(){
  // Lecture du message Json
  StaticJsonDocument<500> doc;
  JsonVariant parse_msg;

  // Lecture sur le port Seriel
  DeserializationError error = deserializeJson(doc, Serial);
  shouldRead_ = false;

  // Si erreur dans le message
  if (error) {
    Serial.print("deserialize() failed: ");
    Serial.println(error.c_str());
    return;
  }
  
  // Analyse des éléments du message message
  parse_msg = doc["hauteurObstacle"];
  if(!parse_msg.isNull()){
     hauteur_obstacle = doc["hauteurObstacle"].as<float>();
  }

  parse_msg = doc["pulsePWM"];
  if(!parse_msg.isNull()){
     pulsePWM_ = doc["pulsePWM"].as<float>();
  }

  parse_msg = doc["pulseTime"];
  if(!parse_msg.isNull()){
     pulseTime_ = doc["pulseTime"].as<float>();
  }

  parse_msg = doc["pulse"];
  if(!parse_msg.isNull()){
     shouldPulse_ = doc["pulse"];
  }
}

void computeAngleGoal(){
  // h = L(1-cos(theta))
  hauteur_obstacle *= 0.01;
  angle_goal = 180*acos(1 - hauteur_obstacle / longueur_pendule)/PI;
}

void computePowerEnergy(){
  power_ax = AX_.getVoltage() * AX_.getCurrent();
  energy_ax = power_ax / inter_time;
}

// Fonctions pour le PID

double pulseToMeters(){
    //3200 pulses par tour de roue
    //conversion vers rads: encoches/ 3200 * 2 * pi
    //longueur de l'arc: angle_en_rads * r
    return AX_.readEncoder(0) / float(PASPARTOUR * RAPPORTVITESSE) * 2 * PI * 0.05;   
}

//fonction pour mesurer la vitesse actuelle
double getAngle(){
  // Lecture de tension recentree
  int pot_read = analogRead(POTPIN);
  pot_read -= POTAVG;

  // Conversion tension a angle
  float pot_ratio = float(POTMAX - POTMIN) / ANGULAR_RANGE;
  pot_angle = pot_read / pot_ratio;
  
  return pot_angle;  
}

double getVel(){
  // devrait lire la valeur de la vitesse
  //pos actuelle
  cur_p = pulseToMeters();
  //temps actuel
  cur_T = millis() / 1000;
  //intervalle de temps
  inter_time = cur_T - lastT;
  //calcul vitesse
  cur_v = (cur_p - prev_p)/inter_time;
  //store cur pos as prev pos
  prev_p = cur_p;
  //store cur t as prev t
  lastT = cur_T;
  //retourne la vitesse calculee
  return cur_v;
}  
//mesure la distance parcourue
double computePIDPos(){
  return pulseToMeters();
}
//mesure l'angle du pendule
double computePIDAng(){
  return getAngle();
}

void PIDcommand(double cmd){
  //tcmd utilise pour pouvoir voir la valeur de la cmd envoyee
  tcmd = cmd;
  //Comportement du PID a verifier
  if(cmd > 1){
    AX_.setMotorPWM(0, 1);
    AX_.setMotorPWM(1, -1);
  }
  else if(cmd < -1){
    AX_.setMotorPWM(0, -1);
    AX_.setMotorPWM(1, 1);
  }
  else{
    AX_.setMotorPWM(0, cmd);
    AX_.setMotorPWM(1, -cmd);
  }
}

//lorsque objectif atteint arrete au complet et recommence les encodeurs pour mesurer une nouvelle distance
void PIDgoalReached(){
  // To do
  AX_.setMotorPWM(0, 0);
  AX_.setMotorPWM(1, 0);
  //Serial.println("Valeur de distance mesuree:");
  //Serial.println(pulseToMeters());
  AX_.resetEncoder(1);
}

//premier essaie pour le controleur de la position
/* ceci etait cree avant l'app: il est possibe de le modifier pour utiliser un plus semblable a ce qui
a ete fait dans l'app 6 */
/* void commandPos(double cmd){
  //commande si positif
  if(pid_pos.getGoal() > 0){
    if(cmd >= 0.05){
      AX_.setMotorPWM(0, 1.0);
      AX_.setMotorPWM(1, 1.0);
    }
    else if(0.01 <= cmd && cmd < 0.05 ){
      AX_.setMotorPWM(0, 0.8);
      AX_.setMotorPWM(1, 0.8);
    }
    else if(0 < cmd && cmd < 0.01){
      AX_.setMotorPWM(0, 0.3);
      AX_.setMotorPWM(1, 0.3);
    }
    else{
      AX_.setMotorPWM(0, 0);
      AX_.setMotorPWM(1, 0);
    }
  }
  //commande si negatif
  else{
    if(cmd <= -0.05){
      AX_.setMotorPWM(0, -1.0);
      AX_.setMotorPWM(1, -1.0);
    }
    else if(-0.01 >= cmd && cmd > -0.05 ){
      AX_.setMotorPWM(0, -0.8);
      AX_.setMotorPWM(1, -0.8);
    }
    else if(0 > cmd && cmd > -0.01){
      AX_.setMotorPWM(0, -0.3);
      AX_.setMotorPWM(1, -0.3);
    }
    else{
      AX_.setMotorPWM(0, 0);
      AX_.setMotorPWM(1, 0);
    }
  } 
  
}*/