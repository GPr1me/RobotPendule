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
/*------------------------------ Constantes ---------------------------------*/

#define BAUD            115200      // Frequence de transmission serielle
#define UPDATE_PERIODE  100         // Periode (ms) d'envoie d'etat general

#define MAGPIN          32          // Port numerique pour electroaimant
#define POTPIN          A5          // Port analogique pour le potentiometre

#define PASPARTOUR      64          // Nombre de pas par tour du moteur
#define RAPPORTVITESSE  50          // Rapport de vitesse du moteur

/*---------------------------- variables globales ---------------------------*/

ArduinoX AX_;                       // objet arduinoX
MegaServo servo_;                   // objet servomoteur
VexQuadEncoder vexEncoder_;         // objet encodeur vex
IMU9DOF imu_;                       // objet imu
PID pid_;                           // objet PID
PID pid_pos;
PID pid_pendule;


volatile bool shouldSend_ = false;  // drapeau prêt à envoyer un message
volatile bool shouldRead_ = false;  // drapeau prêt à lire un message
volatile bool shouldPulse_ = false; // drapeau pour effectuer un pulse
volatile bool isInPulse_ = false;   // drapeau pour effectuer un pulse

SoftTimer timerSendMsg_;            // chronometre d'envoie de messages
SoftTimer timerPulse_;              // chronometre pour la duree d'un pulse

uint16_t pulseTime_ = 0;            // temps dun pulse en ms
float pulsePWM_ = 0;                // Amplitude de la tension au moteur [-1,1]


float Axyz[3];                      // tableau pour accelerometre
float Gxyz[3];                      // tableau pour giroscope
float Mxyz[3];                      // tableau pour magnetometre

enum engines{
REAR,
FRONT

};

/*------------------------- Prototypes de fonctions -------------------------*/

void timerCallback();
void startPulse();
void endPulse();
void sendMsg(); 
void readMsg();
void serialEvent();

// Fonctions pour le PID
double PIDmeasurement();
void PIDcommand(double cmd);
void PIDgoalReached();
double pulseToMeters();
void commandPos(double cmd);

/*---------------------------- fonctions "Main" -----------------------------*/

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
  /*
  pid_.setGains(5, 0.01 , 0);
    // Attache des fonctions de retour
  pid_.setMeasurementFunc(PIDmeasurement);
  pid_.setCommandFunc(PIDcommand);
  pid_.setAtGoalFunc(PIDgoalReached);
  pid_.setEpsilon(0.001); //TODO
  
  pid_.setPeriod(1/4.8);
  */
  pid_pos.setGains(5, 0.01 , 0);
    // Attache des fonctions de retour
  pid_pos.setMeasurementFunc(pulseToMeters);
  pid_pos.setCommandFunc(PIDcommand);
  pid_pos.setAtGoalFunc(PIDgoalReached);
  pid_pos.setEpsilon(0.001); //TODO
  pid_pos.setPeriod(1/4.8);

  pid_pendule.setGains(0.2, 0.01 , 0);

}

/* Boucle principale (infinie)*/
void loop() {
  AX_.setMotorPWM(REAR, 1);
  AX_.setMotorPWM(FRONT, -1);  
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
  pid_.run();
}

/*---------------------------Definition de fonctions ------------------------*/

void serialEvent(){shouldRead_ = true;}

void timerCallback(){shouldSend_ = true;}

void startPulse(){
  /* Demarrage d'un pulse */
  timerPulse_.setDelay(pulseTime_);
  timerPulse_.enable();
  timerPulse_.setRepetition(1);
  AX_.setMotorPWM(REAR, pulsePWM_);
  AX_.setMotorPWM(FRONT, pulsePWM_);
  shouldPulse_ = false;
  isInPulse_ = true;
}

void endPulse(){
  /* Rappel du chronometre */
  AX_.setMotorPWM(REAR, 0);
  AX_.setMotorPWM(FRONT, 1);
  timerPulse_.disable();
  isInPulse_ = false;
}

void sendMsg(){
  /* Envoit du message Json sur le port seriel */
  StaticJsonDocument<500> doc;
  // Elements du message

  doc["time"] = millis();
  doc["potVex"] = analogRead(POTPIN);
  doc["encVex"] = vexEncoder_.getCount();
  doc["goal"] = pid_.getGoal();
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
  doc["isGoal"] = pid_.isAtGoal();

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


// Fonctions pour le PID
double PIDmeasurement(){
  // To do
  return 0;
}
void PIDcommand(double cmd){
  // To do
}
void PIDgoalReached(){
  // To do
}

double pulseToMeters(){
    //3200 pulses par tour de roue
    //conversion vers rads: encoches/ 3200 * 2 * pi
    //longueur de l'arc: angle_en_rads * r
    return AX_.readEncoder(1) / 3200 * 2 * PI * 0.05;   
}

void commandPos(double cmd){
  //commande si positif
  if(pid_pos.getGoal() > 0){
    if(cmd >= 0.05){
      AX_.setMotorPWM(REAR, 1.0);
      AX_.setMotorPWM(FRONT, 1.0);
    }
    else if(0.01 <= cmd && cmd < 0.05 ){
      AX_.setMotorPWM(REAR, 0.8);
      AX_.setMotorPWM(FRONT, 0.8);
    }
    else if(0 < cmd && cmd < 0.01){
      AX_.setMotorPWM(REAR, 0.3);
      AX_.setMotorPWM(FRONT, 0.3);
    }
    else{
      AX_.setMotorPWM(REAR, 0);
      AX_.setMotorPWM(FRONT, 0);
    }
  }
  //commande si negatif
  else{
    if(cmd <= -0.05){
      AX_.setMotorPWM(REAR, -1.0);
      AX_.setMotorPWM(FRONT, -1.0);
    }
    else if(-0.01 >= cmd && cmd > -0.05 ){
      AX_.setMotorPWM(REAR, -0.8);
      AX_.setMotorPWM(FRONT, -0.8);
    }
    else if(0 > cmd && cmd > -0.01){
      AX_.setMotorPWM(REAR, -0.3);
      AX_.setMotorPWM(FRONT, -0.3);
    }
    else{
      AX_.setMotorPWM(REAR, 0);
      AX_.setMotorPWM(FRONT, 0);
    }
  } 
}