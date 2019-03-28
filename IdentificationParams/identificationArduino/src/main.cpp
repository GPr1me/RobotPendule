/* Code exemple pour projet S3 GRO
 * Auteurs: Jean-Samuel Lauzon     
 * date:    17 mars 2019
*/

/*------------------------------ Librairies ---------------------------------*/
#include <LibS3GRO.h>
#include <ArduinoJson.h>

/*------------------------------ Constantes ---------------------------------*/
#define BAUD            115200 // Frequence de transmission serielle
#define UPDATE_PERIODE  100  // Periode (ms) d'envoie d'etat general

#define MAGPIN          32  // Port numerique pour electroaimant
#define POTPIN          A5  // Port analogique pour le potentiometre

/*---------------------------- variables globales ---------------------------*/
ArduinoX AX_; // objet arduinoX
MegaServo servo_; // objet servomoteur
VexQuadEncoder vexEncoder_; // encodeur vex
IMU9DOF imu_; // encodeur vex

volatile bool shouldSend_ = false;  // drapeau prêt à envoer un message
volatile bool shouldRead_ = false;  // drapeau prêt à lire un message
volatile bool shouldPulse_ = false; // drapeau pour effectuer un pulse
volatile bool isInPulse_ = false; // drapeau pour effectuer un pulse

SoftTimer timerSendMsg_;    // chronometre d'envoie de messages
SoftTimer timerPulse_;      // chronometre pour la duree d'un pulse

uint16_t pulseTime_ = 0; // temps dun pulse en ms
float pulsePWM_ = 0;     // Amplitude de la tension au moteur [-1,1]


float Axyz[3]; // Accelerometre
float Gxyz[3]; // Giroscope
float Mxyz[3]; // Magnetometre

/*------------------------- Prototypes de fonctions -------------------------*/

void timerCallback();
void startPulse();
void endPulse();
void sendMsg();
void readMsg();
void serialEvent();

/*---------------------------- fonctions "Main" -----------------------------*/

void setup() {
  Serial.begin(BAUD); // initialisation de la communication serielle
  AX_.init(); // initialisation de la carte ArduinoX 
  imu_.init(); // initialisation de la centrale inertielle
  vexEncoder_.init(2,3);// initialisation de l'encodeur VEX
  attachInterrupt(vexEncoder_.getPinInt(), []{vexEncoder_.isr();}, FALLING);
  
  // Chronometre envoie message
  timerSendMsg_.setDelay(UPDATE_PERIODE);
  timerSendMsg_.setCallback(timerCallback);
  timerSendMsg_.enable();

  // Chronometre duration pulse
  timerPulse_.setCallback(endPulse);
}

/* Fonction boucle infinie*/
void loop() {
  if(shouldRead_){
    readMsg();
  }
  if(shouldSend_){
    sendMsg();
  }
  if(shouldPulse_){
    startPulse();
  }
  // Mise a jour des chronometres
  timerSendMsg_.update();
  timerPulse_.update();
}

/*---------------------------Definition de fonctions ------------------------*/

void serialEvent(){shouldRead_ = true;}

void timerCallback(){shouldSend_ = true;}

void startPulse(){
  /* Demarrage d'un pulse */
  timerPulse_.setDelay(pulseTime_);
  timerPulse_.enable();
  timerPulse_.setRepetition(1);
  AX_.setSpeedMotor(0,pulsePWM_);
  AX_.setSpeedMotor(1,pulsePWM_);
  shouldPulse_ = false;
  isInPulse_ = true;
}

void endPulse(){
  /* Rappel du chronometre */
  AX_.setSpeedMotor(0,0);
  AX_.setSpeedMotor(1,0);
  timerPulse_.disable();
  isInPulse_ = false;
}

void sendMsg(){
  /* Envoit du message Json sur le port seriel */
  StaticJsonDocument<200> doc;
  // elements du message
  doc["time"] = millis();
  doc["potVex"] = analogRead(POTPIN);
  doc["encVex"] = vexEncoder_.getCount();
  doc["voltage"] = AX_.getVoltage();
  doc["current"] = AX_.getCurrent(); 
  doc["pulsePWM"] = pulsePWM_;
  doc["pulseTime"] = pulseTime_;
  doc["inPulse"] = isInPulse_;
  doc["accelX"] = imu_.getAccelX();
  doc["accelY"] = imu_.getAccelY();
  doc["accelZ"] = imu_.getAccelZ();
  // Serialisation
  serializeJson(doc, Serial);
  // Envoit
  Serial.println();
  shouldSend_ = false;
}

void readMsg(){
  // Lecture du message Json
  StaticJsonDocument<200> doc;
  JsonVariant parse_msg;

  // Lecture sur le port Seriel
  DeserializationError error = deserializeJson(doc, Serial);;
  shouldRead_ = false;

  // Si erreur dans le message
  if (error) {
    Serial.print("deserialize() failed: ");
    Serial.println(error.c_str());
    return;
  }
  
  // Analyse du message
  parse_msg = doc["pulsePWM"];
  if(!parse_msg.isNull()){
     pulsePWM_ = doc["pulsePWM"];
  }

  parse_msg = doc["pulseTime"];
  if(!parse_msg.isNull()){
     pulseTime_ = doc["pulseTime"];
  }

  parse_msg = doc["pulse"];
  if(!parse_msg.isNull()){
     shouldPulse_ = doc["pulse"];
  }
}