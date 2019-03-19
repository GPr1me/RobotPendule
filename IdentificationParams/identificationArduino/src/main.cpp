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
ArduinoX __AX__; // objet arduinoX
MegaServo __servo__; // objet servomoteur
VexQuadEncoder __vex__; // encodeur vex

volatile bool __shouldSend__ = false;  // drapeau prêt à envoer un message
volatile bool __shouldRead__ = false;  // drapeau prêt à lire un message
volatile bool __shouldPulse__ = false; // drapeau pour effectuer un pulse

SoftTimer __timerSendMsg__;    // chronometre d'envoie de messages
SoftTimer __timerPulse__;      // chronometre pour la duree d'un pulse

uint16_t __pulseTime__ = 0; // temps dun pulse en ms
float __pulsePWM__ = 0;     // Amplitude de la tension au moteur [-1,1]

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
  __AX__.init(); // initialisation de la carte ArduinoX 
  __vex__.init(2,3);// initialisation de l'encodeur VEX
  attachInterrupt(__vex__.getPinInt(), []{__vex__.isr();}, FALLING);
  
  // Chronometre envoie message
  __timerSendMsg__.setDelay(UPDATE_PERIODE);
  __timerSendMsg__.setCallback(timerCallback);
  __timerSendMsg__.enable();

  // Chronometre duration pulse
  __timerPulse__.setCallback(endPulse);
}

/* Fonction boucle infinie*/
void loop() {
  if(__shouldRead__){
    readMsg();
  }
  if(__shouldSend__){
    sendMsg();
  }
  if(__shouldPulse__){
    startPulse();
  }
  // Mise a jour des chronometres
  __timerSendMsg__.update();
  __timerPulse__.update();
}

/*---------------------------Definition de fonctions ------------------------*/

void serialEvent(){__shouldRead__ = true;}

void timerCallback(){__shouldSend__ = true;}

void startPulse(){
  /* Demarrage d'un pulse */
  __timerPulse__.setDelay(__pulseTime__);
  __timerPulse__.enable();
  __timerPulse__.setRepetition(1);
  __AX__.setSpeedMotor(0,__pulsePWM__);
  __AX__.setSpeedMotor(1,__pulsePWM__);
  __shouldPulse__ = false;
}

void endPulse(){
  /* Rappel du chronometre */
  __AX__.setSpeedMotor(0,0);
  __AX__.setSpeedMotor(1,0);
  __timerPulse__.disable();
}

void sendMsg(){
  /* Envoit du message Json sur le port seriel */
  StaticJsonDocument<200> doc;
  // elements du message
  doc["pot_vex"] = analogRead(POTPIN);
  doc["voltage"] = __AX__.getVoltage();
  doc["current"] = __AX__.getCurrent(); 
  doc["pulsePWM"] = __pulsePWM__;
  doc["pulseTime"] = __pulseTime__;
  doc["pulse"] = __shouldPulse__;
  // Serialisation
  serializeJson(doc, Serial);
  // Envoit
  Serial.println();
  __shouldSend__ = false;
}

void readMsg(){
  // Lecture du message Json
  StaticJsonDocument<200> doc;
  JsonVariant parse_msg;

  // Lecture sur le port Seriel
  DeserializationError error = deserializeJson(doc, Serial);;
  __shouldRead__ = false;

  // Si erreur dans le message
  if (error) {
    Serial.print("deserialize() failed: ");
    Serial.println(error.c_str());
    return;
  }
  
  // Analyse du message
  parse_msg = doc["pulsePWM"];
  if(!parse_msg.isNull()){
     __pulsePWM__ = doc["pulsePWM"];
  }

  parse_msg = doc["pulseTime"];
  if(!parse_msg.isNull()){
     __pulseTime__ = doc["pulseTime"];
  }

  parse_msg = doc["pulse"];
  if(!parse_msg.isNull()){
     __shouldPulse__ = doc["pulse"];
  }
}