/* 
 * Fichier: main.cpp
 * Auteur(s):
 *   Lauzon,   Jean-Samuel
 *   Cabana,   Gabriel      - cabg2101
 *   Lalonde,  Philippe     - lalp2803
 *   Pereira,  Santiago     - pers2113
 *   Roy,      Olivier      - royo2206
 *   Theroux,  Philippe     - thep3103
 *   Thibault, Marc-Olivier - thim1611 
 * Date(s):
 *   2019-05-01 (Creation)
 *   2019-07-30 (Derniere modification)
 * Description: Code de demarrage du robot mobile.
 * GRO302
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
//IMU9DOF imu_;                // objet imu
//PID pid_;                    // objet PID
RobotController *controller; // objet robotControlleur

volatile bool shouldSend_ = false;  // drapeau prêt à envoyer un message
volatile bool shouldRead_ = false;  // drapeau prêt à lire un message
volatile bool shouldPulse_ = false; // drapeau pour effectuer un pulse
volatile bool isInPulse_ = false;   // drapeau pour effectuer un pulse

SoftTimer timerSendMsg_; // chronometre d'envoie de messages
SoftTimer timerPulse_;   // chronometre pour la duree d'un pulse

uint16_t pulseTime_ = 0; // temps dun pulse en ms
float pulsePWM_ = 0;     // Amplitude de la tension au moteur [-1,1]

//float Axyz[3];                      // tableau pour accelerometre
//float Gxyz[3];                      // tableau pour giroscope
//float Mxyz[3];                      // tableau pour magnetometre

// Identification des moteurs
enum engines
{
  REAR,
  FRONT
};

namespace
{
// Variables temporelles
double current_time_;
double previous_time_ = 0;
double time_interval_;

// Variables angulaires
int POTMIN = 90;
int POTMAX = 1023;
int POTAVG = 452;
double ANGULAR_RANGE = 197.0; // °
double pot_ratio_ = (POTMAX - POTMIN) / ANGULAR_RANGE;
double pot_angle_; // °

// Variables lineaires
double current_position_;
double previous_position_ = 0;
double current_speed_;

// Variables de consommation
double power_ax_;
double energy_ax_;

// Variables de commandes
double position_command_;
double angle_command_;
} // namespace

/*------------------------- Prototypes de fonctions -------------------------*/

// Fonctions d'appels et d'envoi de messages
void timerCallback();
void startPulse();
void endPulse();
void sendMsg();
void readMsg();
void serialEvent();

// Fonctions pour calcul de position
double getAngle();

// Fonctions pour calcul de vitesse
double getLinearVelocity();

// Fonctions pour le PID
double computePIDPosition();
double computePIDAngle();
void PIDCommandPosition(double cmd);
void PIDCommandAngle(double cmd);
void goalReachedPosition();
void goalReachedAngle();

// Fonction pour le calcul de consommation
void computePowerEnergy();

/*---------------------------- fonctions "Main" -----------------------------*/

void setup()
{
  Serial.begin(BAUD); // Initialisation de la communication serielle
  AX_.init();         // Initialisation de la carte ArduinoX
  //imu_.init();            // Initialisation de la centrale inertielle
  vexEncoder_.init(2, 3); // Initialisation de l'encodeur VEX
  // Attache de l'interruption pour encodeur vex
  attachInterrupt(vexEncoder_.getPinInt(), [] { vexEncoder_.isr(); }, FALLING);

  // Chronometre pour l'envoi de messages
  timerSendMsg_.setDelay(UPDATE_PERIODE);
  timerSendMsg_.setCallback(timerCallback);
  timerSendMsg_.enable();

  // Chronometre pour la duree de l'impulsion
  timerPulse_.setCallback(endPulse);

  // Initialisation du controleur et des PID
  controller = new RobotController();
  controller->setupPOS(computePIDPosition, PIDCommandPosition, goalReachedPosition);
  controller->setupANGLE(getAngle, computePIDAngle, PIDCommandAngle, goalReachedAngle);
}

/* Boucle principale (infinie) */
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

  // Mise a jour des chronometres
  timerSendMsg_.update();
  timerPulse_.update();

  // Mise a jour du PID
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
  AX_.setMotorPWM(FRONT, pulsePWM_);
  AX_.setMotorPWM(REAR, -pulsePWM_);
  shouldPulse_ = false;
  isInPulse_ = true;
}

void endPulse()
{
  /* Rappel du chronometre */
  AX_.setMotorPWM(FRONT, 0);
  AX_.setMotorPWM(REAR, 0);
  timerPulse_.disable();
  isInPulse_ = false;
}

void sendMsg()
{
  /* Envoit du message Json sur le port seriel */
  StaticJsonDocument<500> doc;

  // Elements du message
  doc["time"]             = millis();
  doc["potVex"]           = analogRead(POTPIN);
  doc["encVex"]           = vexEncoder_.getCount();
  doc["goal"]             = controller->getActiveController->getGoal();
  doc["isGoal"]           = controller->getActiveController->isAtGoal();
  doc["activeController"] = controller->getActiveController->ToString();
  doc["cmd_pos"]          = position_command_;
  doc["cmd_ang"]          = angle_command_;
  doc["motorPos"]         = current_position_;
  //doc["voltage"]          = AX_.getVoltage();
  //doc["current"]          = AX_.getCurrent();
  doc["power"]            = power_ax_;
  doc["energy"]           = energy_ax_;
  doc["pulsePWM"]         = pulsePWM_;
  doc["pulseTime"]        = pulseTime_;
  doc["inPulse"]          = isInPulse_;
  // doc["accelX"]           = imu_.getAccelX();
  // doc["accelY"]           = imu_.getAccelY();
  // doc["accelZ"]           = imu_.getAccelZ();
  // doc["gyroX"]            = imu_.getGyroX();
  // doc["gyroY"]            = imu_.getGyroY();
  // doc["gyroZ"]            = imu_.getGyroZ();
  

  // Serialisation
  serializeJson(doc, Serial);

  // Envoi
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

// Fonction qui calcule la vitesse
double getLinearVelocity()
{
  current_time_ = millis() / 1000;
  time_interval_ = current_time_ - previous_time_;
  current_speed_ = (current_position_ - previous_position_) / time_interval_;
  previous_position_ = current_position_;
  previous_time_ = current_time_;

  return current_speed_;
}

// Fonction pour mesurer l'angle actuel
double getAngle()
{
  // Lecture de tension recentree
  int pot_read_ = analogRead(POTPIN);
  pot_read_ -= POTAVG;

  // Conversion tension a angle
  pot_angle_ = -(pot_read_ / pot_ratio_);

  return pot_angle_;
}

// Mesure la distance parcourue
double computePIDPosition()
{
  current_position_ = AX_.readEncoder(REAR) / float(PASPARTOUR * RAPPORTVITESSE) * 2 * PI * 0.05;
  return current_position_;
}

// Mesure l'angle du pendule
double computePIDAngle()
{
  // TO DO
  return 0;
}

// Fonctions pour envoyer la commande aux moteurs
void PIDCommandPosition(double cmd)
{
  // Variable globale utilisee pour Json
  position_command_ = cmd;

  if (position_command_ > 1)
  {
    AX_.setMotorPWM(FRONT, -1);
    AX_.setMotorPWM(REAR, 1);
  }
  else if (position_command_ < -1)
  {
    AX_.setMotorPWM(FRONT, 1);
    AX_.setMotorPWM(REAR, -1);
  }
  else
  {
    AX_.setMotorPWM(FRONT, -position_command_);
    AX_.setMotorPWM(REAR, position_command_);
  }
}

void PIDCommandAngle(double cmd)
{
  // Variable globale utilisee pour Json
  angle_command_ = cmd;

  if (angle_command_ > 1)
  {
    AX_.setMotorPWM(FRONT, -1);
    AX_.setMotorPWM(REAR, 1);
  }
  else if (angle_command_ < -1)
  {
    AX_.setMotorPWM(FRONT, 1);
    AX_.setMotorPWM(REAR, -1);
  }
  else
  {
    AX_.setMotorPWM(FRONT, -angle_command_);
    AX_.setMotorPWM(REAR, angle_command_);
  }
}

void goalReachedPosition()
{
  // TO DO
}

void goalReachedAngle()
{
  // TO DO
}

// Fonction qui calcule la puissance et l'energie de la carte Arduino X
void computePowerEnergy()
{
  power_ax_ = AX_.getVoltage() * AX_.getCurrent();
  energy_ax_ = power_ax_ / time_interval_;
}