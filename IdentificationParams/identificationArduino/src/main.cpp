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
#include <AngleController.h>
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
PID pid_ang;
//AngleController pid_ang;  //TODO

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
  int POTAVG = 450; //559
  double ANGULAR_RANGE = 197.0;      // °
  double pot_angle;                 // °

  double hauteur_obstacle;          // cm
  double largeur_foret;             // cm
  double distance_boite;            // cm
  double longueur_pendule = 0.548;  // m
  double angle_goal;                // °
  bool starto;

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
  unsigned long timerAngle;
  unsigned long initTW; // timer pour oscillations
  bool beginO;          //bool pour start timer
  
  //controle oscillation
  bool noSwing;
  bool hasTree;

  //timer pour test comportement du electroaimant
  unsigned long timer;  
  unsigned long timer2;

  //variable pour mettre un delai avant le debut de 2 secondes
  bool doT;
}

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
  pid_pos.setGains(2, 0.02 , 0); //gains de la simulation 5, 0.02 , 0 
    // Attache des fonctions de retour
  pid_pos.setMeasurementFunc(computePIDPos);
  pid_pos.setCommandFunc(PIDcommand);
  pid_pos.setAtGoalFunc(PIDgoalReached);
  pid_pos.setEpsilon(0.015); //TODO: valeur par defaut en ce moment. Effet a verifier
  pid_pos.setPeriod(100); //1000 / 10: le pid est ajuste 10 fois par seconde (valeur peut etre changee) 

  //pour test sans qt
  //pid_pos.setGoal(0.3); //valeur en distance a atteindre
  //pid_pos.enable();

  //PID pour angle
  pid_ang.setGains(0.008, 0.0002, 0); // test: 0.008, 0, 0.0002 autre gains de la simulation (valeurs a verifier) 
    // Attache des fonctions de retour
  pid_ang.setMeasurementFunc(computePIDAng);
  pid_ang.setCommandFunc(PIDcommandAngle);
  pid_ang.setAtGoalFunc(goalReachedAngle);
  pid_ang.setEpsilon(2); //TODO: valeur par defaut en ce moment. 
  pid_ang.setPeriod(20);
  //pid_ang.setGoal(10);
  //pid_ang.enable();

  AX_.resetEncoder(0);

  computeAngleGoal();

  timer = millis();
  timer2 = millis();

  //code pour activer le potentiometre et le electroaimant
  pinMode(MAGPIN, OUTPUT);
  pinMode(POTPIN, INPUT);

  //set valeur initiales
  wFlag = true;
  firstRun = false;
  energy_ax = 0;
  power_ax = 0;
  //endRun = true;
  acmd = 0;
  cur_T = millis() / 1000.0;
  countA = 0;

  prev_Ti = 0;
  prev_phi = 0;
  beginO = 1;
  noSwing = 1;
  starto = false;
  
  //active aimant
  digitalWrite(MAGPIN, 1);
}

/* Boucle principale (infinie)*/
void loop() {

  
  //test pour voir Vmax selon mesures
  //Serial.println(getVel());
  if(!starto){
    //arret a la fin
    AX_.setMotorPWM(REAR, 0);
    AX_.setMotorPWM(FRONT, 0);
     
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
    if(starto){
      //set valeur initiales
      firstRun = false;
      //endRun = true;
      acmd = 0;
      cur_T = millis() / 1000.0;
      countA = 0;

      prev_Ti = 0;
      prev_phi = 0;
      beginO = 1;
      noSwing = 1;
      //active aimant
      digitalWrite(MAGPIN, 1);
      timer = millis();
      wFlag = false;
      doT = true;
      energy_ax = 0;  
    }
    pid_pos.run();
    pid_ang.run();
  }
  //run
  else{
    if(3000 <= millis() - timer && doT){
      wFlag = true;
      doT = false;      
    }
    
    //code test pour activer le electroaimant pendant 10 secondes
    /* if(20000 >  ctime){
      digitalWrite(MAGPIN, 1);
    }
    else{
      digitalWrite(MAGPIN, 0);
    }*/
    
    // Comm avec le PI
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

    // test controleur pendule(baseball game)
    //wFlag mis a false quand objectif atteint
    if(wFlag){
      reachAngle(-30);
    }
    if(firstRun){
      pid_pos.setGoal(1.1);//test pos 0 to check only angle
      pid_pos.enable();
      firstRun = false;
    }
    pid_pos.run();
    pid_ang.run();
  }

  // if(!pid_pos.isAtGoal())
  // {
  //   pid_pos.run();
  // }
  // else if (!pid_ang.isAtGoal())
  // {
  //   pid_ang.run();
  // }
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
  computePowerEnergy();
  /* Envoit du message Json sur le port seriel */
  StaticJsonDocument<500> doc;
  // Elements du message

  doc["cmd"] = tcmd;
  doc["acmd"] = acmd;
  doc["globalPos"] = AX_.readEncoder(1) / float(PASPARTOUR * RAPPORTVITESSE) * 2 * PI * 0.05;
  doc["time"] = millis();
  doc["potVex"] = getAngle();
  //doc["encVex"] = vexEncoder_.getCount();
  doc["Posgoal"]   = pid_pos.getGoal();
  doc["Anggoal"]   = pid_ang.getGoal();
  doc["motorPos"]  = dist_;
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
  doc["PosGoal"]   = pid_pos.isAtGoal();
  doc["AngGoal"]   = pid_ang.isAtGoal();

  // Serialisation
  serializeJson(doc, Serial);
  // Envoit
  Serial.println();
  shouldSend_ = false;
  timer2 = millis();
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
  parse_msg = doc["hauteur"];
  if(!parse_msg.isNull()){
     hauteur_obstacle = doc["hauteur"].as<float>();
  }

  parse_msg = doc["largeur"];
  if(!parse_msg.isNull()){
     largeur_foret = doc["largeur"].as<float>();
  }

  parse_msg = doc["distance"];
  if(!parse_msg.isNull()){
     distance_boite = doc["distance"].as<float>();
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

  parse_msg = doc["setGoal"];
  if(!parse_msg.isNull()){
    pid_pos.disable();
    pid_pos.setGains(doc["setGoal"][0], doc["setGoal"][1], doc["setGoal"][2]);
    pid_pos.setEpsilon(doc["setGoal"][3]);
    pid_.setGoal(doc["setGoal"][4]);
    pid_.enable();
  }

  parse_msg = doc["start"];
  if(!parse_msg.isNull()){
     starto = doc["start"];
  }
}

void computeAngleGoal(){
  // h = L(1-cos(theta))
  hauteur_obstacle *= 0.01;
  angle_goal = 180*acos(1 - hauteur_obstacle / longueur_pendule)/PI;
}

void computePowerEnergy(){
  power_ax = AX_.getVoltage() * AX_.getCurrent();
  energy_ax += power_ax *(millis() - timer2)/1000;
}

//fonction pour osciller a un angle voulu
void reachAngle(double angle){
  // pid_pos.disable();
  if(beginO){
    initTW = millis();
    beginO = false;
  }
  tWave = millis() - initTW;
  double scmd = 0.4*sin(4.8*tWave/1000.0);
  AX_.setMotorPWM(REAR, scmd);
  AX_.setMotorPWM(FRONT, -scmd);
  if(angle < 0){
    
    if(getAngle() < angle){
      wFlag = 0;
      AX_.setMotorPWM(REAR, 0);
      AX_.setMotorPWM(FRONT, 0);
      wFlag = false;
      firstRun = true;
      countA++;
      beginO = true;
    }
    /*while(getAngle() < angle/2){


    }*/
  }
  else{
    if(getAngle() > angle){
      wFlag = 0;
      AX_.setMotorPWM(REAR, 0);
      AX_.setMotorPWM(FRONT, 0);
      wFlag = false;
      firstRun = true;
      countA++;
      beginO = true;
    }
    
  }
  
  /*if(firstRun){
    pid_pos.setGoal(0.9);
    pid_pos.enable();
  }*/

}

// Fonctions pour le PID

//calcul de la vitesse angulaire
double getAngleSpeed(){
  double cur_phi = getAngle();
  double cur_Ti = millis();
  speed_phi = (cur_phi - prev_phi)/((cur_Ti - prev_Ti)/1000.0);
  prev_Ti = cur_Ti;
  prev_phi = cur_phi;
  return speed_phi;
}

double pulseToMeters(){
    //3200 pulses par tour de roue
    //conversion vers rads: encoches/ 3200 * 2 * pi
    //longueur de l'arc: angle_en_rads * r
    dist_ = AX_.readEncoder(0) / float(PASPARTOUR * RAPPORTVITESSE) * 2 * PI * 0.05;
    return dist_;
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
  // double angle = getAngle();
  // if(angle < 2 && angle >= 0){
  //   double sp = getAngleSpeed();
  //   if(sp > -40 && sp < 40){
  //     return angle + sp*0.1;
  //   }
  //   else{
  //     return angle;
  //   }  
  // }
  // else if(angle > -2 && angle < 0){
  //   double sp = getAngleSpeed();
  //   if(sp > -40 && sp < 40){
  //     return angle * 2 + sp*0.1;
  //   }
  //   else{
  //     return angle;
  //   }
  // }
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
  //Serial.println("Valeur de distance mesuree:");
  //Serial.println(pulseToMeters());
  
  pid_pos.disable();
  if(countA >= 3){
    AX_.setMotorPWM(0, 0);
    AX_.setMotorPWM(1, 0);
    timerAngle = millis();
    digitalWrite(MAGPIN, 0);
    
    if(countA < 4){
      pid_pos.setGoal(-1.1);
      pid_pos.enable();
    }
    if(countA >= 4){
      starto = false;
    }
  }
  AX_.resetEncoder(0);
  //attends a que l'angle soit negatif
  while(getAngle() < 0){
    //condition do exit if no wait needed
    if(countA > 3){
      break;
    }
  }
  countA++;
  if(countA < 3){
    pid_ang.setGoal(0);
    pid_ang.enable();
  }
  
}

//gestion desiree, coninuer a maintenir l'angle pendant une distance voulue
void goalReachedAngle(){
  double sA = getAngleSpeed();
  if(sA > 20 || sA < -20){
    pid_ang.setGoal(0);
    pid_ang.enable();
    // countA++;
  }
  else{
    AX_.setMotorPWM(0, 0);
    AX_.setMotorPWM(1, 0);
    //Serial.println("Valeur de distance mesuree:");
    //Serial.println(pulseToMeters());
    pid_pos.setGoal(0);
    pid_pos.enable();
    countA++;
  }
}

//fonction pour mesurer l'angle actuel
double getAngle(){
  // Lecture de tension recentree
  int pot_read = analogRead(POTPIN);
  pot_read -= POTAVG;

  // Conversion tension a angle
  pot_angle = pot_read / -pot_ratio;
  
  return pot_angle;  
}

void PIDcommandAngle(double cmd){
  //acmd utilise pour pouvoir voir la valeur de la cmd envoyee
  //addition afin d'essayer de maintenir la vitesse une fois la commande rendu a 0
  if(getAngle() < -1){
    noSwing = false;
  }

  acmd = cmd;
  //Comportement du PID
  
  //empeche le pendule de revenir trop au premier retour
  if(noSwing){
    acmd = acmd/2;
  }

  if(acmd > 1){
    //tWave = millis() - initTW;
    //double scmd = 0.4*sin(4.8*tWave/1000.0);
    AX_.setMotorPWM(0, 1);
    AX_.setMotorPWM(1, -1);
  }
  else if(acmd < -1){
    //tWave = millis() - initTW;
    //double scmd = 0.4*sin(4.8*tWave/1000.0);
    AX_.setMotorPWM(0, -1);
    AX_.setMotorPWM(1, 1);
  }
  else{
    AX_.setMotorPWM(0, acmd);
    AX_.setMotorPWM(1, -acmd);
  }
}

void AngleCommand(double scmd)
{
  AX_.setMotorPWM(REAR, scmd);
  AX_.setMotorPWM(FRONT, -scmd);
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