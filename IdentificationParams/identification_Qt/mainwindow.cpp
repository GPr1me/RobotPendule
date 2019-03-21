#include "mainwindow.h"
#include "ui_mainwindow.h"


MainWindow::MainWindow(QString portName, int updateRate, QWidget *parent) :
    QMainWindow(parent)
{
    // Constructeur de la classe
    // Initialisation du UI
    ui = new Ui::MainWindow;
    ui->setupUi(this);
    ui->graphicsView->setScene(&scene);

    // Fonctions de connections events/slots
    connectTimers(updateRate);
    connectButtons();
    connectSpinBoxes();

    // Protocol seriel
    serialCom = new SerialProtocol(portName, BAUD_RATE);
    connectSerialPortRead();

    // Affichage de donnees
    potVex.setDataLen(500);
    potVex.setColor(255,0,0);
    potVex.setGain(1);

    // initialisation du timer
    updateTimer_.setInterval(DEFAULT_UPDATE_RATE);
    updateTimer_.start();

}

MainWindow::~MainWindow(){
    // Destructeur de la classe
    updateTimer_.stop();
    delete serialCom;
    delete ui;
}

void MainWindow::closeEvent(QCloseEvent *event){
    // Fonction appelee lorsque la fenetre est detruite
    event->accept();
}

void MainWindow::receiveFromSerial(QString msg) {
    // Fonction appelee lors de reception sur port serie
    // Accumulation des morceaux de message
    msgBuffer += msg;

    //Si un message est termine
    if(msgBuffer.endsWith('\n')){
        // Passage ASCII vers structure Json
        QJsonDocument jsonResponse = QJsonDocument::fromJson(msgBuffer.toUtf8());

        // Analyse du message Json
        if(~jsonResponse.isEmpty()){
            QJsonObject jsonObj = jsonResponse.object();
            QString buff = jsonResponse.toJson(QJsonDocument::Indented);
            ui->textBrowser->setText(buff.mid(2,buff.length()-4));

            // Affichage des donnees
            scene.clear();
            potVex.addData((jsonObj["pot_vex"].toDouble()-512.0)/5.0);
            potVex.draw(&scene);

            // Fonction de reception de message (vide pour l'instant)
            msgReceived_ = msgBuffer;
            onMessageReceived(msgReceived_);

            // Si les donnees doivent etre enregistrees
            if(record){
                writer_->write(jsonObj);
            }
            // Reinitialisation du message tampon
            msgBuffer = "";
        }
    }
}


void MainWindow::connectTimers(int updateRate) {
    // Fonction de connection de timers
    connect(&updateTimer_, &QTimer::timeout, this, [this]{onPeriodicUpdate();});
    updateTimer_.start(updateRate);
}

void MainWindow::connectSerialPortRead() {
    // Fonction de connection au message de la classe (serialProtocol)
    connect(serialCom, SIGNAL(newMessage(QString)), this, SLOT(receiveFromSerial(QString)));
}

void MainWindow::connectButtons(){
    // Fonction de connection du boutton Send
    connect(ui->pulseButton, SIGNAL(clicked()), this, SLOT(sendPulseStart()));
    connect(ui->checkBox, SIGNAL(stateChanged(int)), this, SLOT(manageRecording(int)));
}

void MainWindow::connectSpinBoxes() {
    // Fonction de connection des spin boxes
    connect(ui->DurationBox, SIGNAL(valueChanged(int)), this, SLOT(sendPulseSetting()));
    connect(ui->PWMBox, SIGNAL(valueChanged(double)), this, SLOT(sendPulseSetting()));

}

void MainWindow::sendPulseSetting(){
    // Fonction pour envoyer les paramettre de pulse
    double PWM_val = ui->PWMBox->value();
    int duration_val = ui->DurationBox->value();
    QJsonObject jsonObject
    {// pour minimiser le nombre de decimales( QString::number)
        {"pulsePWM", QString::number(PWM_val)},
        {"pulseTime", duration_val}
    };
    QJsonDocument doc(jsonObject);
    QString strJson(doc.toJson(QJsonDocument::Compact));
    sendMessage(strJson);
}

void MainWindow::sendPulseStart(){
    // Fonction pour envoyer la commande de pulse
    QJsonObject jsonObject
    {
        {"pulse", 1}
    };
    QJsonDocument doc(jsonObject);
    QString strJson(doc.toJson(QJsonDocument::Compact));
    sendMessage(strJson);
}


void MainWindow::sendMessage(QString msg) {
    // Fonction d'ecriture sur le port serie
    serialCom->sendMessage(msg);
    qDebug().noquote() <<"Message du RPI: "  <<msg;
}

void MainWindow::setUpdateRate(int rateMs) {
    // Fonction d'initialisation du chronometre
    updateTimer_.start(rateMs);
}

void MainWindow::manageRecording(int stateButton){
    // Fonction qui determine l'etat du bouton
    if(stateButton == 2){
        startRecording();
    }
    if(stateButton == 0){
        stopRecording();
    }
}

void MainWindow::startRecording(){
    // Creation d'un nouveau fichier csv
    record = true;
    writer_ = new CsvWriter("/home/pi/Desktop/");
    ui->label_pathCSV->setText(writer_->folder+writer_->filename);
}

void MainWindow::stopRecording(){
    // Fonction permettant d'arreter l'ecriture
    record = false;
    delete writer_;
}
void onMessageReceived(QString msg){
    // Fonction appelee lors de reception de message
    qDebug().noquote() << "Message du Arduino: " << msg;
}

void onPeriodicUpdate(){
    // Fonction appelee a intervalle
    qDebug().noquote() << "*";
}
