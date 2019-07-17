#include "startupmainwindow.h"
#include "ui_startupmainwindow.h"
#include "mainwindow.h"
#include <QDebug>

bool h = 0;
bool d = 0;
bool l = 0;
SerialProtocol* serialCom_=nullptr;

StartupMainWindow::StartupMainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::StartupMainWindow)
{

    delayMs = 1000;
    ui->setupUi(this);
    //f = false;
    g = new MainWindow(delayMs, this);
    connect(ui->SendHauteur, SIGNAL(clicked(bool)), this, SLOT(HauteurSent()));
    connect(ui->SendDistance, SIGNAL(clicked(bool)), this, SLOT(DistanceSent()));
    connect(ui->SendLargeur, SIGNAL(clicked(bool)), this, SLOT(LargeurSent()));
    connectComboBox();
    portCensus();
}

StartupMainWindow::~StartupMainWindow()
{
    delete ui;
    delete g;
}

void StartupMainWindow::on_WtvButton_clicked()
{
    checkButton();
}

void StartupMainWindow::HauteurSent()
{
    h = true;
    ui->SendHauteur->setEnabled(false);
    ui->Spinh->setEnabled(false);
    ui->SendLargeur->setEnabled(true);
    ui->Spinl->setEnabled(true);
    ui->Back->setEnabled(true);
}

void StartupMainWindow::LargeurSent()
{
    l = true;
    ui->SendLargeur->setEnabled(false);
    ui->Spinl->setEnabled(false);
    ui->SendDistance->setEnabled(true);
    ui->Spind->setEnabled(true);
    ui->Back->setEnabled(true);

}

void StartupMainWindow::DistanceSent()
{
    d = true;
    ui->SendDistance->setEnabled(false);
    ui->Spind->setEnabled(false);
    ui->WtvButton->setEnabled(true);
    ui->Back->setEnabled(true);

}

void StartupMainWindow::checkButton()
{
    if (h && d && l){
      sendPulseSetting();
      hide();
      g->show();
    }
}


void StartupMainWindow::sendPulseSetting(){
    // Fonction SLOT pour envoyer les paramettres de pulse
    double hauteur = ui->Spinh->value();
    int distance = ui->Spind->value();
    int largeur = ui->Spinl->value();
    QJsonObject jsonObject
    {// pour minimiser le nombre de decimales( QString::number)
        {"largeur", largeur},
        {"hauteur", QString::number(hauteur)},
        {"distance", distance}
    };
    QJsonDocument doc(jsonObject);
    QString strJson(doc.toJson(QJsonDocument::Compact));
    sendMessage(strJson);
}

void StartupMainWindow::sendMessage(QString msg){
    // Fonction SLOT d'ecriture sur le port serie
    if(serialCom_==nullptr){
        qDebug().noquote() <<"Erreur aucun port serie !!!";
        return;
    }
    serialCom_->sendMessage(msg);
    qDebug().noquote() <<"Message du RPI: "  <<msg;
}

void StartupMainWindow::on_Back_clicked()
{
    if(d == true){
        d = false;
        ui->SendDistance->setEnabled(true);
        ui->Spind->setEnabled(true);
        ui->WtvButton->setEnabled(false);
    }
    else if(l == true){
        l = false;
        ui->SendLargeur->setEnabled(true);
        ui->Spinl->setEnabled(true);
        ui->SendDistance->setEnabled(false);
        ui->Spind->setEnabled(false);
    }
    else if(h == true){
        h = false;
        ui->SendHauteur->setEnabled(true);
        ui->Spinh->setEnabled(true);
        ui->SendLargeur->setEnabled(false);
        ui->Spinl->setEnabled(false);
        ui->Back->setEnabled(false);
    }
}

void StartupMainWindow::on_Close_clicked()
{
    this->close();
}

void StartupMainWindow::connectComboBox(){
    // Fonction de connection des entrees deroulantes
    connect(ui->SelectPort, SIGNAL(activated(QString)), this, SLOT(startSerialCom(QString)));
}


void StartupMainWindow::portCensus(){
    // Fonction pour recenser les ports disponibles
    ui->SelectPort->clear();
    Q_FOREACH(QSerialPortInfo port, QSerialPortInfo::availablePorts()) {
        ui->SelectPort->addItem(port.portName());
    }
}

void StartupMainWindow::startSerialCom(QString portName){
     qDebug("Waddup");
    // Fonction SLOT pour demarrer la communication serielle
    qDebug().noquote() << "Connection au port"<< portName;
    if(serialCom_!=nullptr){
        delete serialCom_;
    }
    serialCom_ = new SerialProtocol(portName, BAUD_RATE);
    g->setSerialCom(serialCom_);
    //g->connectSerialPortRead();
}

//void MainWindow::connectSerialPortRead();
    // Fonction de connection au message de la classe (serialProtocol)
    //connect(serialCom_, SIGNAL(newMessage(QString)), this, SLOT(donothing()));
//}

//void MainWindow::donothing(){
    //qDebug("Waddup1");
//}







