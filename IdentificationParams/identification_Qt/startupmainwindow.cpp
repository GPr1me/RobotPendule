#include "startupmainwindow.h"
#include "ui_startupmainwindow.h"
#include "mainwindow.h"
#include <QDebug>
#include <QSerialPortInfo>
#include <QtGlobal>
#include "skipdialog.h"
#include "passdialog.h"

StartupMainWindow::StartupMainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::StartupMainWindow)
{
    h = false;
    d = false;
    l = false;
    ss = false;
    serialCom_ = nullptr;
    portName_ = " ";
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
    //qDebug() << qPrintable(portName_);
    if(serialCom_ == nullptr || portName_ == "ttyAMA0"){
        PassDialog passDialog;
        passDialog.setModal(true);
        passDialog.exec();
    }
    else{
        checkButton();
    }
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
        {"hauteur", hauteur},
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

void StartupMainWindow::startSerialCom(QString portName)
{
    portName_ = portName;
    //qDebug("Waddup");
    // Fonction SLOT pour demarrer la communication serielle
    qDebug().noquote() << "Connection au port"<< portName;
    if(serialCom_!=nullptr){
        delete serialCom_;
    }
    serialCom_ = new SerialProtocol(portName, BAUD_RATE);
    g->setSerialCom(serialCom_);
    //g->connectSerialPortRead();
}


void StartupMainWindow::on_skipButton_clicked()
{
    SkipDialog skipDialog;
    skipDialog.setModal(true);
    //skipDialog.exec();
    //qDebug("%d", ss);
    if(skipDialog.exec() == SkipDialog::Accepted){
        hide();
        g->show();
    }
}

void StartupMainWindow::changeState(){
    ss = true;
}



