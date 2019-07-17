#ifndef STARTUPMAINWINDOW_H
#define STARTUPMAINWINDOW_H

#include "mainwindow.h"
#include <QMainWindow>
#include <QTimer>
#include <QCloseEvent>
#include <QDebug>
#include <QtWidgets>
#include <QJsonObject>
#include <QJsonDocument>
#include <QSerialPortInfo>

#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts>


// Propres librairies
#include "csvwriter.h"
#include "serialprotocol.h"

namespace Ui {
class StartupMainWindow;
}

class StartupMainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit StartupMainWindow(QWidget *parent = 0);
    ~StartupMainWindow();
    void sendMessage(QString msg);
    const qint32 BAUD_RATE = 115200;


private slots:
    //void donothing();
    void on_WtvButton_clicked();
    void HauteurSent();
    void DistanceSent();
    void LargeurSent();
    void startSerialCom(QString);
    void on_Back_clicked();
    void connectComboBox();
    void on_Close_clicked();


private:
    void portCensus();
    QString msgBuffer_{""};
    Ui::StartupMainWindow *ui;
    MainWindow *g;
    int delayMs;
    //bool f;
    //void connectSerialPortRead();
    void checkButton();
    void sendPulseSetting();

};

#endif // STARTUPMAINWINDOW_H
