#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QCloseEvent>
#include <QDebug>
#include <QtWidgets>
#include <QJsonObject>
#include <QJsonDocument>

// Propres librairies
#include "csvwriter.h"
#include "serialprotocol.h"
#include "plot.h"

// Classe definissant l'application
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    int TIMEOUT_MS = 100;
    int DEFAULT_UPDATE_RATE = 1000;
    const qint32 BAUD_RATE = 115200;

    explicit MainWindow(QString portName, int updateRate, QWidget *parent = 0);
    virtual ~MainWindow();
    void closeEvent(QCloseEvent *event) override;

    void sendMessage(QString msg);
    void setUpdateRate(int rateMs);

    void onPeriodicUpdate() {}
    void onMessageReceived(QString /*msg*/) {}

private slots:
    void receiveFromSerial(QString);
    void sendPulseSetting();
    void sendPulseStart();
    void manageRecording(int);
    void changeJsonKeyValue();

private:
    void connectTimers(int updateRate);
    void connectButtons();
    void connectSerialPortRead();
    void connectSpinBoxes();
    void startRecording();
    void stopRecording();
    void connectTextInputs();

    bool record = false;
    CsvWriter* writer_;
    QTimer updateTimer_;
    QString msgReceived_{""};
    QString msgBuffer{""};
    SerialProtocol* serialCom;
    QGraphicsScene scene;
    Plot plot;
    QString JsonKey;


protected:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
