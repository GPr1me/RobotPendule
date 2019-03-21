#include "mainwindow.h"
#include <QApplication>

QString port = "/dev/ttyACM0"; // Port de communication du Arduino
int delay = 100; // Frequence du compteur de la fonction "onPeriodicUpdate"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w(port, delay);
    w.show();
    return a.exec();
}
