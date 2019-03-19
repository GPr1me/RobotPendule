#include "mainwindow.h"
#include <QApplication>

QString port = "/dev/ttyACM0";
int delay = 100;

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w(port, delay);
    w.show();
    return a.exec();
}
