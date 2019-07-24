#include "startupmainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    StartupMainWindow w;
    w.show();
    return a.exec();
}
