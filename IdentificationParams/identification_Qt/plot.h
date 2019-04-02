#ifndef PLOT_H
#define PLOT_H

#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsItem>
#include <QPen>
#include <math.h>
#include <QDebug>

// Classe pour afficher des graphes simples
class Plot: public QObject
{
    Q_OBJECT

public:
    Plot();
    void setColor(int,int,int);
    void setDataLen(int);
    void addData(double);
    void draw(QGraphicsScene *);
    void clear();

private:
    double gain = 0;
    double GAINCST = 70;
    int dataBufferLen;
    QVector<double> data;
    QPen pen;
};
#endif // PLOT_H
