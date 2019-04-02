#include "plot.h"
Plot::Plot(){
}

void Plot::setColor(int r, int g, int b){
    QColor color;
    color.setAlpha(255);
    color.setRed(r);
    color.setGreen(g);
    color.setBlue(b);
    pen.setColor(color);
}

void Plot::setDataLen(int dataLen){
    dataBufferLen = dataLen;
}

void Plot::addData(double newData){
    if(fabs(newData) > gain){
        gain = fabs(newData);
    }
    if (data.length()>dataBufferLen){
        data.pop_front();
    }
    data.append(newData);
}

void Plot::draw(QGraphicsScene* scene){

    QPainterPath curve;
    curve.moveTo(0,-GAINCST*(data[0])/gain);
    for (int i = 0; i < data.length(); ++i){
        curve.lineTo(i,-GAINCST*(data[i])/gain);
    }
    scene->addPath(curve, pen);
}

void Plot::clear(){
    data.clear();
    gain = 0;
}
