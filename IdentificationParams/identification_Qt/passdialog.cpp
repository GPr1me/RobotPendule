#include "passdialog.h"
#include "ui_passdialog.h"
#include "startupmainwindow.h"

PassDialog::PassDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::PassDialog)
{
    ui->setupUi(this);
}

PassDialog::~PassDialog()
{
    delete ui;
}


void PassDialog::on_pushButton_clicked()
{
    this->close();
}
