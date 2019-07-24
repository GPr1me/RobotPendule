#include "skipdialog.h"
#include "ui_skipdialog.h"

SkipDialog::SkipDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SkipDialog)
{
    ui->setupUi(this);
}

SkipDialog::~SkipDialog()
{
    delete ui;
}

void SkipDialog::on_buttonBox_accepted()
{
    this->close();
}

void SkipDialog::on_buttonBox_rejected()
{
    this->close();
}
