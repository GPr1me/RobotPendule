#ifndef SKIPDIALOG_H
#define SKIPDIALOG_H

#include <QDialog>
#include "startupmainwindow.h"

namespace Ui {
class SkipDialog;
}

class SkipDialog : public QDialog
{
    Q_OBJECT

public:
    explicit SkipDialog(QWidget *parent = 0);
    ~SkipDialog();

private slots:
    void on_buttonBox_accepted();

    void on_buttonBox_rejected();

private:
    Ui::SkipDialog *ui;
};

#endif // SKIPDIALOG_H
