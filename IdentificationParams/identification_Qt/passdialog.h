#ifndef PASSDIALOG_H
#define PASSDIALOG_H

#include <QDialog>

namespace Ui {
class PassDialog;
}

class PassDialog : public QDialog
{
    Q_OBJECT

public:
    explicit PassDialog(QWidget *parent = 0);
    ~PassDialog();

private slots:


    void on_pushButton_clicked();

private:
    Ui::PassDialog *ui;
};

#endif // PASSDIALOG_H
