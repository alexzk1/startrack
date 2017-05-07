#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "saveable_widget.h"

#include "runners.h"
#include <memory>

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow, public utility::SaveableWidget<MainWindow>
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
protected:
    virtual void changeEvent(QEvent *e) override;
    virtual void recurseRead(QSettings& settings, QObject* object) override;
    virtual void recurseWrite(QSettings& settings, QObject* object) override;
private slots:
    void on_pushButton_clicked();
    void on_cbPorts_currentIndexChanged(int);
    void arduinoRead(float az_rad, float el_rad);
private:
    Ui::MainWindow *ui;
    utility::runner_t comThread;

    void startComPoll();
signals:
    void onArduinoRead(float az_rad, float el_rad);
};

#endif // MAINWINDOW_H
