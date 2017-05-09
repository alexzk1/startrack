#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "saveable_widget.h"

#include "runners.h"
#include <memory>
#include <atomic>
#include <map>
#include <QTcpServer>
#include <QPointer>
#include <QTcpSocket>

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

    void on_cbNight_toggled(bool checked);

private:
    Ui::MainWindow *ui;
    utility::runner_t comThread;
    std::atomic_flag skipWrite;
    float volatile gaz;
    float volatile gel;
    std::shared_ptr<QTcpServer> server;
    std::map<uint64_t, QPointer<QTcpSocket>> stellarium; //decided to have many possible connections, each one can track or rule device
    std::atomic<bool> readyToTrackMap;

    void startComPoll();
    void writeToArduino(float az_rad, float el_rad);
    int64_t getMicrosNow();
    void onStellariumDataReady(QTcpSocket *p);
signals:
    void onArduinoRead(float az_rad, float el_rad);
};

#endif // MAINWINDOW_H
