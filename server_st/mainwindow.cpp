#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QSerialPortInfo>
#include <QSerialPort>
#include "../arduino_st/simple_comm.h"
#include <thread>
#include <chrono>
#include "union_cast.h"
#include <QDebug>

#define RAD_TO_DEG 57.295779513082320876798154814105
#define degrees(rad) ((rad)*RAD_TO_DEG)


Q_DECLARE_METATYPE(QSerialPortInfo);

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    connect(this, &MainWindow::onArduinoRead, this, &MainWindow::arduinoRead, Qt::QueuedConnection);

    on_pushButton_clicked();
    readSettings(this);
}

MainWindow::~MainWindow()
{
    writeSettings(this);
    delete ui;
}

void MainWindow::changeEvent(QEvent *e)
{
    QMainWindow::changeEvent(e);
    switch (e->type()) {
        case QEvent::LanguageChange:
            ui->retranslateUi(this);
            break;
        default:
            break;
    }
}

void MainWindow::recurseRead(QSettings &settings, QObject *object)
{
    Q_UNUSED(object);
    auto ps = settings.value("COMPort", "").toString();
    ui->cbPorts->setCurrentText(ps);
}

void MainWindow::recurseWrite(QSettings &settings, QObject *object)
{
    Q_UNUSED(object);
    auto ps  = ui->cbPorts->currentText();
    settings.setValue("COMPort", ps);
}

void MainWindow::on_pushButton_clicked()
{
    comThread.reset();
    ui->cbPorts->clear();
    auto pl = QSerialPortInfo::availablePorts();
    for (const auto& p : pl)
        ui->cbPorts->addItem(p.portName(), QVariant::fromValue<QSerialPortInfo>(p));

}

void MainWindow::on_cbPorts_currentIndexChanged(int)
{
    startComPoll();
}

void MainWindow::arduinoRead(float az_rad, float el_rad)
{
    ui->lblAz-> setText(QString("%1").arg(degrees(az_rad)));
    ui->lblAlt->setText(QString("%1").arg(degrees(el_rad)));
}

void MainWindow::startComPoll()
{
    comThread = utility::startNewRunner([this](auto stop)
    {
        using namespace std::chrono_literals;
        using namespace ard_st;
        QSerialPort port(ui->cbPorts->currentData().value<QSerialPortInfo>());
        port.setBaudRate(115200);
        //8N1 default settings
        port.setDataBits(QSerialPort::Data8);
        port.setParity(QSerialPort::Parity::NoParity);
        port.setStopBits(QSerialPort::OneStop);

        const bool op = port.open(QIODevice::ReadWrite);
        std::this_thread::sleep_for(15s); // device gets reset, have to wait
        port.setDataTerminalReady(false); //disable autoreset of device
        while (!(*stop))
        {
            if (op) //if failed to open, still should start thread which does nothing
            {
                const static std::string header(MESSAGE_HDR);
                Message msg;
                msg.message.Command = 'R';
                port.write(header.c_str(), header.size());
                port.write(msg.buffer, sizeof(msg.buffer));
                port.waitForBytesWritten();

                for(int i = 0; i < 100 && !(*stop) && port.bytesAvailable() < sizeof(msg.message.value); ++i)
                    std::this_thread::sleep_for(20ms);

                if (sizeof(msg.message.value.buffer) == port.read(msg.message.value.buffer, sizeof(msg.message.value.buffer)))
                {
                    float az, el;
                    readAzEl(msg.message.value, az, el);
                    //qDebug() <<"Az = " <<degrees(az) << ";  El = "<<degrees(el);
                    emit this->onArduinoRead(az, el);
                }
            }
            std::this_thread::sleep_for(500ms);
        }
    });
}
