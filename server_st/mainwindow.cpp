#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QSerialPortInfo>
#include <QSerialPort>
#include "../arduino_st/simple_comm.h"
#include <thread>
#include <chrono>
#include "union_cast.h"
#include <QDebug>
#include <QTcpServer>
#include <QTcpSocket>
#include "stell_msg.h"
#include "star_math.h"


Q_DECLARE_METATYPE(QSerialPortInfo);

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    comThread(nullptr),
    skipWrite(ATOMIC_FLAG_INIT),
    gaz(0), gel(0),
    server(nullptr)
{
    ui->setupUi(this);
    setWindowFlags( (windowFlags() | Qt::CustomizeWindowHint) & ~Qt::WindowMaximizeButtonHint);

    ui->latBox->setPrefixType(AngleSpinBox::PrefixType::Latitude);
    ui->lonBox->setPrefixType(AngleSpinBox::PrefixType::Longitude);

    connect(this, &MainWindow::onArduinoRead, this, &MainWindow::arduinoRead, Qt::QueuedConnection);


    server = std::make_shared<QTcpServer>(nullptr);
    connect(server.get(), &QTcpServer::newConnection, this, [this]()
    {
        if (stellarium) //holding only 1 connection to server
            stellarium->disconnectFromHost();

        stellarium = server->nextPendingConnection();
        connect(stellarium, &QAbstractSocket::disconnected,
                stellarium, &QObject::deleteLater);

        connect(stellarium, &QTcpSocket::readyRead, this, &MainWindow::onStellariumDataReady);
    });
    server->listen(QHostAddress::Any, 10001);

    on_pushButton_clicked();
    readSettings(this);
}

MainWindow::~MainWindow()
{
    comThread.reset();
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
    ui->lonBox->setRadians(settings.value("Longitude", 0).toDouble());
    ui->latBox->setRadians(settings.value("Latitude", 0).toDouble());
}

void MainWindow::recurseWrite(QSettings &settings, QObject *object)
{
    Q_UNUSED(object);
    auto ps  = ui->cbPorts->currentText();
    settings.setValue("COMPort", ps);
    settings.setValue("Longitude", ui->lonBox->valueRadians());
    settings.setValue("Latitude",  ui->latBox->valueRadians());
}

void MainWindow::on_pushButton_clicked()
{
    comThread.reset();
    ui->cbPorts->clear();
    ui->cbPorts->addItem("Select to Stop");
    ui->cbPorts->blockSignals(true);
    auto pl = QSerialPortInfo::availablePorts();
    for (const auto& p : pl)
        ui->cbPorts->addItem(p.portName(), QVariant::fromValue<QSerialPortInfo>(p));
    ui->cbPorts->setCurrentIndex(0);
    ui->cbPorts->blockSignals(false);
}

void MainWindow::on_cbPorts_currentIndexChanged(int index)
{
    if (index > 0)
        startComPoll();
    else
        comThread.reset();
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
        while (!(*stop)) //device reconnect loop
        {
            QSerialPort port(ui->cbPorts->currentData().value<QSerialPortInfo>());
            port.setBaudRate(115200);
            //8N1 default settings
            port.setDataBits(QSerialPort::Data8);
            port.setParity(QSerialPort::Parity::NoParity);
            port.setStopBits(QSerialPort::OneStop);

            const bool op = port.open(QIODevice::ReadWrite);
            std::this_thread::sleep_for(15s); // device gets reset, have to wait
            port.setDataTerminalReady(false); //disable autoreset of device
            skipWrite.test_and_set();

            while (!(*stop))
            {
                if (op) //if failed to open, still should start thread which does nothing
                {
                    const static std::string header(MESSAGE_HDR);
                    Message msg;
                    packAzEl(msg.message.value, gaz, gel);
                    bool shouldSet = !skipWrite.test_and_set();
                    msg.message.Command = (shouldSet)?'S':'R';
                    port.write(header.c_str(), static_cast<int>(header.size()));
                    port.write(msg.buffer, sizeof(msg.buffer));
                    if (!port.waitForBytesWritten(5000))
                        break;
                    if (!shouldSet)
                    {
                        const static auto msz = static_cast<decltype (port.bytesAvailable())>(sizeof(msg.message.value));
                        for(int i = 0; i < 100 && !(*stop) && port.bytesAvailable() < msz; ++i)
                            std::this_thread::sleep_for(20ms);

                        if (sizeof(msg.message.value.buffer) == port.read(msg.message.value.buffer, sizeof(msg.message.value.buffer)))
                        {
                            float az, el;
                            readAzEl(msg.message.value, &az, &el);
                            //qDebug() <<"Az = " <<degrees(az) << ";  El = "<<degrees(el);
                            emit this->onArduinoRead(az, el);
                        }
                    }
                }
                else
                    break;
                std::this_thread::sleep_for(1500ms);
            }
            if (!(*stop))
                std::this_thread::sleep_for(5000ms);
        }
    });
}

void MainWindow::write(float az_rad, float el_rad)
{
    gaz = az_rad;
    gel = el_rad;
    skipWrite.clear();
}

void MainWindow::on_pushButton_2_clicked()
{
    write(3.1415f, 0);
    /*
     *
     *   QByteArray block;
    QDataStream out(&block, QIODevice::WriteOnly);
    out.setVersion(QDataStream::Qt_4_0);

    out << fortunes.at(qrand() % fortunes.size());

*/
}

void MainWindow::onStellariumDataReady()
{
    auto all = stellarium->readAll();
    u_int16_t size = *reinterpret_cast<uint16_t*>(all.data()); //size counts itself, but we dont place it in packet
    //qDebug() << "Packet Size: "<<size;
    if (!(size < 4 || size > all.size() || size - 2 > sizeof(StellBasicMessage)))
    {
        StellBasicMessage msg;
        memcpy(msg.buffer, all.data() + 2, size - 2);
        //qDebug() << msg.msg.clientMicros << msg.msg.ra_int << msg.msg.dec_int;

        //took from ServerDummy.cpp, it is RADIANS now as we want
        const double ra  = msg.msg.ra_int  * (M_PI/static_cast<uint64_t>(0x80000000));
        const double dec = msg.msg.dec_int * (M_PI/static_cast<uint64_t>(0x80000000));
        qDebug() << "RA(hrs): "<< degrees(ra) / 15 << " DEC(deg): "<< degrees(dec);
        double az, alt;
        convert(ra, dec, ui->latBox->valueRadians(), ui->lonBox->valueRadians(), az, alt);
        qDebug() << "Az(deg): "<< degrees(az)<<" ALT(deg): "<<degrees(alt);
        write(static_cast<float>(az), static_cast<float>(alt));
    }
}
