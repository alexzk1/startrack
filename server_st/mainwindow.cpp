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
#include <QDir>
#include <QFileDialog>
#include <QtSql/QtSql>

//if defined - writes all Z-readings to the file (for debug purposes)
//#define EXPORT_Z

#define SQL_DRIVER "QSQLITE"

#ifdef EXPORT_Z
#include <QFile>


#ifndef NO_STELLARIUM_RUN
#define NO_STELLARIUM_RUN
#endif

#endif

Q_DECLARE_METATYPE(QSerialPortInfo);

static const QString nightScheme = "QWidget {background-color: #660000;}";

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    comThread(nullptr),
    skipWrite(ATOMIC_FLAG_INIT),
    gaz(0), gel(0),
    server(nullptr),
    readyToTrackMap(false)
{
    ui->setupUi(this);
    setWindowFlags( (windowFlags() | Qt::CustomizeWindowHint) & ~Qt::WindowMaximizeButtonHint);

    ui->latBox->setPrefixType(AngleSpinBox::PrefixType::Latitude);
    ui->lonBox->setPrefixType(AngleSpinBox::PrefixType::Longitude);

    database = QSqlDatabase::addDatabase(SQL_DRIVER, "DB");

    connect(this, &MainWindow::onArduinoRead, this, &MainWindow::arduinoRead, Qt::QueuedConnection);


    server = std::make_shared<QTcpServer>(nullptr);
    connect(server.get(), &QTcpServer::newConnection, this, [this]()
    {
        static uint64_t connCounter = 0;
        auto dumb = connCounter++;
        QPointer<QTcpSocket> tmp = stellarium[dumb] = server->nextPendingConnection();
        connect(tmp, &QAbstractSocket::disconnected, this, [this, dumb, tmp]()
        {
            if (stellarium.count(dumb))
                stellarium.erase(dumb);

            if (tmp)
                tmp->deleteLater();
        }, Qt::QueuedConnection);

        connect(tmp, &QTcpSocket::readyRead, this, [this, tmp]()
        {
            if (tmp)
                onStellariumDataReady(tmp);
        }, Qt::QueuedConnection);

    }, Qt::QueuedConnection);
    server->listen(QHostAddress::Any, 10001);

    on_pushButton_clicked();
    readSettings(this);
}

MainWindow::~MainWindow()
{
    comThread.reset();
    writeSettings(this);
    if (database.isOpen())
        database.close();
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
    ui->cbNight->setChecked(settings.value("Night", false).toBool());
    ui->cbUseDb->setChecked(settings.value("UseDb", true).toBool());
    ui->editFolder->setText(settings.value("Folder", QDir::homePath()).toString());
}

void MainWindow::recurseWrite(QSettings &settings, QObject *object)
{
    Q_UNUSED(object);
    auto ps  = ui->cbPorts->currentText();
    settings.setValue("COMPort", ps);
    settings.setValue("Longitude", ui->lonBox->valueRadians());
    settings.setValue("Latitude",  ui->latBox->valueRadians());
    settings.setValue("Night", ui->cbNight->isChecked());
    settings.setValue("UseDb", ui->cbUseDb->isChecked());
    settings.setValue("Folder", ui->editFolder->text());
}

void MainWindow::on_pushButton_clicked()
{
    readyToTrackMap = false;
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

void MainWindow::startComPoll()
{
    comThread = utility::startNewRunner([this](auto stop)
    {
        using namespace std::chrono_literals;
        using namespace ard_st;

#ifdef EXPORT_Z
        QFile of (QDir::homePath()+"/sensor_values.csv");
        long os_counter = 0;
        of.open(QFile::WriteOnly | QFile::Truncate | QFile::Text);
        of.write("Azimuth, Altitude, W, X, Y, Z, Sample\n");
#endif

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
            readyToTrackMap = false;
            uint64_t counter = 0;
            while (!(*stop))
            {
                if (op) //if failed to open, still should start thread which does nothing
                {
                    const static std::string header(MESSAGE_HDR);
                    Message msg;
                    packAzEl(msg.message.value, gaz, gel);
                    bool shouldSet = !skipWrite.test_and_set();

                    if (shouldSet)
                    {
                        //cleansing all sensor data there and let'em gather some
                        msg.message.Command = 'C';
                        port.write(header.c_str(), static_cast<int>(header.size()));
                        port.write(msg.buffer, sizeof(msg.buffer));
                        if (*stop || !port.waitForBytesWritten(5000))
                            break;
                        std::this_thread::sleep_for(500ms);
                    }

                    msg.message.Command = (shouldSet)?'S':'R';
                    port.write(header.c_str(), static_cast<int>(header.size()));
                    port.write(msg.buffer, sizeof(msg.buffer));

                    if (*stop || !port.waitForBytesWritten(5000))
                        break;

                    readyToTrackMap = readyToTrackMap || shouldSet;
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
#ifdef EXPORT_Z

                            const static double div = 1 << (8 * sizeof(msg.message.value.vals.current_quat[0]) - 1);
                            of.write(QString("%1,%2,%3,%4,%5,%6,%7\n")
                                     .arg(degrees(az))
                                     .arg(degrees(el))
                                     .arg(msg.message.value.vals.current_quat[0] / div)
                                    .arg(msg.message.value.vals.current_quat[1] / div)
                                    .arg(msg.message.value.vals.current_quat[2] / div)
                                    .arg(msg.message.value.vals.current_quat[3] / div)
                                    .arg(os_counter++).toUtf8());
#endif
                        }
                    }
                    else
                        ++counter;
                }
                else
                    break;
                std::this_thread::sleep_for(500ms);
            }
            readyToTrackMap = false;
            if (!(*stop))
                std::this_thread::sleep_for(5000ms);
        }
    });
}

void MainWindow::writeToArduino(float az_rad, float el_rad)
{
    gaz = az_rad;
    gel = el_rad;
    skipWrite.clear();
}

int64_t MainWindow::getMicrosNow()
{
    using namespace std::chrono;
    return duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
}

void MainWindow::arduinoRead(float az_rad, float el_rad)
{
    ui->lblAz-> setText(QString("%1").arg(degrees(az_rad)));
    ui->lblAlt->setText(QString("%1").arg(degrees(el_rad)));
    if (readyToTrackMap && stellarium.size())
    {
        //writting alt/az from arduino to stellarium
        double ra, dec;
        convertAZ_RA(static_cast<double>(az_rad), static_cast<double>(el_rad), ui->latBox->valueRadians(), ui->lonBox->valueRadians(), ra, dec);
        //qDebug() << "Az(deg): "<< degrees(az_rad)<<" ALT(deg): "<<degrees(el_rad);
        //qDebug() << "RA(hrs): "<< degrees(ra) / 15 << " DEC(deg): "<< degrees(dec);
        if (stellarium.size())
        {

            ToStellMessage msg;
            msg.msg.size = sizeof(ToStellMessage);
            msg.msg.type   = 0;
            msg.msg.status = 0;
            msg.msg.clientMicros = getMicrosNow();
            //from ServerDummy.cpp
            msg.msg.ra_int  = static_cast<decltype(msg.msg.ra_int)>(floor(0.5   +  ra * static_cast<uint64_t>(0x80000000)/M_PI));
            msg.msg.dec_int = static_cast<decltype(msg.msg.dec_int)>(floor(0.5 +  dec * static_cast<uint64_t>(0x80000000)/M_PI)); //yes, uint64_t or u get negated dec
            for (const auto & s : stellarium)
            {

                s.second->write(msg.buffer, sizeof(ToStellMessage));
            }
            if (ui->cbUseDb->isChecked())
            {
                if (!database.isOpen())
                {
                    const static QString datestr = QDate::currentDate().toString("dd_MMM_yyyy");
                    QString sqlDb = ui->editFolder->text() + QDir::separator() + datestr + "_startrack.db";

                    database.setDatabaseName(sqlDb);
                    if (database.open())
                    {
                        //ensruing it has all needed tables
                        QSqlQuery sql(database);
                        qDebug() <<"CREATE TABLE: " <<
                                   sql.exec("CREATE TABLE IF NOT EXISTS Track(ID integer primary key asc, "
                                            "Az_rad REAL not null, Alt_rad REAL not null, LocalDt INTEGER not null, RA_rad not null, DEC_rad not null, "
                                            "W INTEGER, X INTEGER, Y INTEGER, Z INTEGER);"
                                            );
                        qDebug() <<"CREATE INDEX: " <<
                                   sql.exec("CREATE UNIQUE INDEX IF NOT EXISTS Track_LocalDt ON Track(LocalDt);");
                    }
                }

                if (database.isOpen())
                {
                    QSqlQuery sql(database);
                    sql.exec(QString("INSERT OR REPLACE INTO Track(Az_rad, Alt_rad, RA_rad, DEC_rad, LocalDt) VALUES(%1, %2, %3, %4, %5)")
                             .arg(az_rad).arg(el_rad).arg(ra).arg(dec)
                             .arg(QDateTime::currentMSecsSinceEpoch()));
                }
            }
        }
    }
}

void MainWindow::onStellariumDataReady(QTcpSocket *p)
{
    auto all = p->readAll();
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
        // qDebug() << "RA(hrs): "<< degrees(ra) / 15 << " DEC(deg): "<< degrees(dec);
        double az, alt;
        convertRA_AZ(ra, dec, ui->latBox->valueRadians(), ui->lonBox->valueRadians(), az, alt);
        //qDebug() << "Az(deg): "<< degrees(az)<<" ALT(deg): "<<degrees(alt);
        writeToArduino(static_cast<float>(az), static_cast<float>(alt));
    }
}

void MainWindow::on_cbNight_toggled(bool checked)
{
    if (checked)
        qApp->setStyleSheet(nightScheme);
    else
        qApp->setStyleSheet("");
}


void MainWindow::on_btnSelectFodler_clicked()
{
    QString dir = QFileDialog::getExistingDirectory(this, tr("Select folderto save tracks"), ui->editFolder->text(),
                                                    QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (!dir.isEmpty())
    {
        ui->editFolder->setText(dir);
        if (database.isOpen())
            database.close();
    }
}
