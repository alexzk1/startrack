#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    a.setApplicationName("StarTrackerQt");
    a.setApplicationVersion("0.1");
    a.setApplicationDisplayName("StarTracker");
    a.setOrganizationDomain("pasteover.net");
    a.setOrganizationName("pasteover.net");

    MainWindow w;
    w.show();

    return a.exec();
}
