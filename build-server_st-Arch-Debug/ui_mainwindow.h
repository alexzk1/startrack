/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.8.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <AngleSpinBox.hpp>
#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QFormLayout *formLayout;
    QLabel *label;
    QComboBox *cbPorts;
    QPushButton *pushButton;
    QLabel *label_2;
    QLabel *lblAz;
    QLabel *label_3;
    QLabel *lblAlt;
    QLabel *label_4;
    QLabel *label_5;
    AngleSpinBox *latBox;
    QLabel *label_6;
    AngleSpinBox *lonBox;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(358, 425);
        MainWindow->setContextMenuPolicy(Qt::NoContextMenu);
        MainWindow->setUnifiedTitleAndToolBarOnMac(true);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        formLayout = new QFormLayout(centralWidget);
        formLayout->setSpacing(6);
        formLayout->setContentsMargins(11, 11, 11, 11);
        formLayout->setObjectName(QStringLiteral("formLayout"));
        label = new QLabel(centralWidget);
        label->setObjectName(QStringLiteral("label"));

        formLayout->setWidget(0, QFormLayout::LabelRole, label);

        cbPorts = new QComboBox(centralWidget);
        cbPorts->setObjectName(QStringLiteral("cbPorts"));

        formLayout->setWidget(0, QFormLayout::FieldRole, cbPorts);

        pushButton = new QPushButton(centralWidget);
        pushButton->setObjectName(QStringLiteral("pushButton"));

        formLayout->setWidget(1, QFormLayout::FieldRole, pushButton);

        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QStringLiteral("label_2"));

        formLayout->setWidget(3, QFormLayout::LabelRole, label_2);

        lblAz = new QLabel(centralWidget);
        lblAz->setObjectName(QStringLiteral("lblAz"));

        formLayout->setWidget(3, QFormLayout::FieldRole, lblAz);

        label_3 = new QLabel(centralWidget);
        label_3->setObjectName(QStringLiteral("label_3"));

        formLayout->setWidget(4, QFormLayout::LabelRole, label_3);

        lblAlt = new QLabel(centralWidget);
        lblAlt->setObjectName(QStringLiteral("lblAlt"));

        formLayout->setWidget(4, QFormLayout::FieldRole, lblAlt);

        label_4 = new QLabel(centralWidget);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setAlignment(Qt::AlignCenter);

        formLayout->setWidget(5, QFormLayout::SpanningRole, label_4);

        label_5 = new QLabel(centralWidget);
        label_5->setObjectName(QStringLiteral("label_5"));

        formLayout->setWidget(6, QFormLayout::LabelRole, label_5);

        latBox = new AngleSpinBox(centralWidget);
        latBox->setObjectName(QStringLiteral("latBox"));

        formLayout->setWidget(6, QFormLayout::FieldRole, latBox);

        label_6 = new QLabel(centralWidget);
        label_6->setObjectName(QStringLiteral("label_6"));

        formLayout->setWidget(7, QFormLayout::LabelRole, label_6);

        lonBox = new AngleSpinBox(centralWidget);
        lonBox->setObjectName(QStringLiteral("lonBox"));

        formLayout->setWidget(7, QFormLayout::FieldRole, lonBox);

        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 358, 22));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);
#ifndef QT_NO_SHORTCUT
        label->setBuddy(cbPorts);
        label_5->setBuddy(latBox);
        label_6->setBuddy(lonBox);
#endif // QT_NO_SHORTCUT

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "Arduino<->Stellarium Connector", Q_NULLPTR));
        label->setText(QApplication::translate("MainWindow", "&Device Port:", Q_NULLPTR));
        pushButton->setText(QApplication::translate("MainWindow", "&Refresh", Q_NULLPTR));
        label_2->setText(QApplication::translate("MainWindow", "Azimuth:", Q_NULLPTR));
        lblAz->setText(QApplication::translate("MainWindow", "-", Q_NULLPTR));
        label_3->setText(QApplication::translate("MainWindow", "Alt:", Q_NULLPTR));
        lblAlt->setText(QApplication::translate("MainWindow", "-", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        label_4->setToolTip(QApplication::translate("MainWindow", "...ensure system uses proper clock set too!", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        label_4->setText(QApplication::translate("MainWindow", "Current Place (set the same as in Stellarium)", Q_NULLPTR));
        label_5->setText(QApplication::translate("MainWindow", "L&atitude:", Q_NULLPTR));
        label_6->setText(QApplication::translate("MainWindow", "L&ongitude:", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
