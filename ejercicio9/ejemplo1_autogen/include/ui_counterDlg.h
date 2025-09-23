/********************************************************************************
** Form generated from reading UI file 'counterDlg.ui'
**
** Created by: Qt User Interface Compiler version 6.4.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_COUNTERDLG_H
#define UI_COUNTERDLG_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLCDNumber>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Counter
{
public:
    QGridLayout *gridLayout_3;
    QSlider *horizontalSlider;
    QPushButton *stopButton;
    QPushButton *resetButton;
    QLCDNumber *periodo;
    QLCDNumber *contador;

    void setupUi(QWidget *Counter)
    {
        if (Counter->objectName().isEmpty())
            Counter->setObjectName("Counter");
        Counter->resize(418, 354);
        gridLayout_3 = new QGridLayout(Counter);
        gridLayout_3->setObjectName("gridLayout_3");
        horizontalSlider = new QSlider(Counter);
        horizontalSlider->setObjectName("horizontalSlider");
        horizontalSlider->setMaximum(1000);
        horizontalSlider->setValue(500);
        horizontalSlider->setOrientation(Qt::Horizontal);

        gridLayout_3->addWidget(horizontalSlider, 6, 0, 1, 2);

        stopButton = new QPushButton(Counter);
        stopButton->setObjectName("stopButton");

        gridLayout_3->addWidget(stopButton, 3, 1, 1, 1);

        resetButton = new QPushButton(Counter);
        resetButton->setObjectName("resetButton");

        gridLayout_3->addWidget(resetButton, 4, 1, 1, 1);

        periodo = new QLCDNumber(Counter);
        periodo->setObjectName("periodo");

        gridLayout_3->addWidget(periodo, 6, 2, 1, 1);

        contador = new QLCDNumber(Counter);
        contador->setObjectName("contador");
        contador->setMaximumSize(QSize(16777215, 443));

        gridLayout_3->addWidget(contador, 0, 0, 1, 3);


        retranslateUi(Counter);

        QMetaObject::connectSlotsByName(Counter);
    } // setupUi

    void retranslateUi(QWidget *Counter)
    {
        Counter->setWindowTitle(QCoreApplication::translate("Counter", "Counter", nullptr));
        stopButton->setText(QCoreApplication::translate("Counter", "STOP", nullptr));
        resetButton->setText(QCoreApplication::translate("Counter", "RESET", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Counter: public Ui_Counter {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_COUNTERDLG_H
