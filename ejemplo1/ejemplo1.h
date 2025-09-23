#ifndef ejemplo1_H
#define ejemplo1_H

#include <QtGui>
#include "ui_counterDlg.h"

class ejemplo1 : public QWidget, public Ui_Counter
{
    Q_OBJECT
    public:
        ejemplo1();

    public slots:
        void doStop();
        void doCount();
        void doReset();
        void doSlider(int value);

    private:
        QTimer timer;
        int cont;
};

#endif // ejemplo1_H
