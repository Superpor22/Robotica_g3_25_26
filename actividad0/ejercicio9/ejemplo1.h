#ifndef ejemplo1_H
#define ejemplo1_H

#include <QtGui>
#include "Timer.h"
#include "ui_counterDlg.h"

class ejemplo1 : public QWidget, public Ui_Counter
{
    Q_OBJECT
    public:
        ejemplo1();

    public slots:
        void doStop();
        void doCount(int a);
        void doReset();
        void doSlider(int value);

    private:
        Timer timer;
        int cont;
};

#endif // ejemplo1_H
