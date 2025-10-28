#include "ejemplo1.h"
#include <QTimer>

ejemplo1::ejemplo1(): Ui_Counter()
{

	setupUi(this);
	show();
	connect(stopButton, SIGNAL(clicked()), this, SLOT(doStop()) );
	connect(&timer, SIGNAL(timeout()), this, SLOT(doCount()) );
	connect(resetButton, SIGNAL(clicked()), this, SLOT(doReset()) );
	connect(horizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(doSlider(int)) );
	timer.start(horizontalSlider->value());
	cont = 0;
}

void ejemplo1::doCount()
{
	contador->display(cont++);
}

void ejemplo1::doStop()
{
	timer.stop();
}

void ejemplo1::doReset()
{
	cont=0;
}

void ejemplo1::doSlider(int value)
{
	periodo->display(value);
	timer.setInterval(value);
}

