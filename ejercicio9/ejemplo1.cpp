#include "ejemplo1.h"
//#include <QTimer>
#include "Timer.h"

ejemplo1::ejemplo1(): Ui_Counter()
{

	setupUi(this);
	show();
	int a = 0;
	timer.connect(std::bind(&ejemplo1::doCount,this, a));
	timer.connect([](){std::cout<<"Hello World!"<<std::endl;});
	timer.connect(std::bind(&ejemplo1::doCount,this, a));
	timer.start(horizontalSlider->value());
	cont = 0;
}

void ejemplo1::doCount(int a)
{
	contador->display(cont++);
	printf("%d\n",a);
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

