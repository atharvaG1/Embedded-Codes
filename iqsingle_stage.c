#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "inc/tm4c1294ncpdt.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
//#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/adc.h"
#include "driverlib/uart.h"
#include "IQmath/IQmathLib.h"
#include "math.h"
#include <string.h>

int Dinit = 214;
_iq13 iqDinit;
int Dmax = 230;
_iq13 iqDmax;
int Dmin = 195;
_iq13 iqDmin;
int flag = 0, flag1 = 0;
float deltaD = 0.003, D;
_iq13 iqdeltaD;
float P, V, I, dV, dP, Vpu, Ipu, Vcap, Vd, Vq, Id, Iq, Vac, Iac;
_iq13 iqP, iqV, iqI, iqdV, iqdP, iqVpu, iqIpu, iqVcap, iqVd, iqVq, iqId, iqIq, iqVac, iqIac;
float Pold = 0, Vold = 0, Dold = 0;

double ts = 0, t = 0;
double delta = 0.0001;//100uS
float freq = 50;
float w = 314;//6.28*freq;
_iq13 iqw;
float apf1, xpf1 = 0, xpf2 = 0, ypf1 = 0, ypf2 = 0, ipf1 = 0, ipf2 = 0, iipf1 = 0, lf = 0, rf = 0;
_iq13 iqypf1, iqipf1;
float Vkp = 10, Vki = 200;
float edc;
float a0, a1, yn1, yn2, un;
float ikp = 0.15, iki = 6.6, ia0, ia1, eid, eiq, iyn1, iyn2, iqy1, iqy2, iqu, iun, iqq1, Vd1, Vq1, mod, d1, d2;
_iq13 iqVd1, iqVq1;
_iq13 iqyn1, iqiqq1;
_iq13 iqwt;


/***ADC***/
uint32_t adc[9];
volatile uint32_t Vpv;
volatile uint32_t Ipv;
volatile uint32_t Va;
volatile uint32_t Ia;
volatile uint32_t Vcp;

/***Sogi Variables***/
float al1, al2, al3, a1, b0, adc1, adc2, adc3, a2, be1, qb0, qb1, qb2, be2, be3, Q;
_iq13 iqbe1, iqal1, iqQ, iqmod;
float x;
float y;
float b0;
float b2;
float a1;
float a2;
float qb0;
float qb1;
float qb2;
float k = 0.7;
float twopow;

float val = (6.28/360)*25;

/*** Initalize ADC & TIMER ***/

void init_adc()
{
	uint32_t ui32Period1;
    uint32_t ui32SysClkFreq;

    //set the system clock to 120Mhz
    ui32SysClkFreq = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);

    //enable the peripheral N and timer0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

    //enable adc0 and portE, portD
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    //ulsequence=0
    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);

    //ADCSequenceStepConfigure(ulBase, ulSequenceNum, ulStep i, ulConfig);
    // ulConfig= channel to be sampled is selected
    // ulSequenceNum= Valid sample sequencers
    //range from zero to three; sequencer zero captures up to eight samples, sequencers one and
    //two capture up to four samples, and sequencer three captures a single sample.
    // ulstep = The ulStep parameter determines the order in which the samples are captured by the ADC when the trigger occurs

	ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH8);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH7);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 2, ADC_CTL_CH6);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 3, ADC_CTL_CH5);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 4, ADC_CTL_CH4);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 5, ADC_CTL_CH3);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 6, ADC_CTL_CH2);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 7, ADC_CTL_CH1|ADC_CTL_IE|ADC_CTL_END);

    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 );
	GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 );

    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    //TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);

    //timer
    ui32Period1 = ui32SysClkFreq/8192; // timerisr

    //ui32period = ui32SysClkFreq/0.1;  //  Uart timer

    TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period1 -1);///////   1

    IntEnable(INT_TIMER0A);

    //TimerLoadSet(TIMER1_BASE, TIMER_A, ui32period -1);/////////  2

   // IntEnable(INT_TIMER1A);

    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    //TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    IntMasterEnable();

    TimerEnable(TIMER0_BASE, TIMER_A);

   // TimerEnable(TIMER1_BASE, TIMER_A);
}


/*** PWM INITIALIZE ***/

void init_pwm()

{
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

    //enable peripherals for PWM pins and module 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);  //The Tiva Launchpad has 4 modules (0 and 1,2,3).

    //enable pwm pins pf2,pf3,pg0,pg1
    GPIOPinConfigure(GPIO_PF2_M0PWM2);
    GPIOPinConfigure(GPIO_PF3_M0PWM3);
    GPIOPinConfigure(GPIO_PG0_M0PWM4);
    GPIOPinConfigure(GPIO_PG1_M0PWM5);

    //Configures pin(s) for use by the PWM peripheral
    GPIOPinTypePWM(GPIO_PORTF_BASE,  GPIO_PIN_2 | GPIO_PIN_3);
    GPIOPinTypePWM(GPIO_PORTG_BASE,  GPIO_PIN_0 | GPIO_PIN_1);

    //Configures a PWM generator.
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);//uses either of PWM_GEN_0,1,2 and 3
    PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    //Synchronizes the counters in one or multiple PWM generator blocks. logical of either of the mentioned generators
    //each generators produces 2 pwm
    PWMSyncTimeBase(PWM0_BASE,PWM_GEN_1|PWM_GEN_2);

    /*This function sets the period of the specified PWM generator block, where the period of the
    generator block is defined as the number of PWM clock ticks between pulses on the generator
    block zero signal.*/
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 6667);  //PWM freq is 6667 = 18KHz
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, 6667);

    //Enables the timer/counter for a PWM generator block
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
    PWMGenEnable(PWM0_BASE, PWM_GEN_2);

    /*This function sets the dead bands for the specified PWM generator, where the dead bands
    are defined as the number of PWM clock ticks from the rising or falling edge of the generator’s
    OutA signal.*/
    PWMDeadBandEnable(PWM0_BASE, PWM_GEN_1, 150,150);
    PWMDeadBandEnable(PWM0_BASE, PWM_GEN_2, 150,150);

    /*This function enables or disables the selected PWM outputs. The outputs are selected using
     the parameter ulPWMOutBits. The parameter bEnable determines the state of the selected
     outputs. If bEnable is true, then the selected PWM outputs are enabled, or placed in the active
     state. If bEnable is false, then the selected outputs are disabled or placed in the inactive state. */
    PWMOutputState(PWM0_BASE, (PWM_OUT_2_BIT | PWM_OUT_3_BIT), true);
    PWMOutputState(PWM0_BASE, (PWM_OUT_4_BIT | PWM_OUT_5_BIT), true);

}


/*** P&O ALGORITHM ***/

void po()
{

	P = V * I;
	dV = V - Vold;
	dP = P - Pold;

	if (dP != 0)
	{
		if(dP < 0)
		{
			if(dV < 0)
			{
				D = Dold + deltaD;
			}
			else
			{
				D = Dold - deltaD;
			}
		}
		else if (dV < 0)
		{
			D = Dold - deltaD;
		}
		else
		{
			D = Dold + deltaD;
		}
	}
	else
	{
		D = Dold;
	}

	if (D >= Dmax || D <= Dmin)
	{
		D = Dold;
	}

	Dold = D;
	Vold = V;
	Pold = P;
}

/*** PLL USING SOGI ***/

void pll()
{
	w = 2 * 3.14 * freq;
	iqw = _IQ13(w);
    x = 2 * k * w * delta;
    y = w * w * delta * delta;
    b0 = x / (x + y + 4);
    b2 = -b0;
    a1 = (2 * (4 - y)) / (x + y + 4);
    a2 = (x - y - 4) / (x + y + 4);
    qb0 = (k * y) / (x + y + 4);
    qb1 = 2 * qb0;
    qb2 = qb0;

	al1 = b0*(Vpu-adc3)+a1*al2 + a2*al3;
	iqal1 = _IQ13(al1);
	al3 = al2;
	al2 = al1;
	be1 = qb0*adc1 + qb1*adc2 + qb2*adc3 +a1*be2 + a2*be3;
	iqbe1 = _IQ13(be1);
	be3 = be2;
	be2 = be1;
	adc3 = adc2;
	adc2 = Vpu;

	//Q = be1*sin(w*t)+al1*cos(w*t);

	iqQ = _IQ13mpy(iqbe1, _IQ13sin(_IQ13mpy(iqw,t))) + _IQ13mpy(iqal1,_IQ13sin(_IQ13mpy(iqw,t)));
	Q = iqQ * twopow;

	freq = freq + 0.002 * Q;
}

/*** With anti islanding ***/

void anti_islanding()
{

	flag = 1;

	while(1)
	{
		if(Vac < -0.5 || flag1 == 1)  //???????
		{
			/** Chk ZERO CROSSING HERE **/

			if (al1>= 0 && al2<0)
			{
				t = 0;
				D=Dinit ;
				P=0;
				freq=50;
				/*** flag ***/
				flag = 0;
				flag1 = 0;
				break;
			}
		}

		d1 = 0;

		d2 = 0;
	}
}


int main(void)
{
	init_adc();

	init_pwm();

	twopow = pow(2,-13);

	while(1);
}



/*** Enter after every 100uS (INVERTER_CONTROL) ***/

void timerisr(void)
{
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);


    ADCSequenceEnable(ADC0_BASE, 0);

    ADCIntClear(ADC0_BASE, 0);

    ADCProcessorTrigger(ADC0_BASE, 0);

    while(!ADCIntStatus(ADC0_BASE, 0, false)) ;      //We need to wait for the conversion to complete

    ADCSequenceDataGet(ADC0_BASE, 0, adc);

    Vpv = adc[0];

    Ipv = adc[1];

    Va = adc[2];

	Ia = adc[3];

	Vcp = adc[4];



	/*** Vpv, Do appropriate calculations, Calibration results is stored in V ***/

    /*** Ipv, Do appropriate calculations, Calibration results is stored in I ***/

    /*** Va, Do appropriate calculations, Calibration results is stored in Vac ***/

	/*** Ia, Do appropriate calculations, Calibration results is stored in Iac ***/

	/*** Vcp,  Do appropriate calculations, Calibration results is stored in Vcap ***/

	Vpu = Vac/155.5;
	iqVpu = _IQ13(Vpu);

	Ipu = (Iac*110)/(1200*1.41);

	iqIpu = _IQ13(Ipu);


	pll();

	if(flag == 0)
	{
		/*** Call po Function ***/

		po();

		/*** INVERTER_CONTROL ***/



		apf1 = (w*delta-2)/(w*delta+2);
		ypf1 = apf1*Vpu + xpf1-apf1*ypf2;
		iqypf1 = _IQ13(ypf1);
		ypf2 = ypf1;
		xpf1 = Vpu;

		ipf1 = apf1*Ipu + iipf1-apf1*ipf2;
		ipf2 = ipf1;
		iqipf1 = _IQ13(ipf1);
		iipf1 = Ipu;

/*		Vd = Vpu*sin(w*t) - ypf1*cos(w*t);

		Vq = Vpu*cos(w*t) + ypf1*sin(w*t);

		Id = Ipu*sin(w*t) - ipf1*cos(w*t);

		Iq = Ipu*cos(w*t) + ipf1*sin(w*t);

*/

		iqwt = _IQ13mpy(iqw,t);

		/*** IQ CALCULATION ***/

		iqVd = _IQ13mpy(iqVpu,_IQ13sin(iqwt)) - _IQ13mpy(iqypf1,_IQ13cos(iqwt));
		iqVq = _IQ13mpy(iqVpu,_IQ13cos(iqwt)) + _IQ13mpy(iqypf1,_IQ13sin(iqwt));
		iqId = _IQ13mpy(iqIpu,_IQ13sin(iqwt)) - _IQ13mpy(iqipf1,_IQ13cos(iqwt));
		iqIq = _IQ13mpy(iqIpu,_IQ13cos(iqwt)) + _IQ13mpy(iqipf1,_IQ13sin(iqwt));

		Vd = iqVd*twopow;
		Vq = iqVq*twopow;
		Id = iqId*twopow;
		Iq = iqIq*twopow;

		/***Vcap ADC reading***/

		edc = (Vcap - D)/220;

		a0 = 0.5*(Vki*delta+2*Vkp);
		a1 = 0.5*(Vki*delta-2*Vkp);

		yn1 = a0*edc + a1*un + yn2;
		if (yn1 > 1.5)
		{
			yn1 = 1.5;
		}
		else if(yn1 < -1.5)
		{
			yn1 = -1.5;
		}
		un = edc;
		yn2 = yn1;


		/*** With Anti Islanding ***/

		ia0 = ikp;
		ia1 = iki*ts - ikp;

	/*	yn1 = yn1*cos(((6.28/360)*25*sin((3.14/2)*(w-50)/3)));

		iqq1 = yn1*sin(((6.28/360)*25*sin((3.14/2)*(w-50)/3)));*/

		/*** IQ CALCULATION ***/

		iqyn1 = _IQ13(yn1);

		iqyn1 =  _IQ13mpy(iqyn1,_IQ13cos(_IQ13mpy(_IQ13(val),_IQ13sin(_IQ13mpy(_IQ13(1.57),_IQ13div((iqw-_IQ13(50)),_IQ13(3)))))));

		iqiqq1 = _IQ13mpy(iqyn1,_IQ13sin(_IQ13mpy(_IQ13(val),_IQ13sin(_IQ13mpy(_IQ13(1.57),_IQ13div((iqw-_IQ13(50)),_IQ13(3)))))));

		yn1 = iqyn1*twopow;

		iqq1 = iqiqq1*twopow;

		eid = yn1 - Id;
		eiq = iqq1- Iq;

		iyn1 = ia0*eid + ia1*iun + iyn2;
		iqy1 = ia0*eiq + ia1*iqu + iqy2;

		iun = eid;
		iqu = eiq;
		iyn2 = iyn1;
		iqy2 = iqy1;

		/*** END With Anti Islanding ***/

		/************** OR **************/

		/*** Without Anti Islanding ***/

		ia0 = ikp;
		ia1 = iki*ts - ikp;

		eid = yn1 - Id;
		eiq = 0 - Iq;

		iyn1 = ia0*eid + ia1*iun + iyn2;
		iqy1 = ia0*eiq + ia1*iqu + iqy2;

		iun = eid;
		iqu = eiq;
		iyn2 = iyn1;
		iqy2 = iqy1;

		/*** END Without Anti Islanding ***/

		Vd1 = iyn1 + Vd - Iq*lf + Id*rf;

		iqVd1 = _IQ13(Vd1);

		Vq1 = iqy1 + Vq + Id*lf + Iq*rf;

		iqVq1 = _IQ13(Vq1);

	//	mod = Vd1*sin(w*t) + Vq1*cos(w*t);

		iqmod = _IQ13mpy(iqVd1,_IQ13sin(iqwt)) + _IQ13mpy(iqVq1,_IQ13cos(iqwt));

		mod = iqmod*twopow;

		if (mod<-1)

		{
			//ed = 0; //?????????

			//eq = 0;

			mod = -1;
		}

		else if (mod>1)

		{
			//ed = 0;

			//eq = 0;

			mod = 1;
		}

		d1 = (mod+1)*0.5;

		d2 = (-mod+1)*0.5;

		d1=(unsigned long)(d1*6250);

		d2=  (unsigned long)(d2*6250);

		//t = t + delta;

		t = t + 1;


		if(freq<=49.5 || freq>=50.5)
		{
			anti_islanding();
		}
	}

	/*********** PWM GENERATION ***********/

	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, d1);   //1   PF2  d1

	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, d2);   //2   PF3 d2

	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, d2);   //3   PG1 d2

	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, d1);   //4   PG0 d1

}
