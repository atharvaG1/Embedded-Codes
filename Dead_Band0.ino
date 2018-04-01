//# include "driverlib/pwm.h"

//#include "driverlib/pin_map.h"

#include <stdint.h>
#include <stdbool.h>

//#include "inc/hw_gpio.h"
//#include "inc/hw_types.h"

#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
unsigned long desired_frequency;  // =?
unsigned long clock_ticks;

//Uncomment line below if frequency is entered.. If nothing is entered pwm will run at 31.5 khz
//clock_ticks= (SYSCTL_SYSDIV_1)/desired_frequency;
void init_pwm0(void)
{

  
SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC |   SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);


   //Configure PWM Clock to match system

SysCtlPWMClockSet(SYSCTL_PWMDIV_64);


   // Enable the peripherals used by this program.

    //SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);  //The Tiva Launchpad has 4 modules (0 and 1,2,3). 

    
 ///PF0 is locked by default in GPIO only NMI pin, check Table 10-1. GPIO Pins With Special Considerations on the datasheet. You need to unlock it 
HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
HWREG(GPIO_PORTF_BASE + GPIO_O_CR)   |= 0x01;  
  
  
GPIOPinConfigure(GPIO_PF2_M0PWM2);
    
    // uncomment these lines for complementary pwms accross PF1 AND PF0
GPIOPinConfigure(GPIO_PF3_M0PWM3);
 
GPIOPinTypePWM(GPIO_PORTF_BASE,  GPIO_PIN_2 | GPIO_PIN_3);


  //Comment line below if you want a complementary pwm
//GPIOPinTypePWM(GPIO_PORTF_BASE,  GPIO_PIN_1);
  
  
  
  
  
  
  
  // Configure the PWM generator for count down mode with immediate updates
  // to the parameters.
  //
//PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_SYNC|PWM_GEN_MODE_DB_SYNC_LOCAL);
//PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_SYNC|PWM_GEN_MODE_DB_SYNC_LOCAL);
//PWMSyncTimeBase(PWM0_BASE,PWM_GEN_1|PWM_GEN_2);
//PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
//PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
//PWMSyncTimeBase(PWM0_BASE,PWM_GEN_1|PWM_GEN_2);

PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_UP_DOWN |PWM_GEN_MODE_GEN_SYNC_LOCAL | PWM_GEN_MODE_DB_SYNC_LOCAL |PWM_GEN_MODE_DBG_RUN);
PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_UP_DOWN |PWM_GEN_MODE_GEN_SYNC_LOCAL | PWM_GEN_MODE_DB_SYNC_LOCAL |PWM_GEN_MODE_DBG_RUN);
//PWMSyncTimeBase(PWM0_BASE,PWM_GEN_1|PWM_GEN_2);





//PWMSyncUpdate(PWM0_BASE,PWM_GEN_1|PWM_GEN_2);


//
//PWMGenConfigure(PWM0_BASE, PWM_GEN_0,PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
//
// Set the period. For a 31.5 KHz frequency, the period = 1/31500,  For a 125 MHz clock, this translates to 4000 clock ticks.
// Use this value to set the period.
//

// uncomment these lines if desired frequency is entered...
/*
 * 
 * PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, clock_ticks);
 */
PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 6993);    //4000 = System clock/Desired Frequency
//
// Set the pulse width of PWM0 for a 37.5% duty cycle.
//
//PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 0);
//
// Set the pulse width of PWM1 for a 75% duty cycle.
// uncomment line below for complementary
//PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, 3000);   /// PF0= 62.5,, PF1= 37.5
//
// Start the timers in generator 0.
//
PWMGenEnable(PWM0_BASE, PWM_GEN_1);
//
// Enable the outputs.
// uncomment below line if you want complementary
//PWMDeadBandEnable(PWM0_BASE,PWM_GEN_1,0,4000);
PWMDeadBandEnable(PWM0_BASE, PWM_GEN_1, 200,200);
PWMOutputState(PWM0_BASE, (PWM_OUT_2_BIT | PWM_OUT_3_BIT), true);    
//PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);
}


void init_pwm1()
{

  
SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC |   SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);


   //Configure PWM Clock to match system

SysCtlPWMClockSet(SYSCTL_PWMDIV_64);


   // Enable the peripherals used by this program.

    //SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);  //The Tiva Launchpad has 4 modules (0 and 1,2,3). 

    
 ///PF0 is locked by default in GPIO only NMI pin, check Table 10-1. GPIO Pins With Special Considerations on the datasheet. You need to unlock it 
//HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
//HWREG(GPIO_PORTF_BASE + GPIO_O_CR)   |= 0x01;  
  
  
GPIOPinConfigure(GPIO_PG0_M0PWM4);
    
    // uncomment these lines for complementary pwms accross PF1 AND PF0
GPIOPinConfigure(GPIO_PG1_M0PWM5);
 
GPIOPinTypePWM(GPIO_PORTG_BASE,  GPIO_PIN_0 | GPIO_PIN_1);


  //Comment line below if you want a complementary pwm
//GPIOPinTypePWM(GPIO_PORTG_BASE,  GPIO_PIN_1);
  
  
  
  
  
  
  
  // Configure the PWM generator for count down mode with immediate updates
  // to the parameters.
  //

//PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

//PWMGenConfigure(PWM0_BASE, PWM_GEN_0,PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
//
// Set the period. For a 31.5 KHz frequency, the period = 1/31500,  For a 125 MHz clock, this translates to 4000 clock ticks.
// Use this value to set the period.
//

// uncomment these lines if desired frequency is entered...
/*
 * 
 * PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, clock_ticks);
 */
PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, 6993);    //4000 = System clock/Desired Frequency

//
// Set the pulse width of PWM0 for a 37.5% duty cycle.
//
//PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 0);
//
// Set the pulse width of PWM1 for a 75% duty cycle.
// uncomment line below for complementary
//PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, 3000);   /// PF0= 62.5,, PF1= 37.5
//
// Start the timers in generator 0.
//7
PWMGenEnable(PWM0_BASE, PWM_GEN_2);
//
// Enable the outputs.
// uncomment below line if you want complementary
//PWMDeadBandEnable(PWM0_BASE,PWM_GEN_1,0,4000);
PWMDeadBandEnable(PWM0_BASE, PWM_GEN_2,200,200);

PWMOutputState(PWM0_BASE, (PWM_OUT_4_BIT | PWM_OUT_5_BIT), true);    
//PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);

  
}
  
void init_pwm2(void)
{

  
SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC |   SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);


   //Configure PWM Clock to match system

SysCtlPWMClockSet(SYSCTL_PWMDIV_64);


   // Enable the peripherals used by this program.

    //SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);  //The Tiva Launchpad has 4 modules (0 and 1,2,3). 

    
 ///PF0 is locked by default in GPIO only NMI pin, check Table 10-1. GPIO Pins With Special Considerations on the datasheet. You need to unlock it 
//HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
//HWREG(GPIO_PORTF_BASE + GPIO_O_CR)   |= 0x01;  
  
  
GPIOPinConfigure(GPIO_PK4_M0PWM6);
    
    // uncomment these lines for complementary pwms accross PF1 AND PF0
//GPIOPinConfigure(GPIO_PF3_M0PWM3);
 
GPIOPinTypePWM(GPIO_PORTK_BASE,  GPIO_PIN_4);


  //Comment line below if you want a complementary pwm
//GPIOPinTypePWM(GPIO_PORTF_BASE,  GPIO_PIN_1);
  
  
  
  
  
  
  
  // Configure the PWM generator for count down mode with immediate updates
  // to the parameters.
  //

PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
//PWMSyncUpdate(PWM0_BASE,PWM_GEN_1|PWM_GEN_2);
//PWMGenConfigure(PWM0_BASE, PWM_GEN_0,PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
//
// Set the period. For a 31.5 KHz frequency, the period = 1/31500,  For a 125 MHz clock, this translates to 4000 clock ticks.
// Use this value to set the period.
//

// uncomment these lines if desired frequency is entered...
/*
 * 
 * PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, clock_ticks);
 */
PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, 625);    //4000 = System clock/Desired Frequency
//
// Set the pulse width of PWM0 for a 37.5% duty cycle.
//
//PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 0);
//
// Set the pulse width of PWM1 for a 75% duty cycle.
// uncomment line below for complementary
//PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, 3000);   /// PF0= 62.5,, PF1= 37.5
//
// Start the timers in generator 0.
//
PWMGenEnable(PWM0_BASE, PWM_GEN_3);
//
// Enable the outputs.
// uncomment below line if you want complementary
//PWMDeadBandEnable(PWM0_BASE,PWM_GEN_1,0,4000);
//PWMDeadBandEnable(PWM0_BASE, PWM_GEN_3, 200,200);
PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT, true);    
//PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);
}


void setup() {
 
  // put your setup code here, to run once:
//PWMSyncUpdate(PWM0_BASE,PWM_GEN_1|PWM_GEN_2);

init_pwm0();

init_pwm1();
unsigned long freq= _SysCtlFrequencyGet();
//init_pwm2();
Serial.begin(9600);
Serial.println(freq);
PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, 3497);   //1   pf3  pg1
PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 3497);  //2    PF2  pg0

PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, 3497);   //3    PG1
PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, 3497);   //4    PG0
//PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, 312);  //PK4
}

void loop() {
  
  // put your main code here, to run repeatedly:    //You can change Duty cycle /Period here in void loop By just calling corresponding API.
    // Here enter an unsigned long number = duty cycle* clock_ticks
  //delay(10000);
  //PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 1000);
 // delay(10000);
}
