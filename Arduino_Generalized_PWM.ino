unsigned long duty1,duty2;
// Duty Cycle in terms of a percentage.
unsigned long plus;
// Value read from A1, in case plus mode is activated
float xxx;
// Float numbers to calculate duty for PWM 1 and PWM 2
float yyy;
unsigned long pwm1;
// Value read from A0 and A2 to give PWM duty cycle output in terms // of 0-5V
unsigned long pwm2;
void setup(){

pinMode(9, OUTPUT);
pinMode(10, OUTPUT);
TCCR1A = _BV(COM1A1) | _BV(COM1B1) ; // phase and frequency correct mode. NON-inverted mode
// TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(COM1B0)  ;                 //   ************************ FOR Inverted PWM
//phase/frequency correct mode. SELECT THIS FOR INVERTED OUTPUTS.
TCCR1B = _BV(WGM13) | _BV(CS11);
// Select mode 8 and select divide by 8 on main clock.

}
 void init_pwm()
{

ICR1 = 10000; // 100Hz - Default value to 100Hz for A3 = 0V

/*
 * 
 ICR1 = 5000;            // 200Hz
 ICR1 = 2500;            // 400Hz
 ICR1 = 1000;            // 1000Hz
 ICR1 = 500;             // 2000Hz
 ICR1 = 333;             // 3000Hz
 ICR1 = 250;             // 4000Hz
 ICR1 = 100;             // 10000Hz
 ICR1 = 1000;            // Default value to 1kHz for A3 = 5V

*/
//ICR1 = 1000; // for ICR1 = 1000, frequency = 1kHz.
 }

void loop(){

// Program that lets different values on A3 choose different values of frequency, e.g. 100,200,400,500,1k,2k,3k,4k,10k,
//etc in relation with a free input.

//CHANGE DUTY CYCLE

//Assign values to OCR Registers, which output the PWM duty cycle.
OCR1B = int(xxx);  //  (0 to 100);     // for complimentary pwm xxx== yyy;
OCR1A = int(yyy);  // (0 to 100);      // for Dead band keep yyy slightly more ( by 1 or 2) than xxx.
}

