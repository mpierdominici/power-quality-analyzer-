#include "arduinoFFT.h"
#define PIN_TEST 8
//el sampleo esta razonablemente bien, lo probe con matlab y una senal de 2.5 k estubo bien.
#define MAX_RESULTS  512
arduinoFFT FFT = arduinoFFT();
//fyente https://github.com/kosme/arduinoFFT
const byte adcPin = 0;  // A0

volatile double results [MAX_RESULTS];
//volatile double img [MAX_RESULTS];
volatile int resultNumber;
//fuente: https://blog.wildan.us/2017/11/03/arduino-fast-er-sampling-rate/
// ADC complete ISR
ISR (ADC_vect)
  {
  if (resultNumber >= MAX_RESULTS)
    ADCSRA = 0;  // turn off ADC
  else{
    //digitalWrite(PIN_TEST,HIGH);
    results[resultNumber] = ADC;//genera mucho gitter esta operacion a 50k de samplink rate
    //img[resultNumber]=0;

    resultNumber++;
      //results [resultNumber++]=(ADCH << 8) | ADCL;// a 25k tiene jitter pero menos
    
    //digitalWrite(PIN_TEST,LOW);
  }  // end of ADC_vect
  }
  
EMPTY_INTERRUPT (TIMER1_COMPB_vect);
 
void setup ()
  {
  Serial.begin(115200); // set baudrate
  Serial.println();
  pinMode(PIN_TEST,OUTPUT);
  // reset Timer 1
  TCCR1A  = 0;
  TCCR1B  = 0;
  TCNT1   = 0;
  TCCR1B  = bit (CS11) | bit (WGM12);  // CTC, prescaler of 8
  TIMSK1  = bit (OCIE1B); 
  OCR1A   = 380;    
  OCR1B   = 380; // 39 20 uS - sampling frequency 50 kHz

  ADCSRA  =  bit (ADEN) | bit (ADIE) | bit (ADIF); // turn ADC on, want interrupt on completion
  ADCSRA |= bit (ADPS2);  // Prescaler of 16
  ADMUX   = bit (REFS0) | (adcPin & 7);
  ADCSRB  = bit (ADTS0) | bit (ADTS2);  // Timer/Counter1 Compare Match B
  ADCSRA |= bit (ADATE);   // turn on automatic triggering
}

void loop () {
   double img [MAX_RESULTS];
  for (int i = 0; i < MAX_RESULTS; i++)
  {
    img[i]=0;
  }
  while (resultNumber < MAX_RESULTS) { }
  FFT.Windowing(results, MAX_RESULTS, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(results, img, MAX_RESULTS, FFT_FORWARD);
  FFT.ComplexToMagnitude(results, img, MAX_RESULTS);
  for (int i = 0; i < MAX_RESULTS; i++)
  {
    Serial.println (results[i]);
    //Serial.println (i+0.1);
  }
 while(1);
  
 
}
