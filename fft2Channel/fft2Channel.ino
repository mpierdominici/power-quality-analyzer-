#include "arduinoFFT.h"
#define PIN_TEST 8
//el sampleo esta razonablemente bien, lo probe con matlab y una senal de 2.5 k estubo bien.
#define SAMPLES_AMAUNT  512
arduinoFFT FFT = arduinoFFT();
//fyente https://github.com/kosme/arduinoFFT
const byte adcPin = 0;  // A0

volatile double voltage [SAMPLES_AMAUNT];
volatile double current [SAMPLES_AMAUNT];
//volatile double img [SAMPLES_AMAUNT];
volatile unsigned int resultNumber;
//fuente: https://blog.wildan.us/2017/11/03/arduino-fast-er-sampling-rate/
// ADC complete ISR
double * samplesMeas[]={voltage,current};

ISR (ADC_vect)
  {
  if (resultNumber >= 2*SAMPLES_AMAUNT)//uso el dbole de la cantidad de muestras xq estoy sampleando dos canales
    ADCSRA = 0;  // turn off ADC
  else{
    (samplesMeas[resultNumber&1])[(resultNumber-(resultNumber&1))>>1]=ADC;//resultNumber pares son mediciones de voltaje, despues  (resultNumber-(resultNumber&1))>>1 corrige el indice de cada arreglo
    resultNumber++;
    ADMUX   = bit (REFS0) | ((resultNumber&1) & 7);//selecciono los mux
  }  
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
  OCR1A   = 190;    
  OCR1B   = 190; //frecuencia de sampleo del adc 10.26KHz

  ADCSRA  =  bit (ADEN) | bit (ADIE) | bit (ADIF); // turn ADC on, want interrupt on completion
  ADCSRA |= bit (ADPS2);  // Prescaler of 16
  ADMUX   = bit (REFS0) | (adcPin & 7);
  ADCSRB  = bit (ADTS0) | bit (ADTS2);  // Timer/Counter1 Compare Match B
  ADCSRA |= bit (ADATE);   // turn on automatic triggering
}

void loop () {
   double img [SAMPLES_AMAUNT];
  for (int i = 0; i < SAMPLES_AMAUNT; i++)
  {
    img[i]=0;
  }
  while (resultNumber < 2*SAMPLES_AMAUNT) { }//sampleo de dos canales
  FFT.Windowing(voltage, SAMPLES_AMAUNT, FFT_WIN_TYP_HAMMING, FFT_FORWARD);//fft para la tension
  FFT.Compute(voltage, img, SAMPLES_AMAUNT, FFT_FORWARD);
  FFT.ComplexToMagnitude(voltage, img, SAMPLES_AMAUNT);

  for (int i = 0; i < SAMPLES_AMAUNT; i++)
  {
    img[i]=0;
  }
  FFT.Windowing(current, SAMPLES_AMAUNT, FFT_WIN_TYP_HAMMING, FFT_FORWARD);//fft para la corriente
  FFT.Compute(current, img, SAMPLES_AMAUNT, FFT_FORWARD);
  FFT.ComplexToMagnitude(current, img, SAMPLES_AMAUNT);
  
  
  for (int i = 0; i < SAMPLES_AMAUNT; i++)
  {
    Serial.print(voltage[i]);
    Serial.print(" ");
    Serial.println(current[i]);
    //Serial.println (i+0.1);
  }
 while(1);
  
 
}
