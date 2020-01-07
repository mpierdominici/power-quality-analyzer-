#include "arduinoFFT.h"
#define PIN_TEST 8
#define CERO_MEAS 400
//el sampleo esta razonablemente bien, lo probe con matlab y una senal de 2.5 k estubo bien.
#define SAMPLES_AMAUNT  512
#define RMS_PERIOD 102



#define PIN_LR 24
#define PIN_LA 26
#define PIN_LV 28
arduinoFFT FFT = arduinoFFT();
//fyente https://github.com/kosme/arduinoFFT
const byte adcPin = 0;  // A0

volatile double voltage [SAMPLES_AMAUNT];
volatile double current [SAMPLES_AMAUNT];
//volatile double img [SAMPLES_AMAUNT];
volatile unsigned int resultNumber;
//fuente: https://blog.wildan.us/2017/11/03/arduino-fast-er-sampling-rate/
// ADC complete ISR
double * samplesMeas[]={current,voltage};

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
      pinMode(PIN_LR,OUTPUT);
  pinMode(PIN_LA,OUTPUT);
  pinMode(PIN_LV,OUTPUT);
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
  double irms=0;
  double vrms=0;
  double pinst=0;
   double img [SAMPLES_AMAUNT];
   digitalWrite(PIN_LR,HIGH);
  
  for (int i = 0; i < SAMPLES_AMAUNT; i++)
  {
    
    img[i]=0;
  }
  while (resultNumber < 2*SAMPLES_AMAUNT) { }//sampleo de dos canales
  for (int i = 0; i < SAMPLES_AMAUNT; i++)
  {
    voltage[i]=(voltage[i]-CERO_MEAS)*(5.0/1023.0)*(282.84);
     //Serial.println(voltage[i]);
    current[i]=(current[i]-CERO_MEAS)*(5.0/1023.0)*(50/3.0);
    
  }
  for (int i = 0; i < RMS_PERIOD; i++)
  {
    pinst=pinst+(voltage[i]*current[i]);
    vrms= vrms + (voltage[i]* voltage[i]);
    irms= irms +(current[i] * current[i]);
  }
  vrms=vrms/RMS_PERIOD;
  irms=irms/RMS_PERIOD;
  pinst=pinst/RMS_PERIOD;
  
  vrms=sqrt(vrms);
  irms=sqrt(irms);
  
  Serial.println(vrms);
  Serial.println(irms);
  Serial.println(pinst);
  
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

  digitalWrite(PIN_LR,LOW);
 // digitalWrite(PIN_AMARILLO,HIGH);
 while(1);
  
 
}
