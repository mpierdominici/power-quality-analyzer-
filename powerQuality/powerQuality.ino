#include "arduinoFFT.h"
#define PIN_TEST 8
#define CERO_MEAS 400
//el sampleo esta razonablemente bien, lo probe con matlab y una senal de 2.5 k estubo bien.
#define SAMPLES_AMAUNT  512
#define RMS_PERIOD 102
#define BIN2_SEND 10 //lacantidad de armonicos a enviar

volatile double voltageBin[1+(BIN2_SEND/2)];


#define PIN_LR 24
#define PIN_LA 26
#define PIN_LV 28


void procesamientoMediciones(void);


arduinoFFT FFT = arduinoFFT();
//fyente https://github.com/kosme/arduinoFFT
const byte adcPin = 0;  // A0

volatile double voltage [SAMPLES_AMAUNT];
volatile double current [SAMPLES_AMAUNT];

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
  Serial3.begin(9600);
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
  double irms=0;//variable para almcanera la corriente rms
  double vrms=0;//variable para almacenara la tension rms
  double pinst=0;//potencia activa
  double pact=0;//potencia aparente
  double pf=0;//factor de potencia
void loop () {
  String commandRecived;
  
  if(Serial3.available()){
     commandRecived=Serial3.readString();
    if(commandRecived.indexOf("med")>=0){
      
      digitalWrite(PIN_LR,HIGH);
      digitalWrite(PIN_LV,LOW);
      resultNumber=0;
      ADCSRA =  bit (ADEN) | bit (ADIE) | bit (ADIF)| bit (ADPS2) | bit (ADATE); // turn ADC ON
      procesamientoMediciones();
      digitalWrite(PIN_LR,LOW);
      digitalWrite(PIN_LV,HIGH);
    }
    if(commandRecived.indexOf("getF")>=0){
      digitalWrite(PIN_LA,HIGH);
        Serial.print("Tension rms: ");
        Serial.print(vrms);
        Serial.println("V");
      
        Serial.print("Corriente rms: ");
        Serial.print(irms);
        Serial.println("A");
      
        Serial.print("Potencia aparente: ");
        Serial.print(pact);
        Serial.println("VA");
      
        Serial.print("Potencia activa: ");
        Serial.print(pinst);
        Serial.println("W");
      
        Serial.print("Factor de potencia: ");
        Serial.print(pf);
        Serial.println("");
      
        Serial.println("");
        Serial.println("");
        Serial.println("");
        Serial.println("");
        Serial.println("");
        Serial.println("");
        digitalWrite(PIN_LA,LOW);
      
    } else if(commandRecived.indexOf("get")>=0){
      digitalWrite(PIN_LA,HIGH);
      Serial3.println(vrms);
      Serial3.println(irms);
      Serial3.println(pact);
      Serial3.println(pinst);
      Serial3.println(pf);  
      digitalWrite(PIN_LA,LOW);
    } else if(commandRecived.indexOf("CA")>=0){
      double tempcthd=0;
      for(int i=2;i<=BIN2_SEND;i++){
        tempcthd+=max(current[5*i],current[(5*i)-1]);
        //Serial3.println(max(current[5*i],current[(5*i)-1]));
      }
      Serial3.println(tempcthd/max(current[5],current[(5)-1]));
    }else if(commandRecived.indexOf("VA")>=0){
      double tempvthd=0;
      for(int i=2;i<=BIN2_SEND;i++){
        tempvthd+=max(voltage[5*i],voltage[(5*i)-1]);
        //Serial3.println(max(voltage[5*i],voltage[(5*i)-1]));
      }
      Serial3.println(tempvthd/max(voltage[5],voltage[(5)-1]));
    }else if(commandRecived.indexOf("all")>=0){
      for (int i = 0; i < SAMPLES_AMAUNT; i++)
  {
    Serial3.print(voltage[i]);
    Serial3.print(" ");
    Serial3.println(current[i]);
    //Serial.println (i+0.1);
  }
    }else if(commandRecived.indexOf("PA")>=0){
      for(int i=0;i<=(BIN2_SEND/2);i++){
       //Serial.println(powerBIN[i]);
      }      
    }else if(commandRecived.indexOf("i")>=0){

      Serial3.println(irms);
      
    }else if(commandRecived.indexOf("v")>=0){
      Serial3.println(vrms);
      
    }else if(commandRecived.indexOf("p")>=0){
      Serial3.println(pinst);
      
    }else if(commandRecived.indexOf("f")>=0){
      Serial3.println(pf);
    }
    
  }


 
  
 
}

void procesamientoMediciones(void){

  double img [SAMPLES_AMAUNT];
  
  irms=0;
  
  for (int i = 0; i < SAMPLES_AMAUNT; i++)
  { 
    img[i]=0;
  }
  while (resultNumber < 2*SAMPLES_AMAUNT) { }//sampleo de dos canales
  for (int i = 0; i < SAMPLES_AMAUNT; i++)//adecuo las mediciones del adc a los valores reales
  {
    voltage[i]=(voltage[i]-CERO_MEAS)*(5.0/1023.0)*(282.84);
    //current[i]=(current[i]-CERO_MEAS)*(5.0/1023.0)*(50/3.0);
    current[i]=(current[i]-CERO_MEAS)*(5.0/1023.0)*50.0;
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

  
  pact=vrms*irms;

  pf=pinst/pact;
  
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


  
  


  
}
