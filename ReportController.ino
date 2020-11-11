//2nd order compensator

//2nd order comp variabls
const double sampleTime = 0.5e-3;//sample time

const double a1 = 3.55072e8;
const double a2 = -6.97844e8;
const double a3 = 3.42859e8;
const double b1 = 11;
const double b2 = -16.0;
const double b3 = 5;
const double k = 0.0005;

const double linVoltage = 1.84946;//Linearisation Voltage
const double linPos = 7.15e-3;//linearisation Point

double u=0;//Control output
double e=0;//error
double lastE=0;//previous error
double lastU=linVoltage;
double lastLastE=0;
double lastLastU=0;

double currentTime=0;//time
double lastTime=0;//previous time
double ellapsedTime=0;

//include libs
#include <ADC.h>
#include <ADC_util.h>
//define pins
const int outP=A21;//actual DA0 (next to 39)
const int posP = A20;//actual 39
const int refP = A9;//actual 23
const int pwmP = 2;

//global variables for ADC read
ADC *adc;
float pos=linPos;
float setPoint=0;
//time variable
elapsedMicros time;

const int btn = 20;//pin used for push button


void setup() {
  Serial.begin(9600);
  delay(100);//small delay to allow serial to start
  Serial.println("Setup Start");
  
  //setup ADC
  initADC();
  //setup any buttons
  initBtns();
  //setup PWM
  initPWM();

  Serial.println("Setup End - starting in 5 seconds");
  writeVoltage(0.8*linVoltage);
  delay(5000);
}

ADC::Sync_result result;

void loop() {
  readADC();//read position
  currentTime=time;//get time
  setPoint=0;//use until pot input is added
  e=linPos-pos;//error 
  ellapsedTime=(currentTime-lastTime)/1e6;

  if(digitalRead(btn) and pos>0 and pos<0.015){//only output calculated current if button is pressed and object in opperatig range
    u = (e*k*a1 + lastE*k*a2 + lastLastE*k*a3 - lastU*b2 - lastLastU*b3)/b1;
    u=(u*-1)+ linVoltage;
    writeVoltage(u);
    Serial.print(time, DEC);//print time
    Serial.print(", ");
    Serial.print(pos*1000);//print current pos
    Serial.print(", ");
    Serial.println(u);//print control input
  }else{//otherwise default to 0
    writeVoltage(0.8*linVoltage);
  }

  lastTime=currentTime;
  lastLastE=lastE;
  lastLastU=lastU;
  lastE=e;
  lastU=u;
  
  currentTime=time;
  while(currentTime-(lastTime) < (sampleTime*1e6)){
    //doNothing
    currentTime=time;
  }
    
}//end loop

//setup function for buttons
void initBtns(){
    pinMode(btn,INPUT);
}

//setup function for ADC
void initADC(){
  //create ADC object
  adc = new ADC();
  //setup pins
  pinMode(posP, INPUT);
  pinMode(refP, INPUT);
  //setup ADC's
  ///// ADC0 //// adc used for setpoint
  adc->adc0->setAveraging(32); // set number of averages - slows speed but less susceptable to noise
  adc->adc0->setResolution(8); // set bits of resolution
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED); // change the conversion speed to highest stable speed
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED); // change the sampling speed to highest stable speed

  ////// ADC1 ///// adc used for position sensing
  adc->adc1->setAveraging(32); // set number of averages
  adc->adc1->setResolution(12); // set bits of resolution
  adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED); // change the conversion speed
  adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED); // change the sampling speed
  //start reading values - 
  adc->startSynchronizedContinuous(refP, posP);
}

//make the ADC read. values are stored in pos and setPoint
void readADC(){
  //read ADC's
  result = adc->readSynchronizedContinuous();
  //fetch results
  result.result_adc0 = (uint16_t)result.result_adc0;
  result.result_adc1 = (uint16_t)result.result_adc1;
  setPoint=result.result_adc0*3.3/adc->adc0->getMaxValue();
  pos     =result.result_adc1*3.3/adc->adc1->getMaxValue();

  pos=(51.729*pos-101.7)/1e3;
}


//setup function for pwm
void initPWM(){
  analogWriteResolution(11);//set range from 0 to 2047
  analogWriteFrequency(pwmP, 3662);//sets frequency to ideal of 29296.875hz
}

void writeVoltage(float volt){//range from 0-2047
  if(volt>0){
    volt=volt+0.605537;  
  }
  int freq=0;
  double clim=4;//limit output to 2.5A
  double R=3.3;//resistance from vcc to gnd
  if(volt/R > clim){
    volt=clim*R;
  }else if(volt<0){
    volt=0;
  }
  freq=volt*2047/12;
  analogWrite(pwmP,freq);
}

