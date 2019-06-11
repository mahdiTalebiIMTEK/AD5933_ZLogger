/*
ad5933-test
    Reads impedance values from the AD5933 over I2C and prints them serially.
*/

#include <Wire.h>
#include "AD5933.h"
#include <Math.h>
#include "Fifo.h"

#define START_FREQ  (80000)
#define FREQ_INCR   (0)
#define NUM_INCR    (0)


double Mag; 
double feedbackR = 205000; 
double phaseShift; 
double GF;

byte ImpData[4];

FifoE realData;
FifoE imgData;

double Calc_Real, Calc_Img;
double gain[NUM_INCR+1];
int phase[NUM_INCR+1];
int DataReal, DataImg;
void setup(void)
{
  // Begin I2C
  Wire.begin();
  Wire.setClock(400000);

  // Begin serial at 9600 baud for output
  Serial.begin(115200);
  Serial.println("AD5933 Test Started!");
  Serial.println("frq:");
  Serial.println(START_FREQ);

  // Perform initial configuration. Fail if any one of these fail.
  if (!(AD5933::reset() &&
        AD5933::setInternalClock(true) &&
        AD5933::setStartFrequency(START_FREQ) &&
        AD5933::setIncrementFrequency(FREQ_INCR) &&
        AD5933::setNumberIncrements(NUM_INCR) &&
        AD5933::setPGAGain(PGA_GAIN_X1)))
        {
            Serial.println("FAILED in initialization!");
            while (true) ;
        }

   Serial.print("status register after initialization: ");
   Serial.println(AD5933::readStatusRegister());     



    byte FrByte1, FrByte2, FrByte3;

    FrByte1 = AD5933::readRegister(0x82);
    FrByte2 = AD5933::readRegister(0x83);
    FrByte3 = AD5933::readRegister(0x84);
    Serial.println("bytes in frequency registers (to check):");
//    Serial.println(FrByte1);
//    Serial.println(FrByte2);
//    Serial.println(FrByte3);

    for (byte i=0; i<8; i++) {
      byte state = bitRead(FrByte1, 7-i);
      Serial.print(state);
      }
    for (byte i=0; i<8; i++) {
      byte state = bitRead(FrByte2, 7-i);
      Serial.print(state);
      }
    for (byte i=0; i<8; i++) {
      byte state = bitRead(FrByte3, 7-i);
      Serial.print(state);
    }  
    Serial.println("")   ;
    
    delay(200);

    AD5933::enableTemperature(CTRL_TEMP_MEASURE);
    double Temperature;
    Temperature = AD5933::getTemperature();
    Serial.print("Temperature:  ");
    Serial.println(Temperature)   ;
    
    AD5933::SetSettlingTime(4, 4); 
    //set the number of cycles and settling time 
    
    AD5933::GetGainFactor(feedbackR, 0, 15 , &GF, &phaseShift, true);
    Serial.print("Gain Factor: ");
    Serial.println(GF, 13);
    AD5933::SetSettlingTime(1, 1); 

}
int counter = 0;
void loop(void)
{
  AD5933::setPowerMode(CTRL_STANDBY_MODE);
  AD5933::reset();
  AD5933::setControlMode(CTRL_INIT_START_FREQ);
  AD5933::setControlMode(CTRL_START_FREQ_SWEEP);
  AD5933::setControlMode(CTRL_REPEAT_FREQ);
  
  delay(3000);
  
  Serial.print("Status register : ");
  Serial.println(AD5933::readStatusRegister());    
  Serial.print("NUM_SCYCLES_1: ");
  Serial.println(AD5933::readRegister(NUM_SCYCLES_1)) ;
  Serial.print("NUM_SCYCLES_2: ");
  Serial.println(AD5933::readRegister(NUM_SCYCLES_2)) ;
  Serial.print("Measurement started");
  int rounds = 11000 ;
  pinMode(52, OUTPUT);
  digitalWrite(52, HIGH);
  delay(10);
  digitalWrite(52, LOW);
  //put a delay because camera needs time to start 
  
  long start = millis();
  while(rounds){
   AD5933::Blockread(0x94, 4, ImpData);
   AD5933::setControlMode(CTRL_REPEAT_FREQ);
   DataReal = (int16_t)(((ImpData[0] << 8) | ImpData[1]) & 0xFFFF);
   DataImg = (int16_t)(((ImpData[2] << 8) | ImpData[3]) & 0xFFFF);
   realData.putIn(DataReal);
   imgData.putIn(DataImg);
   
//   Serial.println(ImpData[0]);
//   Serial.println(ImpData[1]);
//   Serial.println(ImpData[2]);
//   Serial.println(ImpData[3]);
//   Mag = sqrt( ( pow(DataReal,2) + pow(DataImg,2)) );
//   Serial.println(1./(Mag*GF));
   rounds--;
  }

  
  Serial.print("measurement time:");
  Serial.println(millis()-start);
  AD5933::setControlMode(CTRL_POWER_DOWN_MODE);

//  Serial.println("Real: ");
//  while(realData.count()>0){
//    Serial.println(realData.getOut());
//  }
  
  Serial.println("Real, Img ");
  while(imgData.count()>0 && realData.count()>0){
    Serial.print(realData.getOut());
    Serial.print(", ");
    Serial.println(imgData.getOut());
    //Serial.println("");
  }

  
  delay(1000);
  counter++;
  if (counter == 1)
    { 
    Serial.println("######");
    Serial.println("measuerements done.");
      while(1){}
      }
}
