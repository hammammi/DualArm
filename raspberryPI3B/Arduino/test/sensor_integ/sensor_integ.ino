#include <Wire.h>
#include <Arduino.h>
#include <VL53L1X.h>
#include <SPI.h>

#define tca1 0x70
#define tca2 0X71


//TCA9548A I2CMux;
VL53L1X sen;
uint8_t defaultadd = 0b0101001;
uint8_t ntca[] = {tca1,tca2};
uint16_t sensorread[]={0,0,0,0,0,0,0};

// set pin 10 as the slave select for the encoder:
const int CS[] = {2,3};
word data_register = 0x3FFF;
word uncor_data_register = 0x3FFE;
word err_register = 0x0001;
word diag_register = 0x3FFC;
long value, value2;   
long avg1, avg2;
word mask_results = 0b0011111111111111; // this grabs the returned data without the read/write or parity bits.

long stack1[7] ={0,0,0,0,0,0,0};
long stack2[7] ={0,0,0,0,0,0,0};


void TCA9548A(uint8_t tca, uint8_t bus)
{
  Wire.beginTransmission(tca);  // TCA9548A address is 0x70
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
}

void closeall(uint8_t tca){
  Wire.beginTransmission(tca);  
  Wire.write(0b00000000);          
  Wire.endTransmission();
}

long readRegister(int cs) {
  byte inByte = 0x00;   // incoming byte from the SPI
  byte inByte2 = 0x00;
  long result = 0;   // result to return
  digitalWrite(cs, LOW);
  delayMicroseconds(10);
  inByte = SPI.transfer(0x00);
  inByte2 = SPI.transfer(0x00);
  // combine the byte with the previous one:
  result = inByte << 8 | inByte2;
  // take the chip select high to de-select:
  digitalWrite(cs, HIGH);
  delayMicroseconds(10);
  // return the result:
  return result;
}


void setup() {
  Serial.begin(115200);

  SPI.begin();
  SPI.setBitOrder(MSBFIRST);  
  SPI.setDataMode(SPI_MODE1);  
  SPI.setClockDivider(10);
  for (int i =0; i<sizeof(CS)/sizeof(CS[0]); i++){
    pinMode (CS[i], OUTPUT);  
    digitalWrite(CS[i], HIGH);
  }
  
  Wire.begin();
  Wire.setClock(400000);
  
  closeall(ntca[0]);
  closeall(ntca[1]);
  for (int k=0;k<2;k++){
    Serial.print("TCA number ");
    Serial.print(ntca[k]);
    for (int i=0; i<2; i++){
      TCA9548A(ntca[k],i);
      sen.setTimeout(500);
      if (!sen.init())
      {
        Serial.print("Failed to detect and initialize sensor ");
        Serial.println(i);
        while (1);
      }
      sen.startContinuous(50);
    }
    Serial.println(" initialized");
  }

  closeall(ntca[0]);
  closeall(ntca[1]); 
  
  Serial.println("start");
}

  
void loop() {
//  openall(ntca[0]);
  for (int k=0;k<2;k++){
    
    for (int i=0; i<2; i++){
      TCA9548A(ntca[k],i);
      sensorread[i] = sen.read();
      if (sen.timeoutOccurred()) { 
        Serial.print("TCA number ");
        Serial.println(ntca[k]);
        Serial.println("sen TIMEOUT"); }
    }
  Serial.print(ntca[k]);
  Serial.print("\t");
  Serial.print(sensorread[0]);
  Serial.print("\t");
  Serial.println(sensorread[1]);
  delay(500);
  }
  value = readRegister(CS[0]) & mask_results; // I'm precomputing the parity
    value2 = readRegister(CS[1]) & mask_results; // I'm precomputing the parity

    avg1=long((value+stack1[0]+stack1[1]+stack1[2]+stack1[3]+stack1[4]+stack1[5]+stack1[6])/8);
    avg2=long((value2+stack2[0]+stack2[1]+stack2[2]+stack2[3]+stack2[4]+stack2[5]+stack2[6])/8);
//    if(Serial.read() == 'a'){

    //Serial.println(String(value)+","+String(value2));
    //byte low = lowByte(value2);
    //Serial.print(highByte(value2),HEX);
//    Serial.println((avg1| 0b1000000000000000  ),HEX);//| 0b1000000000000000
    Serial.print((avg1| 0b1000000000000000  ),DEC);
    Serial.print("\t"); 
    Serial.println((avg2| 0b1000000000000000  ), DEC);//| 0b1000000000000000 
    
    //Serial.println((avg1));
    //Serial.println((avg2));
//    }
    
    //Serial.println("1");

    stack1[6] = stack1[5];
    stack1[5] = stack1[4];
    stack1[4] = stack1[3];
    stack1[3] = stack1[2];
    stack1[2] = stack1[1];
    stack1[1] = stack1[0];
    stack1[0] = value;
    
    stack2[6] = stack2[5];
    stack2[5] = stack2[4];
    stack2[4] = stack2[3];
    stack2[3] = stack2[2];
    stack2[2] = stack2[1];
    stack2[1] = stack2[0];
    stack2[0] = value2;
    
    
    delayMicroseconds(10);

}
