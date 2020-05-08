#define USE_USBCON  // for using ros with due

#include <ros.h>
#include <Wire.h>
#include <Arduino.h>
#include <VL53L1X.h>
#include <SPI.h>

#include <vehicle_control/as5047Msg.h>
#include <vehicle_control/vl53l1xMsg.h>

#define interval 10

#define tca1 0x70
#define tca2 0X71


//TCA9548A I2CMux;
VL53L1X sen;
uint8_t defaultadd = 0b0101001;
uint8_t ntca[] = {tca1,tca2};
uint16_t sensorread[]={0,0,0,0,0,0,0};

// set pin 10 as the slave select for the encoder:
const int CS[] = {2,3,4,5};
word data_register = 0x3FFF;
word uncor_data_register = 0x3FFE;
word err_register = 0x0001;
word diag_register = 0x3FFC;
long value1, value2, value3, value4;      
long avg1, avg2, avg3, avg4;
word mask_results = 0b0011111111111111; // this grabs the returned data without the read/write or parity bits.

long stack1[7] ={0,0,0,0,0,0,0};
long stack2[7] ={0,0,0,0,0,0,0};
long stack3[7] ={0,0,0,0,0,0,0};
long stack4[7] ={0,0,0,0,0,0,0};

ros::NodeHandle nh;
vehicle_control::as5047Msg enc_data;
vehicle_control::vl53l1xMsg laser_data;

ros::Publisher enc_chatter("magEnc", &enc_data);
ros::Publisher laser_chatter("laser", &laser_data);

unsigned long old_time=0;


void TCA9548A(uint8_t tca, uint8_t bus)
{
  Wire.beginTransmission(tca);  // TCA9548A address is 0x70
  Wire.write(0b00000000);
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
//  Serial.begin(115200);
//  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(enc_chatter);
  nh.advertise(laser_chatter);

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
//    Serial.print("TCA number ");
//    Serial.print(ntca[k]);
    for (int i=0; i<7; i++){
      TCA9548A(ntca[k],i);
      sen.setTimeout(500);
      if (!sen.init())
      {
//        Serial.print("Failed to detect and initialize sensor ");
//        Serial.println(i);
        while (1);
      }
      sen.startContinuous(50);
    }
//    Serial.println(" initialized");
  }

  closeall(ntca[0]);
  closeall(ntca[1]); 
  
//  Serial.println("start");
}

  
void loop() {
//  openall(ntca[0]);
  for (int k=0;k<2;k++){
    
    for (int i=0; i<7; i++){
      TCA9548A(ntca[k],i);
//      sensorread[i] = sen.read();
      laser_data.laser[k*7+i] = sen.read();
      if (sen.timeoutOccurred()) { 
//        ROSINFO("TCA number ");
//        Serial.println(ntca[k]);
        ROSINFO("sen TIMEOUT"); }
    }
    closeall(ntca[k]);
    }
//  Serial.print(ntca[k]);
//  Serial.print("\t");
//  Serial.print(sensorread[0]);
//  Serial.print("\t");
//  Serial.println(sensorread[1]);
  delay(500);
  }
  
    value1 = readRegister(CS[0]) & mask_results; 
    value2 = readRegister(CS[1]) & mask_results; 
    value3 = readRegister(CS[2]) & mask_results; 
    value4 = readRegister(CS[3]) & mask_results; 

    avg1=long((value1+stack1[0]+stack1[1]+stack1[2]+stack1[3]+stack1[4]+stack1[5]+stack1[6])/8);
    avg2=long((value2+stack2[0]+stack2[1]+stack2[2]+stack2[3]+stack2[4]+stack2[5]+stack2[6])/8);
    avg3=long((value3+stack3[0]+stack3[1]+stack3[2]+stack3[3]+stack3[4]+stack3[5]+stack3[6])/8);
    avg4=long((value4+stack4[0]+stack4[1]+stack4[2]+stack4[3]+stack4[4]+stack4[5]+stack4[6])/8);

    enc_data.mag_enc[0] = avg1|0b1000000000000000;
    enc_data.mag_enc[1] = avg2|0b1000000000000000;
    enc_data.mag_enc[2] = avg3|0b1000000000000000;
    enc_data.mag_enc[3] = avg4|0b1000000000000000;

    stack1[6] = stack1[5];
    stack1[5] = stack1[4];
    stack1[4] = stack1[3];
    stack1[3] = stack1[2];
    stack1[2] = stack1[1];
    stack1[1] = stack1[0];
    stack1[0] = value1;
    
    stack2[6] = stack2[5];
    stack2[5] = stack2[4];
    stack2[4] = stack2[3];
    stack2[3] = stack2[2];
    stack2[2] = stack2[1];
    stack2[1] = stack2[0];
    stack2[0] = value2;

    stack3[6] = stack3[5];
    stack3[5] = stack3[4];
    stack3[4] = stack3[3];
    stack3[3] = stack3[2];
    stack3[2] = stack3[1];
    stack3[1] = stack3[0];
    stack3[0] = value3;
    
    stack4[6] = stack4[5];
    stack4[5] = stack4[4];
    stack4[4] = stack4[3];
    stack4[3] = stack4[2];
    stack4[2] = stack4[1];
    stack4[1] = stack4[0];
    stack4[0] = value4;
    
    
    delayMicroseconds(10);
    
    unsigned long currentMillis = millis();
    if (currentMillis >= old_time + interval){
    old_time = currentMillis + interval;

    enc_chatter.publish(&enc_data);
    laser_chatter.publish(&laser_data);
    
     
    }
    
  nh.spinOnce(); 
    

}
