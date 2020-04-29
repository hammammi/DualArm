// include the SPI library:
#include <SPI.h>
#include <ros.h>
#include <vehicle_control/AS5047Msg.h>

#define interval 10

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
vehicle_control::AS5047Msg data;
ros::Publisher chatter("magEnc", &data);

unsigned long old_time=0;

void setup() {
  nh.initNode();
  nh.advertise(chatter);
  Serial.begin(115200);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);  
  SPI.setDataMode(SPI_MODE1);  
  SPI.setClockDivider(10);
  for (int i =0; i<sizeof(CS)/sizeof(CS[0]); i++){
    pinMode (CS[i], OUTPUT);  
    digitalWrite(CS[i], HIGH);
  }
  Serial.print("Start");
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

void loop() {

    value1 = readRegister(CS[0]) & mask_results; 
    value2 = readRegister(CS[1]) & mask_results; 
    value3 = readRegister(CS[2]) & mask_results; 
    value4 = readRegister(CS[3]) & mask_results; 

    avg1=long((value1+stack1[0]+stack1[1]+stack1[2]+stack1[3]+stack1[4]+stack1[5]+stack1[6])/8);
    avg2=long((value2+stack2[0]+stack2[1]+stack2[2]+stack2[3]+stack2[4]+stack2[5]+stack2[6])/8);
    avg3=long((value3+stack3[0]+stack3[1]+stack3[2]+stack3[3]+stack3[4]+stack3[5]+stack3[6])/8);
    avg4=long((value4+stack4[0]+stack4[1]+stack4[2]+stack4[3]+stack4[4]+stack4[5]+stack4[6])/8);

//    Serial.print((avg1| 0b1000000000000000  ),DEC);
//    Serial.print("\t"); 
//    Serial.println((avg2| 0b1000000000000000  ), DEC);//| 0b1000000000000000 

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
    data.wheel1 = avg1|0b1000000000000000;
    data.wheel2 = avg2|0b1000000000000000;
    data.wheel3 = avg3|0b1000000000000000;
    data.wheel4 = avg4|0b1000000000000000;
    chatter.publish(&data);
    }
    

}
