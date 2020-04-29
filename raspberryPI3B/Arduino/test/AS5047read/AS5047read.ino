// inslude the SPI library:
#include <SPI.h>
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
    value = readRegister(CS[0]) & mask_results; // I'm precomputing the parity
    value2 = readRegister(CS[1]) & mask_results; // I'm precomputing the parity

    avg1=long((value+stack1[0]+stack1[1]+stack1[2]+stack1[3]+stack1[4]+stack1[5]+stack1[6])/8);
    avg2=long((value2+stack2[0]+stack2[1]+stack2[2]+stack2[3]+stack2[4]+stack2[5]+stack2[6])/8);
//    if(Serial.read() == 'a'){

    //Serial.println(String(value)+","+String(value2));
    //byte low = lowByte(value2);
    //Serial.print(highByte(value2),HEX);
//    Serial.println((avg1| 0b1000000000000000  ),HEX);//| 0b1000000000000000
    Serial.println((avg1| 0b1000000000000000  ),DEC); 
//    Serial.print((avg2| 0b1000000000000000  ), HEX);//| 0b1000000000000000 
    
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
