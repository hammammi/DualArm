#include <Wire.h>
#include <Arduino.h>
#include <VL53L1X.h>
//#include <TCA9548A.h>

#define tca1 = 0x70;
#define tca2 = 0X71;


//TCA9548A I2CMux;
VL53L1X sen0, sen1, sen2, sen3, sen4, sen5, sen6, sen7;
uint8_t defaultadd = 0b0101001;
uint8_t ntca[] = {tca1,tca2};
uint16_t sensorread[]={};

void TCA9548A(uint8_t tca, uint8_t bus)
{
  Wire.beginTransmission(tca);  // TCA9548A address is 0x70
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
}
//void TCA9548A_2(uint8_t bus)
//{
//  Wire.beginTransmission(tca2);  // TCA9548A address is 0x70
//  Wire.write(1 << bus);          // send byte to select bus
//  Wire.endTransmission();
//}

void closeall(uint8_t tca){
  Wire.beginTransmission(tca);  
  Wire.write(0b00000000);          
  Wire.endTransmission();
}

void openall(uint8_t tca){
  Wire.beginTransmission(tca);  
  Wire.write(0b01111111);          
  Wire.endTransmission();
}

void addsetting(uint8_t tca){
  Serial.print("TCA number ");
  Serial.println(tca);
  TCA9548A(tca,0);
  sen0.setTimeout(500);
  if (!sen0.init())
  {
    Serial.println("Failed to detect and initialize sensor0!");
    while (1);
  }
  sen0.setAddress(defaultadd + 0);
  Serial.print("sensor0 Addr: ");
  Serial.println(sen0.getAddress());
  sen0.startContinuous(50);

  TCA9548A(tca,1);
  delay(500);
  sen1.setTimeout(500);  
  if (!sen1.init())
  {
    Serial.println("Failed to detect and initialize sensor1!");
    while (1);
  }
  sen1.setAddress(defaultadd + 1);
  Serial.print("sensor1 Addr: ");
  Serial.println(sen1.getAddress());
  sen1.startContinuous(50);
  
  TCA9548A(tca,2);
  sen2.setTimeout(500);
  if (!sen2.init())
  {
    Serial.println("Failed to detect and initialize sensor2!");
    while (1);
  }
  sen2.setAddress(defaultadd + 2);
  Serial.print("sensor2 Addr: ");
  Serial.println(sen2.getAddress());
  sen2.startContinuous(50);

  TCA9548A(tca,3);
  delay(500);
  sen3.setTimeout(500);  
  if (!sen3.init())
  {
    Serial.println("Failed to detect and initialize sensor3!");
    while (1);
  }
  sen3.setAddress(defaultadd + 3);
  Serial.print("sensor3 Addr: ");
  Serial.println(sen3.getAddress());
  sen3.startContinuous(50);
  
  TCA9548A(tca,4);
  sen4.setTimeout(500);
  if (!sen4.init())
  {
    Serial.println("Failed to detect and initialize sensor4!");
    while (1);
  }
  sen4.setAddress(defaultadd + 4);
  Serial.print("sensor4 Addr: ");
  Serial.println(sen4.getAddress());
  sen4.startContinuous(50);

  TCA9548A(tca,5);
  delay(500);
  sen5.setTimeout(500);  
  if (!sen5.init())
  {
    Serial.println("Failed to detect and initialize sensor5!");
    while (1);
  }
  sen5.setAddress(defaultadd + 5);
  Serial.print("sensor5 Addr: ");
  Serial.println(sen5.getAddress());
  sen5.startContinuous(50);
  
  TCA9548A(tca,6);
  sen6.setTimeout(500);
  if (!sen6.init())
  {
    Serial.println("Failed to detect and initialize sensor6!");
    while (1);
  }
  sen6.setAddress(defaultadd + 6);
  Serial.print("sensor6 Addr: ");
  Serial.println(sen6.getAddress());
  sen6.startContinuous(50);

  closeall(tca);
}


void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  
  closeall(ntca[1]);
  closeall(ntca[2]);
  
  addsetting(ntca[1]);
  addsetting(ntca[2]);
  
  Serial.println("start");
}

  
void loop() {
  openall(ntca[1]);
  Serial.println(ntca[1]);
  sensorread[1] = sen0.read();
  if (sen0.timeoutOccurred()) { Serial.println("sen0 TIMEOUT"); }
  sensorread[2] = sen1.read();
  if (sen1.timeoutOccurred()) { Serial.println("sen1 TIMEOUT"); }
  sensorread[3] = sen2.read();
  if (sen2.timeoutOccurred()) { Serial.println("sen2 TIMEOUT"); }
  sensorread[4] = sen3.read();
  if (sen3.timeoutOccurred()) { Serial.println("sen3 TIMEOUT"); }
  sensorread[5] = sen4.read();
  if (sen4.timeoutOccurred()) { Serial.println("sen4 TIMEOUT"); }
  sensorread[6] = sen5.read();
  if (sen5.timeoutOccurred()) { Serial.println("sen5 TIMEOUT"); }
  sensorread[7] = sen6.read();
  if (sen6.timeoutOccurred()) { Serial.println("sen6 TIMEOUT"); }
  Serial.println(sensorread);
  closeall(ntca[1]);
  openall(ntca[2]);
  Serial.println(ntca[2]);
  sensorread[1] = sen0.read();
  if (sen0.timeoutOccurred()) { Serial.println("sen0 TIMEOUT"); }
  sensorread[2] = sen1.read();
  if (sen1.timeoutOccurred()) { Serial.println("sen1 TIMEOUT"); }
  sensorread[3] = sen2.read();
  if (sen2.timeoutOccurred()) { Serial.println("sen2 TIMEOUT"); }
  sensorread[4] = sen3.read();
  if (sen3.timeoutOccurred()) { Serial.println("sen3 TIMEOUT"); }
  sensorread[5] = sen4.read();
  if (sen4.timeoutOccurred()) { Serial.println("sen4 TIMEOUT"); }
  sensorread[6] = sen5.read();
  if (sen5.timeoutOccurred()) { Serial.println("sen5 TIMEOUT"); }
  sensorread[7] = sen6.read();
  if (sen6.timeoutOccurred()) { Serial.println("sen6 TIMEOUT"); }
  Serial.println(sensorread);
  closeall(ntca[2]);
}
