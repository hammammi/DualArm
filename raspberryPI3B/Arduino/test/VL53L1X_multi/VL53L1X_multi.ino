#include <Wire.h>
#include <Arduino.h>
#include <VL53L1X.h>
//#include <TCA9548A.h>

#define tca1 0x70
#define tca2 0X71


//TCA9548A I2CMux;
VL53L1X sen;
uint8_t defaultadd = 0b0101001;
uint8_t ntca[] = {tca1,tca2};
uint16_t sensorread[]={0,0,0,0,0,0,0};

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


void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  
  closeall(ntca[0]);
  closeall(ntca[1]);
  for (int k=0;k<2;k++){
    Serial.print("TCA number ");
    Serial.print(ntca[k]);
    for (int i=0; i<7; i++){
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

}
