#define UniqueIDbuffer 8

uint64_t chipid;  
uint8_t id[UniqueIDbuffer];

void setup() {
  Serial.begin(115200);
}

void loop() {
  
//  chipid=ESP.getEfuseMac();//The chip ID is essentially its MAC address(length: 6 bytes).
//  Serial.printf("ESP32 Chip ID = %04X",(uint16_t)(chipid>>32));//print High 2 bytes
//  Serial.printf("%08X\n",(uint32_t)chipid);//print Low 4bytes.

  chipid = ESP.getEfuseMac();
  id[0] = 0;
  id[1] = 0;
  id[2] = chipid;
  id[3] = chipid >> 8;
  id[4] = chipid >> 16;
  id[5] = chipid >> 24;
  id[6] = chipid >> 32;
  id[7] = chipid >> 40;

  for (int i = 0; i < UniqueIDbuffer; i++) {
    Serial.print(id[i]);
    Serial.print(":");
  }
  Serial.println();
  
  delay(3000);

}
