#include <GyverOLED.h>
//GyverOLED<SSD1306_128x32, OLED_BUFFER> oled;
//GyverOLED<SSD1306_128x32, OLED_NO_BUFFER> oled;
//GyverOLED<SSD1306_128x64, OLED_BUFFER> oled;
//GyverOLED<SSD1306_128x64, OLED_NO_BUFFER> oled;
//GyverOLED<SSD1306_128x64, OLED_BUFFER, OLED_SPI, 8, 7, 6> oled;
#include <LSM6DS3.h>
#include <Wire.h>

GyverOLED<SSH1106_128x64> oled;
LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A
float aX, aY, aZ, gX, gY, gZ;
const float accelerationThreshold = 2.5; // threshold of significant in G's
const int numSamples = 119;
int samplesRead = numSamples;

int ALIGN_COORD = 22;

void setup() {
  Serial.begin(9600);
  oled.init();              // инициализация
  myIMU.begin();
  /* x: y: z: fissi */
  oled.clear();
  oled.setScale(1);
  oled.setCursor(0, 0);
  oled.println("AX: ");
  oled.println("AY: ");
  oled.println("AZ: ");
  oled.println("GX: ");
  oled.println("GY: ");
  oled.println("GZ: ");

}


void loop()
{
  aX = myIMU.readFloatAccelX();
  aY = myIMU.readFloatAccelY();
  aZ = myIMU.readFloatAccelZ();
  gX = myIMU.readFloatGyroX();
  gY = myIMU.readFloatGyroY();
  gZ = myIMU.readFloatGyroZ();
  myOledDisplayAGValues(aX,aY,aZ, gX, gY, gZ);
}


void myOledDisplayAccelarator(float x, float y, float z)
{
  char ax[32];
  char ay[32];
  char az[32];

  sprintf(ax,"X: %f", x);
  sprintf(ay,"Y: %f", y);
  sprintf(az,"Z: %f", z);

  oled.clear();
  oled.setScale(1);
  oled.setCursor(0, 0);
  oled.println(ax);
  oled.setCursor(1, 1);
  oled.println(ay);
  oled.setCursor(2, 2);
  oled.println(az);
  oled.update();
  delay(1000);
}

void myOledDisplayAccelaratorValues(float x, float y, float z)
{
  oled.setCursor(ALIGN_COORD, 0);
  oled.print(x);
  oled.setCursor(ALIGN_COORD, 1);
  oled.print(y);
  oled.setCursor(ALIGN_COORD, 2);
  oled.print(z);
  oled.update();
}

void myOledDisplayAGValues(float ax, float ay, float az, float gx, float gy, float gz)
{
  oled.setCursor(ALIGN_COORD, 0);
  oled.print(ax);
  oled.setCursor(ALIGN_COORD, 1);
  oled.print(ay);
  oled.setCursor(ALIGN_COORD, 2);
  oled.print(az);
  oled.setCursor(ALIGN_COORD, 3);
  oled.print(gx);
  oled.setCursor(ALIGN_COORD, 4);
  oled.print(gy);
  oled.setCursor(ALIGN_COORD, 5);
  oled.print(gz);
  oled.update();
}

void printScale(byte x) {
  oled.clear();
  oled.setScale(x);
  for (byte i = 0; i < 8; i += x) {
    oled.setCursor(0, i);
    oled.print("Hello!");
  }
  oled.update();
  delay(1000);
}

void party() {
  oled.clear();
  uint32_t tmr = millis();
  oled.setScale(3);
  oled.setCursor(10, 2);
  oled.print("ПЕННАЯ");
  oled.setScale(2);
  oled.setCursor(6, 5);
  oled.print("ВЕЧЕРИНКА!");
  oled.update();
  for (;;) {
    oled.invertDisplay(true);
    delay(200);
    oled.invertDisplay(false);
    delay(200);
    if (millis() - tmr > 5000) return;
  }
}

