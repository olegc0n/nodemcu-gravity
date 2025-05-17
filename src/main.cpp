#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_GC9A01A.h" 
#include "Wire.h"

const char* ssid = "onc-wifi-5g";
const char* password = "Passw0rd123";
const unsigned int LIGHT_SENSOR = A0;
#define TFT_DC D3
#define TFT_CS D8
#define DEG2RAD 0.0174532925  
// some extra colors
#define BLACK      0x0000
#define RED        0xF800
#define WHITE      0xFFFF
#define ORANGE     0xFBE0
#define GREY       0x84B5
#define BORDEAUX   0xA000

float sx = 0, sy = 1, mx = 1, my = 0, hx = -1, hy = 0;                              // saved H, M, S x & y multipliers
float sdeg = 0, mdeg= 0, hdeg = 0;
uint16_t osx = 120, osy = 120, omx = 120, omy = 120, ohx = 120, ohy = 120;          // saved H, M, S x & y coords
uint16_t x0=0, x1=0, yy0=0, yy1=0;  
Adafruit_GC9A01A tft(TFT_CS, TFT_DC); 
//WiFiServer server(80);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "ntp0.ntp-servers.net", 0, 60000);  // Update every 60 seconds


// MPU6050 Slave Device Address
const uint8_t MPU6050SlaveAddress = 0x68;

void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data){
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();
}

//configure MPU6050
void MPU6050_Init()
{
  Wire.begin(D2, D1);
  delay(150);
  // MPU6050 few configuration register addresses
  const uint8_t MPU6050_REGISTER_SMPLRT_DIV   =  0x19;
  const uint8_t MPU6050_REGISTER_USER_CTRL    =  0x6A;
  const uint8_t MPU6050_REGISTER_PWR_MGMT_1   =  0x6B;
  const uint8_t MPU6050_REGISTER_PWR_MGMT_2   =  0x6C;
  const uint8_t MPU6050_REGISTER_CONFIG       =  0x1A;
  const uint8_t MPU6050_REGISTER_GYRO_CONFIG  =  0x1B;
  const uint8_t MPU6050_REGISTER_ACCEL_CONFIG =  0x1C;
  const uint8_t MPU6050_REGISTER_FIFO_EN      =  0x23;
  const uint8_t MPU6050_REGISTER_INT_ENABLE   =  0x38;
  const uint8_t MPU6050_REGISTER_SIGNAL_PATH_RESET  = 0x68;
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SMPLRT_DIV, 0x07);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_1, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_2, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_CONFIG, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_GYRO_CONFIG, 0x00);//set +/-250 degree/second full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG, 0x00);// set +/- 2g full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_FIFO_EN, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_INT_ENABLE, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SIGNAL_PATH_RESET, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_USER_CTRL, 0x00);
}

// values from MPU6050
int16_t AccelX, AccelY, AccelZ, Temperature, GyroX, GyroY, GyroZ;

void Read_MPU_RawValue(uint8_t deviceAddress)
{
  const uint8_t MPU6050_REGISTER_ACCEL_XOUT_H =  0x3B;
  Wire.beginTransmission(deviceAddress);
  Wire.write(MPU6050_REGISTER_ACCEL_XOUT_H);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, (uint8_t)14);
  AccelX = (((int16_t)Wire.read()<<8) | Wire.read());
  AccelY = (((int16_t)Wire.read()<<8) | Wire.read());
  AccelZ = (((int16_t)Wire.read()<<8) | Wire.read());
  Temperature = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroX = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroY = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroZ = (((int16_t)Wire.read()<<8) | Wire.read());
}

void readMPUData(float &roll, float &pitch) 
{
  // sensitivity scale factor respective to full scale setting provided in datasheet 
  const uint16_t AccelScaleFactor = 16384;
  const uint16_t GyroScaleFactor = 131;
  Read_MPU_RawValue(MPU6050SlaveAddress);
  //divide each with their sensitivity scale factor
  const double Ax = (double)AccelX/AccelScaleFactor;
  const double Ay = (double)AccelY/AccelScaleFactor;
  const double Az = (double)AccelZ/AccelScaleFactor;
  const double T = (double)Temperature/340+36.53; //temperature formula
  const double Gx = (double)GyroX/GyroScaleFactor;
  const double Gy = (double)GyroY/GyroScaleFactor;
  const double Gz = (double)GyroZ/GyroScaleFactor;

  Serial.print("Ax: "); Serial.print(Ax);
  Serial.print(" Ay: "); Serial.print(Ay);
  Serial.print(" Az: "); Serial.print(Az);
  Serial.print(" T: "); Serial.print(T);
  Serial.print(" Gx: "); Serial.print(Gx);
  Serial.print(" Gy: "); Serial.print(Gy);
  Serial.print(" Gz: "); Serial.println(Gz);

  // formula from https://wiki.dfrobot.com/How_to_Use_a_Three-Axis_Accelerometer_for_Tilt_Sensing
  roll = atan2(Ay , Az) * 180.0 / PI;
  pitch = atan2(-Ax , sqrt(Ay * Ay + Az * Az)) * 180.0 / PI; //account for roll already applied

  Serial.print("roll = ");
  Serial.print(roll,1);
  Serial.print(", pitch = ");
  Serial.println(pitch,1);
}

void createDial ()
{
  tft.setTextColor (WHITE, GREY);  
  tft.fillCircle(120, 120, 120, BLACK);   

  // line segments for seconds parts
  for (int i = 0; i<360; i+= 2)
  {                                                   
     sx = cos((i-90)*DEG2RAD);
     sy = sin((i-90)*DEG2RAD);
     x0 = sx*118+120;
     yy0 = sy*118+120;
     x1 = sx*114+120;
     yy1 = sy*114+120;
     tft.drawLine(x0, yy0, x1, yy1, WHITE);
  }

  // line segments for seconds
  for (int i = 0; i<360; i+= 6)
  {                                                   
     sx = cos((i-90)*DEG2RAD);
     sy = sin((i-90)*DEG2RAD);
     x0 = sx*118+120;
     yy0 = sy*118+120;
     x1 = sx*110+120;
     yy1 = sy*110+120;
     tft.drawLine(x0, yy0, x1, yy1, WHITE);
  }
   
  // line segments for hours parts
  for (int i = 0; i<360; i+= 30)
  {
     sx = cos((i-90)*DEG2RAD);
     sy = sin((i-90)*DEG2RAD);
     x0 = sx*108+120;
     yy0 = sy*108+120;
     x1 = sx*85+120;
     yy1 = sy*85+120;
     tft.drawLine(x0, yy0, x1, yy1, WHITE);
   
     if(i==0)
     { 
      tft.fillCircle (x0-5, yy0, 4, WHITE);
      tft.fillCircle (x0+5, yy0, 4, WHITE);
     }
  }
}

void setup() {
  Serial.begin(115200);
  MPU6050_Init();
  tft.begin (); 
  tft.setRotation (2);
  tft.fillScreen (BLACK);
  tft.setTextSize (1);
  tft.setTextColor (WHITE, BLACK);
  tft.setCursor(0, 120);
  
  // Connect to WiFi network
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  tft.print("Connecting to ");
  tft.println(ssid);
 
  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    tft.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  tft.fillScreen (BLACK);
  tft.setCursor(0, 120);
  tft.print("WiFi connected ");
  tft.println(WiFi.localIP());
 
  // Start the server
  //server.begin();
  //Serial.println("Server started");
 
  // Print the IP address
  Serial.print("Use this URL to connect: ");
  Serial.print("http://");
  Serial.print(WiFi.localIP());
  Serial.println("/");

  tft.fillScreen (BLACK);  
  createDial (); 

  timeClient.begin();
}

void loop() 
{
  // get current light 
  static bool initial = true;

  // update the time
  timeClient.update();
  Serial.println(timeClient.getFormattedTime()); // print the light

  // get the light level
  const int raw_light = analogRead(LIGHT_SENSOR); // read the raw value from light_sensor pin (A3)
  const int light = map(raw_light, 0, 1023, 0, 100); // map the value from 0, 1023 to 0, 100
  Serial.print("Light level: "); 
  Serial.println(light); // print the light 

  // get the rool and pitch
  float roll, pitch;
  readMPUData(roll, pitch);

  // pre-compute hand degrees, x & y coords for a fast screen update
  const int ss = timeClient.getSeconds();
  sdeg = ss * 6;                                                                     // 0-59 -> 0-354
  mdeg = timeClient.getMinutes() * 6 + sdeg * 0.01666667;                                                     // 0-59 -> 0-360 - includes seconds
  hdeg = ((timeClient.getHours() + 3) % 12) * 30 + mdeg * 0.0833333;                                                     // 0-11 -> 0-360 - includes minutes and seconds
  hx = cos((hdeg - 90) * DEG2RAD);    
  hy = sin((hdeg - 90) * DEG2RAD);
  mx = cos((mdeg - 90) * DEG2RAD);    
  my = sin((mdeg - 90) * DEG2RAD);
  sx = cos((sdeg - 90) * DEG2RAD);    
  sy = sin((sdeg - 90) * DEG2RAD);

  // erase hour and minute hand positions every minute
  if (ss == 0 || initial) 
  {
    initial = 0;
    tft.drawLine(ohx, ohy, 120, 121, BLACK);                                     
    ohx = hx * 62 + 121;    
    ohy = hy * 62 + 121;
    tft.drawLine(omx, omy, 120, 121, BLACK);
    omx = mx * 84 + 120;
    omy = my * 84 + 121;
  }

  // Output values to the screen
  tft.setTextSize(2);
  tft.setTextColor(WHITE, BLACK);
  tft.setCursor(92, 60);
  tft.printf("%.1f", roll);
  tft.setCursor(92, 180);
  tft.printf("%.1f", pitch);
  tft.setCursor(50, 112);
  tft.printf("%d", light);

  // Redraw new hand positions, hour and minute hands not erased here to avoid flicker
  tft.drawLine(osx, osy, 120, 121, BLACK);
  osx = sx * 90 + 121;
  osy = sy * 90 + 121;
  tft.drawLine(osx, osy, 120, 121, RED);
  tft.drawLine(ohx, ohy, 120, 121, WHITE);
  tft.drawLine(omx, omy, 120, 121, WHITE);
  tft.drawLine(osx, osy, 120, 121, RED);
  tft.fillCircle(120, 121, 3, RED);

  /*
  // Check if a client has connected
  WiFiClient client = server.available();
  if (!client) {
    return;
  }
  // Wait until the client sends some data
  Serial.println("new client");
  while(!client.available()){
    delay(1);
  }
 
  // Read the first line of the request
  String request = client.readStringUntil('\r');
  Serial.println(request);
  client.flush();
 
  // Match the request
 
  // Return the response
  timeClient.update();
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println(""); //  do not forget this one
  client.println("<!DOCTYPE HTML>");
  client.println("<html lang=\"en\">");
  client.println("<head>");
  client.println("    <meta charset=\"UTF-8\">");
  client.println("    <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">");
  client.println("    <title>Integer Display</title>");
  client.println("</head>");
  client.println("<body>");
  client.println("    <div id=\"id_light\">Current light level: " + String(light) + "</div>");
  client.println("    <div id=\"id_time\">Current time: " + timeClient.getFormattedTime() + "</div>");
  client.println("</body>");
  client.println("</html>");
  */
}