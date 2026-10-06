#include <Arduino.h>
#include "MPU6050.h"
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_GC9A01A.h" 
#include "Wire.h"

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
#define GREEN      0x07E0
#define BLUE       0x001F
const uint16_t AccelScaleFactor = 16384;

Adafruit_GC9A01A tft(TFT_CS, TFT_DC); 
MPU6050 mpu;

// values from MPU6050
int16_t AccelX = 0;
int16_t AccelY = 0;
int16_t AccelZ = 0;
int16_t GyroX = 0;
int16_t GyroY = 0;
int16_t GyroZ = 0;

void readMPUData(float &roll, float &pitch) 
{
  // sensitivity scale factor respective to full scale setting provided in datasheet 
  const uint16_t GyroScaleFactor = 131;
  mpu.getMotion6(&AccelX, &AccelY, &AccelZ, &GyroX, &GyroY, &GyroZ);
  //divide each with their sensitivity scale factor
  const double Ax = (double)AccelX/AccelScaleFactor;
  const double Ay = (double)AccelY/AccelScaleFactor;
  const double Az = (double)AccelZ/AccelScaleFactor;
  const double Gx = (double)GyroX/GyroScaleFactor;
  const double Gy = (double)GyroY/GyroScaleFactor;
  const double Gz = (double)GyroZ/GyroScaleFactor;

  // formula from https://wiki.dfrobot.com/How_to_Use_a_Three-Axis_Accelerometer_for_Tilt_Sensing
  roll = atan2(Ay , Az) * 180.0 / PI;
  pitch = atan2(-Ax , sqrt(Ay * Ay + Az * Az)) * 180.0 / PI; //account for roll already applied
  return;
  Serial.print("Ax: "); Serial.print(Ax);
  Serial.print(" Ay: "); Serial.print(Ay);
  Serial.print(" Az: "); Serial.print(Az);
  Serial.print(" Gx: "); Serial.print(Gx);
  Serial.print(" Gy: "); Serial.print(Gy);
  Serial.print(" Gz: "); Serial.print(Gz);
  Serial.print(" Roll: "); Serial.print(roll, 1);
  Serial.print(" Pitch: "); Serial.println(pitch,1);
}

void createDial ()
{
  tft.setTextColor (WHITE, GREY);  
  tft.fillCircle(120, 120, 120, BLACK);   

  // line small segment ruller
  for (int i = 0; i < 360; i+= 2)
  {                                                   
     const float sx = cos((i-90)*DEG2RAD);
     const float sy = sin((i-90)*DEG2RAD);
     const int16_t x0 = sx*118+120;
     const int16_t yy0 = sy*118+120;
     const int16_t x1 = sx*114+120;
     const int16_t yy1 = sy*114+120;
     tft.drawLine(x0, yy0, x1, yy1, WHITE);
  }
   
  // draw 45 degree segments
  for (int i = 0; i<360; i+= 45)
  {
     const float sx = cos((i-90)*DEG2RAD);
     const float sy = sin((i-90)*DEG2RAD);
     const int16_t x0 = sx*108+120;
     const int16_t yy0 = sy*108+120;
     const int16_t x1 = sx*85+120;
     const int16_t yy1 = sy*85+120;
     tft.drawLine(x0, yy0, x1, yy1, WHITE);
  }
}

// Indicator class definition
class Indicator 
{
public:
    Indicator(int x, int y, bool showInt = true) : m_x(x), m_y(y), m_value(0.0f),
      m_oldShownValue(0.), m_visible(true), m_showInt(showInt)
    {
    }
    void setVisible(bool visible) 
    {
      if(m_visible != visible) 
        {
          m_visible = visible;
          if (m_visible) 
            update(); // update the indicator if it is visible
        }
    }
    void setValue(float value) 
    {
      // update the value
      m_value = value; // store the current value
      update();
    }
    void update()
    {
      if (m_showInt)
      {
        const int value2show = abs((int)m_value);
        if ((int)m_oldShownValue != value2show)
        {
          tft.fillRect(m_x, m_y, 50, 16, BLACK);
          tft.setCursor(m_x, m_y);
          if(m_value < 0)
            tft.setTextColor (BLUE, BLACK);
          else
            tft.setTextColor (RED, BLACK);
          tft.print(value2show);
          m_oldShownValue = value2show; // update old shown value
        }
      }
      else
      {
        const float value2show = m_value;
        if (abs(m_oldShownValue - value2show) > 0.05)
        {
          tft.fillRect(m_x, m_y, 85, 16, BLACK);
          tft.setCursor(m_x, m_y);
          if(m_value < 0)
            tft.setTextColor (BLUE, BLACK);
          else
            tft.setTextColor (RED, BLACK);
          tft.print(value2show, 2);
          m_oldShownValue = value2show; // update old shown value
        }
      }
    }
private:
    int m_x = 0;
    int  m_y = 0;
    float m_oldShownValue = 0.;
    float m_value = 0.;
    bool m_visible = true;
    bool m_showInt = true; // show integer values or float
};

Indicator rollIndicator(40, 85, false);
Indicator pitchIndicator(160, 85, false);
//Indicator accelIndicator(90, 160, false); // show float values

void drawCurrentState()
{
  // get the roll and pitch
  float roll, pitch;
  readMPUData(roll, pitch);
  // constants to draw 
  const int centerX = 120;
  const int centerY = 120;
  const int maxOffset = 70;
  const int radius = 5;
  // variables to store old data
  static int oldX = -120; // set to a value that is not possible
  static int oldY = -120; // set to a value that is not possible

  tft.setTextSize(2);

  // draw roll
  rollIndicator.setValue(roll); // update the indicator with roll value
  
  // draw pitch
  pitchIndicator.setValue(pitch); // update the indicator with pitch value
  
  // Calculate summary acceleration (magnitude)
  //double Ax = (double)AccelX / AccelScaleFactor;
  //double Ay = (double)AccelY / AccelScaleFactor;
  //double Az = (double)AccelZ / AccelScaleFactor;
  //double summaryAccel = sqrt(Ax * Ax + Ay * Ay + Az * Az);
  //accelIndicator.setValue(summaryAccel); // update the indicator with summary acceleration value

  // Draw red circle based on roll and pitch
  // Map roll and pitch to display coordinates
  // Assume roll and pitch range from -45 to +45 degrees
  const float maxAngle = 90.0;
  // Clamp roll and pitch
  float clampedRoll = constrain(roll, -maxAngle, maxAngle);
  float clampedPitch = constrain(pitch, -maxAngle, maxAngle);
  // Map to display (move circle from center to edge)
  const int x = centerX - (int)(clampedRoll / maxAngle * (maxOffset - radius));
  const int y = centerY + (int)(clampedPitch / maxAngle * (maxOffset - radius));
  // draw only if position changed
  if(x != oldX || y != oldY)
  {
    // Clear the previous circle
    tft.fillCircle(oldX, oldY, radius, BLACK);
    // draw central mark
    tft.fillCircle(centerX, centerY, radius + 2, WHITE);
    // Draw the red circle
    tft.fillCircle(x, y, radius, RED);
    oldX = x; // update oldX
    oldY = y; // update oldY 
  }
}

void calibration() 
{
  const int BUFFER_SIZE = 100; // number of measurements for averaging
  long offsets[6];
  long offsetsOld[6];
  int16_t mpuGet[6];
  // use standard accuracy
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  // reset offsets
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  delay(10);
  tft.setTextColor (WHITE, BLACK);
  tft.setCursor(20, 100);
  tft.setTextSize(1);
  tft.println("Calibration start. It will take about 5 seconds");
  for (byte n = 0; n < 10; n++) 
  {     // 10 calibration iterations
    for (byte j = 0; j < 6; j++) 
    {    // reset calibration array
      offsets[j] = 0;
    }
    for (byte i = 0; i < 100 + BUFFER_SIZE; i++) 
    { // perform BUFFER_SIZE measurements for averaging
      mpu.getMotion6(&mpuGet[0], &mpuGet[1], &mpuGet[2], &mpuGet[3], &mpuGet[4], &mpuGet[5]);
      if (i >= 99) 
      {                         // skip the first 99 measurements
        for (byte j = 0; j < 6; j++) 
        {
          offsets[j] += (long)mpuGet[j];   // write to calibration array
        }
      }
    }
    for (byte i = 0; i < 6; i++) 
    {
      offsets[i] = offsetsOld[i] - ((long)offsets[i] / BUFFER_SIZE); // take into account previous calibration
      if (i == 2) offsets[i] += 16384;                               // if Z axis, calibrate to 16384
      offsetsOld[i] = offsets[i];
    }
    // set new offsets
    mpu.setXAccelOffset(offsets[0] / 8);
    mpu.setYAccelOffset(offsets[1] / 8);
    mpu.setZAccelOffset(offsets[2] / 8);
    mpu.setXGyroOffset(offsets[3] / 4);
    mpu.setYGyroOffset(offsets[4] / 4);
    mpu.setZGyroOffset(offsets[5] / 4);
    delay(2);
  }
  // output to port
  tft.println("Calibration end. Your offsets:");
  tft.println("accX accY accZ gyrX gyrY gyrZ");
  tft.print(mpu.getXAccelOffset()); tft.print(", ");
  tft.print(mpu.getYAccelOffset()); tft.print(", ");
  tft.print(mpu.getZAccelOffset()); tft.print(", ");
  tft.print(mpu.getXGyroOffset()); tft.print(", ");
  tft.print(mpu.getYGyroOffset()); tft.print(", ");
  tft.print(mpu.getZGyroOffset()); tft.println(" ");

  Serial.println("Calibration end. Your offsets:");
  Serial.println("accX accY accZ gyrX gyrY gyrZ");
  Serial.println(mpu.getXAccelOffset()); tft.print(", ");
  Serial.println(mpu.getYAccelOffset()); tft.print(", ");
  Serial.println(mpu.getZAccelOffset()); tft.print(", ");
  Serial.println(mpu.getXGyroOffset()); tft.print(", ");
  Serial.println(mpu.getYGyroOffset()); tft.print(", ");
  Serial.println(mpu.getZGyroOffset()); tft.println(" ");
}
void setup() 
{
  Wire.begin(D2, D1);
  Serial.begin(115200);
  mpu.initialize();
  delay(1000);
  tft.begin (); 
  tft.setRotation (2);
  tft.fillScreen (BLACK);  
  delay(1000);
  if(1)
  {
    // I got this values from calibration
    mpu.setXAccelOffset(-7254);
    mpu.setYAccelOffset(4320);
    mpu.setZAccelOffset(9998);
    mpu.setXGyroOffset(-37);
    mpu.setYGyroOffset(-41);
    mpu.setZGyroOffset(-36);
  }
  else
  {
    // do calibration
    calibration();
  }
  delay(1000);
  createDial ();
}

void loop() 
{
  drawCurrentState();
}