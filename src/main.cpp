#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_GC9A01A.h" 

const char* ssid = "onc-wifi-5g";
const char* password = "Passw0rd123";
const unsigned int LIGHT_SENSOR = A0;
#define TFT_DC D2
#define TFT_CS D8
#define DEG2RAD 0.0174532925  
// some extra colors
#define BLACK      0x0000
#define BLUE       0x001F
#define RED        0xF800
#define GREEN      0x07E0
#define CYAN       0x07FF
#define MAGENTA    0xF81F
#define YELLOW     0xFFE0
#define WHITE      0xFFFF
#define ORANGE     0xFBE0
#define GREY       0x84B5
#define BORDEAUX   0xA000

float sx = 0, sy = 1, mx = 1, my = 0, hx = -1, hy = 0;                              // saved H, M, S x & y multipliers
float sdeg = 0, mdeg= 0, hdeg = 0;
uint16_t osx = 120, osy = 120, omx = 120, omy = 120, ohx = 120, ohy = 120;          // saved H, M, S x & y coords
uint16_t x0=0, x1=0, yy0=0, yy1=0;  
Adafruit_GC9A01A tft(TFT_CS, TFT_DC); 
WiFiServer server(80);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "ntp0.ntp-servers.net", 0, 60000);  // Update every 60 seconds


void createDial (){

  tft.setTextColor (WHITE, GREY);  
  tft.fillCircle(120, 120, 118, BORDEAUX);                                           // creates outer ring
  tft.fillCircle(120, 120, 110, BLACK);   

  for (int i = 0; i<360; i+= 30)                                                     // draw 12 line segments at the outer ring 
     {                                                   
     sx = cos((i-90)*DEG2RAD);
     sy = sin((i-90)*DEG2RAD);
     x0 = sx*114+120;
     yy0 = sy*114+120;
     x1 = sx*100+120;
     yy1 = sy*100+120;
     tft.drawLine(x0, yy0, x1, yy1, GREEN);
     }
                                                            
  for (int i = 0; i<360; i+= 6)                                                      // draw 60 dots - minute markers
     {
     sx = cos((i-90)*DEG2RAD);
     sy = sin((i-90)*DEG2RAD);
     x0 = sx*102+120;
     yy0 = sy*102+120;    
     tft.drawPixel(x0, yy0, WHITE);
   
     if(i==0  || i==180) tft.fillCircle (x0, yy0, 2, WHITE);                         // draw main quadrant dots
     if(i==90 || i==270) tft.fillCircle (x0, yy0, 2, WHITE);
    }
 
  tft.fillCircle(120, 121, 3, WHITE);                                               // pivot 
}


void setup() {
  Serial.begin(9600);
  tft.begin (); 
  tft.setRotation (2);
  tft.fillScreen (BLACK);  
  delay (200);
  tft.fillScreen (RED);
  delay (200);
  tft.fillScreen (GREEN);
  delay (200);
  tft.fillScreen (BLUE);
  delay (200);
  tft.fillScreen (BLACK);  
  delay (200);
  tft.fillScreen (GREY); 

  delay(200);
  createDial (); 


  
  // Connect to WiFi network
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
 
  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
 
  // Start the server
  server.begin();
  Serial.println("Server started");
 
  // Print the IP address
  Serial.print("Use this URL to connect: ");
  Serial.print("http://");
  Serial.print(WiFi.localIP());
  Serial.println("/");

  timeClient.begin();
}

void loop() {
  // get current light 
  static bool initial = true;
  const int raw_light = analogRead(LIGHT_SENSOR); // read the raw value from light_sensor pin (A3)
  const int light = map(raw_light, 0, 1023, 0, 100); // map the value from 0, 1023 to 0, 100
 
  Serial.print("Light level: "); 
  Serial.println(light); // print the light 

  timeClient.update();
  Serial.println(timeClient.getFormattedTime()); // print the light

          
  // pre-compute hand degrees, x & y coords for a fast screen update
  const int ss = timeClient.getSeconds();
  sdeg = ss*6;                                                                     // 0-59 -> 0-354
  mdeg = timeClient.getMinutes()*6+sdeg*0.01666667;                                                     // 0-59 -> 0-360 - includes seconds
  hdeg = ((timeClient.getHours() + 3)%12)*30+mdeg*0.0833333;                                                     // 0-11 -> 0-360 - includes minutes and seconds
  hx = cos ((hdeg-90)*DEG2RAD);    
  hy = sin ((hdeg-90)*DEG2RAD);
  mx = cos ((mdeg-90)*DEG2RAD);    
  my = sin ((mdeg-90)*DEG2RAD);
  sx = cos ((sdeg-90)*DEG2RAD);    
  sy = sin ((sdeg-90)*DEG2RAD);

  if (ss==0 || initial) 
      {
      initial = 0;
      tft.drawLine (ohx, ohy, 120, 121, BLACK);                                     // erase hour and minute hand positions every minute
      ohx = hx*62+121;    
      ohy = hy*62+121;
      tft.drawLine (omx, omy, 120, 121, BLACK);
      omx = mx*84+120;    
      omy = my*84+121;
      }

  tft.drawLine (osx, osy, 120, 121, BLACK);                                      // redraw new hand positions, hour and minute hands not erased here to avoid flicker
  osx = sx*90+121;    
  osy = sy*90+121;
  tft.drawLine (osx, osy, 120, 121, RED);
  tft.drawLine (ohx, ohy, 120, 121, WHITE);
  tft.drawLine (omx, omy, 120, 121, WHITE);
  tft.drawLine (osx, osy, 120, 121, RED);
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