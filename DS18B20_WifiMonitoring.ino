/*
 * DS18B20 Temperatures Wifi Monitoring
 *  Purpose: 
 *    Allow to report at regular update the temperatur evalues monitored on DS18B20.
 *    DS18B20 are seek on the bus, up to eight temperature sensors can be managed.
 *  Author:
 *    Vincent(.)Cruvellier(@]gmail(.)com
 *  Date of release:
 *     13-MAY-2017
 *  Hardware:
 *    MCU : ESP-01
  *  Interface:
 *    Main Serial at 115 200 bauds
 *    
 *    Current chipId = 10363774
 *    https://dweet.io:443/get/latest/dweet/for/10363774
 *    
 */ 


 /* 
 *  Include SECTION
 */
#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>
#include "WiFiManager.h"          //https://github.com/tzapu/WiFiManager
#include <Base64.h>
#include <OneWire.h>
#include <ArduinoJson.h>          //https://bblanchon.github.io/ArduinoJson/

// Debug Section definition
#define DEBUG_PROG 
#ifdef DEBUG_PROG
  #define DEBUG_PRINTLN(x)  Serial.println(x)
  #define DEBUG_PRINT(x)    Serial.print(x)
#else
  #define DEBUG_PRINTLN(x) 
  #define DEBUG_PRINT(x)
#endif

/*
 * GLOBAL VARIABLES
 */
const int nsensors = 2;           // Force Management of 2 sensors
byte sensors[][8] = {
   { 0x28, 0xFF, 0x0F, 0xA8, 0x6C, 0x14, 0x04, 0x3A },
   { 0x28, 0xFF, 0xBC, 0x02, 0x6D, 0x14, 0x04, 0x22 },
   { 0x28, 0xC1, 0x02, 0x64, 0x04, 0x00, 0x00, 0x35 },
   { 0x28, 0xE7, 0x0B, 0x63, 0x04, 0x00, 0x00, 0x44 }
   //28 FF F A8 6C 14 4 3A
   //28 FF BC 2 6D 14 4 22  // 28 FF F A8 6C 14 4 3A
}; // Table Management for Sensor
int16_t tempraw[nsensors];
unsigned long nextprint = 0;
OneWire  ds(2);  // on pin 2 (a 4.7K pullup is necessary)
/* 
 *  Use of DWEET.IO service
 */
#define SERVER_DWEET "dweet.io"
#define PORT_DWEET 80 
char dweet_host[40] = "dweet.io";
char thing_name[40] = "MyThing";
bool _useDweetIO = true;
char updatefreq[5] = "15"; // 
/*
 * WIFI data
 */
WiFiClient _client;
ESP8266WebServer server(80);
WiFiManager wifiManager;

uint REPORT_INTERVAL = 60; // in sec

/* 
 *  ds18process:
 *  
 *  Process the sensor data in stages.Each stage will run quickly. the conversion 
 * delay is done via a millis() based delay. a 5 second wait between reads reduces self
 * heating of the sensors.
 */
void ds18process() {
  static byte stage = 0;
  static unsigned long timeNextStage = 0;
  static byte sensorindex = 100;
  byte i, j;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];

  if(stage == 0 && millis() > timeNextStage) {
    if (!ds.search(addr)) {
      //no more, reset search and pause
      ds.reset_search();
      timeNextStage = millis() + 5000; //5 seconds until next read
      return;
    } else {
      if (OneWire::crc8(addr, 7) != addr[7]) {
        Serial.println("CRC is not valid!");
        return;
      }
      //got one, start stage 1
      stage = 1;
    }
  }
  if(stage==1) {
    Serial.print("ROM =");
    for ( i = 0; i < 8; i++) {
      Serial.write(' ');
      Serial.print(addr[i], HEX);
    }
    //find sensor
    for(j=0; j<nsensors; j++){
      sensorindex = j;
      for(i=0; i<8; i++){
        if(sensors[j][i] != addr[i]) {
          sensorindex = 100;
          break; // stop the i loop
        }
      }
      if (sensorindex < 100) { 
        break; //found it, stop the j loop
      }
    }
    if(sensorindex == 100) {
      Serial.println("  Sensor not found in array");
      stage = 0;
      return;
    }
    Serial.print("  index="); Serial.println(sensorindex); 
    ds.reset();
    ds.select(sensors[sensorindex]);
    ds.write(0x44, 0);        // start conversion, with parasite power off at the end
    stage = 2; //now wait for stage 2
    timeNextStage = millis() + 1000; //wait 1 seconds for the read
  }
  
  if (stage == 2 && millis() > timeNextStage) {
    // the first ROM byte indicates which chip
    switch (sensors[sensorindex][0]) {
      case 0x10:
        Serial.print("  Chip = DS18S20");  // or old DS1820
        Serial.print("  index="); Serial.println(sensorindex);
        type_s = 1;
        break;
      case 0x28:
        Serial.print("  Chip = DS18B20");
        Serial.print("  index="); Serial.println(sensorindex);
        type_s = 0;
        break;
      case 0x22:
        Serial.print("  Chip = DS1822");
        Serial.print("  index="); Serial.println(sensorindex);
        type_s = 0;
        break;
      default:
        Serial.println("Device is not a DS18x20 family device.");
        stage=0;
        return;
    }
  
    present = ds.reset();
    ds.select(sensors[sensorindex]);
    ds.write(0xBE);         // Read Scratchpad
  
    Serial.print("  Data = ");
    Serial.print(present, HEX);
    Serial.print(" ");
    for ( i = 0; i < 9; i++) {           // we need 9 bytes
      data[i] = ds.read();
      Serial.print(data[i], HEX);
      Serial.print(" ");
    }
    Serial.print(" CRC=");
    Serial.print(OneWire::crc8(data, 8), HEX);
    Serial.print(" index="); Serial.print(sensorindex);
    Serial.println();
  
    int16_t raw = (data[1] << 8) | data[0];
    if (type_s) {
      raw = raw << 3; // 9 bit resolution default
      if (data[7] == 0x10) {
        // "count remain" gives full 12 bit resolution
        raw = (raw & 0xFFF0) + 12 - data[6];
      }
    } else {
      byte cfg = (data[4] & 0x60);
      // at lower res, the low bits are undefined, so let's zero them
      if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
      else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
      else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
      //// default is 12 bit resolution, 750 ms conversion time
    }
    tempraw[sensorindex] = raw;
    stage=0;
  }
}

/* Converts raw temp to Celsius or Fahrenheit
 * scale: 0=celsius, 1=fahrenheit
 * raw: raw temp from sensor
 * 
 * Call at any time to get the last save temperature
 */
float ds18temp(byte scale, int16_t raw) 
{
  switch(scale) {
    case 0: //Celsius
      return (float)raw / 16.0;
      break;
    case 1: //Fahrenheit
      return (float)raw / 16.0 * 1.8 + 32.0;
      break;
    default: //er, wut
      return -255;
  }
}


/*************************************************************
 * configModeCallback (WiFiManager *myWiFiManager)
 */

void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
}

void make_json(){
  StaticJsonBuffer<200> jsonBuffer;

  JsonObject& root = jsonBuffer.createObject();
  root["application"] = "DS18B20_WIFI_MONITOING";
  root["time"] = millis();
  
  JsonArray& data = root.createNestedArray("sensors");
  data.add(ds18temp(0, tempraw[0]), 6);  // 6 is the number of decimals to print
  data.add(ds18temp(0, tempraw[1]), 6);   // if not specified, 2 digits are printed
  //root.printTo(server);
  //String root = "{}";
    String response = "";
    root.printTo(response);
    server.send(200, "application/json", response);
  
}

/*************************************************************
 * handle_root()
 */
void handle_root() {
  String debugstr = "\n\nApplication data:\n";
         for(byte j=0; j<nsensors; j++){
          debugstr += "Sensor: " + String(j);
          String sensortype="Unknow";
          Serial.print(j); Serial.print("=> ");
           switch (sensors[j][0]) {
              case 0x10:
                Serial.print(" Chip = DS18S20");  // or old DS1820
                sensortype = "DS18S20";
                break;
              case 0x28:
                Serial.print(" Chip = DS18B20");
                sensortype = "DS18B20";
                break;
              case 0x22:
                Serial.print(" Chip = DS1822");
                sensortype = "DS1822";
              default:
                Serial.println(" Device is not a DS18x20 family device.");
              }
              debugstr += " Type: "+sensortype + " Rom: ";
              Serial.print("  ROM =");
              for ( int i = 0; i < 8; i++) {
                Serial.write(' ');
                Serial.print(sensors[j][i], HEX);
                debugstr += " ";
                if (sensors[j][i] <= 0xF) debugstr += "0";
                debugstr += String(sensors[j][i], HEX);
              }
              Serial.print("Temp C = ");
              Serial.print(ds18temp(0, tempraw[j])); Serial.println("");
              debugstr += " Temp: " + String(ds18temp(0, tempraw[j])) + "\n";
        }
        debugstr += "\n";
        debugstr += "ESP chip Id: " + String(ESP.getChipId());
    server.send(200, "text/plain", "Hello from the DS18B20_Wifi_Monitoring, read data from /json. Reset Wifi settings /reset."+ debugstr);
    delay(100);
}

long unsigned lastTimeStampdataSent=0;

/*************************************************************
 * setup()
 */
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Start...");
/*
  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManagerParameter p_custom_text1("<br>*** DWEET.IO feature ***<br>");
  WiFiManagerParameter p_custom_text2("Server: ");
  WiFiManagerParameter p_custom_dweet_host("server", "dweet.io", dweet_host, 40, "");
  WiFiManagerParameter p_custom_text3("ThingName: ");
  WiFiManagerParameter p_custom_dweet_thing_name("ThingName", "ThingName.io", thing_name, 40, "");
  WiFiManagerParameter p_custom_text4("Update Frequency (minutes): ");
  WiFiManagerParameter p_custom_dweet_updatefreq("updatefreq", "UpdateFred", updatefreq, 4, "");

  char customhtml[24] = "type=\"checkbox\"";
  if (_useDweetIO) {
       strcat(customhtml, " checked");
  }
  WiFiManagerParameter p_custom_text5("<br>Service Enabled: ");
  WiFiManagerParameter p_useDweetIO("DweetIO:", "use DweetIO", "T", 2, customhtml);
  //add all your parameters here
  wifiManager.addParameter(&p_custom_text1);
  wifiManager.addParameter(&p_custom_text2);
  wifiManager.addParameter(&p_custom_dweet_host);
  wifiManager.addParameter(&p_custom_text3);
  wifiManager.addParameter(&p_custom_dweet_thing_name);
  wifiManager.addParameter(&p_custom_text4);
  wifiManager.addParameter(&p_custom_dweet_updatefreq);
  wifiManager.addParameter(&p_custom_text5);
  wifiManager.addParameter(&p_useDweetIO);
  //wifiManager.addParameter(&p_custom_text2);
*/   
  //reset settings - for testing
  //wifiManager.resetSettings();

  //set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  wifiManager.setAPCallback(configModeCallback);

  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  if(!wifiManager.autoConnect()) {
    Serial.println("failed to connect and hit timeout");
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(1000);
  } 

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");
  /*Serial.println("p_useDweetIO :"+String(p_useDweetIO.getValue()));
  _useDweetIO = (strncmp(p_useDweetIO.getValue(), "T", 1) == 0);
  REPORT_INTERVAL = atoi(p_custom_dweet_updatefreq.getValue());
  REPORT_INTERVAL = 3; // 3 Minutes
  Serial.println("p_custom_dweet_updatefreq:"+String(p_custom_dweet_updatefreq.getValue()));
  Serial.println("p_custom_dweet_updatefreq:"+String(p_custom_dweet_updatefreq.getValue()).toInt());
  Serial.println("REPORT_INTERVAL = ");*/
  String tmpstr = String(ESP.getChipId());
  tmpstr.toCharArray(thing_name, tmpstr.length()+1);
  Serial.print("Thing name="); Serial.println(thing_name);
  // Start the server
  server.on("/", handle_root);
  server.on("/reset", []() {
    wifiManager.resetSettings();
    ESP.reset();
    delay(5000);
  });
  server.on("/json", []() {
    make_json();
  });
  server.begin();
  Serial.println("Server started");
  // Print the IP address
  Serial.println(WiFi.localIP());
  ds18process();
  Serial.println("DSB18B20 started");
}

/*************************************************************
 * push_dsb_data()
 *  
 *  Push temperature date to dweet
 */
void push_dsb_data() {
   // put your main code here, to run repeatedly:
    String all = "sensor0=" + String(ds18temp(0, tempraw[0]),5);
    all += "&sensor1=" + String(ds18temp(0, tempraw[1]),5);
    if (_client.connect(SERVER_DWEET, PORT_DWEET)){
          Serial.println(F("Posting your variables"));
          _client.print(F("POST /dweet/for/"));
          _client.print(thing_name);
          _client.print(F("?"));
          _client.print(all);
          _client.println(F(" HTTP/1.1"));
          _client.println(F("Host: dweet.io"));
          _client.println(F("User-Agent: ESP8266-WiFi/1.0"));
          _client.println(F("Connection: close"));
          _client.println();          
    }
    while(!_client.available());
    while (_client.available()){
        char c = _client.read();
        Serial.write(c);
    }
    //currentValue = 0;
    _client.stop();
  delay(100);
}


void loop() {
  REPORT_INTERVAL = 1;
  ds18process(); //call this every loop itteration, the more calls the better.
  if ((millis() - lastTimeStampdataSent) > (1000 * 60 * REPORT_INTERVAL)) {
       Serial.print("Temp F: ");
       for(byte j=0; j<nsensors; j++){
          Serial.print(j); Serial.print("=");
          Serial.print(ds18temp(0, tempraw[j])); Serial.print("  ");
      }
    push_dsb_data();
    lastTimeStampdataSent = millis();
  }
  server.handleClient();
  delay(10);
}
