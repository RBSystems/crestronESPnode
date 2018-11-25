#include <credentials.h>
#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <SimpleTimer.h>                                        
#include "EmonLib.h"                   // Include Emon Library
#include <IRremoteESP8266.h>
#include <IRsend.h>
#include "TSL2561.h"  
#include <BME280_t.h>
#include <TelnetSpy.h>
TelnetSpy SerialAndTelnet;
//#include <IRremote.h>

//#define SERIAL  Serial
#define SERIAL  SerialAndTelnet
#define SEND_PIN  D3                              // Samsung Bluray
#define SEND_PIN2 D2                              // LR Westinghouse TV
#define ASCII_ESC 27
#define MYALTITUDE  150.50                        // ft above sea level

SimpleTimer timer;
EnergyMonitor emon1;   
WiFiClient espClient;

IRsend irsend(SEND_PIN);
IRsend irsend2(SEND_PIN2);
TSL2561 tsl(TSL2561_ADDR_FLOAT);                      
BME280<> BMESensor;

unsigned long lastSend;
unsigned long lastlux;
uint16_t oldlux;

unsigned long lstRecon;
unsigned long lastCurr;
uint16_t oldIrms;

//////////////////////////////////////////////////  Motion Sensor    
int calibrationTime = 15;        
long unsigned int lowIn;         
long unsigned int pause = 60000;  
boolean lockLow = true;
boolean takeLowTime;  
int motnPin = D0;

//  Samsung BD Series Bluray IR Codes (RAW)
uint16_t brPowerSig[77] = {4500,4450, 500,450, 550,450, 550,450, 500,450, 550,450, 550,1450, 550,450, 500,450, 550,450, 550,450, 550,450, 550,450, 550,400, 600,400, 550,450, 550,450, 550,4400, 550,1400, 550,1450, 550,1400, 600,400, 550,450, 550,400, 600,400, 600,400, 600,400, 550,450, 550,400, 600,400, 600,1400, 550,1400, 600,1400, 550,1400, 600,1400, 550,1400, 600,1400, 550,1400, 600};  // UNKNOWN E4CD1208
uint16_t brPlaySig[77] = {4550,4350, 500,500, 500,500, 500,500, 500,450, 550,450, 500,1500, 500,450, 500,500, 550,450, 550,450, 500,500, 500,450, 500,500, 550,400, 600,400, 550,500, 500,4400, 550,1450, 500,1450, 550,1400, 550,450, 550,450, 550,450, 550,1400, 550,450, 550,1450, 500,450, 550,450, 550,450, 550,1400, 550,1450, 550,450, 500,1450, 550,450, 550,1400, 600,1400, 550,1400, 600};  // UNKNOWN CBE92448
uint16_t brPauseSig[77] = {4450,4450, 500,500, 500,500, 500,500, 500,450, 500,500, 500,1500, 500,450, 500,500, 500,500, 500,500, 500,500, 500,450, 500,500, 550,450, 500,500, 500,500, 500,4400, 550,1450, 550,1400, 550,1450, 500,450, 550,450, 550,1400, 600,400, 550,450, 550,1450, 500,1450, 550,450, 500,500, 500,1450, 550,450, 500,1450, 550,1450, 500,500, 500,450, 550,1450, 500,1450, 550};  // UNKNOWN C2E35B04
uint16_t brStopSig[77] = {4450,4500, 500,450, 500,500, 550,450, 500,500, 500,500, 500,1450, 500,500, 500,500, 500,450, 500,500, 500,500, 500,500, 500,500, 500,450, 500,500, 500,500, 500,4400, 550,1450, 500,1500, 500,1450, 500,500, 500,1450, 550,1450, 500,500, 500,500, 500,1450, 500,500, 500,500, 500,500, 500,450, 500,500, 500,1500, 500,1450, 550,450, 550,1400, 550,1400, 600,1400, 550};  // UNKNOWN 6AC61222
uint16_t brRewSig[77] = {4400,4450, 550,500, 500,450, 500,500, 500,500, 500,450, 550,1450, 500,500, 500,450, 550,500, 500,450, 550,450, 500,500, 500,500, 500,450, 550,450, 500,500, 500,4450, 500,1500, 500,1450, 500,1500, 500,450, 500,500, 500,1500, 500,500, 500,450, 500,1500, 500,450, 550,450, 500,500, 500,1500, 500,450, 500,1500, 500,1450, 500,500, 500,1500, 500,1450, 500,1500, 500};  // UNKNOWN FF4C3E20
uint16_t brFfwdSig[77] = {4500,4400, 550,400, 600,400, 550,450, 600,400, 550,450, 550,1400, 550,450, 550,450, 550,400, 600,400, 550,450, 550,450, 600,400, 550,400, 600,400, 550,450, 600,4350, 550,1400, 600,1400, 550,1400, 600,400, 550,1450, 550,400, 600,1400, 550,450, 550,1400, 550,450, 550,450, 550,450, 550,400, 600,1400, 550,450, 550,1400, 550,450, 550,1450, 550,1400, 550,1400, 600};  // UNKNOWN 555A6302
uint16_t brEjectSig[77] = {4450,4400, 500,500, 500,450, 550,500, 500,450, 550,450, 500,1500, 500,450, 550,500, 500,450, 500,500, 500,500, 500,450, 550,450, 550,450, 500,500, 500,500, 500,4400, 550,1450, 500,1500, 500,1450, 500,450, 550,1500, 500,450, 500,500, 500,500, 500,500, 500,450, 550,450, 500,500, 500,450, 550,1450, 500,1500, 500,1450, 500,1500, 500,1450, 500,1500, 500,1450, 500};  // UNKNOWN 6E6FE942
// end bluray codes //

void waitForConnection() {
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    SERIAL.print(".");
  }
  SERIAL.println(" Connected!");
}

void waitForDisconnection() {
  while (WiFi.status() == WL_CONNECTED) {
    delay(500);
    SERIAL.print(".");
  }
  SERIAL.println(" Disconnected!");
}

void telnetConnected() {
  SERIAL.println("Telnet connection established.");
}

void telnetDisconnected() {
  SERIAL.println("Telnet connection closed.");
}

void myDelay(int ms) {
  int i;
  for(i=1;i!=ms;i++) {
    delay(1);
    if(i%100 == 0) {
      ESP.wdtFeed(); 
      yield();
    }
  }
}

void restartesp() {
  ESP.restart();
  myDelay(1000);
}

void setup() {
  SerialAndTelnet.setWelcomeMsg("C\n\n");
  SerialAndTelnet.setCallbackOnConnect(telnetConnected);
  SerialAndTelnet.setCallbackOnDisconnect(telnetDisconnected);
  SERIAL.begin(115200);
  delay(100); // Wait for serial port
  SERIAL.setDebugOutput(false);
  SERIAL.print("\n\nConnecting to WiFi ");

//  Serial.begin(115200,SERIAL_8N1,SERIAL_TX_ONLY);
  emon1.current(A0, 111.1);                         // Current: input pin, calibration.
  Wire.begin(D7,D6);
  tsl.begin();
  tsl.setGain(TSL2561_GAIN_16X);                    // set 16x gain (for dim situations)
  tsl.setTiming(TSL2561_INTEGRATIONTIME_101MS);     // medium integration time (medium light)
  BMESensor.begin();
  irsend.begin();
  irsend2.begin();
  const char *nodename = "node0";
  WiFi.begin(ssid, password);
  WiFi.mode(WIFI_STA);

  lstRecon = 0;
  oldIrms = 3;

  ArduinoOTA.onStart([]() {                             // OTA updates
  });
  ArduinoOTA.onEnd([]() {
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.setHostname(nodename);
  ArduinoOTA.setPassword(password);
  ArduinoOTA.begin();

  SERIAL.println("Ready");
  //SERIAL.print("IP address: ");
  SERIAL.println(WiFi.localIP());
  
  //SERIAL.println("\nType 'C' for WiFi connect.\nType 'D' for WiFi disconnect.\nType 'R' for WiFi reconnect.");
  //SERIAL.println("Type 'c' for getlux... and more. All other chars will be echoed. Play around...\n");
}

void current1() {
  uint16_t Irms = emon1.calcIrms(1480);
  if (oldIrms != Irms) {
    char msg[3];
    dtostrf(Irms,0, 0, msg);
    if (Irms > 5 && Irms < 20 && oldIrms < Irms) {
      SERIAL.println("warming");
      oldIrms = Irms;
    }
    else if (Irms < 2 && oldIrms > Irms) {
      SERIAL.println("cooling");
      oldIrms = Irms;
    }
  }
}

void current2() {
  uint16_t Irms = emon1.calcIrms(1480);
  char msg[3];
  dtostrf(Irms,0, 0, msg);
  SERIAL.print("|");
  SERIAL.print(msg);
  SERIAL.println("|");
}

void tvstatus() {
  uint16_t Irms = emon1.calcIrms(1480);
  if (Irms > 4  && Irms < 20) {
    SERIAL.println("warming");
  }
  else if (Irms < 1) {
    SERIAL.println("cooling");
  }
}

void bmetemp() {
    char bufout[10];
    sprintf(bufout,"",ASCII_ESC);
    dtostrf((int)round(1.8*BMESensor.temperature+32),0, 0, bufout);
    SERIAL.print("@");
    SERIAL.print(bufout);
    SERIAL.println("@");
}

void bmepress() {
    char bufout[10];
    sprintf(bufout,"",ASCII_ESC);
    double relativepressure = (BMESensor.seaLevelForAltitude(MYALTITUDE)/3389.39);
    dtostrf(relativepressure,5, 2, bufout);
    SERIAL.print("$");
    SERIAL.print(bufout);
    SERIAL.println("^");
}

void bmehum() {
    char bufout[10];
    sprintf(bufout,"",ASCII_ESC);
    dtostrf(BMESensor.humidity,0, 0, bufout);
    SERIAL.print("!");
    SERIAL.print(bufout);
    SERIAL.println("!");
    BMESensor.refresh();
}

void motion() {
  if(digitalRead(motnPin) == HIGH){
    if(lockLow){  
      lockLow = false;            
      SERIAL.println("m1");
    }         
    takeLowTime = true;
    }
    if(digitalRead(motnPin) == LOW){       
      if(takeLowTime){
        lowIn = millis();          //save the time of the transition from high to LOW
        takeLowTime = false;       //make sure this is only done at the start of a LOW phase
        }
       if(!lockLow && millis() - lowIn > pause){  
           SERIAL.println("m0");
           lockLow = true;                        
           }
       }
}

/////////////////////////////////////////////////////////////////////// IR Codes ////////////////////////////////////////

void mute() {
    irsend2.sendSony(0x290, 12); 
}                                   //  Mute

void volDown() {
  irsend2.sendSony(0xC90, 12);
}

void volUp() {
  irsend2.sendSony(0xC90, 12);
}

void source() {
  irsend2.sendSony(0xA50, 12);
}

void hdmi() {
  irsend2.sendSony(0x430, 12);
}

void sleeptv() {
  irsend2.sendSony(0x6D0, 12);
}

void BRTVOn() {
  irsend2.sendSony(0xA90, 12);   
  myDelay(3000);
  current1();
}

void BRTVOff() {
  irsend2.sendSony(0xA90, 12);
  myDelay(1000);
  current1();
}

void chUp() {
  irsend2.sendSony(0x090, 12); 
}

void chDown() {
  irsend2.sendSony(0x890, 12);
}

void ratio() {
  irsend2.sendSony(0x390, 12);
}

/////// Blu-Ray commands

void brPowerTog() {
  irsend.sendRaw(brPowerSig, 77, 32);
}
void brPlayIR() {
  irsend.sendRaw(brPlaySig, 77, 32);
}
void brPauseIR() {
  irsend.sendRaw(brPauseSig, 77, 32);
}
void brStopIR() {
  irsend.sendRaw(brStopSig, 77, 32);
}
void brRewIR() {
  irsend.sendRaw(brRewSig, 77, 32);
}
void brFfwdIR() {
  irsend.sendRaw(brFfwdSig, 77, 32);
}
void brEjectIR() {
  irsend.sendRaw(brEjectSig, 77, 32);
}

/*
/// hdmi switcher IR codes ////////
void hdmi1() {
  irsend.sendNEC(0x40BFFB04, 32);
}

void hdmi2() {
  irsend.sendNEC(0x40BFF906, 32);
}

void hdmi3() {
  irsend.sendNEC(0x40BFC33C, 32);
}

void hdmi4() {
  irsend.sendNEC(0x40BF19E6, 32);
}

void hdmi5() {
  irsend.sendNEC(0x40BFE916, 32);
}
*/

void getlux() {
  uint16_t lux = tsl.getLuminosity(TSL2561_VISIBLE);   
  char msg[5];
  dtostrf(lux,0, 0, msg);
  SERIAL.print("x");
  SERIAL.print(msg);
  SERIAL.println("%");
}

void alive() {
  SERIAL.println("OK");
}

void loop() {
  SerialAndTelnet.handle();
  ArduinoOTA.handle();

  if (SERIAL.available() > 0) {
    char c = SERIAL.read();
    switch (c) {
      case '\r':
        SERIAL.println();
        break;
      case 'C':
        SERIAL.print("\nConnecting ");
        WiFi.begin(ssid, password);
        waitForConnection();
        break;
      case 'D':
        SERIAL.print("\nDisconnecting ...");
        WiFi.disconnect();
        waitForDisconnection();
        break;
      case 'R':
        SERIAL.print("\nReconnecting ");
        WiFi.reconnect();
        waitForDisconnection();
        waitForConnection();
        break;
      case 'q':
        BRTVOn();
        break;
      case 'r':
        BRTVOff(); 
        break;
//      case 'w':     
//        tvstatus();     
//        break;
      case '9':
        restartesp();
        break;
      case 'f':
        mute();
        break;
      case '4':     
        volUp();
        break;
      case '5':   
        volDown();
        break;
      case '8':
        hdmi();
        break;
      case 'z':
        sleeptv();
        break;
      case 't':
        bmetemp();
        break;
      case 'b':
        alive();
        break;
      case 'c':
        getlux();
        break;
      case 'i':
        current1();
        break;
      case 'n':
        ratio();
        break;
      case 'h':
        bmehum();
        break;
      case 'p':
        bmepress();
        break;
      case 'd':
        brPowerTog();
        break;
      case 's':
        brEjectIR();
        break;
      case 'u':
        brPlayIR();
        break;
      case 'v':
        brPauseIR();
        break;
      case 'w':
        brStopIR();
        break;
      case 'x':
        brRewIR();
        break;
      case 'y':
        brFfwdIR();
        break;
      default:
        SERIAL.print(c);
        break;
/*
      case 'e':
        hdmi1();
        break;
      case 'k':
        hdmi2();
        break;
      case 'g':
        hdmi3();
        break;  
      case 'h':
        hdmi4();
        break;  
      case 'j':
        hdmi5();
        break;
*/
    }
  }
}
