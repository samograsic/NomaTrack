#include <ThingerCC3000.h>
#include <ThingerClient.h>
#include <ThingerENC28J60.h>
#include <ThingerESP32.h>
#include <ThingerESP8266.h>
#include <ThingerESP8266AT.h>
#include <ThingerEthernet.h>
#include <ThingerLinkItOneGPRS.h>
#include <ThingerLinkItOneWifi.h>
#include <ThingerSmartConfig.h>
#include <ThingerTinyGSM.h>
#include <ThingerWebConfig.h>
#include <ThingerWifi.h>
#include <ThingerWifi101.h>


    #define RFM95_CS  2    // "E"
    #define RFM95_RST 16   // "D"
    #define RFM95_INT 15   // "B"
    // Change to 434.0 or other frequency, must match RX's freq!
    #define RF95_FREQ 434.0
   
    #include <SPI.h>
    #include <RH_RF95.h>
    #include <ESP8266WiFi.h>

    RH_RF95 rf95(RFM95_CS, RFM95_INT);
    int16_t packetnum = 0;  // packet counter, we increment per xmission


    ThingerWifi thing("username", "deviceId", "deviceCredential");

void setup() {
  thing.add_wifi("wifi_ssid", "wifi_credentials");
}

void loop() {
  thing.handle();
}     
    const char* ssid     = "Heimlauta4G";
    const char* password = "thelia1216";
     
    const char* host = "wifitest.adafruit.com";
     
    void setup() {
      Serial.begin(115200);
      delay(100);
      // LORA Radio Init
      pinMode(RFM95_RST, OUTPUT);
      digitalWrite(RFM95_RST, HIGH);
      Serial.println("Feather LoRa RX Test!");
      // manual reset
      digitalWrite(RFM95_RST, LOW);
      delay(10);
      digitalWrite(RFM95_RST, HIGH);
      yield();
      delay(10);
      while (!rf95.init()) 
      {
        Serial.println("LoRa radio init failed");
        yield();
      }
      Serial.println("LoRa radio init OK!");
     
      // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
      if (!rf95.setFrequency(RF95_FREQ)) {
        Serial.println("setFrequency failed");
        yield();
      }
      Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
      // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
      // The default transmitter power is 13dBm, using PA_BOOST.
      // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
      // you can set transmitter powers from 5 to 23 dBm:
      rf95.setTxPower(23, false);
     
      // We start by connecting to a WiFi network
     
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
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
    }
     
    int value = 0;
     
    void loop() {
      delay(5000);
      ++value;

  Serial.println("LORA Transmitting..."); // Send a message to rf95_server
  
  char radiopacket[20] = "Hello World #      ";
  itoa(packetnum++, radiopacket+13, 10);
  Serial.print("Sending "); Serial.println(radiopacket);
  radiopacket[19] = 0;
  
  Serial.println("LORA Sending..."); delay(10);
  rf95.send((uint8_t *)radiopacket, 20);
 
  Serial.println("LORA Waiting for packet to complete..."); delay(10);
  rf95.waitPacketSent();
  
     
      Serial.print("connecting to ");
      Serial.println(host);
      
      // Use WiFiClient class to create TCP connections
      WiFiClient client;
      const int httpPort = 80;
      if (!client.connect(host, httpPort)) {
        Serial.println("connection failed");
        return;
      }
      
      // We now create a URI for the request
/*      String url = "/testwifi/index.html";
      Serial.print("Requesting URL: ");
      Serial.println(url);
      
      // This will send the request to the server
 //     client.print(String("GET ") + url + " HTTP/1.1\r\n" +
                   "Host: " + host + "\r\n" + 
                   "Connection: close\r\n\r\n");
      delay(500);
      
      // Read all the lines of the reply from server and print them to Serial
      while(client.available()){
        String line = client.readStringUntil('\r');
        Serial.print(line);
      }
 */     
      Serial.println();
      Serial.println("closing connection");
    }
