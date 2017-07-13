#define _DEBUG_

    #include <SPI.h>
    #include <RH_RF95.h>
    #include <ESP8266WiFi.h>
    #include <ThingerWifi.h>


    #define RFM95_CS  2    // "E"
    #define RFM95_RST 16   // "D"
    #define RFM95_INT 15   // "B"
    // Change to 434.0 or other frequency, must match RX's freq!
    #define RF95_FREQ 434.0
    #define CALLTIMEOUT 10000 //10 SEC call timeout
    #define MAXNROFNODES 60
    
    #define USERNAME "samograsic"
    #define DEVICE_ID "Gateway433MHz"
    #define DEVICE_CREDENTIAL "dc6Lqdzlb6Sa"
    #define SSID "OptNet"
    #define SSID_PASSWORD "krat11jekrat11je"
   
    RH_RF95 rf95(RFM95_CS, RFM95_INT);
    int16_t packetnum = 0;  // packet counter, we increment per xmission
    ThingerWifi thing(USERNAME, DEVICE_ID, DEVICE_CREDENTIAL);

    int value = 0;

struct NodeData
{
  uint32_t nodeId;
  float RSSI;
};

struct GPSRecord
{
  uint32_t nodeId;
  uint8_t hour, minute, seconds, year, month, day;
  float speed;
  float latitudeDegrees, longitudeDegrees;
  float altitude;
  float temperature;
  uint8_t satellites;
  float RSSI;
};

void loop() 
{
  uint16_t numberOfNodes = 0;
  NodeData nodeData[MAXNROFNODES];
  Serial.println("Preparing to call Nodes..."); // Send a message to rf95_server
  char radioBeacon[] = {0x01,0xFF,0xFF,0xFF,0xFE}; //0x01 Beacon Type, rest is address
  rf95.init();
  //while(rf95.isChannelActive()==true)
  //  yield();
  rf95.send((uint8_t *)radioBeacon,5);
  rf95.waitPacketSent();
  unsigned long startTime = millis();
  Serial.println("Sending call, waiting for clients...");
  while(((numberOfNodes<MAXNROFNODES)&&((millis()-startTime)<CALLTIMEOUT)))
  {
    if(rf95.available()==true)
    {
      //Process response
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);
     // Serial.println(len,DEC);
      rf95.recv(buf, &len);
      RH_RF95::printBuffer("Received: ", buf, len);
      Serial.println("Received response...");
      if(buf[0]==0x02) //expected response type //0x02
      {
        Serial.println("Processing response...");
        nodeData[numberOfNodes].nodeId=buf[4]+(buf[3]*0xFF)+(buf[2]*0xFFFF)+(buf[1]*0xFFFFFF);
        Serial.print("Added node ID:");Serial.println(nodeData[numberOfNodes].nodeId,DEC);
        numberOfNodes++;
      }
      else
      {
        Serial.println("Wrong response!");
      }     
    }
    yield();
   }
   if(numberOfNodes>0)
   {
     Serial.println("Makin request list of nodes..");
     //Send request to nodes for instant data
     uint8_t instantRequest[RH_RF95_MAX_MESSAGE_LEN];
     instantRequest[0]=0x03;//0x03 Instant Request Type 
     instantRequest[1]=numberOfNodes;
     uint8_t index=0;
     while(index<numberOfNodes)
     {
      instantRequest[index+2]=(uint8_t)(nodeData[index].nodeId)%0xFFF;
      instantRequest[index+3]=(uint8_t)(nodeData[index].nodeId/0xFFF)%0xFF;
      instantRequest[index+4]=(uint8_t)(nodeData[index].nodeId/0xFF)%0xF;
      instantRequest[index+5]=(uint8_t)(nodeData[index].nodeId/0xF);
      index=index+4;
     }
     rf95.send((uint8_t *)instantRequest,index+1);
     yield();
     rf95.waitPacketSent();
     startTime = millis();
     Serial.println("Request to nodes sent, waiting for data...");
     while(((millis()-startTime)<CALLTIMEOUT))
     {
        if(rf95.available()==true)
        {
          uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
          uint8_t len = sizeof(buf);
          rf95.recv(buf, &len);
          RH_RF95::printBuffer("Received data: ", buf, len);
          Serial.println("Received GPS data!!!");
          startTime = millis();
        }
        yield();
     }
     Serial.println("End of waiting for data...");
   }
   else
    Serial.println("No node responded!");
       //  StopTime = millis();
       //  Serial.println("Received response, needed time:");
       //  Serial.println(StopTime-StartTime,DEC);

        //  delay(5000);

/*  //MQTT Stuff
  Serial.println("Handling THING.IO..."); // Send a message to rf95_server
  thing.handle();
  yield();


      delay(5000);
      ++value;

  
  char radiopacket[20] = "Hello World #      ";
  itoa(packetnum++, radiopacket+13, 10);
  Serial.print("Sending "); Serial.println(radiopacket);
  radiopacket[19] = 0;
  
  Serial.println("LORA Sending..."); delay(10);
  rf95.send((uint8_t *)radiopacket, 20);
 
  Serial.println("LORA Waiting for packet to complete..."); delay(10);
  rf95.waitPacketSent();
int out;
thing["LORA-MSG"] >> [](pson& out){
      out = 100;
};
*/
}

void setup() 
{
      //Init LORA
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
      thing.add_wifi(SSID, SSID_PASSWORD);
    
}
