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
  float latitudeDegrees, longitudeDegrees;
  float altitude;
  float speed;
  uint8_t satellites;
  uint8_t RSSI;
  uint8_t battery;
  int8_t temperature;
};


GPSRecord lastGPSRecord;


uint8_t writeStringToGPSRecord(uint8_t  *str)
  {
    lastGPSRecord.nodeId = (str[1] <<  24) | (str[2] << 16) | (str[3] << 8) | str[4];
    lastGPSRecord.hour=str[5];lastGPSRecord.minute=str[6];lastGPSRecord.seconds=str[7];
    lastGPSRecord.year=str[8];lastGPSRecord.month=str[9];lastGPSRecord.day=str[10];
    lastGPSRecord.latitudeDegrees = ((str[11] <<  24) | (str[12] << 16) | (str[13] << 8) | str[14] )/100000.0;
    lastGPSRecord.longitudeDegrees = ((str[15] <<  24) | (str[16] << 16) | (str[17] << 8) | str[18] )/100000.0;
    lastGPSRecord.altitude=((str[19] << 8) | str[20] )/10.0;
    lastGPSRecord.speed=((str[21] << 8) | str[22] )/100.0;
    lastGPSRecord.satellites=str[23];
    lastGPSRecord.RSSI=str[24];
    lastGPSRecord.battery=str[25];
    return 26;

}

void printGPSData()
{
    Serial.print("\nGPS RECORD DATA:");
    Serial.print("\nNodeId:");
    Serial.print(lastGPSRecord.nodeId, HEX); Serial.print(':');
    Serial.print("\nTime: ");
    Serial.print(lastGPSRecord.hour, DEC); Serial.print(':');
    Serial.print(lastGPSRecord.minute, DEC); Serial.print(':');
    Serial.print(lastGPSRecord.seconds, DEC); Serial.print('.');
    Serial.print("Date: ");
    Serial.print(lastGPSRecord.day, DEC); Serial.print('/');
    Serial.print(lastGPSRecord.month, DEC); Serial.print("/20");
    Serial.println(lastGPSRecord.year, DEC);
    Serial.print("Location: ");
    Serial.print(lastGPSRecord.latitudeDegrees, 4);
    Serial.print(", ");
    Serial.print(lastGPSRecord.longitudeDegrees, 4); 
    Serial.print("\nSpeed(km/h): "); Serial.println(lastGPSRecord.speed);
    Serial.print("Altitude: "); Serial.println(lastGPSRecord.altitude);
    Serial.print("Satellites: "); Serial.println((int)lastGPSRecord.satellites);
    Serial.print("RSSI(%): "); Serial.println(lastGPSRecord.RSSI);
    Serial.print("Battery(%): "); Serial.println(lastGPSRecord.battery);
    Serial.print("Temperature(C): "); Serial.println(lastGPSRecord.temperature);
}

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
        nodeData[numberOfNodes].nodeId = (buf[1] <<  24) | (buf[2] << 16) | (buf[3] << 8) | buf[4];
        Serial.print("Added node ID:");Serial.println(nodeData[numberOfNodes].nodeId,HEX);
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
     instantRequest[0]=0x03;//0x03 Instant Data Request Type 
     instantRequest[1]=numberOfNodes;
     uint8_t index=0;
     while(index<numberOfNodes)
     {         
      instantRequest[index+2]=nodeData[index].nodeId>> 24;
      instantRequest[index+3]=nodeData[index].nodeId>> 16;
      instantRequest[index+4]=nodeData[index].nodeId>> 8;
      instantRequest[index+5]=nodeData[index].nodeId;
      index=index+4;
     }
     rf95.send((uint8_t *)instantRequest,(index*4)+2);
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
          
          writeStringToGPSRecord(buf);
            Serial.println("Received GPS data!!!");
          printGPSData();
          
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
