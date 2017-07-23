//#include <Adafruit_SleepyDog.h>

#include <Adafruit_GPS.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <math.h>

#define NODEID 0xAAAAAAA2

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

#define MINDELAY 1000
#define MAXDELAY 2000
#define RESPONSETIMEOUT 10000
#define RADIOTIMESLOT 2000

//Protocol State Machine
#define WAIT 0
#define INDENTIFY 1
#define DATA 2
  
  
#define VBATPIN A7

uint8_t protocolState=0;
uint32_t currentServerId=0;
uint32_t nodeId=NODEID;
unsigned long lastPacketSentTime;
uint8_t timeSlotIndex;

#define GPSSerial Serial1


struct GPSRecord
{
  uint32_t nodeId;
  uint8_t hour, minute, seconds, year, month, day;
  float latitudeDegrees, longitudeDegrees;
  float altitude;
  float speed;
  uint8_t satellites;
  uint8_t RSSI;
  float battery;
  int8_t temperature;
};

GPSRecord lastGPSRecord;

// Change to 434.0 or other frequency, must match RX's freq!
//#define RF95_FREQ 434.0
#define RF95_FREQ 868.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

// Blinky on receipt
#define LED 13

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

uint32_t timer = millis();




uint8_t processPacket()
{
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);
      rf95.recv(buf, &len);
      RH_RF95::printBuffer("Received: ", buf, len);
      Serial.print("RSSI: "); Serial.print(rf95.lastRssi(), DEC); Serial.print(" "); Serial.print(rssiToPercentage(rf95.lastRssi()), DEC); Serial.println("%");
      lastGPSRecord.RSSI=rssiToPercentage(rf95.lastRssi());
      switch (buf[0]) {
        case 0x01:
        {
          //Expecting init call
         //if we are in avaliable we accept server ID
          if (protocolState==WAIT)
          {
              currentServerId=(buf[1] <<  24) | (buf[2] << 16) | (buf[3] << 8) | buf[4];
              Serial.print("Answering to nodeID:");Serial.println(currentServerId,HEX);
              protocolState=INDENTIFY;
          }
          else
              Serial.print("Received 0x01, but we are in other state!");
          return 0x01;          
        }
        break;
        case 0x03:
        {
        //Expeciting request for data and timeslot
          Serial.println("Processing requested data response...");
          uint8_t numberOfNodes=buf[1];
          uint8_t index=0;
          Serial.print("Searching our time slot...");
          timeSlotIndex=255;
          while((index<numberOfNodes)&&(timeSlotIndex==255))
          {
             uint32_t parsedId=(buf[2] <<  24) | (buf[3] << 16) | (buf[4] << 8) | buf[5];
         //    parsedId=buf[index+5]+((uint32_t)buf[index+4]*0xFF)+((uint32_t)buf[index+3]*0xFFFF)+(buf[index+2]*0xFFFFFF);
         /*    Serial.print("Buf+2:");Serial.println(buf[index+2],HEX);
             Serial.print("Buf+3:");Serial.println(buf[index+3],HEX);
             Serial.print("Buf+4:");Serial.println(buf[index+4],HEX);
             Serial.print("Buf+5:");Serial.println(buf[index+5],HEX);
            Serial.print("ID:");Serial.println(parsedId,HEX);*/
             if(parsedId==NODEID)
                 timeSlotIndex=index/4;
             index=index+4; 
          }
          protocolState=DATA;
          return 0x03;
        }
      break;
     default:
       {
        // if nothing else matches
       Serial.println("Unknow type of packet received!!!!");
       return 0xFF;
       }
      break;
    }
}

//Protocol state machine
void execProtocol()
{
  //Serial.println("Protocoling...");
    if(rf95.available()==true)
    {
      uint8_t packetType=processPacket();
      if(packetType!=0xFF)
      switch (protocolState) {
       case WAIT:
        //Waiting for the init call, we really do nothing
       Serial.println("WAIT:  Waiting...");
      break;
    case INDENTIFY:
      //Introduce ourself
      Serial.println("INDENTIFY: Indetifiying ourself...");
      //Than we anwser with our id
      uint8_t data[RH_RF95_MAX_MESSAGE_LEN];
      data[0]=0x02; 
      data[1]=nodeId>>24;data[2]=nodeId>>16;data[3]=nodeId>>8;data[4]=nodeId;
       //while(rf95.isChannelActive()==true)
      delay(random(MINDELAY, MAXDELAY));
      rf95.send(data,5);
      rf95.waitPacketSent();
      delay(100);
      lastPacketSentTime = millis();
      Serial.println("Responded to call, waiting for request...");
      break;
     case DATA:
        //Send data in in our time slot -if we got it
          if(timeSlotIndex!=255)
          {
                      Serial.print("OurTimeSlot:");Serial.println(timeSlotIndex,DEC);
                      Serial.println("Waiting for my time...");
                      delay(500); delay(timeSlotIndex*RADIOTIMESLOT);
                      //send current GPS record
                      uint8_t GPSData[RH_RF95_MAX_MESSAGE_LEN];
                      GPSData[0]=0x04;
                      uint8_t size=writeGPSRecordToString(GPSData);
                      rf95.send(GPSData,size);
                      rf95.waitPacketSent();                      
                      Serial.print("Sent data package with size:"); Serial.println(size,DEC);
                      printGPSData();
                          
          }
          else
            Serial.println("No timeslot for us... Going to WAIT");
          protocolState=WAIT;  
       break;
     default:
       // if nothing else matches, do the default and init protocol
       protocolState=WAIT;
      break;
    }
    }
}

void loop()
{
  readGPS();
  execProtocol();
  

  /*
  while(rf95.available()==false)
  {
    //We wait for a radio call
    readGPS();
  }
  //After we check if it a valid radio all
  //LORA Stuff

  if ()
  {
      //Debug
      digitalWrite(LED, HIGH);
      RH_RF95::printBuffer("Received: ", buf, len);
      Serial.print("Got: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.print(rf95.lastRssi(), DEC); Serial.print(" ");
     // Serial.print(rssiToPercentage(rf95.lastRssi()), DEC); Serial.println("%");
      //Checking if it is a right call
      if((buf[0]==0x01)&&(buf[1]==0xFF)&&(buf[2]==0xFF)&&(buf[3]==0xFF)&&(buf[4]==0xFE))
      {
        //Than we anwser with our id
         lastGPSRecord.RSSI=rf95.lastRssi();
         uint8_t data[] = {0x02,0x00,0x00,0x00,0x01};
         unsigned long startTime = millis();
         //while(rf95.isChannelActive()==true)
         delay(random(MINDELAY, MAXDELAY));
         rf95.send(data, sizeof(data));
         rf95.waitPacketSent();
         startTime = millis();
         Serial.println("Responded to call, waiting for request...");
         while((millis()-startTime)<RESPONSETIMEOUT)
         {
              if(rf95.available()==true)
              {
                //Process response
                uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
                uint8_t len = sizeof(buf);
                // Serial.println(len,DEC);
                rf95.recv(buf, &len);
                RH_RF95::printBuffer("Received request response: ", buf, len);
                if(buf[0]==0x03) //expected instant data response type //0x03
                {
                    Serial.println("Processing instat data response...");
                    uint8_t numberOfNodes=buf[1];
                    uint8_t index=0;
                    Serial.println("Searching our time slot...");
                    uint8_t timeSlotIndex=255;
                    while((index<numberOfNodes)&&(timeSlotIndex==255))
                    {
                      uint32_t parsedId;
                      parsedId=buf[index+2]+(buf[index+3]*0xF)+(buf[index+4]*0xFF)+(buf[index+5]*0xFFF);
                      if(parsedId==NODEID)
                        timeSlotIndex=index/4;
                      index=index+4; 
                    }
                    if(timeSlotIndex!=255)
                    {
                      Serial.print("OurTimeSlot:");Serial.println(timeSlotIndex,DEC);
                      Serial.println("Waiting for my time...");
                      readGPS();
                      delay(timeSlotIndex*RADIOTIMESLOT);
                      //send current GPS record
                      uint8_t GPSData[RH_RF95_MAX_MESSAGE_LEN];
                      uint8_t size=writeGPSRecordToString(lastGPSRecord,GPSData);
                      rf95.send(GPSData,size);
                       Serial.print("Size:"); Serial.println(size,DEC);
                      //Buffer printOut

                      printPacket(GPSData);                      
                      
                      rf95.waitPacketSent();
                      Serial.println("Instant GPS Data Sent...");



                      
                    }
                    else
                    {
                      Serial.print("No time slot for me...");
                    }
                }
            }
        }
      }
      else
      {
        Serial.println("Not call for us!");
      }
  }
  */
}


void initLora()
{
 
  //LORA Init
  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
   Serial.println("Feather LoRa RX Test!");
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

//rf95.setTxPower(23); // use PA_BOOST transmitter pin

rf95.setTxPower(23, false); // use PA_RFO pin transmitter pin
//rf95.setModemConfig(RH_RF95::Bw125Cr48Sf4096 );
//rf95.setModemConfig(RH_RF95::Bw500Cr45Sf128);

//rf95.printRegisters();
}
 
void setup()
{
  delay(100);
  Serial.begin(115200);
  delay(500);

  lastGPSRecord.nodeId=NODEID;
  initLora();

  //GPS Init
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz
  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);
}


void printPacket(uint8_t *ptr)
{
  for(uint8_t i=0;i<RH_RF95_MAX_MESSAGE_LEN; i++)
  {
  Serial.print(i,DEC);Serial.print(" :0x");Serial.println(ptr[i],HEX);
  }
}

void readGPS()
{
  //GPS Stuff
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  // if (GPSECHO)
  // if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false

    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis()) timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
  /*  Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }*/
  //Fill in lastGPSRecord
  lastGPSRecord.hour=GPS.hour;
  lastGPSRecord.minute=GPS.minute;
  lastGPSRecord.seconds=GPS.seconds;
  lastGPSRecord.year=GPS.year;
  lastGPSRecord.month=GPS.month;
  lastGPSRecord.day=GPS.day;
  lastGPSRecord.speed=GPS.speed*1.852; //coversion from knots to KM/h
  lastGPSRecord.latitudeDegrees=convertDegMinToDecDeg(GPS.latitude);
  lastGPSRecord.longitudeDegrees=convertDegMinToDecDeg(GPS.longitude);
  lastGPSRecord.altitude=GPS.altitude;
  lastGPSRecord.satellites=GPS.satellites;

   float measuredvbat = analogRead(VBATPIN);
    measuredvbat *= 2;    // we divided by 2, so multiply back
    measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
    measuredvbat /= 1024; // convert to voltage
   //lastGPSRecord.battery=((measuredvbat-3)/0.7)*100;
    lastGPSRecord.battery=measuredvbat;//-3)/0.7)*100;

  }
}


uint8_t writeGPSRecordToString(uint8_t  *str)
  {
    str[1]=lastGPSRecord.nodeId>>24;str[2]=lastGPSRecord.nodeId>>16;str[3]=lastGPSRecord.nodeId>>8;str[4]=lastGPSRecord.nodeId;
    str[5]=lastGPSRecord.hour;str[6]=lastGPSRecord.minute;str[7]=lastGPSRecord.seconds;
    str[8]=lastGPSRecord.year;str[9]=lastGPSRecord.month;str[10]=lastGPSRecord.day;
    int32_t temp=lastGPSRecord.latitudeDegrees*100000;
    str[11]=temp>>24;str[12]=temp>>16;str[13]=temp>>8;str[14]=temp;
    temp=lastGPSRecord.longitudeDegrees*100000;
    str[15]=temp>>24;str[16]=temp>>16;str[17]=temp>>8;str[18]=temp;
    int16_t temp2=lastGPSRecord.altitude*10;
    str[19]=temp2>>8;str[20]=temp2;
    temp2=lastGPSRecord.speed*100;
    str[21]=temp2>>8;str[22]=temp2;
    str[23]=lastGPSRecord.satellites;
    str[24]=lastGPSRecord.RSSI;
    str[25]=lastGPSRecord.battery*10;
    return 26;
}
// converts lat/long from Adafruit
// degree-minute format to decimal-degrees
double convertDegMinToDecDeg (float degMin) {
  double min = 0.0;
  double decDeg = 0.0;
 
  //get the minutes, fmod() requires double
  min = fmod((double)degMin, 100.0);
 
  //rebuild coordinates in decimal degrees
  degMin = (int) ( degMin / 100 );
  decDeg = degMin + ( min / 60 );
 
  return decDeg;
}

uint8_t rssiToPercentage(float _rssi )
{
  if (_rssi <= -100)  { return 0;} else if (_rssi >= -50) {return 100;}else{return ( 2 * (_rssi + 100)); } return 0;
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

