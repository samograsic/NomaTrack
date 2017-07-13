#include <Adafruit_GPS.h>
#include <SPI.h>
#include <RH_RF95.h>

#define NODEID 0x0001

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

#define MINDELAY 100
#define MAXDELAY 1000
#define RESPONSETIMEOUT 10000
#define RADIOTIMESLOT 2000

#define GPSSerial Serial1

/* Feather m0 w/wing
  #define RFM95_RST     11   // "A"
  #define RFM95_CS      10   // "B"
  #define RFM95_INT     6    // "D"
*/


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

GPSRecord lastGPSRecord;
//lastGPSRecord.nodeId=1;

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 434.0

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
uint8_t writeGPSRecordToString( GPSRecord & value, uint8_t  *str)
  {
    uint8_t *p = ( uint8_t*) &value;
    //const uint8_t *strPtr=
    unsigned int i;
    *str=0x04;
    str++;
    for (i = 0; i < sizeof value; i++)
    {      
      *str = (*p++); 
      str++; 
    }
    return i+1;
}
 
void setup()
{
  //LORA Init
  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  while (!Serial);
  Serial.begin(115200);
  delay(100);
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
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

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

void loop()
{
  while(rf95.available()==false)
  {
    //We wait for a radio call
   // readGPS();
  }
  //After we check if it a valid radio all
  //LORA Stuff
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  if (rf95.recv(buf, &len))
  {
      //Debug
      digitalWrite(LED, HIGH);
      RH_RF95::printBuffer("Received: ", buf, len);
      Serial.print("Got: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.print(rf95.lastRssi(), DEC); Serial.print(" ");
      Serial.print(rssiToPercentage(rf95.lastRssi()), DEC); Serial.println("%");
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
    Serial.print("\nTime: ");
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
    }
  //Fill in lastGPSRecord
  lastGPSRecord.hour=GPS.hour;
  lastGPSRecord.minute=GPS.minute;
  lastGPSRecord.seconds=GPS.seconds;
  lastGPSRecord.year=GPS.year;
  lastGPSRecord.month=GPS.month;
  lastGPSRecord.day=GPS.day;
  lastGPSRecord.speed=GPS.speed;
  lastGPSRecord.latitudeDegrees=GPS.lat;
  lastGPSRecord.longitudeDegrees=GPS.lon;
  lastGPSRecord.altitude=GPS.altitude;
  lastGPSRecord.satellites=GPS.satellites;
  }
}


int rssiToPercentage(float _rssi )
{
  if (_rssi <= -100)
  {
    return 0;
  }
  else if (_rssi >= -50)
  {
    return 100;
  }
  else
  {
    return ( 2 * (_rssi + 100));
  }
  return 0;
}
