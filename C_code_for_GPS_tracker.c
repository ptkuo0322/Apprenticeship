#include <STM32L0.h>
#include <TinyGPS++.h>
#include "LoRaWAN.h"
#include "TimerMillis.h"
#include <CayenneLPP.h>
#include <MicroNMEA.h>

//fill in you DEVEUI/ APPEUI/ APPKEY in MSB form
const char *devEui = "fill_in_data";
const char *appEui = "fill_in_data"; 
const char *appKey = "fill_in_data"; 

//By changing the duration of the interval, the update period would thus be changed
const uint32_t TX_INTERVAL =20000; // 60 Seconds 
TimerMillis timer_send;

float longitude_mdeg;
float latitude_mdeg;

//Define Serial1 for STM32 Nucleo boards
#ifdef ARDUINO_ARCH_STM32
HardwareSerial Serial1(18, 19); //UART 23 24 rx tx
#endif

// Refer to serial devices by use
HardwareSerial &console = Serial;
HardwareSerial &gpscole = Serial1;

// GPS object to process the NMEA data
TinyGPSPlus gps;

// This indicate the maximum payload for the message
CayenneLPP lpp(51);
static volatile bool uplink_attempted;

void setupGPS() {
  console.begin(115200); // console
  Serial.println("Starting GPS Example...");

  gpscole.begin(9600); // gps
}
void readGPS() {
  //While a message is received and there are incoming characters from the GPS
  while(gpscole.available())              
  {
    //This feeds the serial NMEA data into the library one char at a time
    if (gps.encode(gpscole.read())){;           
  
      //This will pretty much be fired all the time anyway but will at least reduce it to only after a package of NMEA data comes in
      if(gps.location.isUpdated()){  

           // Clear Payload
           lpp.reset();

           latitude_mdeg = gps.location.lat();
           longitude_mdeg = gps.location.lng();

           //Add the GPS infomation to the payload and will be uploaded to TTN via LoRaWAN
           lpp.addGPS(1, latitude_mdeg, longitude_mdeg);
      }
    }
  }
}

//This function is implemented to upload the message to the TTN
void async_timer_send() {
  if (LoRaWAN.joined() && !LoRaWAN.busy()) {
    // Send Packet
    LoRaWAN.sendPacket(1, lpp.getBuffer(), lpp.getSize());
    uplink_attempted = true;
  }
}

// Create a byte array for send UBX protocol command to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++) {
    console.write(MSG[i]);
  }
}

// Set NEO 6M into backup mode and STM32L0 chip into standby mode for less power consumption
void setupTRACKERpower() {

 //Set GPS to backup mode (need to be waken up manually)
 uint8_t GPSoff[] = {0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x4D, 0x3B};
 sendUBX(GPSoff, sizeof(GPSoff)/sizeof(uint8_t));
 
 // Turn STM32LO chip into standby mode with RTC on only
 STM32L0.standby(19000); // modify the time here to match with the update period, suggest time difference : 1 second
 delay(19000); // modify the time here to match with the update period, suggest time difference : 1 second
 
 //Manually restart GPS to get the fix
 uint8_t GPSon[] = {0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x4C, 0x37};
 sendUBX(GPSon, sizeof(GPSon)/sizeof(uint8_t));
}

void setup(void) {
  setupGPS();

  while (!Serial) {
  }

  // US Region
  LoRaWAN.begin(US915);
  // Helium SubBand
  LoRaWAN.setSubBand(2);
  // Disable Adaptive Data Rate
  LoRaWAN.setADR(false);
  // Set Data Rate 1 - Max Payload 53 Bytes
  LoRaWAN.setDataRate(1);
  // Device IDs and Key
  LoRaWAN.joinOTAA(appEui, appKey, devEui);

  Serial.println("JOIN( )");

  while (!LoRaWAN.joined() && LoRaWAN.busy()) {
    Serial.println("JOINING( )");
    delay(5000);
  }
  Serial.println("JOINED( )");

  // Start Continuous Uplink Timer
  timer_send.start(async_timer_send, 0, TX_INTERVAL);
}
int i = 0;

void loop(void) {
  if (uplink_attempted) {
   
    uplink_attempted = false;
    i=0;
  } 
  // Turn the Tracker into standby mode to lower the power consumption
  if (i==0){
   setupTRACKERpower();
    i++;
    }
  readGPS();
}