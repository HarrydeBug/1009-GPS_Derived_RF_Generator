
/* Software for Zachtek "GPS referenced RF"
   
   Arduino software that takes incoming data on serial port or I2C and displays it on LED display.
   The Version of this software is stored in the constant "softwareversion" and is displayed on the Serialport att startup
   For Arduino Pro Mini.

   Arduino Output pin 2-9 drives 7 segment LEDs.
   All 7 segment digits are coupled in paralell. 7 segment is common cathode.
   output 10,11,12 are counter output to a 74HC238 3 to 8 demultiplexer that sinks the common cathodes one at a time via a FET transistor for each 7 segment digit.
   output 13 is enable to the 74HC238, it needs to be pulsed every once in while to keep the LED lighted. This a safeguard to make sure a LED is not
   getting overcurrent in case of stuck software or unprogramed Arduinos pulling or sinking random outputs.
   For Arduino Pro Mini.
*/

// Test 2 för att ställa in parametrar på U-blox NEO6 GPS modul med serieport
#include <SoftwareSerial.h>
#include <MicroNMEA.h>
#define LEDIndicator1 5  //LED to indicator for GPS Lock on pin A3
#define LDO_Enable A3    //GPS Voltage regulator Enable on pin A0
SoftwareSerial GPSSerial(2, 3); // RX, TX
const char softwareversion[] = "0.3" ; //Version of this program, sent to serialport at startup

char NMEAbuffer[500];
int  bufferIndex = 0; 
unsigned long LastCheck, LastConfig;
MicroNMEA nmea(NMEAbuffer, sizeof(NMEAbuffer));
Stream& console = Serial;
boolean GPSOK;

 
void setup() {
  GPSSerial.begin(9600);
  Serial.begin (57600);
  delay(500);//Wait for Serialport to be initialized properly
  Serial.print(F("Zachtek GPS referenced RF, Software version: "));
  Serial.println(softwareversion);
   
   Serial.println(F("Initializing.."));
   pinMode(LDO_Enable, OUTPUT); // Set Voltage Regulator Enable pin as output.
   Serial.println (F("Turning on Voltage Regulator for GPS module"));
   digitalWrite(LDO_Enable, HIGH); //Turn on 3.1V Power supply for the Ublox GPS module

  pinMode(LEDIndicator1, OUTPUT); // Set GPS Lock  LED pin as output.
  digitalWrite(LEDIndicator1, LOW); //Turn off Lock LED 
  delay(250);//Wait for GPSmodule to complete it's initialization.

 //Send command to GPS that its RF output will be Zero at GPS unlock and GPS referenced 10.000 MHz at GPS Lock
 if (setGPS_OutputFreq1MHz()) {
  GPSOK=true;
    Serial.println ("GPS module detected OK"); 
    Serial.println ("RF Output programed for 2MHz (Output will be off until GPS Lock)");
    Serial.println ("Initialization is complete.");
    Serial.println ("");
   
  }
  else
  {
    Serial.println ("Error! Could not connect to GPS!"); 
  GPSOK=false;  
  }
 LastConfig=millis();
}

void loop() {
 if(GPSSerial.available())
 {
   char ch = GPSSerial.read();
   nmea.process(ch);//send to NMEA parsing routine
   if( bufferIndex>499) //
   {
     NMEAbuffer[ bufferIndex ] = 0; // terminate the string with a 0     
     bufferIndex = 0;  // reset the index ready for another string
   }
   else
     NMEAbuffer[ bufferIndex++ ] = ch; // add the character into the buffer
 }

/*
if ((millis()-LastConfig) >60000) { //Recheck connection to GPS and configure it every minute.
  LastConfig=millis();
  Serial.println ("Reinitialising the 10MHz Output just in case");
   
  if (setGPS_OutputFreq10MHz()) {
    GPSOK=true;
  }
  else
  {
    GPSOK=false;  
    Serial.println ("Error! Could not connect to GPS!"); 
  }
}

*/
//Read the GPS Serial port every second and parse the data to be able detect any loss of signal
 if ((millis()-LastCheck) >1000) { 
  LastCheck=millis();
    if (nmea.isValid()) {
      digitalWrite(LEDIndicator1, HIGH);   // turn the LED on 
     
    }
    else
     {
      if (GPSOK) { //If the GPS is connected but not locked then short blink
        digitalWrite(LEDIndicator1, HIGH);   // turn the LED on 
        delay(100); 
        digitalWrite(LEDIndicator1, LOW);    // turn the LED off  
     }
    }
  
  if (GPSOK) {
    console.print("Valid fix: ");
    console.println(nmea.isValid() ? "yes" : "no");
    if (nmea.isValid())
     {
      console.print("Nav. system: ");
      if (nmea.getNavSystem())
        console.println(nmea.getNavSystem());
      else
        console.println("none");

      console.print("Num. satellites: ");
      console.println(nmea.getNumSatellites());
   
      console.print("UTC: ");
      console.print(int(nmea.getHour()));
      console.print(':');
      console.print(int(nmea.getMinute()));
      console.print(':');
      console.println(int(nmea.getSecond()));

      long latitude_mdeg = nmea.getLatitude();
      long longitude_mdeg = nmea.getLongitude();
      console.print("Latitude (deg): ");
      console.println(latitude_mdeg / 1000000., 6);

      console.print("Longitude (deg): ");
      console.println(longitude_mdeg / 1000000., 6);

      long alt;
      console.print("Altitude (m): ");
      if (nmea.getAltitude(alt))
        console.println(alt / 1000., 3);
      else
        console.println("not available");
      nmea.clear();
    }
  }
 } 
}




bool setGPS_OutputFreq100kHz()
{
 int gps_set_sucess=0;
 uint8_t setOutputFreq[] = {
 0xB5, 0x62, 0x06, 0x31, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0x00, 0x00, 0x00, 0x01, 0x00,
 0x00, 0x00, 0xA0, 0x86, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00,
 0x00, 0x00, 0xEF, 0x00, 0x00, 0x00, 0x20, 0x1B };

 sendUBX(setOutputFreq, sizeof(setOutputFreq)/sizeof(uint8_t));
 gps_set_sucess=getUBX_ACK(setOutputFreq);
 //Serial.println("Set output Freq Done");
 return gps_set_sucess;
}

bool setGPS_OutputFreq1MHz()
{
 int gps_set_sucess=0;
 uint8_t setOutputFreq[] = {
 0xB5, 0x62, 0x06, 0x31, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0x00, 0x00, 0x00, 0x01, 0x00,
 0x00, 0x00, 0x40, 0x42, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00,
 0x00, 0x00, 0xEF, 0x00, 0x00, 0x00, 0x8A, 0x8B };

 sendUBX(setOutputFreq, sizeof(setOutputFreq)/sizeof(uint8_t));
 gps_set_sucess=getUBX_ACK(setOutputFreq);
 //Serial.println("Set output Freq Done");
 return gps_set_sucess;
}

bool setGPS_OutputFreq2MHz()
{
 int gps_set_sucess=0;
 uint8_t setOutputFreq[] = {
 0xB5, 0x62, 0x06, 0x31, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0x00, 0x00, 0x00, 0x01, 0x00,
 0x00, 0x00, 0x80, 0x84, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00,
 0x00, 0x00, 0xEF, 0x00, 0x00, 0x00, 0x1B, 0x7F };

 sendUBX(setOutputFreq, sizeof(setOutputFreq)/sizeof(uint8_t));
 gps_set_sucess=getUBX_ACK(setOutputFreq);
 //Serial.println("Set output Freq Done");
 return gps_set_sucess;
}

bool setGPS_OutputFreq4MHz()
{
 int gps_set_sucess=0;
 uint8_t setOutputFreq[] = {
 0xB5, 0x62, 0x06, 0x31, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0x00, 0x00, 0x00, 0x01, 0x00,
 0x00, 0x00, 0x00, 0x09, 0x3D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00,
 0x00, 0x00, 0xEF, 0x00, 0x00, 0x00, 0x3F, 0x8C };

 sendUBX(setOutputFreq, sizeof(setOutputFreq)/sizeof(uint8_t));
 gps_set_sucess=getUBX_ACK(setOutputFreq);
 //Serial.println("Set output Freq Done");
 return gps_set_sucess;
}

//8MHz is the highest low-jitter frequency possible
bool setGPS_OutputFreq8MHz()
{
 int gps_set_sucess=0;
 uint8_t setOutputFreq[] = {
 0xB5, 0x62, 0x06, 0x31, 0x20, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
 0x00, 0x00, 0x00, 0x12, 0x7A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00,
 0x00, 0x00, 0x6F, 0x00, 0x00, 0x00, 0xD4, 0x28 };

 sendUBX(setOutputFreq, sizeof(setOutputFreq)/sizeof(uint8_t));
 gps_set_sucess=getUBX_ACK(setOutputFreq);
 //Serial.println("Set output Freq Done");
 return gps_set_sucess;
}

//10 MHz is very jittery. Frequencys that are an integer division of 48MHz will produce
//the lowest jitter.
//If 10MHz low jitter is needed an option is to output 2MHz and filter out the 5th overtone from it arriving at 10MHz
bool setGPS_OutputFreq10MHz()
{
 int gps_set_sucess=0;
 uint8_t setOutputFreq[] = {
 0xB5, 0x62, 0x06, 0x31, 0x20, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
 0x00, 0x00, 0x80, 0x96, 0x98, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00,
 0x00, 0x00, 0x6F, 0x00, 0x00, 0x00, 0xF6, 0x10 };

 sendUBX(setOutputFreq, sizeof(setOutputFreq)/sizeof(uint8_t));
 gps_set_sucess=getUBX_ACK(setOutputFreq);
 //Serial.println("Set output Freq Done");
 return gps_set_sucess;
}

//Ublox Neo-6 is not accurate or stable above 10MHz so only use this in experiments. 
//This will not produce as clean Square wave as lower frequencys.
bool setGPS_OutputFreq16MHz()
{
 int gps_set_sucess=0;
 uint8_t setOutputFreq[] = {
 0xB5, 0x62, 0x06, 0x31, 0x20, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
 0x00, 0x00, 0x00, 0x24, 0xF4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00,
 0x00, 0x00, 0x6F, 0x00, 0x00, 0x00, 0x60, 0x12 };

 sendUBX(setOutputFreq, sizeof(setOutputFreq)/sizeof(uint8_t));
 gps_set_sucess=getUBX_ACK(setOutputFreq);
 //Serial.println("Set output Freq Done");
 return gps_set_sucess;
}

void sendUBX(uint8_t *MSG, uint8_t len) {
 GPSSerial.flush();
 GPSSerial.write(0xFF);
 _delay_ms(500);
 for(int i=0; i<len; i++) {
 GPSSerial.write(MSG[i]);
 }
}

boolean getUBX_ACK(uint8_t *MSG) {
 uint8_t b;
 uint8_t ackByteID = 0;
 uint8_t ackPacket[10];
 unsigned long startTime = millis();
 
// Construct the expected ACK packet
 ackPacket[0] = 0xB5; // header
 ackPacket[1] = 0x62; // header
 ackPacket[2] = 0x05; // class
 ackPacket[3] = 0x01; // id
 ackPacket[4] = 0x02; // length
 ackPacket[5] = 0x00;
 ackPacket[6] = MSG[2]; // ACK class
 ackPacket[7] = MSG[3]; // ACK id
 ackPacket[8] = 0; // CK_A
 ackPacket[9] = 0; // CK_B
 
// Calculate the checksums
 for (uint8_t ubxi=2; ubxi<8; ubxi++) {
 ackPacket[8] = ackPacket[8] + ackPacket[ubxi];
 ackPacket[9] = ackPacket[9] + ackPacket[8];
 }
 
  while (1) {  // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      return true;
    }
 
  // Timeout if no valid response in 3 seconds
  if (millis() - startTime > 3000) {
    return false;
  }
 
  // Make sure data is available to read
  if (GPSSerial.available()) {
    b = GPSSerial.read();
    // Check that bytes arrive in sequence as per expected ACK packet
    if (b == ackPacket[ackByteID]) {
      ackByteID++;
    }
    else {
      ackByteID = 0; // Reset and look again, invalid order
    }//else
  }//If
 }//While
}//getUBX_ACK



