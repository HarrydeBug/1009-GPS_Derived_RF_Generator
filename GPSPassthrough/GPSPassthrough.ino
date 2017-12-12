/*Software for Zachtek "GPS Referenced RF Generator" 
   
   Arduino software that just passes serial data between the Arduino Serial port and the Ublox Neo-6 onboard GPS module.
   This can be useful in trouble shooting scenarias, expriments etc but the main use would be to be able to controll the GPS module from the PC based U-Center software.
   Arduino pin A0 is output - Voltage Regulator Enable
           pin 2 and 3 is Sofware serial port to GPS module
           pin 4 is output  - connected to the IDC connector
           pin 5 is output  - Yellow LED indicator
  
   For Arduino Pro Mini 8MHz
*/


#include <SoftwareSerial.h>
#include <MicroNMEA.h>
#define LEDIndicator1 5  //LED to indicator for GPS Lock on pin A3
#define LDO_Enable A3    //GPS Voltage regulator Enable on pin A0
SoftwareSerial GPSSerial(2, 3); // RX, TX
const char softwareversion[] = "0.1" ; //Version of this program, sent to serialport at startup

char NMEAbuffer[500];
int  bufferIndex = 0; 
unsigned long LastCheck, LastConfig;
MicroNMEA nmea(NMEAbuffer, sizeof(NMEAbuffer));
Stream& console = Serial;
boolean GPSOK;

 
void setup() {
  GPSSerial.begin(9600);
  Serial.begin (9600);
  delay(100);//Wait for Serialport to be initialized properly
 
   pinMode(LDO_Enable, OUTPUT); // Set Voltage Regulator Enable pin as output.
   digitalWrite(LDO_Enable, HIGH); //Turn on 3.1V Power supply for the Ublox GPS module

   pinMode(LEDIndicator1, OUTPUT); // Set GPS Lock  LED pin as output.
   digitalWrite(LEDIndicator1, LOW); //Turn off Lock LED 
   delay(250);//Wait for GPSmodule to complete it's power on.
}

void loop() {
 
  if (GPSSerial.available()) // If GPS data is available
    Serial.write(GPSSerial.read()); // Read it and print to SerialMonitor
  if (Serial.available()) // If SerialMonitor data is available
    GPSSerial.write(Serial.read()); // Read it and send to GPS
}



