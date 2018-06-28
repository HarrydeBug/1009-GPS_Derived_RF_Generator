//Software for Zachtek "GPS Referenced RF generator 1MHz Out" with a 8MHz Mini Pro Arduno
//This will program an NEO-6 Gps module at startup to output an RF signal when locked.
//The NEO-6 has an internal 48MHz PLL that can be diveded down to any freq below 10MHz. 
//Divide to a frequency that is an integer fraction of 48MHz to get the lowest jitter.
//Examples off low jitter freqencies is 1, 2, 4 and 8 MHz  
//To compile you need to install the Library "NeoGps by SlashDevin", you can download it using The Library manager in the Arduino IDE
//ZachTec 2017-2018
//The Version of this software is stored in the constant "softwareversion" and is displayed on the Serialport att startup
//For Arduino Pro Mini.

#include <NMEAGPS.h>
#include <SoftwareSerial.h>
SoftwareSerial gpsPort(2, 3); // RX, TX
#define LEDIndicator1 5  //LED indicator for GPS Lock on pin A3
#define FIXOut 4         //Pin Out at IDC. Indicator for GPS Lock on pin 4
#define LDO_Enable A3    //GPS Voltage regulator Enable on pin A3
boolean GPSOK;
const char softwareversion[] = "1.05" ; //Version of this program, sent to serialport at startup
NMEAGPS  gps; // This parses the GPS characters
gps_fix  fix; // This holds on to the latest values



//--------------------------  SETUP -----------------------

void setup()
{
  Serial.begin(9600);
  while (!Serial);
  Serial.println("");
  Serial.print(F("Zachtek GPS referenced RF, Software version: "));
  Serial.println(softwareversion);

  pinMode(LDO_Enable, OUTPUT); // Set Voltage Regulator Enable pin as output.

  //Blink the Lock led
  Serial.println (F("Blinking Lock  LED"));
  pinMode(LEDIndicator1, OUTPUT); // Set GPS Lock  LED pin as output.
  for (int i = 0; i <= 17; i++) {
    digitalWrite(LEDIndicator1, HIGH); //Turn on Lock LED
    delay (60);
    digitalWrite(LEDIndicator1, LOW); //Turn off Lock LED
    delay (60);
  }

  //Turn on Power to GPS
  digitalWrite(LDO_Enable, HIGH); //Turn on 3.1V Power supply for the Ublox GPS module
  Serial.println (F("Turning on Voltage Regulator for GPS module"));

  pinMode(FIXOut, OUTPUT);        // Set GPS Lock line as output.
  digitalWrite(FIXOut, LOW);        //Go low on Lock line

  //Program GPS
  Serial.println (F("Programming GPS"));
  delay(500);//Wait for GPSmodule to complete it's power on.
  gpsPort.begin(9600);
  GPSOK = true;
  delay(500);//Wait for GPSmodule to complete it's power on.

  //Program GPS to output RF
  if (setGPS_OutputFreq1MHz()) {
    Serial.println ("GPS Initialized to output RF at 1MHz");
    Serial.println ("Initialization is complete.");
    Serial.println ("");
    GPSOK = true;
  }
  else
  {
    Serial.println ("Error! Could not program GPS!");
    GPSOK = false;
  }
}

//--------------------------




//--------------------------  Main loop -----------------------
void loop()
{
  while (gps.available( gpsPort )) {
    fix = gps.read();
    if (fix.valid.location && fix.valid.date && fix.valid.time)
    {
      Serial.print( F("Fix - Location: ") );
      digitalWrite(LEDIndicator1, HIGH);   // turn the LED on
      digitalWrite(FIXOut, HIGH);          // Set Lock Line high
      Serial.print( fix.latitude(), 6 );
      Serial.print( ',' );
      Serial.print( fix.longitude(), 6 );
      Serial.println();

    }
    else
    {
      if (GPSOK) { //If the GPS is connected but not locked then short blink
        digitalWrite(LEDIndicator1, HIGH);   // turn the LED on
        digitalWrite(FIXOut, LOW);          // Set Lock Line low
        delay(100);
        digitalWrite(LEDIndicator1, LOW);    // turn the LED off
        Serial.println(F("Waiting for GPS location fix"));
      }
    }
  }
}


//--------------------------

bool setGPS_OutputFreq100kHz()
{
  int gps_set_sucess = 0;
  uint8_t setOutputFreq[] = {
    0xB5, 0x62, 0x06, 0x31, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0x00, 0x00, 0x00, 0x01, 0x00,
    0x00, 0x00, 0xA0, 0x86, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00,
    0x00, 0x00, 0xEF, 0x00, 0x00, 0x00, 0x20, 0x1B
  };

  sendUBX(setOutputFreq, sizeof(setOutputFreq) / sizeof(uint8_t));
  gps_set_sucess = getUBX_ACK(setOutputFreq);
  //Serial.println("Set output Freq Done");
  return gps_set_sucess;
}

bool setGPS_OutputFreq1MHz()
{
  int gps_set_sucess = 0;
  uint8_t setOutputFreq[] = {
    0xB5, 0x62, 0x06, 0x31, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0x00, 0x00, 0x00, 0x01, 0x00,
    0x00, 0x00, 0x40, 0x42, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00,
    0x00, 0x00, 0xEF, 0x00, 0x00, 0x00, 0x8A, 0x8B
  };

  sendUBX(setOutputFreq, sizeof(setOutputFreq) / sizeof(uint8_t));
  gps_set_sucess = getUBX_ACK(setOutputFreq);
  //Serial.println("Set output Freq Done");
  return gps_set_sucess;
}

bool setGPS_OutputFreq2MHz()
{
  int gps_set_sucess = 0;
  uint8_t setOutputFreq[] = {
    0xB5, 0x62, 0x06, 0x31, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0x00, 0x00, 0x00, 0x01, 0x00,
    0x00, 0x00, 0x80, 0x84, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00,
    0x00, 0x00, 0xEF, 0x00, 0x00, 0x00, 0x1B, 0x7F
  };

  sendUBX(setOutputFreq, sizeof(setOutputFreq) / sizeof(uint8_t));
  gps_set_sucess = getUBX_ACK(setOutputFreq);
  //Serial.println("Set output Freq Done");
  return gps_set_sucess;
}


bool setGPS_OutputFreq4MHz()
{
  int gps_set_sucess = 0;
  uint8_t setOutputFreq[] = {
    0xB5, 0x62, 0x06, 0x31, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0x00, 0x00, 0x00, 0x01, 0x00,
    0x00, 0x00, 0x00, 0x09, 0x3D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00,
    0x00, 0x00, 0xEF, 0x00, 0x00, 0x00, 0x3F, 0x8C
  };

  sendUBX(setOutputFreq, sizeof(setOutputFreq) / sizeof(uint8_t));
  gps_set_sucess = getUBX_ACK(setOutputFreq);
  //Serial.println("Set output Freq Done");
  return gps_set_sucess;
}

//8MHz is the highest low-jitter frequency possible
bool setGPS_OutputFreq8MHz()
{
  int gps_set_sucess = 0;
  uint8_t setOutputFreq[] = {
    0xB5, 0x62, 0x06, 0x31, 0x20, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
    0x00, 0x00, 0x00, 0x12, 0x7A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00,
    0x00, 0x00, 0x6F, 0x00, 0x00, 0x00, 0xD4, 0x28
  };

  sendUBX(setOutputFreq, sizeof(setOutputFreq) / sizeof(uint8_t));
  gps_set_sucess = getUBX_ACK(setOutputFreq);
  //Serial.println("Set output Freq Done");
  return gps_set_sucess;
}

//10 MHz is very jittery. Numbers that can be done with an integer division from 48MHz will produce
//the lowest jitter so 16 ,12 ,8 ,6 ,4 ,2 and 1 MHz is low jitter but 10MHz is not
//If 10MHz low jitter is needed then one option is to output 2MHz and then filter out the 5th overtone arriving at 10MHz in that way.
bool setGPS_OutputFreq10MHz()
{
  int gps_set_sucess = 0;
  uint8_t setOutputFreq[] = {
    0xB5, 0x62, 0x06, 0x31, 0x20, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
    0x00, 0x00, 0x80, 0x96, 0x98, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00,
    0x00, 0x00, 0x6F, 0x00, 0x00, 0x00, 0xF6, 0x10
  };

  sendUBX(setOutputFreq, sizeof(setOutputFreq) / sizeof(uint8_t));
  gps_set_sucess = getUBX_ACK(setOutputFreq);
  //Serial.println("Set output Freq Done");
  return gps_set_sucess;
}

//16MHz is above the specs for lUblox Neo-6, only included for experiments.
//This will not produce as clean Square wave.
bool setGPS_OutputFreq16MHz()
{
  int gps_set_sucess = 0;
  uint8_t setOutputFreq[] = {
    0xB5, 0x62, 0x06, 0x31, 0x20, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
    0x00, 0x00, 0x00, 0x24, 0xF4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00,
    0x00, 0x00, 0x6F, 0x00, 0x00, 0x00, 0x60, 0x12
  };

  sendUBX(setOutputFreq, sizeof(setOutputFreq) / sizeof(uint8_t));
  gps_set_sucess = getUBX_ACK(setOutputFreq);
  //Serial.println("Set output Freq Done");
  return gps_set_sucess;
}


void sendUBX(uint8_t *MSG, uint8_t len) {
  gpsPort.flush();
  gpsPort.write(0xFF);
  _delay_ms(500);
  for (int i = 0; i < len; i++) {
    gpsPort.write(MSG[i]);
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
  for (uint8_t ubxi = 2; ubxi < 8; ubxi++) {
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
    if (gpsPort.available()) {
      b = gpsPort.read();
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
