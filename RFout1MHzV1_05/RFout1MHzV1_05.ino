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
boolean GPSOK, passthrough;
int state; // serial command state machine
uint32_t freq; // Frequency in Hz commanded via serial
#define softwareversion "1.05" //Version of this program, sent to serialport at startup
NMEAGPS  gps; // This parses the GPS characters
gps_fix  fix; // This holds on to the latest values

constexpr auto MHz = 1000000lu;
constexpr auto KHz = 1000lu;
constexpr auto Hz = 1lu;

// This is a little bit arcane but it just lets you keep the user string
// and the actual value in synch automatically
#define SPEED 1
#define UNITS MHz
#define STRINGIFY1(x) #x
#define STRINGIFY(x) STRINGIFY1(x)
#define SPEED_STR STRINGIFY(SPEED) STRINGIFY(UNITS)
#define SPEED_ARG (UINT32_C(SPEED) * UNITS)

//--------------------------  SETUP -----------------------

void setup()
{
  Serial.begin(9600);
  while (!Serial);

  Serial.println(F(""));
  Serial.println(F("Zachtek GPS referenced RF, Software version: " softwareversion));

  if(strapped_for_passthrough()) {
    pinMode(LDO_Enable, OUTPUT); // Set Voltage Regulator Enable pin as output.
    digitalWrite(LDO_Enable, HIGH); //Turn on 3.1V Power supply for the Ublox GPS module
    gpsPort.begin(9600);
    passthrough = 1;
    Serial.println(F("Strapped for passthrough.  Disconnect jumper between SDA/SCL for normal operation"));
    return;
  }

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
  if (setGPS_OutputFreq(SPEED_ARG)) {
    Serial.println ("GPS Initialized to output RF at " SPEED_STR);
    Serial.println ("Initialization is complete.");
    Serial.println ("");
    GPSOK = true;
  }
  else
  {
    Serial.println (F("Error! Could not program GPS!"));
    GPSOK = false;
  }
}

//--------------------------


// Handle serial commands
void handle_serial() {
  int c = Serial.read();
  if(c == 'P') {
    Serial.println(F("Entering passthrough mode -- reset microcontroller to return to normal mode\n"));
    passthrough = true;
  }
  else if(c == 'F' || c == 'f') {
    Serial.print(F("Frequency?"));
    state = 1; freq = 0;
  }
  else if(state == 1) {
    if(c >= '0' && c <= '9') {
      Serial.write(c);
      freq = freq * 10 + c - '0';
    } else if (c == 'M' || c == 'm') {
      Serial.write(c);
      freq *= 1000000lu;
    } else if (c == 'K' || c == 'k') {
      Serial.write(c);
      freq *= 1000lu;
    } else if (c == '\n' || c == '\r') {
      Serial.println(F(""));
      if (setGPS_OutputFreq(freq)) {
        Serial.print ("GPS Initialized to output RF at " );
        Serial.println (freq);
        Serial.println ("Initialization is complete.");
        Serial.println ("");
        GPSOK = true;
      }
      else
      {
        Serial.println (F("Error! Could not program GPS!"));
        GPSOK = false;
      }
      state = 0;
    }
  }
}


//--------------------------  Main loop -----------------------
void loop()
{
  if(passthrough) {
    if(gpsPort.available()) Serial.write(gpsPort.read());
    if(Serial.available()) gpsPort.write(Serial.read());
    return;
  }
  if(Serial.available()) {
    handle_serial();
  }
  if(state != 0) return;
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

// For details of the UBX protocol, see
// https://www.u-blox.com/sites/default/files/products/documents/u-blox7-V14_ReceiverDescriptionProtocolSpec_%28GPS.G7-SW-12001%29_Public.pdf
// Documentation of this packet is under the heading CFG-TP5 (35.19.2) in the current documentation.
uint8_t setOutputFreq[] = {
0xB5, 0x62, 0x06, 0x31, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0x00, 0x00, 0x00, 0x01, 0x00,
0x00, 0x00, 0xA0, 0x86, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00,
0x00, 0x00, 0xEF, 0x00, 0x00, 0x00, 0x20, 0x1B
};

#define OFFSET_FREQUENCY_LOCKED (18)
#define OFFSET_CKSUM (38)

// Note: Normally payload_start is 2 bytes past the start of buf, because the first two bytes
// are the message type and are not checksummed.
void ubx_compute_checksum(uint8_t *payload_start, uint8_t *payload_end, uint8_t *cksum) {
  uint8_t ck_a=0, ck_b=0;
  for(const uint8_t *p = payload_start; p != payload_end; p++)
  {
    ck_a += *p;
    ck_b += ck_a;
  }
  cksum[0] = ck_a;
  cksum[1] = ck_b;
}

bool setGPS_OutputFreq(uint32_t freq) {
  for(int i=0; i<4; i++) {
    setOutputFreq[OFFSET_FREQUENCY_LOCKED+i] = freq & 0xff;
    freq >>= 8;
  }
  ubx_compute_checksum(setOutputFreq+2, setOutputFreq+38, setOutputFreq+38);
  sendUBX(setOutputFreq, sizeof(setOutputFreq) / sizeof(uint8_t));
  bool gps_set_sucess = getUBX_ACK(setOutputFreq);
  // Serial.println(F("Set output Freq Done"));
  return gps_set_sucess;
}

bool setGPS_OutputFreq1Kz()
{
  return setGPS_OutputFreq(1*KHz);
}

bool setGPS_OutputFreq1MHz()
{
  return setGPS_OutputFreq(1*MHz);
}

bool setGPS_OutputFreq2MHz()
{
  return setGPS_OutputFreq(2*MHz);
}


bool setGPS_OutputFreq4MHz()
{
  return setGPS_OutputFreq(4*MHz);
}

//8MHz is the highest low-jitter frequency possible
bool setGPS_OutputFreq8MHz()
{
  return setGPS_OutputFreq(8*MHz);
}

//10 MHz is very jittery. Numbers that can be done with an integer division from 48MHz will produce
//the lowest jitter so 16 ,12 ,8 ,6 ,4 ,2 and 1 MHz is low jitter but 10MHz is not
//If 10MHz low jitter is needed then one option is to output 2MHz and then filter out the 5th overtone arriving at 10MHz in that way.
bool setGPS_OutputFreq10MHz()
{
  return setGPS_OutputFreq(10*MHz);
}

//16MHz is above the specs for lUblox Neo-6, only included for experiments.
//This will not produce as clean Square wave.
bool setGPS_OutputFreq16MHz()
{
  return setGPS_OutputFreq(16*MHz);
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
  ubx_compute_checksum(ackPacket+2, ackPacket+8, ackPacket+8);

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

// If pins 7/8 on the 10-pin header are bridged, this is "strapped for passthrough"
// Detect this by driving one pin high and checking the other pin, then driving low and repeating
// set pins back to inputs before returning
bool strapped_for_passthrough() {
  bool result = true;
  pinMode(A4, OUTPUT);

  digitalWrite(A4, HIGH);
  delay(1);
  if(!digitalRead(A5)) result = false;

  digitalWrite(A4, LOW);
  pinMode(A5, INPUT_PULLUP);
  delay(1);
  if(digitalRead(A5)) result = false;

  pinMode(A4, INPUT);
  pinMode(A5, INPUT);

  return result;
}
