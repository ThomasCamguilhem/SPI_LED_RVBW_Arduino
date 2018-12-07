#include "Led.h"
#include "pins_arduino.h"

#define SPI_MISO D12
#define SPI_MOSI D11
#define SPI_SCK D13
#define SPI_SS D10

#define AMB_R 6
#define AMB_G 5
#define AMB_B 3
#define AMB_W 9

#define VER_R 16
#define VER_G 15
#define VER_B 14

unsigned int NB_ALIVE = 0;
int alive = 1000; // ms

Led Red, Green, Blue, White;
bool V_flash, V_Red, V_Green, V_Blue;
long transi = 0;

byte buf [100];
volatile byte pos;
volatile boolean process_it;

enum NetworkQuality {
  GOOD_CONNECTION = 0,
  UNSTABLE_CONNECTION = 1,
  POOR_CONNECTION = 2,
  BAD_CONNECTION = 3,
  NO_CONNECTION = 4
};
NetworkQuality networkStatus;

uint16_t fletcher16(const uint8_t *data, size_t len)
{
  uint32_t c0, c1;
  unsigned int i;
  for (c0 = c1 = 0; len >= 5802; len -= 5802) {
    for (i = 0; i < 5802; ++i) {
      c0 = c0 + *data++;
      c1 = c1 + c0;
    }
    c0 = c0 % 255;
    c1 = c1 % 255;
  }
  for (i = 0; i < len; ++i) {
    c0 = c0 + *data++;
    c1 = c1 + c0;
  }
  c0 = c0 % 255;
  c1 = c1 % 255;
  return (c1 << 8 | c0);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Booting up ...");
  Red.pin = AMB_R;
  Green.pin = AMB_G;
  Blue.pin = AMB_B;
  White.pin = AMB_W;

  /*Red.minPower = 10;
  Green.minPower = 10;
  Blue.minPower = 150;
  White.minPower = 150;
  */
  pinMode(AMB_R, OUTPUT);
  pinMode(AMB_G, OUTPUT);
  pinMode(AMB_B, OUTPUT);
  pinMode(AMB_W, OUTPUT);

  pinMode(VER_R, OUTPUT);
  pinMode(VER_G, OUTPUT);
  pinMode(VER_B, OUTPUT);

  digitalWrite(AMB_R, HIGH);
  digitalWrite(AMB_G, HIGH);
  digitalWrite(AMB_B, HIGH);
  digitalWrite(AMB_W, HIGH);
  digitalWrite(VER_R, HIGH);
  digitalWrite(VER_G, LOW);
  digitalWrite(VER_B, HIGH);
  networkStatus = GOOD_CONNECTION;
  White.SetMaxPower(150);
  White.SetDelay(5000);
  
  // have to send on master in, *slave out*
  pinMode(MISO, OUTPUT);
  // turn on SPI in slave mode
  SPCR |= _BV(SPE);
  // turn on interrupts
  SPCR |= _BV(SPIE);
  pos = 0;
  process_it = false;
  
  Serial.println("ready to receive as slave");
}


// SPI interrupt routine
ISR (SPI_STC_vect)
{
  byte c = SPDR;
  // add to buffer if room
  if (pos < sizeof buf)
  {
    buf [pos++] = c;
    // example: newline means time to process buffer
    if (c == '\n')
      process_it = true;
  }  // end of room available
}

void loop() {
  // put your main code here, to run repeatedly:
  //Securite();
  if (process_it)
  {
    reception();
    
    pos = 0;
    process_it = false;
  }

  UpdateLight();
  delay(10);
}

int safeGreen = 255;
int dangerRed = 255;
unsigned long lastTime = 0;
int connectionLifespan = 0;

void Securite() {
  if ((millis() - lastTime) > alive)
  {
    //Serial.print(millis() - lastTime);
    //Serial.println(": Pulse");
    if (NB_ALIVE == 0)
    {
      connectionLifespan ++;
      if (networkStatus == GOOD_CONNECTION)
      {
        analogWrite(AMB_G, 255);
        analogWrite(AMB_R, 51);
        if (connectionLifespan == 5)
        {
          networkStatus = UNSTABLE_CONNECTION;
          connectionLifespan = 0;
        }
      } else if (networkStatus == UNSTABLE_CONNECTION)
      {
        analogWrite(AMB_G, 204);
        analogWrite(AMB_R, 102);
        if (connectionLifespan == 5)
        {
          networkStatus = POOR_CONNECTION;
          connectionLifespan = 0;
        }
      } else if (networkStatus == POOR_CONNECTION)
      {
        analogWrite(AMB_G, 153);
        analogWrite(AMB_R, 153);
        if (connectionLifespan == 5)
        {
          networkStatus = BAD_CONNECTION;
          connectionLifespan = 0;
        }
      } else if (networkStatus == BAD_CONNECTION)
      {
        analogWrite(AMB_G, 102);
        analogWrite(AMB_R, 204);
        if (connectionLifespan == 5)
        {
          networkStatus = NO_CONNECTION;
          connectionLifespan = 0;
        }
      } else {
        analogWrite(AMB_G, 51);
        analogWrite(AMB_R, 255);
      }
      delay(100);
      Serial.println("ill");
    } else if (networkStatus != GOOD_CONNECTION) {
      if (connectionLifespan != 0) {
        connectionLifespan--;
      } else {
        switch (networkStatus)
        {
          case NO_CONNECTION:
            networkStatus = BAD_CONNECTION;
            break;
          case BAD_CONNECTION:
            networkStatus = POOR_CONNECTION;
            break;
          case POOR_CONNECTION:
            networkStatus = UNSTABLE_CONNECTION;
            break;
          case UNSTABLE_CONNECTION:
            networkStatus = GOOD_CONNECTION;
            break;
          default:
            networkStatus = NO_CONNECTION;
        }
      }
      //Serial.println("felling better");
    } else {
      //Serial.println(" healthy");
    }
    lastTime = millis();
    NB_ALIVE = 0;
  }
}

void reception ()
{
    unsigned long now = millis();
    int h = now / 3600000;
    int m = (now / 60000) % 60;
    int s = (now / 1000) % 60;
    int ms = now % 1000;
    String time = String(String("T+: ") + String((h < 10 ? "0" : "")) + String(h) + String(":")
                                               + (m < 10 ? "0" : "") + String(m) + ":"
                                               + (s < 10 ? "0" : "") + String(s) + " "
                                       + String((ms < 100 ? String("0") + String((ms < 10 ? "0" : "")) : "")) + String(ms));
     Serial.print(time);
    // the data is 4 bytes for the LED color (rgbw) 
    // + 1 byte for the lock+timing (3 bits(rgb)+4bits(0-15s)) 
    // + 2 bytes of checksum + 1 byte for the end code
   int index = 0;
   if(pos == 8) { 
      buf [pos] = 0;
      uint16_t csum = fletcher16(buf, 5);
      if (!(buf[5] == (csum & 0xFF)) // check for the data integrity
       || !(buf[6] == ((csum >> 8) & 0xFF))) {
        Serial.print(": Data err");
        Serial.print(": 0x");
        while (buf[index] != '\n') {
          if (buf[index] < 10)
            Serial.print(0, HEX);
          Serial.print(buf[index++], HEX);
        }
        Serial.println("");
      } else {
        //Serial.print("received ");
        //Serial.print(howMany);
        //Serial.println(" bytes of data");
        NB_ALIVE ++;
        //Valeur = 0;
        Serial.print(": 0x");
        White.SetMaxPower(buf[0]);    //Serial.println(out);
        Blue.SetMaxPower(buf[1]);     //Serial.println(out);
        Green.SetMaxPower(buf[2]);    //Serial.println(out);
        Red.SetMaxPower(buf[3]);      //Serial.println(out);
        byte out = buf[4];            //Serial.println(out);
        if (White.maxPower < 16)
          Serial.print("0");
        Serial.print(White.maxPower, HEX);
        if (Blue.maxPower < 16)
          Serial.print("0");
        Serial.print(Blue.maxPower, HEX);
        if (Green.maxPower < 16)
          Serial.print("0");
        Serial.print(Green.maxPower, HEX);
        if (Red.maxPower < 16)
          Serial.print("0");
        Serial.print(Red.maxPower, HEX);
        if (out < 16)
          Serial.print("0");
        Serial.print(out, HEX);
        V_flash = (out & 8);
        V_Red = (out & 4);
        V_Green = (out & 2);
        V_Blue = (out & 1);
        transi = (out & 0xF0);
        transi = (transi >> 4);
        Red.SetDelay((transi * 1000));
        Green.SetDelay((transi * 1000));
        Blue.SetDelay((transi * 1000));
        White.SetDelay((transi * 1000));
        Red.Transition();
        Green.Transition();
        Blue.Transition();
        White.Transition();
        Serial.println(";");
      }
    } else {
      Serial.print(": len err ");
      Serial.print(pos - 1);
      Serial.print(": 0x");
        while (buf[index] != '\n') {
          if (buf[index] < 10)
            Serial.print(0, HEX);
          Serial.print(buf[index++], HEX);
        }
        Serial.println("");
    }
}

void UpdateLight()
{
  if (Red.isTransition)
    Red.Transition();
  if (Green.isTransition)
    Green.Transition();
  if (Blue.isTransition)
    Blue.Transition();
  if (White.isTransition)
    White.Transition();
  digitalWrite(VER_R, !V_Red);
  digitalWrite(VER_G, !V_Green);
  digitalWrite(VER_B, !V_Blue);
}

