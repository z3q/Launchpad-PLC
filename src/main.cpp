/*
Launchpad/msp430g2452 pinout
    VCC  1|V|20 GND
Контролл 2| |19 реле
Пов.вх.  3| |18 реле
ШИМ.вых  4| |17xxxx
торм.анал5| |16xxxx
Зажиг.вх 6| |15 SPI MISO
SPI SCK  7| |14 SPI MOSI
SPI CS   8| |13 тормоз.эл.реле
Пов.лев. 9| |12 фары.реле
Пов.пр. 10|_|11 стоп.реле
*/

#define FINAL // TEST or FINAL version

#include <Energia.h>
#include "Mfrc522.h"
#include <SPI.h>
#include "FireTimer.h"

#ifdef TEST
#include "testkey.h"
#else
#include "secretkey.h"
#endif

// Pin definitions:
// 1 - VCC
#define CTRLR_PIN 2  // motor contorller OUTPUT =RED_LED
#define TURN_PIN 3   // turn signal analog INPUT  //serial TX
#define PWM_PIN 4    // PWM OUTPUT                //serial RX   //use only when SPI communication finished
#define BREAKS_PIN 5 // break analog INPUT  =PUSH2
#define IGN_PIN 6    // ignition switch analog INPUT
// 7 - SPI clock
#define CS_PIN 8         // SPI chip select pin
#define TURNLEFT_PIN 9   // turn signal left OUTPUT
#define TURNRIGHT_PIN 10 // turn signal right OUTPUT
#define STOP_PIN 11      // stop signal OUTPUT
#define LIGHTS_PIN 12    // lights OUTPUT
#define ELBREAK_PIN 13   // electric break activation OUTPUT
// 14 - SPI MOSI  =GREEN_LED
// 15 - SPI MISO
// 16 - RESET
// 17 - TEST
#define RELAY_PIN 18    // relay OUTPUT
#define DIGINPUT_PIN 19 // digital INPUT
// 20 - GND

// output pins array
const uint8_t outPins[] = {CTRLR_PIN, PWM_PIN, TURNLEFT_PIN, TURNRIGHT_PIN, STOP_PIN, LIGHTS_PIN, ELBREAK_PIN, RELAY_PIN};
const uint8_t analogInPins[] = {TURN_PIN, BREAKS_PIN, IGN_PIN};
const uint8_t digitalInPins[] = {DIGINPUT_PIN};

FireTimer blink;
FireTimer blink3Hz;

Mfrc522 Mfrc522(CS_PIN); // contains pinMode(chipSelectPin, OUTPUT);
// boolean allowed = false;

/*void toggle_led()
{
  digitalWrite(RED_LED, !digitalRead(RED_LED)); // toggle the LED
  //return true;                                  // repeat? true
}
*/

boolean checkUID();

void setup()
{
  // nalogFrequency(2000);
  for (uint8_t i = 0; i < sizeof(outPins) / sizeof(outPins[0]); i++)
  {
    pinMode(outPins[i], OUTPUT);
    digitalWrite(outPins[i], LOW);
  }
  for (uint8_t i = 0; i < sizeof(analogInPins) / sizeof(analogInPins[0]); i++)
  {
    pinMode(analogInPins[i], INPUT_PULLUP);
  }
  pinMode(TURN_PIN, INPUT_PULLUP);
#ifdef TEST
  Serial.begin(9600);
  Serial.println("Starting communication\n");
#endif

  digitalWrite(CS_PIN, HIGH); // Stop SPI activity
  SPI.begin();
  blink.begin(2000);
  blink3Hz.begin(333);
  Mfrc522.Init();

  while (!checkUID())
    ;
}

void loop()
{
  // input vars
  uint16_t turnValue = analogRead(TURN_PIN);
  uint16_t breaksValue = analogRead(BREAKS_PIN);
  uint16_t ignValue = analogRead(IGN_PIN);
  boolean digInputValue = digitalRead(DIGINPUT_PIN);

  // main part

  // CTRLR_PIN

  // PWM_PIN,
  analogWrite(PWM_PIN, map(turnValue, 0, 1023, 0, 255));

  // TURNLEFT_PIN, TURNRIGHT_PIN,
  if (blink3Hz.fire())
  {
    if (turnValue >= 1000)
    {
      digitalWrite(TURNLEFT_PIN, LOW);
      digitalWrite(TURNRIGHT_PIN, LOW);
    }
    else if ((turnValue < 1000) && (turnValue >= 600))
    {
      digitalWrite(TURNLEFT_PIN, !digitalRead(TURNLEFT_PIN));
      digitalWrite(TURNRIGHT_PIN, LOW);
    }
    else if ((turnValue < 600) && (turnValue >= 200))
    {
      digitalWrite(TURNRIGHT_PIN, !digitalRead(TURNRIGHT_PIN));
      digitalWrite(TURNLEFT_PIN, LOW);
    }
    else
    {
      digitalWrite(TURNLEFT_PIN, !digitalRead(TURNLEFT_PIN));
      digitalWrite(TURNRIGHT_PIN, !digitalRead(TURNRIGHT_PIN));
    }
  }
  
  // STOP_PIN,

  // LIGHTS_PIN,

  // ELBREAK_PIN,

  // RELAY_PIN

}

// returns TRUE only if a card with valid UID recognized
boolean checkUID()
{

  boolean rfidStatus = false;
  unsigned char str[MAX_LEN];
  rfidStatus = (Mfrc522.Request(PICC_REQIDL, str) == MI_OK && Mfrc522.Anticoll(str) == MI_OK);
  Mfrc522.Halt();
  if (rfidStatus)
  {
    if (memcmp(str, validUID, sizeof(validUID)) == 0) // UID check
    {
#ifdef TEST
      Serial.println("Hello 007\n");
      delay(10);
#endif
      blink.start();
      Mfrc522.AntennaOff();
      return true;
    }
    else
    {
#ifdef TEST
      Serial.println("SPECTRE attempting entry!\n");
      delay(10);
#endif
      for (unsigned char i = 0; i < 8; i++)
      {
        digitalWrite(RED_LED, !digitalRead(RED_LED));
        sleep(80);
      }
      digitalWrite(RED_LED, LOW);
    }
  }
  return false;
}