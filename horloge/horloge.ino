#include <Wire.h>
#include <ShiftLCD.h>
#include <RTClib.h>

ShiftLCD lcd(3, 6, 5);
RTC_DS1307 rtc;

bool rtc_ok = true;
char jourSemaine [7][3] = {"Di", "Lu", "Ma", "Me", "Je", "Ve", "Sa"};

byte iconeTemp [8] = //icon for termometer
{
  B00100,
  B01110,
  B01010,
  B01010,
  B01010,
  B01110,
  B11111,
  B01110
};


byte iconeAlarm [8] = {
  B00100,
  B00100,
  B01110,
  B01110,
  B01110,
  B11111,
  B11111,
  B00100
};

const int swqe = 2;
volatile unsigned short nbTick = 0;
volatile bool displayTime = false;

const unsigned short pinTemp = A0;

const unsigned short button = 7;
byte previousButtonState = HIGH; // car pull-up interne
const unsigned short push = 8;
byte previousPushState = HIGH; // car pull-up interne
const unsigned short onoff = 10;
byte previousOnoffState = HIGH;

const unsigned long debounceTime = 20;
unsigned long switchPressTime = 0;

short heures = 0;
short minutes = 0;

#define REGULAR_STATE 0
#define ALARM_STATE 1
#define SET_ALARM_HOUR 2
#define SET_ALARM_MINUTE 3

unsigned short state = REGULAR_STATE;

#define ALARM_UNSET 0
#define ALARM_SET 1
#define ALARM_ON 2

unsigned short alarmState = ALARM_UNSET;

const unsigned short buzzer = 9;

void rtcTick () {
  nbTick++;
  if (nbTick >= 60) {
    nbTick = 0;
    displayTime = true;
  }
}

float getTemperature (const unsigned short pin) {
  unsigned int sensor_data = 0;

  for (unsigned short i = 0; i < 8; i++) {
    unsigned int value = analogRead (pin);
    sensor_data = sensor_data + value;
    delay (10);
  }

  unsigned int sensor_value = (sensor_data / 8);
  float voltage = (sensor_value / 1024.0) * 5.0;
  float temperature = (voltage - 0.5) * 100;
  return temperature;
}

void setup() {
  pinMode (button, INPUT_PULLUP);
  pinMode (push, INPUT_PULLUP);
  pinMode (onoff, INPUT_PULLUP);

  Serial.begin (9600);

  lcd.begin (16, 2);
  lcd.createChar (1, iconeTemp);
  lcd.createChar (2, iconeAlarm);

  lcd.noCursor ();

  lcd.setCursor (9, 0);
  lcd.write ('-');
  lcd.setCursor (0, 1);
  lcd.write (1);
  lcd.setCursor (6, 1);
  lcd.write ((char)223);
  //lcd.setCursor (8, 1);
  //lcd.write (2);

  if (! rtc.begin ()) {
    Serial.println ("RTC absent");
    rtc_ok = false;
  } else {
    if (! rtc.isrunning()) {
      Serial.println ("Ajustement RTC");
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
  }
  //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  DateTime current = rtc.now ();
  char jour [10];
  char heure [6];
  sprintf (&jour[0], "%s %02d/%02d", jourSemaine[current.dayOfTheWeek()], current.day(), current.month());
  sprintf (&heure[0], "%02d:%02d", current.hour(), current.minute());
  lcd.setCursor (0, 0); lcd.print (jour);
  lcd.setCursor (11, 0); lcd.print (heure);

  float temperature = getTemperature (pinTemp);

  Serial.println (temperature);
  lcd.setCursor (2, 1);
  lcd.print (temperature, 1);
  // lcd.setCursor (13,0); lcd.blink ();

  for (int i = 0; i < 10; i++) {
    Serial.println(rtc.readnvram(i), HEX);
  }

  pinMode (swqe, INPUT_PULLUP);
  attachInterrupt (digitalPinToInterrupt (swqe), rtcTick, CHANGE);
  rtc.writeSqwPinMode (0x10);
}

void loop() {
  // lecture bouton on/off
  byte onoffState = digitalRead (onoff);
  if (onoffState != previousOnoffState) {
    if (millis() - switchPressTime > debounceTime) {
      switchPressTime = millis ();
      previousOnoffState = onoffState;
      if (onoffState == LOW) {
        Serial.println ("onoff ferme");
        switch (alarmState) {
          case ALARM_SET:
            lcd.setCursor (8, 1);
            lcd.write (" ");
            alarmState = ALARM_UNSET;
            break;
          case ALARM_ON:
            noTone (buzzer);
            alarmState = ALARM_SET;
        }
      }
    }
  }
  // lecture reglage
  byte pushState = digitalRead (push);
  if (pushState != previousPushState) {
    if (millis() - switchPressTime > debounceTime) {
      switchPressTime = millis ();
      previousPushState = pushState;
      if (pushState == LOW) {
        char data[3];
        Serial.println ("push ferme");
        switch (state) {
          case SET_ALARM_HOUR:
            heures++;
            if (heures > 23) {
              heures = 0;
            }
            lcd.setCursor (0, 1);
            sprintf (&data[0], "%02d", heures);
            lcd.print (data);
            break;
          case SET_ALARM_MINUTE:
            minutes++;
            if (minutes > 59) {
              minutes = 0;
            }
            lcd.setCursor (3, 1);
            sprintf (&data[0], "%02d", minutes);
            lcd.print (data);
            break;
        }
      }
    }
  }

  // lecture bouton
  byte buttonState = digitalRead (button);
  if (buttonState != previousButtonState) {
    if (millis() - switchPressTime > debounceTime) {
      switchPressTime = millis ();
      previousButtonState = buttonState;
      if (buttonState == LOW) {
        Serial.println ("bouton ferme");
        switch (state) {
          case REGULAR_STATE:
            state = SET_ALARM_HOUR;
            lcd.clear();
            lcd.setCursor (0, 0); lcd.print ("Alarme");
            char data[6];
            sprintf (&data[0], "%02d:%02d", heures, minutes);
            lcd.setCursor (0, 1);
            lcd.print (data);
            lcd.setCursor (0, 1); lcd.blink ();
            break;
          case SET_ALARM_HOUR:
            lcd.setCursor (3, 1); lcd.blink ();
            state = SET_ALARM_MINUTE;
            break;
          case SET_ALARM_MINUTE:
            lcd.clear ();
            lcd.noBlink ();
            lcd.setCursor (9, 0);
            lcd.write ('-');
            lcd.setCursor (0, 1);
            lcd.write (1);
            lcd.setCursor (6, 1);
            lcd.write ((char)223);

            // affichage icone alarme
            lcd.setCursor (8, 1);
            lcd.write (2);
            alarmState = ALARM_SET;

            DateTime current = rtc.now ();
            char jour [10];
            char heure [6];
            sprintf (&jour[0], "%s %02d/%02d", jourSemaine[current.dayOfTheWeek()], current.day(), current.month());
            sprintf (&heure[0], "%02d:%02d", current.hour(), current.minute());
            lcd.setCursor (0, 0); lcd.print (jour);
            lcd.setCursor (11, 0); lcd.print (heure);

            float temperature = getTemperature (pinTemp);
            lcd.setCursor (2, 1);
            lcd.print (temperature, 1);
            state = REGULAR_STATE;
            break;
        }
      } 
    }
  }

  if (state == REGULAR_STATE) {
    noInterrupts ();
    if (displayTime == true) {
      displayTime = false;
      interrupts ();

      DateTime current = rtc.now ();

      // verification alarme
      if (alarmState == ALARM_SET) {
        if ((current.hour () == heures) && (current.minute() == minutes)) {
          tone (buzzer, 440);
          alarmState = ALARM_ON;
        }
      }

      char jour [10];
      char heure [6];
      sprintf (&jour[0], "%s %02d/%02d", jourSemaine[current.dayOfTheWeek()], current.day(), current.month());
      sprintf (&heure[0], "%02d:%02d", current.hour(), current.minute());
      lcd.setCursor (0, 0); lcd.print (jour);
      lcd.setCursor (11, 0); lcd.print (heure);
      Serial.println (jour); Serial.println (heure); Serial.println (current.second());

      float temperature = getTemperature (pinTemp);
      Serial.println (temperature);
      lcd.setCursor (2, 1);
      lcd.print (temperature, 1);
    }

    interrupts ();
  }
}
