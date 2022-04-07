#include "Arduino.h"

// Pindefinitionen
const unsigned char PUMPE = 2;
const unsigned char LED_OK = 3;
const unsigned char LED_WARN = 5;
const unsigned char LED_FEHLER = 4;
const unsigned char LUEFTER = 11;
const unsigned char TASTER = 12;
const unsigned char V_BATT = A0;
const unsigned char V_TEMP = A1;

// Minimale Batteriespannung
#define MIN_BAT     10.5 //12.0

// Maximale Temperatur
#define TEMP_MAX  70.0

// #Ticks pro Tag auf Basis des 10s Timer
#define TICKS_PER_DAY   8640


typedef struct {
  uint64_t zeit;
  uint32_t taster;
  float Batt_min, Batt_max;
  float Temp_min, Temp_max;
} statistiken_t;

statistiken_t stat_tag[7], stat_alltime;
uint8_t pos_tag = 0;
uint16_t ticks_tag = TICKS_PER_DAY;

float getTemp()
{
  uint16_t adc;
  float v;

  adc = analogRead(V_TEMP);
  v = 5 * ((float)adc/1024.0);

  return 0.8193*v*v*v*v*v - 9.6569 *v*v*v*v + 43.315 *v*v*v - 91.04 *v*v +107.14 *v -52.94;
}


float getBattVolt()
{
  uint16_t adc;
  
  adc = analogRead(V_BATT);

  return (float)adc * 0.0167;
}


void sendStats()
{
  uint8_t i, pos;
  pos = pos_tag;

  for(i = 0; i < 7; i++)
  {
    Serial.print(i);
    Serial.print(": ");
    Serial.print((float)stat_tag[pos].zeit / 1000);
    Serial.print("s mit ");
    Serial.print(stat_tag[pos].taster);
    Serial.print("x druecken");
    Serial.println();
    
    Serial.print(i);
    Serial.print(": Batterie ");
    Serial.print(stat_tag[pos].Batt_min);
    Serial.print(" bis ");
    Serial.print(stat_tag[pos].Batt_max);
    Serial.print("V  Temperatur: ");
    Serial.print(stat_tag[pos].Temp_min);
    Serial.print(" bis ");
    Serial.print(stat_tag[pos].Temp_max);
    Serial.print("C");
    Serial.println();

    if(!pos)
    {
      pos = 6;
    }
    else
    {
      pos--;
    }
  }

  Serial.println("---");
  Serial.print("A: ");
  Serial.print((float)stat_alltime.zeit / 1000);
  Serial.print("s mit ");
  Serial.print(stat_alltime.taster);
  Serial.print("x druecken");
  Serial.println();
  
  Serial.print("A: Batterie ");
  Serial.print(stat_alltime.Batt_min);
  Serial.print(" bis ");
  Serial.print(stat_alltime.Batt_max);
  Serial.print("V  Temperatur: ");
  Serial.print(stat_alltime.Temp_min);
  Serial.print(" bis ");
  Serial.print(stat_alltime.Temp_max);
  Serial.print("C");
  Serial.println();
}


void setup()
{
  Serial.begin(9600);

  pinMode(PUMPE, OUTPUT);
  pinMode(LED_OK, OUTPUT);
  pinMode(LED_WARN, OUTPUT);
  pinMode(LED_FEHLER, OUTPUT);
  pinMode(LUEFTER, OUTPUT);
  pinMode(TASTER, INPUT_PULLUP);
  pinMode(V_BATT, INPUT);
  pinMode(V_TEMP, INPUT);

  memset(stat_tag, 0, sizeof(statistiken_t) * 7);
  memset(&stat_alltime, 0, sizeof(statistiken_t));
  stat_alltime.Temp_min = 100;
  stat_alltime.Batt_min = 100;
  stat_tag[0].Temp_min = 100;
  stat_tag[0].Batt_min = 100;
}


void loop()
{
  static uint32_t tim10s = 0;        // 1s Timer
  static uint16_t schalter_sperre = 0;  // Schaltersperren
  static uint8_t schalter_prev = LOW;   // vorheriger Schalterzustand
  static uint8_t schalter_now = LOW;
  static uint8_t debounce_cnt = 0;
  static uint32_t pumpe_start = 0;    // millis() wann pumpe angestellt
  static float FETtemp = 0;
  
  float BATvoltage;
  uint8_t ok = 1;

  BATvoltage = getBattVolt();
  FETtemp = getTemp();

  // Unterspannung?
  if(BATvoltage < MIN_BAT)
  {
    ok = 0;
    digitalWrite(LED_OK, LOW);
    digitalWrite(LED_WARN, HIGH);
  }
  else
  {
    digitalWrite(LED_WARN, LOW);
  }

  // Überhitzung?
  
  if(FETtemp > TEMP_MAX)
  {
    ok = 0;
    digitalWrite(LED_OK, LOW);
    digitalWrite(LED_FEHLER, HIGH);
  }
  else
  {
    digitalWrite(LED_FEHLER, LOW);
  }
  

  // Sperre Schalter x Sekunden, nachdem er losgelassen wurde
  if(schalter_sperre)
  {
    digitalWrite(LED_OK, LOW);
    schalter_sperre--;
    ok = 0;
  }
  else
  {
    digitalWrite(LED_OK, HIGH);
  }

  // Taster abfragen mit einfacher Entprellung
  if(digitalRead(TASTER) == LOW)
  {
    debounce_cnt++;
    if(debounce_cnt >= 2)
    {
      schalter_now = HIGH;
    }
  }
  else
  {
    debounce_cnt = 0;
    schalter_now = LOW;
  }

  // falls keine Fehler
  if(ok)
  {
    digitalWrite(LED_OK, HIGH);
    if(schalter_now == HIGH)
    {
      if(schalter_prev == LOW)  // Taster gerade gedrückt
      {
        stat_tag[pos_tag].taster++;
        stat_alltime.taster++;
        pumpe_start = millis();
      }

      if(millis() - pumpe_start >= 5000)  // Wenn Taster länger als 5000ms gedrückt
      {      
        digitalWrite(PUMPE, LOW);
        while(digitalRead(TASTER) == LOW)
          ;     // nichts machen bis Taster losgelassen
      }
      else
      {
        digitalWrite(PUMPE, HIGH);
      }
    }
    else
    {
      if(schalter_prev == HIGH) // gerade losgelassen
      {
        uint32_t pumpzeit = millis() - pumpe_start;
        stat_tag[pos_tag].zeit += pumpzeit;
        stat_alltime.zeit += pumpzeit;
        schalter_sperre = 10;
      }
      digitalWrite(PUMPE, LOW);
    }
  }
  else
  {
    digitalWrite(LED_OK, LOW);
    digitalWrite(PUMPE, LOW);
  }


  // Lüfter ansteuern
  if(FETtemp > 40)
  {
    long lwert;

    lwert = map(FETtemp, 40, 70, 100, 255);
    if(lwert > 255)
    {
      lwert = 255;
    }

    analogWrite(LUEFTER, lwert);
  }
  else
  {
    analogWrite(LUEFTER, 0);
  }

  if(millis() > tim10s)
  {
    Serial.print("Batt: ");
    Serial.println(BATvoltage);
    Serial.print("Temp: ");
    Serial.println(FETtemp);
    Serial.print("OK?: ");
    Serial.println(ok);
    Serial.println();
    tim10s = millis() + 10000;

	if(schalter_now == LOW)
	{
		sendStats();
	}
	
    // Laufzeit zählen
    ticks_tag--;
    if(!ticks_tag)
    {
      ticks_tag = TICKS_PER_DAY;
      pos_tag = (pos_tag + 1) % 7;
      memset(&stat_tag[pos_tag], 0, sizeof(statistiken_t));
      stat_tag[pos_tag].Batt_min = BATvoltage;
      stat_tag[pos_tag].Temp_min = FETtemp;
    }
  }

  // Statistiken für Batteriespannung und Temperatur
  if(BATvoltage < stat_tag[pos_tag].Batt_min)
  {
    stat_tag[pos_tag].Batt_min = BATvoltage;
  }
  if(BATvoltage > stat_tag[pos_tag].Batt_max)
  {
    stat_tag[pos_tag].Batt_max = BATvoltage;
  }
  if(FETtemp < stat_tag[pos_tag].Temp_min)
  {
    stat_tag[pos_tag].Temp_min = FETtemp;
  }
  if(FETtemp > stat_tag[pos_tag].Temp_max)
  {
    stat_tag[pos_tag].Temp_max = FETtemp;
  }

  if(BATvoltage < stat_alltime.Batt_min)
  {
    stat_alltime.Batt_min = BATvoltage;
  }
  if(BATvoltage > stat_alltime.Batt_max)
  {
    stat_alltime.Batt_max = BATvoltage;
  }
  if(FETtemp < stat_alltime.Temp_min)
  {
    stat_alltime.Temp_min = FETtemp;
  }
  if(FETtemp > stat_alltime.Temp_max)
  {
    stat_alltime.Temp_max = FETtemp;
  }
    
  schalter_prev = schalter_now;
  delay(100);
}

