/*
#include <IntervalTimer.h>

#include <TinyGPS.h>
#include <TimeLib.h>

#include <SdFat.h>

  #include <EEPROM.h>
  #include <ArduinoJson.h>
  #include "gcCFG.h"

  #include <LowPower_Teensy3.h>
#include "gcAll.h"

void stop_reading()
{
  noInterrupts();
  gcPlayStat &= ~GC_ST_READING;
  gcPlayStat |= GC_ST_STOP;
  interrupts();
}

void start_reading()
{}

void setup2()
{
  Serial.begin(9600); while (!Serial);
  geocentinela.println(PSTR("Setup ini..."));

  // pines no utilizados:
  pinMode(24, INPUT); //digitalWrite(24, LOW);
  pinMode(25, INPUT); //digitalWrite(25, LOW);
  pinMode(26, INPUT); //digitalWrite(26, LOW);
  pinMode(27, INPUT); //digitalWrite(27, LOW);
  pinMode(28, INPUT); //digitalWrite(28, LOW);
  pinMode(29, INPUT); //digitalWrite(29, LOW);
  pinMode(30, INPUT); //digitalWrite(30, LOW); PIN_USB
  pinMode(31, INPUT); //digitalWrite(31, LOW);
  pinMode(32, INPUT); //digitalWrite(32, LOW);
  pinMode(33, INPUT); //digitalWrite(33, LOW);

  // se lee la configuracion desde la EEPROM
  readCfg();
  geocentinela.println(PSTR("EEPROM"));

  // se configura el RTC
  cfgRTC();
  geocentinela.println(PSTR("RTC...wait!"));

  // se configura el PGA
  cfgPGA();
  geocentinela.println(PSTR("PGA"));

  // se configura la SD
  cfgSD();
  geocentinela.println(PSTR("SD"));

  // se configura el GPRS
  if (geocentinela.getGprs()) {
    if (!cfgM95()) {
      geocentinela.setGprs(false);
    } else
      geocentinela.println(PSTR("GPRS"));
  } else {
    geocentinela.setGprs(true);
    cfgM95();
  }

  // se configura el GPS
  if (geocentinela.getGps()) {
    cfgGPS();
    //if (!cfgGPS()) {
    //  geocentinela.setGps(false);
    //}
  }
  geocentinela.println(PSTR("GPS"));

  // configure ADC
  cfgADC();
  geocentinela.println(PSTR("ADC"));

  // configure DMA
  cfgDMA();
  geocentinela.println(PSTR("DMA"));

  rMessage mlog;
  mlog.setInstrumentId(geocentinela.getId());
  mlog.setDateNow();
  mlog.setFrom("GeoCentinela");
  mlog.setSMS("Started");

  m95.cleanStringIO();
  mlog.serialize(m95.getStringIO(), m95.getStringIOLength(),
    (uint8_t*) adc_ring_buffer, GC_ADC_RING_SIZE_BYTES, false);
  geocentinela.printCmd('j', m95.getStringIO());
}

void setup()
{
  Serial.begin(9600); while (!Serial);
  setup2();

  setPowerUp(M95_MASK);
  for(int8_t i = 0; !(powMask & M95_MASK) && i < 3; i++) {
    Serial.println("power m95 error!");
    delay(1000);
    cfgM95();
    setPowerUp(M95_MASK);
  }
  Serial.println("out setup!");
}

void loop()
{
  uint8_t cmd = 0;
  while (true) {
    while (Serial.available()) {
      cmd = Serial.read();
      if (cmd == '@') {
        cmd = Serial.read();
        switch (cmd) {
          case 0:
            XBEE_IO.begin(9600);
            Serial.println("9600");
            break;
          case 1:
            XBEE_IO.begin(38400);
            Serial.println("38400");
            break;
          case 2:
            XBEE_IO.begin(57600);
            Serial.println("57600");
            break;
          case 3:
            XBEE_IO.begin(115200);
            Serial.println("115200");
            break;
          case 4:
            XBEE_IO.begin(1200);
            Serial.println("1200");
            break;
          case 5:
            XBEE_IO.begin(2400);
            Serial.println("2400");
            break;
          case 6:
            XBEE_IO.begin(4800);
            Serial.println("4800");
            break;
          default:
            Serial.println("0<= ? <= 4");
        }
      }
      XBEE_IO.write(cmd);
    }
    while (XBEE_IO.available()) {
      cmd = XBEE_IO.read();
      Serial.write(cmd);
    }
  }
}
/**/
