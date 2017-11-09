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

#include <DHT.h>
#define DHTPIN GPS_PPS
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

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
  while (!(powMask & M95_MASK)) {
    Serial.println("power m95 error!");
    delay(1000);
  }

  setPowerUp(EXT_MASK);
  while (!(powMask & EXT_MASK)) {
    Serial.println("power ext error!");
    delay(1000);
  }
  dht.begin();
}

uint8_t niter = 0;

void loop()
{
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  Serial.print("Humedad:"); Serial.println(h);
  Serial.print("Temperatura:"); Serial.println(t);
  delay(1000);

  // jsonificando en el buffer
  m95.cleanStringIO();

  StaticSharedJsonBuffer jsonBuffer((uint8_t*)adc_ring_buffer, GC_ADC_RING_SIZE_BYTES);
  JsonObject& root = jsonBuffer.createObject();
  JsonObject& sensor_event = root.createNestedObject("sensor_event");
  sensor_event["timestamp"] = Teensy3Clock.get();
  sensor_event["value"] = t;
  root.printTo(m95.getStringIO(), m95.getStringIOLength());

  uint16_t stateHTTP = m95.httpJsonREST("ultractmsensors.vinstruments.cl", "POST /sensor_events");

  if (0) {
    if (niter <= 0) {
      //geocentinela.c.setGain(niter++);
      niter++;
      //geocentinela.c.setGain(0);
      //geocentinela.write();
      //setup2();

      // start:
      gcPlayStat = GC_ST_PLAY;
      adc_play_cnt = (1000000/geocentinela.c.getTickTime())*5;
      gcStart();
    }

    while (gcPlayStat & GC_ST_READING) {
      while (delta > 3) {
        if (sd_head == GC_SD_BUFFER_SIZE) {
          file.write(sd_buffer, GC_SD_BUFFER_SIZE_BYTES);
          sd_head = 0;
        }

        sd_buffer[sd_head++] = adc_ring_buffer[tail++];
        tail &= GC_ADC_RING_SIZE_HASH;

        noInterrupts();
        delta--;
        interrupts();

        if (trigger_list_head == TRIGGER_LIST_LENGTH) {
          tfile.write(trigger_list, TRIGGER_LIST_LENGTH_BYTES);
          trigger_list_head = 0;
        }
      }

      if (file.getWriteError()) {
        stop_reading();
      }
    }

    if (gcPlayStat & GC_ST_STOP) {
      gcStop();
      gcPlayStat &= ~GC_ST_STOP;
    }
  }
}
/**/
