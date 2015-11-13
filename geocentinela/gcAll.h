void gc_print(const char *log_string);
void gc_println(const char *log_string);
//-------------------------------
#include "gcCFG.h"
gcCFG gc_cfg(PSTR("GC03.CFG"), gc_println);
//-------------------------------
TEENSY3_LP lp = TEENSY3_LP();
sleep_block_t lp_cfg; // sleep configuration
//-------------------------------
#define GC_ADC0_A0 5
#define GC_ADC0_A1 14
#define GC_ADC0_A2 8
#define GC_ADC0_A3 9
#define GC_ADC0_A4 13
#define GC_ADC0_A5 12
#define GC_ADC0_A6 6
#define GC_ADC0_A7 7
#define GC_ADC0_A8 15
#define GC_ADC0_A9 4
#define GC_ADC0_A10 0
#define GC_ADC0_A11 19
#define GC_ADC0_A12 3
#define GC_ADC0_A13 21
#define GC_ADC1_A2 GC_ADC0_A2
#define GC_ADC1_A3 GC_ADC0_A3
#define GC_ADC1_A10 3
#define GC_ADC_TEMP 26
#define GC_ADC_BANDGAP 27
#define GC_ADC_REFSEL 0 // 0->3.3v(ext 2.5v), 1->1.2v
#define GC_ADC0_AZ GC_ADC0_A11
#define GC_ADC0_AZV 0x8000 // 1.25v
#define GC_ADC1_AZ GC_ADC_BANDGAP
#define GC_ADC1_AZV 0x6667 // 1v
//-------------------------------
#define GC_ADC_VREF 2500 // external vref
#define GC_ADC_BITS 16
#define GC_ADC_RING_SIZE 0x2000
#define GC_ADC_RING_SIZE_BYTES (GC_ADC_RING_SIZE*2) // 2 == sizeof(uint16_t)
#define GC_ADC_RING_SIZE_HASH (GC_ADC_RING_SIZE-1)
DMAMEM
volatile uint16_t
  __attribute__((aligned(GC_ADC_RING_SIZE_BYTES)))
  adc_ring_buffer[GC_ADC_RING_SIZE];

volatile uint32_t adc_config[14] = {
  ADC_SC1_ADCH(GC_ADC0_A4), // GC_ADC0_A2 in gc v3.4
  ADC_SC1_ADCH(GC_ADC0_A6), // GC_ADC0_A3 in gc v3.4
  ADC_SC1_ADCH(GC_ADC0_A9), // GC_ADC0_A10 in gc v3.4
  ADC_SC1_ADCH(31), // stop=0b11111=31
  ADC_SC1_ADCH(GC_ADC0_A10), // read battery state
  ADC_SC1_ADCH(31), // stop=0b11111=31
  ADC_SC1_ADCH(GC_ADC0_A11), // read zero vref
  ADC_SC1_ADCH(31), // stop=0b11111=31
  ADC_SC1_ADCH(GC_ADC_TEMP), // read temperature
  ADC_SC1_ADCH(31), // stop=0b11111=31
  ADC_SC1_ADCH(GC_ADC1_A10), //  A2 in gc v3.4
  ADC_SC1_ADCH(GC_ADC1_A10), //  A3 in gc v3.4
  ADC_SC1_ADCH(GC_ADC1_A10), // A10 in gc v3.4
  ADC_SC1_ADCH(31), // stop=0b11111=31
};
//--------------------------------------------------
#define SD_CS 10
#define SD_MOSI 11
#define SD_MISO 12
#define SD_SCK 13
SdFat sd;
SdFile file;
#define GC_SD_BUFFER_SIZE 0x1000
#define GC_SD_BUFFER_SIZE_BYTES (GC_SD_BUFFER_SIZE*2) // 2 == sizeof(uint16_t)
uint16_t sd_buffer[GC_SD_BUFFER_SIZE];
//--------------------------------------------------
// Create an XBee object at the top of your sketch
XBee xbng = XBee();
#define XBEE_SLEEP_TIME 10000

#define XBEE_nSLEEP_ON 9
#define XBEE_SLEEP_RQ 6
#define XBEE_nRESET 5

#define XBEE_IO Serial3
#define XBEE_RX 7
#define XBEE_TX 8
volatile uint32_t xbeeBD = 0;

#define XBEE_nRTS 4
#define XBEE_nCTS 3
//--------------------------------------------------
SdFile qfile;
#define QUAKE_LIST_LENGTH 0x100
#define QUAKE_LIST_LENGTH_BYTES QUAKE_LIST_LENGTH*4 // 4 == sizeof(uint32_t)
uint32_t quake_list[QUAKE_LIST_LENGTH];
uint32_t adc_play_cnt_quake;
uint16_t quake_head;
#define TRIGGER_BY_SOFTWARE 1

#if TRIGGER_BY_SOFTWARE == 0
#elif TRIGGER_BY_SOFTWARE == 1
const uint32_t adc_ring_buffer_mmin = (uint32_t)&(adc_ring_buffer[0]);
const uint32_t adc_ring_buffer_mmax = (uint32_t)&(adc_ring_buffer[0]) + (GC_ADC_RING_SIZE_BYTES-1);
volatile uint16_t quake_min;
volatile uint16_t quake_max;
#endif
//--------------------------------------------------
IntervalTimer adc_play;
//--------------------------------------------------
volatile uint16_t delta = 0;
volatile uint16_t tail = 0;
volatile uint16_t sd_head = 0;

volatile uint32_t adc_errors = 0;
volatile uint32_t buffer_errors = 0;

volatile uint32_t adc_play_cnt = 0;
volatile uint32_t adc_rtc_stop = 0;
//--------------------------------------------------
volatile uint8_t gcPlayStat = 0;// GeoCentinela Play Status
#define GC_ST_SLEEP     0x01 // GeoCentinela is sleeping
#define GC_ST_READING   0x02 // GeoCentinela is running
#define GC_ST_STOP      0x04 // GeoCentinela is stoping
#define GC_ST_CONFIG    0x08
#define GC_ST_PLAY      0x10
#define GC_ST_FILE_OPEN 0x20

volatile uint8_t gcCfgStat = 0;// GeoCentinela Config Status
#define GC_CFG_READ   0x20 // Configure readed!
#define GC_CFG_ADC    0x01 // ADC OK
#define GC_CFG_DMA    0x02 // DMA OK
//--------------------------------------------------
#define FILE_FORMAT 0x05
#define FILENAME_FORMAT "GC000000.???"
#define FILENAME_MAX_LENGH 12 // 8.3 filename
#define FILENAME_NO_DIGITS 2  // GC000000
#define FILENAME_EXT 4        // .???
#define PIN_USB 30
#define WAKE_USB PIN_30
//--------------------------------------------------
#define SDX_POW 19 // 3.3V
#define PGA_POW 22 // 5Va
#define EXT_POW 21 // 3.3VP
#define XBEE_MASK 0b1000
#define SD_MASK   0b0100
#define PGA_MASK  0b0010
#define EXT_MASK  0b0001
#define POWER_UP_MASK 0b10000
volatile uint8_t powMask = 0;
//--------------------------------------------------
#define GAIN_CS 15
#define GAIN_A0 16
#define GAIN_A1 14
#define GAIN_A2 17
//--------------------------------------------------
TinyGPS gps;
#define HOUR_OFFSET -3
#define GPS_RTC_SYNC_TIME 5*60*1000 // 5min
#define GPS_PPS 2
#define GPS_IO Serial1
//--------------------------------------------------
#define LOG_TIMEOUT 1000
#define SEG_A_DAY 86400
//--------------------------------------------------
void stop_reading();
void setPGA(uint8_t g);
//--------------------------------------------------
time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

void cfgRTC()
{
  // se verifica el RTC
  setSyncProvider(getTeensy3Time);
  while (timeStatus() != timeSet) {
    gc_println(PSTR("error: rtc!"));
    delay(LOG_TIMEOUT);
    setSyncProvider(getTeensy3Time);
  }
}
//--------------------------------------------------
void setPowerDown(uint8_t mask)
{
  if ((mask & XBEE_MASK) > 0) {
    if ((powMask & XBEE_MASK) == 0) goto out_xbee;

    digitalWrite(XBEE_SLEEP_RQ, HIGH);
    uint32_t stime = millis();
    while (millis() - stime < XBEE_SLEEP_TIME) {
      if (digitalRead(XBEE_nSLEEP_ON)==LOW) break;
    } delay(200);

    digitalWrite(XBEE_nRESET, LOW);
    digitalWrite(XBEE_nRTS, LOW);

    XBEE_IO.end();

    if ((powMask & SD_MASK) == 0) {
      digitalWrite(SDX_POW, LOW);
      digitalWrite(XBEE_SLEEP_RQ, LOW);
      digitalWrite(XBEE_nRESET, LOW);
    }

    powMask &= ~XBEE_MASK;
  }
out_xbee:

  if ((mask & SD_MASK) > 0) {
    if ((powMask & SD_MASK) == 0) goto out_sd;

    while (sd.card()->isBusy()); delay(1000);
    pinMode(SD_CS, OUTPUT); digitalWrite(SD_CS, LOW);
    pinMode(SD_SCK, OUTPUT); digitalWrite(SD_SCK, LOW);
    pinMode(SD_MOSI, OUTPUT); digitalWrite(SD_MOSI, LOW);

    if ((powMask & XBEE_MASK) == 0) {
      digitalWrite(SDX_POW, LOW);
      digitalWrite(XBEE_SLEEP_RQ, LOW);
      digitalWrite(XBEE_nRESET, LOW);
    }

    powMask &= ~SD_MASK;
  }
out_sd:

  if ((mask & PGA_MASK) > 0) {
    if ((powMask & PGA_MASK) == 0) goto out_pga;

    digitalWrite(GAIN_CS, LOW);
    digitalWrite(PGA_POW, LOW);
    powMask &= ~PGA_MASK;
  }
out_pga:

  if ((mask & EXT_MASK) > 0) {
    if ((powMask & EXT_MASK) == 0) goto out_ext;

    digitalWrite(EXT_POW, LOW);
    powMask &= ~EXT_MASK;
  }
out_ext:

  return;
}

void setPowerUp(uint8_t mask)
{
  if ((mask & XBEE_MASK) > 0) {
    if ((powMask & XBEE_MASK) > 0) goto out_xbee;

    if (xbeeBD != 0) {
      XBEE_IO.begin(xbeeBD);
    } else {
      XBEE_IO.begin(9600);
    }

    digitalWrite(XBEE_nRESET, HIGH);
    digitalWrite(XBEE_nRTS, LOW);

    digitalWrite(SDX_POW, HIGH);

    digitalWrite(XBEE_SLEEP_RQ, LOW);
    uint32_t stime = millis();
    while (millis() - stime < XBEE_SLEEP_TIME/10) {
      if (digitalRead(XBEE_nSLEEP_ON)==HIGH && digitalRead(XBEE_nCTS)==LOW) {
        stime = 0; break;
      }
    } delay(200);

    powMask |= XBEE_MASK;
    if (stime != 0) setPowerDown(XBEE_MASK);
  }
out_xbee:

  if ((mask & SD_MASK) > 0) {
    if ((powMask & SD_MASK) > 0) goto out_sd;

    digitalWrite(SDX_POW, HIGH); delay(500);
    ADC0_SC1A = adc_config[3]; // revisar pq es necesario...

    sd = SdFat();
    while(!sd.begin(SD_CS, SPI_FULL_SPEED)) {
      gc_println(PSTR("error: sdcard"));
      //sd.errorPrint();
      delay(LOG_TIMEOUT);
    }
    while (sd.card()->isBusy()); delay(200);
    sd.chvol();

    powMask |= SD_MASK;

    if ((powMask & XBEE_MASK) == 0) {// hibernate xbee
      setPowerUp(XBEE_MASK);
      setPowerDown(XBEE_MASK);
    }
  }
out_sd:

  if ((mask & PGA_MASK) > 0) {
    if ((powMask & PGA_MASK) > 0) goto out_pga;

    digitalWrite(PGA_POW, HIGH); delay(200);

    powMask |= PGA_MASK;
    setPGA(gc_cfg.gain);
  }
out_pga:

  if ((mask & EXT_MASK) > 0) {
    if ((powMask & EXT_MASK) > 0) goto out_ext;

    digitalWrite(EXT_POW, HIGH); delay(200);

    powMask |= EXT_MASK;
  }
out_ext:

  return;  
}
//--------------------------------------------------
String strDouble(String str, double val, uint8_t precision)
{
  // prints val with number of decimal places determine by precision
  // precision is a number from 0 to 6 indicating the desired decimial places
  // example: lcdPrintDouble( 3.1415, 2); // prints 3.14 (two decimal places)

  if(val < 0.0){
    str += '-';
    val = -val;
  }

  str += long(val);  //prints the int part
  if(precision > 0) {
    str += '.'; // print the decimal point
    while (precision-- > 0) {
      double frac = (val - long(val))*10;
      uint8_t frac1 = long(frac);
      str += frac1;
      val = frac;
    }
  }

  return str;
}

static void cfgGps()
{
  GPS_IO.begin(4800);

  GPS_IO.println(F("$PSRF103,0,0,0,1*24"));
  GPS_IO.println(F("$PSRF103,1,0,0,1*25"));
  GPS_IO.println(F("$PSRF103,2,0,0,1*26"));
  GPS_IO.println(F("$PSRF103,3,0,0,1*27"));
  GPS_IO.println(F("$PSRF103,4,0,0,1*20"));
  GPS_IO.println(F("$PSRF103,5,0,0,1*21"));
  GPS_IO.println(F("$PSRF103,6,0,0,1*22"));
  GPS_IO.println(F("$PSRF103,8,0,0,1*2C"));

  //GPS_IO.println(F("$PSRF103,0,0,1,1*25"));
  //GPS_IO.println(F("$PSRF103,1,0,1,1*26"));
  //GPS_IO.println(F("$PSRF103,2,0,1,1*27"));
  //GPS_IO.println(F("$PSRF103,3,0,1,1*28"));
  GPS_IO.println(F("$PSRF103,4,0,1,1*21"));
  //GPS_IO.println(F("$PSRF103,5,0,1,1*22"));
  //GPS_IO.println(F("$PSRF103,6,0,1,1*23"));
  //GPS_IO.println(F("$PSRF103,8,0,1,1*2D"));
}

static boolean gps2rtcSync()
{
  uint32_t fix_age;
  int Year;
  uint8_t Month, Day, Hour, Minute, Second;
  gps.crack_datetime(&Year, &Month, &Day, &Hour, &Minute, &Second, NULL, &fix_age);
  if (fix_age < 500) {
    // set the Time to the latest GPS reading
    setTime(Hour, Minute, Second, Day, Month, Year);
    adjustTime(HOUR_OFFSET * SECS_PER_HOUR);
  } else return false;

  float flat, flon;
  gps.f_get_position(&flat, &flon, &fix_age);
  if (fix_age < 500 && fix_age != TinyGPS::GPS_INVALID_AGE) {
    return true;
  }

  return false;
}

void syncGps()
{
  if (!gc_cfg.gps) return;

  // GPS power on
  setPowerUp(EXT_MASK);

  cfgGps();

  // sync
  char c = 0;
  uint8_t timeIni = millis();
  boolean rtcSync = false;
  while (!rtcSync && (millis() - timeIni) <= GPS_RTC_SYNC_TIME && (gcPlayStat & GC_ST_CONFIG) == 0) {
    while (GPS_IO.available() > 0 && !rtcSync) {
      c = GPS_IO.read();
      Serial.write(c);
      if (gps.encode(c)) rtcSync = gps2rtcSync();
    }
  }

  // GPS power off
  setPowerDown(EXT_MASK);
}

void cfgGPS()
{
  // se configura la alimentacion
  pinMode(EXT_POW, OUTPUT); digitalWrite(EXT_POW, LOW);

  // se configura el PPS:
  pinMode(GPS_PPS, INPUT);
  *portConfigRegister(GPS_PPS) = PORT_PCR_MUX(1) | PORT_PCR_PE; // INPUT_PULLDOWN

  // se sincroniza el GPS:
  syncGps();
}
//--------------------------------------------------
void cfgPGA()
{
  pinMode(PGA_POW, OUTPUT); digitalWrite(PGA_POW, LOW);

  pinMode(GAIN_CS, OUTPUT); digitalWrite(GAIN_CS, LOW);
  pinMode(GAIN_A0, OUTPUT); digitalWrite(GAIN_A0, LOW);
  pinMode(GAIN_A1, OUTPUT); digitalWrite(GAIN_A1, LOW);
  pinMode(GAIN_A2, OUTPUT); digitalWrite(GAIN_A2, LOW);
}
//--------------------------------------------------
void setPGA(uint8_t gain)
{
  gc_cfg.set_gain(gain);

  if ((powMask & PGA_MASK) == 0) return;

  digitalWrite(GAIN_CS, LOW);
  switch (gain) {
    case 0: {
      digitalWrite(GAIN_A0, LOW);
      digitalWrite(GAIN_A1, LOW);
      digitalWrite(GAIN_A2, LOW);
    } break;
    case 1: {
      digitalWrite(GAIN_A0, HIGH);
      digitalWrite(GAIN_A1, LOW);
      digitalWrite(GAIN_A2, LOW);
    } break;
    case 2: {
      digitalWrite(GAIN_A0, LOW);
      digitalWrite(GAIN_A1, HIGH);
      digitalWrite(GAIN_A2, LOW);
    } break;
    case 3: {
      digitalWrite(GAIN_A0, HIGH);
      digitalWrite(GAIN_A1, HIGH);
      digitalWrite(GAIN_A2, LOW);
    } break;
    case 4: {
      digitalWrite(GAIN_A0, LOW);
      digitalWrite(GAIN_A1, LOW);
      digitalWrite(GAIN_A2, HIGH);
    } break;
    case 5: {
      digitalWrite(GAIN_A0, HIGH);
      digitalWrite(GAIN_A1, LOW);
      digitalWrite(GAIN_A2, HIGH);
    } break;
    case 6: {
      digitalWrite(GAIN_A0, LOW);
      digitalWrite(GAIN_A1, HIGH);
      digitalWrite(GAIN_A2, HIGH);
    } break;
    case 7: {
      digitalWrite(GAIN_A0, HIGH);
      digitalWrite(GAIN_A1, HIGH);
      digitalWrite(GAIN_A2, HIGH);
    } break;
    default: {
    } break;
  }
  digitalWrite(GAIN_CS, HIGH);
  digitalWrite(GAIN_A0, LOW);
  digitalWrite(GAIN_A1, LOW);
  digitalWrite(GAIN_A2, LOW);
}
//--------------------------------------------------
void cfgSD()
{
  // se configura los IO
  pinMode(SD_CS, OUTPUT); digitalWrite(SD_CS, LOW);
  pinMode(SD_MOSI, OUTPUT); digitalWrite(SD_MOSI, LOW);
  pinMode(SD_MISO, INPUT_PULLUP);
  pinMode(SD_SCK, OUTPUT); digitalWrite(SD_SCK, LOW);

  // se lee la configuracion desde la sd
  setPowerUp(SD_MASK); // se enciende la sd
  while(!gc_cfg.read()) {
    while(!gc_cfg.write()) {
      gc_println(PSTR("error: cfg/rw!"));
      delay(LOG_TIMEOUT);
      if (gc_cfg.read()) break;
    }
  }
  gcCfgStat |= GC_CFG_READ;
  setPowerDown(SD_MASK);
}
//--------------------------------------------------
void cfgXBEE()
{
  // se configura los IO
  pinMode(XBEE_nSLEEP_ON, INPUT);
  *portConfigRegister(XBEE_nSLEEP_ON) = PORT_PCR_MUX(1) | PORT_PCR_PE; // INPUT_PULLDOWN

  pinMode(XBEE_SLEEP_RQ, OUTPUT); digitalWrite(XBEE_SLEEP_RQ, LOW);
  pinMode(XBEE_nRESET, OUTPUT); digitalWrite(XBEE_nRESET, LOW);

  pinMode(XBEE_nRTS, OUTPUT); digitalWrite(XBEE_nRTS, LOW);
  pinMode(XBEE_nCTS, INPUT);
  *portConfigRegister(XBEE_nCTS) = PORT_PCR_MUX(1) | PORT_PCR_PE; // INPUT_PULLDOWN

  pinMode(XBEE_RX, OUTPUT); digitalWrite(XBEE_RX, LOW);
  pinMode(XBEE_TX, OUTPUT); digitalWrite(XBEE_TX, LOW);
  XBEE_IO.begin(9600); XBEE_IO.end();

  // se configura AP=2, SM=1 and BD=8
  uint32_t bauds[9] = {
    1200,
    2400,
    4800,
    9600,
    19200,
    38400,
    57600,
    115200,
    230400
  };

  setPowerUp(XBEE_MASK);
  if ((powMask & XBEE_MASK) == 0) return;

  uint8_t loop_max = 3;
  for (int8_t i = 8; i >= 0 && xbeeBD==0; i--) {
    XBEE_IO.begin(bauds[i]); delay(1050);
    XBEE_IO.write(PSTR("+++")); delay(1050);
    uint8_t cmd = 0;
    while (XBEE_IO.available()) {
      cmd = XBEE_IO.read();
      if (cmd == 'O') {
        if (XBEE_IO.available()) {
          cmd = XBEE_IO.read();
          if (cmd == 'K') {
            XBEE_IO.println(F("ATAP2"));delay(50);
            XBEE_IO.println(F("ATSM1"));delay(50);
            XBEE_IO.println(F("ATBD8"));delay(50);
            XBEE_IO.println(F("ATWR"));delay(50);
            XBEE_IO.println(F("ATCN"));delay(50);

            xbeeBD = bauds[8];
            while (XBEE_IO.available()) XBEE_IO.read();
            XBEE_IO.begin(xbeeBD);
          }
        }
      }
    }

    if (xbeeBD != 0) {
      xbng.begin(XBEE_IO);
      break;
    }

    if (i == 0) {
      gc_println(PSTR("xbee baud rate not detected!, try again..."));

      if (loop_max-- > 0) i = 9;
    }
  }
  setPowerDown(XBEE_MASK);
}
//--------------------------------------------------
void cfgSDX()
{
  // se configura la alimentacion
  pinMode(SDX_POW, OUTPUT); digitalWrite(SDX_POW, LOW);

  // se configura el XBEE:
  cfgXBEE();

  // se configura la tarjeta SD:
  cfgSD();
}
//--------------------------------------------------
void clearDMA()
{
  // enable DMA and DMAMUX clock
  SIM_SCGC7 |= SIM_SCGC7_DMA;
  SIM_SCGC6 |= SIM_SCGC6_DMAMUX;

  // Clear before configure DMA channel
  DMAMUX0_CHCFG1 = 0;
  DMAMUX0_CHCFG3 = 0;
  DMA_TCD1_CSR = 0;
  DMA_TCD2_CSR = 0;
  DMA_TCD3_CSR = 0;

  // disable DMA and DMAMUX clock
  //SIM_SCGC6 &= ~SIM_SCGC6_DMAMUX;
  //SIM_SCGC7 &= ~SIM_SCGC7_DMA;

  gcCfgStat &= ~GC_CFG_DMA;
}
//--------------------------------------------------
#if F_BUS == 60000000
  #define ADC_CFG1_16BIT  ADC_CFG1_ADIV(2) | ADC_CFG1_ADICLK(1) // 7.5 MHz
#elif F_BUS == 56000000
  #define ADC_CFG1_16BIT  ADC_CFG1_ADIV(2) | ADC_CFG1_ADICLK(1) // 7 MHz
#elif F_BUS == 48000000
  #define ADC_CFG1_16BIT  ADC_CFG1_ADIV(1) | ADC_CFG1_ADICLK(1) // 12 MHz
#elif F_BUS == 40000000
  #define ADC_CFG1_16BIT  ADC_CFG1_ADIV(1) | ADC_CFG1_ADICLK(1) // 10 MHz
#elif F_BUS == 36000000
  #define ADC_CFG1_16BIT  ADC_CFG1_ADIV(1) | ADC_CFG1_ADICLK(1) // 9 MHz
#elif F_BUS == 24000000
  #define ADC_CFG1_16BIT  ADC_CFG1_ADIV(1) | ADC_CFG1_ADICLK(0) // 12 MHz
#elif F_BUS == 16000000
  #define ADC_CFG1_16BIT  ADC_CFG1_ADIV(0) | ADC_CFG1_ADICLK(0) // 8 MHz
#elif F_BUS == 8000000
  #define ADC_CFG1_16BIT  ADC_CFG1_ADIV(0) | ADC_CFG1_ADICLK(0) // 8 MHz
#elif F_BUS == 4000000
  #define ADC_CFG1_16BIT  ADC_CFG1_ADIV(0) | ADC_CFG1_ADICLK(0) // 4 MHz
#elif F_BUS == 2000000
  #define ADC_CFG1_16BIT  ADC_CFG1_ADIV(0) | ADC_CFG1_ADICLK(0) // 2 MHz
#else
#error "F_BUS must be 60, 56, 48, 40, 36, 24, 4 or 2 MHz"
#endif
boolean cfgADC()
{
  if (!(gcCfgStat & GC_CFG_READ)) {
    gcCfgStat &= ~GC_CFG_ADC;
    return false;
  }

  // clear DMA:
  clearDMA();

  // ADC clock
  SIM_SCGC6 |= SIM_SCGC6_ADC0;

  // vref:
#if GC_ADC_REFSEL == 1
  VREF_TRM = 0
    | VREF_TRM_CHOPEN
    | VREF_TRM_TRIM(0x20);

  VREF_SC = 0
    | VREF_SC_VREFEN     // Internal Voltage Reference enable
    | VREF_SC_REGEN      // Regulator enable
    | VREF_SC_ICOMPEN    // Second order curvature compensation enable
    | VREF_SC_MODE_LV(1) // power buffer mode: 0->Bandgap on only, 1->High, 2->Low
  ;
#else
  VREF_TRM = 0;
  VREF_SC = 0;
#endif

  // general configuration
  uint32_t cfg1 = 0
    //| ADC_CFG1_ADLPC     // lower power: off, on
    //| ADC_CFG1_ADIV(1)   // clock divide: 1, 2, 4, 8
    //| ADC_CFG1_ADLSMP    // sample time: short, long
    | ADC_CFG1_MODE(3)   // conversion mode: 8, 12, 10, 16
    //| ADC_CFG1_ADICLK(1) // input clock: bus, bus/2, alternate, asynchronous
    | ADC_CFG1_16BIT       // set adc_clock, i.e: ADC_CFG1_ADIV and ADC_CFG1_ADICLK
  ;
  ADC0_CFG1 = cfg1;

  uint32_t cfg2 = 0
    | ADC_CFG2_MUXSEL    // adc mux (see man. pag. 98 Connections/Channel Assignment): ADxxa, ADxxb
    //| ADC_CG2_ADACKEN    // asynchronous clock output: disable, enable
    | ADC_CFG2_ADHSC     // high speed configuration: normal, high
    | ADC_CFG2_ADLSTS(0) // long sample time: 20ext, 12ext, 6ext, 2ext
  ;
  ADC0_CFG2 = cfg2;

  // control
  uint32_t sc2 = 0
    //| ADC_SC2_ADTRG                 // trigger select: software, hardware
    //| ADC_SC2_ACFE                  // compare function: disable, enable
    //| ADC_SC2_ACFGT                 // compare function greater than: disable, enable
    //| ADC_SC2_ACREN                 // compare function range: disable, enable
    | ADC_SC2_DMAEN                 // DMA enable
    | ADC_SC2_REFSEL(GC_ADC_REFSEL) // 0->3.3v, 1->1.2v
  ;
  ADC0_SC2 = sc2;

  uint32_t sc3 = 0; // continuous conversion disable, hardware average disable
  if (gc_cfg.average < 4) {
    sc3 = 0
      //| ADC_SC3_ADCO                 // continuous conversion: disable, enable
      | ADC_SC3_AVGE                 // enable hardware average
      | ADC_SC3_AVGS(gc_cfg.average) // average select: 0->4, 1->8, 2->16, 3->32
    ;
  }
  ADC0_SC3 = sc3;

  uint32_t powMaskOld = powMask;

  PMC_REGSC |= PMC_REGSC_BGBE;
  setPowerUp(PGA_MASK);

  // calibration
  ADC0_SC3 |= ADC_SC3_CAL; // begin cal

  uint16_t cal_sum;
  while ((ADC0_SC3 & ADC_SC3_CAL) && (ADC0_SC1A & ADC_SC1_COCO));

  cal_sum = (ADC0_CLPS + ADC0_CLP4 + ADC0_CLP3 + ADC0_CLP2 + ADC0_CLP1 + ADC0_CLP0)/2;
  ADC0_PG = cal_sum | 0x8000;

  cal_sum = (ADC0_CLMS + ADC0_CLM4 + ADC0_CLM3 + ADC0_CLM2 + ADC0_CLM1 + ADC0_CLM0)/2;
  ADC0_MG = cal_sum | 0x8000;

#define NAVERAGE 5000
#define NLOOPS 3
  uint32_t vavrg = 0;
  // set offset:
  ADC0_OFS = 0;
  for (uint16_t j = 0; j < NLOOPS || vavrg != 0; j++) {
    vavrg = 0;
    for (uint16_t i = 0; i < NAVERAGE; i++) {
      ADC0_SC1A = ADC_SC1_ADCH(GC_ADC0_AZ);
      while ((ADC0_SC1A & ADC_SC1_COCO)==0);
      vavrg += ADC0_RA;
    }
    vavrg /= NAVERAGE;
    boolean ofs_sig = vavrg > (uint32_t)GC_ADC0_AZV;
    if (ofs_sig) {
      vavrg = (vavrg - (uint32_t)GC_ADC0_AZV) >> 1;
      ADC0_OFS +=  ((int16_t)vavrg);
    } else {
      vavrg = ((uint32_t)GC_ADC0_AZV - vavrg) >> 1;
      ADC0_OFS += -((int16_t)vavrg);
    }
  }

  // Stop conversion
  ADC0_SC1A = ADC_SC1_ADCH(0b11111);

#if TRIGGER_BY_SOFTWARE == 0
  // ADC clock
  SIM_SCGC3 |= SIM_SCGC3_ADC1;

  // general configuration
  ADC1_CFG1 = cfg1;
  ADC1_CFG2 = cfg2;

  // control
  if (gc_cfg.trigger_level > 0 || gc_cfg.trigger_time_number > 0) {
    ADC1_SC2 = ADC_SC2_ACFE | ADC_SC2_ACFGT | ADC_SC2_ACREN | sc2;

    ADC1_CV1 = 0x0000 + (gc_cfg.trigger_level+1);
    ADC1_CV2 = 0xFFFF - gc_cfg.trigger_level;
  } else {
    ADC1_SC2 = sc2;

    ADC1_CV1 = 0;
    ADC1_CV2 = 0;
  }
  ADC1_SC3 = sc3;

  // calibration
  ADC1_SC3 |= ADC_SC3_CAL; // begin cal
  while ((ADC1_SC3 & ADC_SC3_CAL) && (ADC1_SC1A & ADC_SC1_COCO));

  cal_sum = (ADC1_CLPS + ADC1_CLP4 + ADC1_CLP3 + ADC1_CLP2 + ADC1_CLP1 + ADC1_CLP0)/2;
  ADC1_PG = cal_sum | 0x8000;

  cal_sum = (ADC1_CLMS + ADC1_CLM4 + ADC1_CLM3 + ADC1_CLM2 + ADC1_CLM1 + ADC1_CLM0)/2;
  ADC1_MG = cal_sum | 0x8000;

  // set offset:
  ADC1_OFS = 0;
  for (uint16_t j = 0; j < NLOOPS || vavrg != 0; j++) {
    vavrg = 0;
    for (uint16_t i = 0; i < NAVERAGE; i++) {
      ADC1_SC1A = ADC_SC1_ADCH(GC_ADC1_AZ);
      while ((ADC1_SC1A & ADC_SC1_COCO)==0);
      vavrg += ADC1_RA;
    }
    vavrg /= NAVERAGE;
    boolean ofs_sig = vavrg > (uint32_t)GC_ADC1_AZV;
    if (ofs_sig) {
      vavrg = (vavrg - (uint32_t)GC_ADC1_AZV) >> 1;
      ADC1_OFS +=  ((int16_t)vavrg);
    } else {
      vavrg = ((uint32_t)GC_ADC1_AZV - vavrg) >> 1;
      ADC1_OFS += -((int16_t)vavrg);
    }
  }

  // Stop conversion
  ADC1_SC1A = ADC_SC1_ADCH(0b11111);
#endif

  if ((powMaskOld & PGA_MASK)==0) setPowerDown(PGA_MASK);
  PMC_REGSC &= ~PMC_REGSC_BGBE;

  gcCfgStat |= GC_CFG_ADC;
  return true;
}

void cfgDMA()
{
  uint32_t mod = 1+log2(GC_ADC_RING_SIZE);

  clearDMA();

  // enable DMA and DMAMUX clock
  SIM_SCGC7 |= SIM_SCGC7_DMA;
  SIM_SCGC6 |= SIM_SCGC6_DMAMUX;

  // channels priority
  DMA_CR |= DMA_CR_ERCA; // enable round robin scheduling

  // configure the DMA transfer control descriptor 2
  DMA_TCD2_SADDR = &(adc_config[1]);
  DMA_TCD2_SOFF = 4; // Nº bytes between data
  DMA_TCD2_ATTR = 0
    | DMA_TCD_ATTR_SMOD(0)
    | DMA_TCD_ATTR_SSIZE(2)
    | DMA_TCD_ATTR_DMOD(0)
    | DMA_TCD_ATTR_DSIZE(2);
  DMA_TCD2_NBYTES_MLNO = 4; // Nº bytes to be transferred
  DMA_TCD2_SLAST = 0;
  DMA_TCD2_DADDR = &(ADC0_SC1A); // must be 'nbytes*dsize' aligned
  DMA_TCD2_DOFF = 0; // Nº bytes between data
  DMA_TCD2_DLASTSGA = 0;

  DMA_TCD2_CITER_ELINKNO = DMA_TCD2_BITER_ELINKNO = 1;

  DMA_TCD2_CSR &= ~DMA_TCD_CSR_DONE;
  DMA_TCD2_CSR = 0
  // disable scatter/gatter processing ESG=0
  // ERQ bit is not affected when the major loop is complete DREQ=0
  //| DMA_TCD_CSR_MAJORELINK // enable major loop channel to channel linking
    | DMA_TCD_CSR_BWC(0) // no eDMA engine stall
  //  | DMA_TCD_CSR_MAJORLINKCH(2) // major loop channel to channel linking set to 2
  //  | DMA_TCD_CSR_INTMAJOR // enable the end-of-major loop interrupt
  ;

  // configure the DMA transfer control descriptor 1
  DMA_TCD1_SADDR = &(ADC0_RA);
  DMA_TCD1_SOFF = 0; // Nº bytes between data
  DMA_TCD1_ATTR = 0
    | DMA_TCD_ATTR_SMOD(0)
    | DMA_TCD_ATTR_SSIZE(1)
    | DMA_TCD_ATTR_DMOD(mod) // mod = log2(nbytes*dsize)
    | DMA_TCD_ATTR_DSIZE(1);
  DMA_TCD1_NBYTES_MLNO = 2;// Nº bytes to be transferred
  DMA_TCD1_SLAST = 0;
  DMA_TCD1_DADDR = &(adc_ring_buffer[0]); // must be 'nbytes*dsize' aligned
  DMA_TCD1_DOFF = 2; // Nº bytes between data
  DMA_TCD1_DLASTSGA = 0;

  DMA_TCD1_CITER_ELINKNO = DMA_TCD1_BITER_ELINKNO = 1
  //DMA_TCD1_CITER_ELINKYES = DMA_TCD1_BITER_ELINKYES = 1
  //  | DMA_TCD_ITER_ELINK // enable minor loop channel linking
  //  | DMA_TCD_ITER_LINKCH(2) // link to the second channel.
  ;

  DMA_TCD1_CSR &= ~DMA_TCD_CSR_DONE;
  DMA_TCD1_CSR = 0
  // disable scatter/gatter processing ESG=0
  // ERQ bit is not affected when the major loop is complete DREQ=0
    | DMA_TCD_CSR_MAJORELINK // enable major loop channel to channel linking
    | DMA_TCD_CSR_BWC(0) // no eDMA engine stall
    | DMA_TCD_CSR_MAJORLINKCH(2) // major loop channel to channel linking set to 2
  //  | DMA_TCD_CSR_INTMAJOR // enable the end-of-major loop interrupt
  ;

  // to connect ADC with DMA
  DMA_SERQ = 1; // enable channel 1 requests

  // configure the DMAMUX so that the ADC DMA request triggers DMA channel 1
  DMAMUX0_CHCFG1 = DMAMUX_ENABLE | DMAMUX_SOURCE_ADC0;

#if TRIGGER_BY_SOFTWARE == 0
  // configure the DMA transfer control descriptor 3
  DMA_TCD3_SADDR = &(adc_config[1]);
  DMA_TCD3_SOFF = 4; // Nº bytes between data
  DMA_TCD3_ATTR = 0
    | DMA_TCD_ATTR_SMOD(0)
    | DMA_TCD_ATTR_SSIZE(2)
    | DMA_TCD_ATTR_DMOD(0)
    | DMA_TCD_ATTR_DSIZE(2);
  DMA_TCD3_NBYTES_MLNO = 4; // Nº bytes to be transferred
  DMA_TCD3_SLAST = 0;
  DMA_TCD3_DADDR = &(ADC1_SC1A); // must be 'nbytes*dsize' aligned
  DMA_TCD3_DOFF = 0; // Nº bytes between data
  DMA_TCD3_DLASTSGA = 0;

  DMA_TCD3_CITER_ELINKNO = DMA_TCD3_BITER_ELINKNO = 1;

  DMA_TCD3_CSR &= ~DMA_TCD_CSR_DONE;
  DMA_TCD3_CSR = 0
  // disable scatter/gatter processing ESG=0
  // ERQ bit is not affected when the major loop is complete DREQ=0
  //| DMA_TCD_CSR_MAJORELINK // enable major loop channel to channel linking
    | DMA_TCD_CSR_BWC(0) // no eDMA engine stall
  //  | DMA_TCD_CSR_MAJORLINKCH(2) // major loop channel to channel linking set to 2
  //  | DMA_TCD_CSR_INTMAJOR // enable the end-of-major loop interrupt
  ;

  // to connect ADC with DMA
  DMA_SERQ = 3; // enable channel 3 requests

  // configure the DMAMUX so that the ADC DMA request triggers DMA channel 3
  DMAMUX0_CHCFG3 = DMAMUX_ENABLE | DMAMUX_SOURCE_ADC1;
#endif

  gcCfgStat |= GC_CFG_DMA;
}
//----------------------------------------------------------------------
float getTemp()
{
  if (!(gcCfgStat & GC_CFG_ADC)) return 0;

  // Se desconecta el DMA
  clearDMA();

  // Se prende el voltage de referencia
  uint32_t powMaskOld = powMask;
  setPowerUp(PGA_MASK);

  // Se lee la temperatura
  uint32_t bvalue = 0;
  uint16_t n = 1000;
  for(uint16_t i = 0; i < n; i++) {
    ADC0_SC1A = adc_config[8];
    while(!(ADC0_SC1A & ADC_SC1_COCO));
    bvalue += ADC0_RA;
  }
  bvalue /= n;

  // se para la conversion
  ADC0_SC1A = adc_config[3];

  // Si estaba apagado el PGA se apaga:
  if ((powMaskOld & PGA_MASK) == 0) setPowerDown(PGA_MASK);

  // se reconecta el DMA
  cfgDMA();

  float mV_value = 2500.0*bvalue/65536.0;
  return 25.0-((mV_value-719)/1.715);// Temp = 25 - ((mV_temp - mV_temp25)/m)
}
//----------------------------------------------------------------------
float getVBat()
{
  if (!(gcCfgStat & GC_CFG_ADC)) return 0;

  // Se desconecta el DMA
  clearDMA();

  // Se prende el voltage de referencia
  uint32_t powMaskOld = powMask;
  setPowerUp(PGA_MASK);

  // Se lee el votaje de la bateria
  uint32_t bvalue = 0;
  uint16_t n = 1000;
  for(uint16_t i = 0; i < n; i++) {
    ADC0_SC1A = adc_config[4];
    while(!(ADC0_SC1A & ADC_SC1_COCO));
    bvalue += ADC0_RA;
  }
  bvalue /= n;

  // se para la conversion
  ADC0_SC1A = adc_config[3];

  // Si estaba apagado el PGA se apaga:
  if ((powMaskOld & PGA_MASK) == 0) setPowerDown(PGA_MASK);

  // se reconecta el DMA
  cfgDMA();

  float mV_value = 2.5*bvalue/65536.0;
  return 21*mV_value;
}
//----------------------------------------------------------------------
// User callback handler
void callbackhandler()
{
  setSyncProvider(getTeensy3Time);
}

uint32_t deep_sleep()
{
  if (!(gcCfgStat & GC_CFG_READ)) return 0;

  // reset lp_cfg
  memset(&lp_cfg, 0, sizeof(sleep_block_t));

  // OR together different wake sources
  lp_cfg.modules = GPIO_WAKE;

  // GPIO alarm wakeup
  lp_cfg.gpio_pin = WAKE_USB;

  // user callback function
  lp_cfg.callback = callbackhandler;

  if (HIGH == digitalRead(PIN_USB)) return 0;

  // sleep
  uint32_t powMaskOld = powMask;
  setPowerDown(SD_MASK|XBEE_MASK|PGA_MASK|EXT_MASK);
  lp.DeepSleep(&lp_cfg);
  setPowerUp(powMaskOld);

  return 0;
}

uint32_t sleep_chrono()
{
  if (!(gcCfgStat & GC_CFG_READ)) return 0;

  // reset lp_cfg
  memset(&lp_cfg, 0, sizeof(sleep_block_t));

  // OR together different wake sources
  lp_cfg.modules = (GPIO_WAKE | RTCA_WAKE);

  // GPIO alarm wakeup
  lp_cfg.gpio_pin = WAKE_USB;

  // RTC alarm wakeup in seconds:
  lp_cfg.rtc_alarm = gc_cfg.time_begin_seg;
  uint32_t dseg = gc_cfg.time_end_seg;

  // user callback function
  lp_cfg.callback = callbackhandler;

  if (HIGH == digitalRead(PIN_USB)) return 0;

  // sleep
  if (lp_cfg.rtc_alarm > 0) {
    uint32_t powMaskOld = powMask;
    setPowerDown(SD_MASK|XBEE_MASK|PGA_MASK|EXT_MASK);
    lp.DeepSleep(&lp_cfg);
    setPowerUp(powMaskOld);

    if (lp_cfg.wake_source == WAKE_USB) {
      return 0;
    } else {
      // sync RTC with the GPS
      syncGps();
    }
  }

  return dseg*(1000000/gc_cfg.tick_time_useg);
}

uint32_t sleep_daily()
{
  if (!(gcCfgStat & GC_CFG_READ)) return 0;

  // reset lp_cfg
  memset(&lp_cfg, 0, sizeof(sleep_block_t));

  // OR together different wake sources
  lp_cfg.modules = (GPIO_WAKE | RTCA_WAKE);

  // GPIO alarm wakeup
  lp_cfg.gpio_pin = WAKE_USB;

  uint32_t time_n = Teensy3Clock.get() % SEG_A_DAY;
  uint32_t dseg = 0;

  // RTC alarm wakeup in seconds:
  if (gc_cfg.time_begin_seg < gc_cfg.time_end_seg) {
    dseg = gc_cfg.time_end_seg - gc_cfg.time_begin_seg;
    if (time_n <= gc_cfg.time_begin_seg) {
      lp_cfg.rtc_alarm = gc_cfg.time_begin_seg - time_n;
    } else if (time_n >= gc_cfg.time_end_seg) {
      lp_cfg.rtc_alarm = (gc_cfg.time_begin_seg + SEG_A_DAY) - time_n;
    } else {
      lp_cfg.rtc_alarm = 0;
      dseg = gc_cfg.time_end_seg - time_n;
    }
  } else {
    if (time_n < gc_cfg.time_end_seg) {
      lp_cfg.rtc_alarm = 0;
      dseg = gc_cfg.time_end_seg - time_n;
    } else if (time_n > gc_cfg.time_begin_seg) {
      lp_cfg.rtc_alarm = 0;
      dseg = (gc_cfg.time_end_seg + SEG_A_DAY) - time_n;
    } else {
      lp_cfg.rtc_alarm = gc_cfg.time_begin_seg - time_n;
      dseg = (gc_cfg.time_end_seg + SEG_A_DAY) - gc_cfg.time_begin_seg;
    }
  }

  // user callback function
  lp_cfg.callback = callbackhandler;

  if (HIGH == digitalRead(PIN_USB)) return 0;

  // sleep
  if (lp_cfg.rtc_alarm > 0) {
    uint32_t powMaskOld = powMask;
    setPowerDown(SD_MASK|XBEE_MASK|PGA_MASK|EXT_MASK);
    lp.DeepSleep(&lp_cfg);
    setPowerUp(powMaskOld);

    if (lp_cfg.wake_source == WAKE_USB) {
      return 0;
    } else { // GPS rtc sync first!
      // sync RTC with the GPS
      syncGps();
    }
  }

  return dseg*(1000000/gc_cfg.tick_time_useg);
}
//----------------------------------------------------------------------
boolean files_close();
boolean gcStop()
{
  if (!(gcPlayStat & GC_ST_STOP)) return false;

  if (!(gcPlayStat & GC_ST_FILE_OPEN)) {
    gcPlayStat &= ~GC_ST_STOP;
    return true;
  }

  // stop adc_play
  adc_play.end();

  int8_t wadc = 3;
  while (ADC0_SC1A != adc_config[3] && wadc-- > 0) {
    gc_println(PSTR("warning:gc_stop: waiting for the ADC!"));
    delay(LOG_TIMEOUT);
  }

  // save tail
  while (delta > 0) {
    if (sd_head == GC_SD_BUFFER_SIZE) {
      file.write(sd_buffer, GC_SD_BUFFER_SIZE_BYTES);
      sd_head = 0;
    }

    sd_buffer[sd_head++] = adc_ring_buffer[tail++];

    tail &= GC_ADC_RING_SIZE_HASH;
    delta--;
  }
  file.write(sd_buffer, sd_head * sizeof(uint16_t));
  qfile.write(quake_list, quake_head * sizeof(uint32_t));

  // save errors
  file.write((uint8_t*)&buffer_errors, sizeof(uint32_t)); // 4
  file.write((uint8_t*)&adc_errors, sizeof(uint32_t)); // +4 = 8

  // save adc_rtc_stop
  file.write((uint8_t*)&adc_rtc_stop, sizeof(uint32_t)); // +4 = 12

  // save act_play_cnt
  file.write((uint32_t*)&adc_play_cnt, sizeof(uint32_t)); // +4 = 16

  // write float vbat:
  float vbat = getVBat();
  file.write((uint8_t*)&vbat, sizeof(float)); // +4 = 20

  // write float temp:
  float temp = getTemp();
  file.write((uint8_t*)&temp, sizeof(float)); // +4 = 24

  // file timestamp
  file.timestamp(T_WRITE, year(), month(), day(), hour(), minute(), second());
  file.timestamp(T_ACCESS, year(), month(), day(), hour(), minute(), second());
  qfile.timestamp(T_WRITE, year(), month(), day(), hour(), minute(), second());
  qfile.timestamp(T_ACCESS, year(), month(), day(), hour(), minute(), second());

  if (!files_close()) goto fail;

  setPowerDown(PGA_MASK|SD_MASK);
  gcPlayStat &= ~GC_ST_STOP;
  return true;

fail:
  setPowerDown(PGA_MASK|SD_MASK);
  gcPlayStat &= ~GC_ST_STOP;
  return false;
}

boolean files_close()
{
  boolean close_file = true;
  boolean close_qfile = true;

  if (file.isOpen()) {
    if (file.getWriteError()) {
      gc_println(PSTR("error:stop: write file!"));
      if (!file.close()) {
        gc_println(PSTR("error:stop: close file!"));
      }
      close_file = false;
    }
    if (!file.close()) {
      gc_println(PSTR("error:stop: close file!"));
      close_file = false;
    }
  }

  if (qfile.isOpen()) {
    if (qfile.getWriteError()) {
      gc_println(PSTR("error:stop: write qfile!"));
      if (!qfile.close()) {
        gc_println(PSTR("error:stop: close qfile!"));
      }
      close_qfile = false;
    }
    if (!qfile.close()) {
      gc_println(PSTR("error:stop: close qfile!"));
      close_qfile = false;
    }
  }

  gcPlayStat &= ~GC_ST_FILE_OPEN;
  return (close_file && close_qfile);
}

boolean file_cfg()
{
  // create a new file
  static char filename[FILENAME_MAX_LENGH+1] = FILENAME_FORMAT;
  filename[FILENAME_MAX_LENGH-3] = 'X';
  filename[FILENAME_MAX_LENGH-2] = 'Y';
  filename[FILENAME_MAX_LENGH-1] = 'Z';
  static uint32_t n = 0;
  for (; n < pow(10, FILENAME_MAX_LENGH-(FILENAME_NO_DIGITS+FILENAME_EXT));) {
    for (uint8_t i = 0; i < (FILENAME_MAX_LENGH-(FILENAME_NO_DIGITS+FILENAME_EXT)); i++) {
      uint32_t pw = pow(10, i+1);
      filename[(FILENAME_MAX_LENGH-FILENAME_EXT)-(i+1)] = '0' + ((n%pw)/(pw/10));
    }

    if (file.open(filename, O_CREAT | O_EXCL | O_TRUNC | O_WRITE)) {
      file.timestamp(T_CREATE, year(), month(), day(), hour(), minute(), second());
      file.timestamp(T_WRITE, year(), month(), day(), hour(), minute(), second());
      file.timestamp(T_ACCESS, year(), month(), day(), hour(), minute(), second());

      filename[FILENAME_MAX_LENGH-3] = 'T';
      filename[FILENAME_MAX_LENGH-2] = 'R';
      filename[FILENAME_MAX_LENGH-1] = 'G';

      qfile.open(filename, O_CREAT | O_TRUNC | O_WRITE);
      qfile.timestamp(T_CREATE, year(), month(), day(), hour(), minute(), second());
      qfile.timestamp(T_WRITE, year(), month(), day(), hour(), minute(), second());
      qfile.timestamp(T_ACCESS, year(), month(), day(), hour(), minute(), second());

      n++;
      if (n == pow(10, FILENAME_MAX_LENGH-(FILENAME_NO_DIGITS+FILENAME_EXT))) n = 0;
      break;
    }

    n++;
    if (n == pow(10, FILENAME_MAX_LENGH-(FILENAME_NO_DIGITS+FILENAME_EXT))) n = 0;
  }

  if (!file.isOpen()) {
    gc_println(PSTR("log:file open failed"));

    file.timestamp(T_CREATE, 1980, month(), day(), hour(), minute(), second());
    file.timestamp(T_WRITE, 1980, month(), day(), hour(), minute(), second());
    file.timestamp(T_ACCESS, 1980, month(), day(), hour(), minute(), second());
    file.close();

    qfile.timestamp(T_CREATE, 1980, month(), day(), hour(), minute(), second());
    qfile.timestamp(T_WRITE, 1980, month(), day(), hour(), minute(), second());
    qfile.timestamp(T_ACCESS, 1980, month(), day(), hour(), minute(), second());
    qfile.close();

    gcPlayStat &= ~GC_ST_FILE_OPEN;
    return false;
  } else {
    // write uint8_t file format
    file.write((uint8_t)FILE_FORMAT); // 1

    // write uint32_t SIM_UIDH
    file.write((uint8_t*)&SIM_UIDH, sizeof(uint32_t)); // +4 = 5

    // write uint32_t SIM_UIDMH
    file.write((uint8_t*)&SIM_UIDMH, sizeof(uint32_t)); // +4 = 9

    // write uint32_t SIM_UIDML
    file.write((uint8_t*)&SIM_UIDML, sizeof(uint32_t)); // +4 = 13

    // write uint32_t SIM_UIDL
    file.write((uint8_t*)&SIM_UIDL, sizeof(uint32_t)); // +4 = 17

    // write uint16_t GC_ADC_VREF in mV and bits:
    uint16_t bitsVref = ((GC_ADC_BITS << 12) & 0xF000) | (GC_ADC_VREF & 0x0FFF);
    file.write((uint8_t*)&bitsVref, sizeof(uint16_t)); // +2 = 19

    // write float sensitivity:
    file.write((uint8_t*)&gc_cfg.sensitivity, sizeof(float)); // +4 = 23

    // write uint16_t trigger_level:
    file.write((uint8_t*)&gc_cfg.trigger_level, sizeof(uint16_t)); // +2 = 25

    // write uint32_t trigger_time_number:
    file.write((uint8_t*)&gc_cfg.trigger_time_number, sizeof(uint32_t)); // +4 = 29

    // write uint8_t ((gain << 4)| average):
    file.write((uint8_t)((gc_cfg.gain << 4) | gc_cfg.average)); // +1 = 30

    // write uint32_t tick_time_useg:
    file.write((uint8_t*)&gc_cfg.tick_time_useg, sizeof(uint32_t)); // +4 = 34

    // write uint32_t time_begin_seg:
    file.write((uint8_t*)&gc_cfg.time_begin_seg, sizeof(uint32_t)); // +4 = 38

    // write uint32_t time_end_seg:
    file.write((uint8_t*)&gc_cfg.time_end_seg, sizeof(uint32_t)); // +4 = 42

    // write float vbat:
    float vbat = getVBat();
    file.write((uint8_t*)&vbat, sizeof(float)); // +4 = 46

    // write float temp:
    float temp = getTemp();
    file.write((uint8_t*)&temp, sizeof(float)); // +4 = 50

    gcPlayStat |= GC_ST_FILE_OPEN;
    return true;
  }
}

void adc_play_callback()
{
  if (adc_play_cnt-- == 0) {
    adc_play_cnt = 0;
    adc_rtc_stop = Teensy3Clock.get();
    stop_reading();
    return;
  }

  if (ADC0_SC1A != adc_config[3]) {
    adc_errors++;
    adc_play_cnt++;
    return;
  }

  if (delta+3 >= GC_ADC_RING_SIZE) {
    buffer_errors++;
    adc_play_cnt++;
    return;
  }

  delta += 3;

#if TRIGGER_BY_SOFTWARE == 0
#elif TRIGGER_BY_SOFTWARE >= 1
  uint16_t *adc_value = (uint16_t*)DMA_TCD1_DADDR;
#endif

  DMA_TCD2_SADDR = &(adc_config[1]);
  ADC0_SC1A = adc_config[0];

  if (adc_play_cnt_quake > adc_play_cnt) {
#if TRIGGER_BY_SOFTWARE == 0
    if (ADC1_SC1A != adc_config[3]) {
      if (adc_play_cnt > gc_cfg.trigger_time_number)
        adc_play_cnt_quake = adc_play_cnt - gc_cfg.trigger_time_number;
      else
        adc_play_cnt_quake = 0;

      quake_list[quake_head++] = adc_play_cnt+2;
      quake_head &= (QUAKE_LIST_LENGTH-1);
    }

    DMA_TCD3_SADDR = &(adc_config[11]);
    ADC1_SC1A = adc_config[10];
#elif TRIGGER_BY_SOFTWARE == 1
    uint16_t* x = (uint16_t*)(((uint32_t)(adc_value-3) | adc_ring_buffer_mmin) & adc_ring_buffer_mmax);
    uint16_t* y = (uint16_t*)(((uint32_t)(adc_value-2) | adc_ring_buffer_mmin) & adc_ring_buffer_mmax);
    uint16_t* z = (uint16_t*)(((uint32_t)(adc_value-1) | adc_ring_buffer_mmin) & adc_ring_buffer_mmax);
    if (quake_min > *x || *x > quake_max ||
        quake_min > *y || *y > quake_max ||
        quake_min > *z || *z > quake_max) {
      if (adc_play_cnt > gc_cfg.trigger_time_number)
        adc_play_cnt_quake = adc_play_cnt - gc_cfg.trigger_time_number;
      else
        adc_play_cnt_quake = 0;

      quake_list[quake_head++] = adc_play_cnt+2;
      quake_head &= (QUAKE_LIST_LENGTH-1);
    }
#endif
  }
}

boolean gcStart()
{
  if (gcPlayStat & GC_ST_READING) return false;

  uint32_t cfg_all = GC_CFG_READ|GC_CFG_ADC|GC_CFG_DMA;
  if (!((gcCfgStat & cfg_all)==cfg_all)) return false;

  // restart
  delta = 0; tail = 0; sd_head = 0;
  adc_errors = 0; buffer_errors = 0;
  adc_rtc_stop = 0;

#if TRIGGER_BY_SOFTWARE == 0
#elif TRIGGER_BY_SOFTWARE == 1
  quake_min = 0x0000 + (gc_cfg.trigger_level+1);
  quake_max = 0xFFFF - gc_cfg.trigger_level;
#endif
  quake_head = 0;
  if (gc_cfg.trigger_level > 0 || gc_cfg.trigger_time_number > 0) adc_play_cnt_quake = adc_play_cnt-1;
  else adc_play_cnt_quake = 0;

  setPowerUp(PGA_MASK|SD_MASK);

  // configure file
  if (file_cfg()) { // => gcPlayStat += GC_ST_FILE_OPEN
    // read rtc time
    uint32_t rtc_time = Teensy3Clock.get();

    // write uint32_t rtc_time_sec:
    sd_buffer[sd_head++] = ((uint16_t*)&rtc_time)[0];
    sd_buffer[sd_head++] = ((uint16_t*)&rtc_time)[1]; // +4 = 54

    // write uint32_t adc_play_cnt:
    sd_buffer[sd_head++] = ((uint16_t*)&adc_play_cnt)[0];
    sd_buffer[sd_head++] = ((uint16_t*)&adc_play_cnt)[1]; // +4 = 58

    // configure adc_play
    sd_buffer[sd_head++] = 0;
    sd_buffer[sd_head++] = 0; // +4 = 62

    if (!adc_play.begin(adc_play_callback, gc_cfg.tick_time_useg)) {
      adc_play.end();

      sd_buffer[sd_head-1] = 61153;

      if (!files_close()) goto fail;

      gc_println(PSTR("error:start: adc_play!"));
      goto fail;
    }

    gcPlayStat |= GC_ST_READING;
    return true;
  } else { // gcPlayStat -= GC_ST_FILE_OPEN
    gc_println(PSTR("error:start: file!"));
    goto fail;
  }

fail:
  setPowerDown(PGA_MASK|SD_MASK);
  return false;
}
//-----------------------------------------------------------------------
void gc_cmd(uint8_t* cmd, uint8_t n)
{
  uint8_t head[9] = { '\xaa', '\xaa', '\xaa',
                      '\xff', '\xff', '\xff',
                      '\x00', '\x00', '\x00'};
  Serial.write(head, 9);
  Serial.write(cmd, n);
}

void gc_cmd(uint8_t cmd)
{
  uint8_t head[10] = { '\xaa', '\xaa', '\xaa',
                      '\xff', '\xff', '\xff',
                      '\x00', '\x00', '\x00', cmd};
  Serial.write(head, 10);
}

void gc_print(const char *log_string)
{
  uint8_t cmd[1] = { 't' };
  gc_cmd(cmd, 1);

  Serial.print(log_string);
}

void gc_println(const char *log_string)
{
  uint8_t cmd[1] = { 't' };
  gc_cmd(cmd, 1);

  Serial.println(log_string);
}

void printDigits(int digits, boolean first)
{
  // utility function for digital clock display: prints preceding colon and leading 0
  if (!first) Serial.print(F(":"));
  if(digits < 10)
    Serial.print(F("0"));
  Serial.print(digits);
}

void digitalClockDisplay()
{
  // digital clock display of the time:
  Serial.print(year());
  Serial.print(F("-"));
  Serial.print(month());
  Serial.print(F("-"));
  Serial.print(day());
  Serial.print(F(" "));
  printDigits(hour(), true);
  printDigits(minute(), false);
  printDigits(second(), false);
  Serial.println();
  //Serial.print(F(" "));
  //Serial.println(Teensy3Clock.get() % SEG_A_DAY);
}
