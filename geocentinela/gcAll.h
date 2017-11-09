#include "dcodec.h"
//-------------------------------
rInstrument geocentinela;
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
#define GC_ADC0_BAT GC_ADC0_A10
#define GC_ADC1_BAT GC_ADC1_A10
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

volatile uint32_t adc_config[18] = {
  ADC_SC1_ADCH(GC_ADC0_A2), // ADC0_ch0
  ADC_SC1_ADCH(GC_ADC0_A5), // ADC0_ch1
  ADC_SC1_ADCH(GC_ADC0_A9), // ADC0_ch2
  ADC_SC1_ADCH(31), // stop=0b11111=31
  ADC_SC1_ADCH(GC_ADC0_BAT), // read battery state
  ADC_SC1_ADCH(31), // stop=0b11111=31
  ADC_SC1_ADCH(GC_ADC0_AZ), // read zero vref
  ADC_SC1_ADCH(31), // stop=0b11111=31
  ADC_SC1_ADCH(GC_ADC1_A10), //  A2 in gc v3.4 ADC1_ch0
  ADC_SC1_ADCH(GC_ADC1_A10), //  A3 in gc v3.4 ADC1_ch1
  ADC_SC1_ADCH(GC_ADC1_A10), // A10 in gc v3.4 ADC1_ch2
  ADC_SC1_ADCH(31), // stop=0b11111=31
  ADC_SC1_ADCH(GC_ADC1_BAT), // read zero vref
  ADC_SC1_ADCH(31), // stop=0b11111=31
  ADC_SC1_ADCH(GC_ADC1_AZ), // read zero vref
  ADC_SC1_ADCH(31), // stop=0b11111=31
  ADC_SC1_ADCH(GC_ADC_TEMP), // read temperature
  ADC_SC1_ADCH(31)  // stop=0b11111=31
};
//--------------------------------------------------
#define SD_CS   10
#define SD_MOSI 11
#define SD_MISO 12
#define SD_SCK  13
SdFat sd;
SdFile file;
#define GC_SD_BUFFER_SIZE 0x1000
#define GC_SD_BUFFER_SIZE_BYTES (GC_SD_BUFFER_SIZE*2) // 2 == sizeof(uint16_t)
uint16_t sd_buffer[GC_SD_BUFFER_SIZE];
//--------------------------------------------------
SdFile tfile;
#define TRIGGER_LIST_LENGTH 0x100
#define TRIGGER_LIST_LENGTH_BYTES TRIGGER_LIST_LENGTH*4 // 4 == sizeof(uint32_t)
uint32_t trigger_list[TRIGGER_LIST_LENGTH];
volatile uint32_t adc_play_cnt_trigger_check;
volatile uint16_t trigger_list_head;
volatile uint32_t trigger_time_cnt;
volatile uint32_t send_trigger_time_cnt;
#define TRIGGER_BY_SOFTWARE 1

#if TRIGGER_BY_SOFTWARE == 0
#elif TRIGGER_BY_SOFTWARE == 1
const uint32_t adc_ring_buffer_mmin = (uint32_t)&(adc_ring_buffer[0]);
const uint32_t adc_ring_buffer_mmax = (uint32_t)&(adc_ring_buffer[0]) + (GC_ADC_RING_SIZE_BYTES-1);
volatile uint16_t trigger_min;
volatile uint16_t trigger_max;
#endif

SdFile synclog;
SdFile syncaux;
#define FILENAME_SYNC_LOG "SYNC.LOG"
#define FILENAME_SYNC_AUX "SYNC.AUX"

SdFile cfile;
#define FILENAME_CACHE_EVF "CACHE.EVF"

SdFile syslog;
SdFile sysaux;
#define FILENAME_SYS_LOG "SYS.LOG"
#define FILENAME_SYS_AUX "SYS.AUX"
//--------------------------------------------------
IntervalTimer adc_play;
//--------------------------------------------------
volatile uint16_t delta = 0;
volatile uint16_t tail = 0;
volatile uint16_t sd_head = 0;

volatile uint32_t adc_errors = 0;
volatile uint32_t buffer_errors = 0;

volatile uint32_t adc_play_cnt = 0;
volatile uint32_t adc_play_stop = 0;
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
#define FILE_FORMAT 0x06
#define FILE_HEAD 84
#define FILE_TAIL 24
#define FILENAME_FORMAT "GC000000.???"
#define FILENAME_MAX_LENGH 12 // 8.3 filename
#define FILENAME_NO_DIGITS 2  // GC000000
#define FILENAME_EXT 4        // .???
char filename[FILENAME_MAX_LENGH+1] = FILENAME_FORMAT;

#define PIN_USB 30
#define WAKE_USB PIN_30
//--------------------------------------------------
#define SDX_POW 03 // 3.3V
#define PGA_POW 22 // 5Va
#define EXT_POW 02 // 3.3VP
#define M95_MASK      0b001000
#define SD_MASK       0b000100
#define PGA_MASK      0b000010
#define EXT_MASK      0b000001
#define ALL_MASK      0b001111
#define POWER_UP_MASK 0b100000
volatile uint8_t powMask = 0;
//--------------------------------------------------
#define GAIN_CS 05
#define GAIN_A0 18
#define GAIN_A1 14
#define GAIN_A2 17
//--------------------------------------------------
TinyGPS gps;
#define GPS_RTC_SYNC_TIME 5*60*1000 // 5min
#define GPS_PPS 04
#define GPS_IO Serial1
#define GPS_RX 00
#define GPS_TX 01
//--------------------------------------------------
#define LOG_TIMEOUT 1000
//--------------------------------------------------
#define XBEE_nSLEEP_ON 20
#define XBEE_SLEEP_RQ   9
#define XBEE_nRESET    06

#define XBEE_IO Serial3
#define XBEE_RX  8
#define XBEE_TX 07

#define XBEE_nRTS 15
#define XBEE_nCTS 21

#include "M95.h"
M95 m95(Serial3,
  (char*)adc_ring_buffer, GC_ADC_RING_SIZE_BYTES,
  (char*)sd_buffer, GC_SD_BUFFER_SIZE_BYTES);
//--------------------------------------------------
void stop_reading();
void setPGA(uint8_t g);
//--------------------------------------------------
void setPowerDown(uint8_t mask)
{
  if ((mask & M95_MASK) > 0) {
    if (!(powMask & M95_MASK)) goto out_m95;

    if (m95.powerDown()) powMask &= ~M95_MASK;
  }
out_m95:

  if ((mask & SD_MASK) > 0) {
    if (!(powMask & SD_MASK)) goto out_sd;

    while (sd.card()->isBusy()); delay(1000);
    pinMode(SD_CS, OUTPUT); digitalWrite(SD_CS, LOW);
    pinMode(SD_SCK, OUTPUT); digitalWrite(SD_SCK, LOW);
    pinMode(SD_MOSI, OUTPUT); digitalWrite(SD_MOSI, LOW);

    powMask &= ~SD_MASK;
  }
out_sd:

  if ((mask & PGA_MASK) > 0) {
    if (!(powMask & PGA_MASK)) goto out_pga;

    pinMode(GAIN_CS, OUTPUT); digitalWrite(GAIN_CS, LOW);
    pinMode(PGA_POW, OUTPUT); digitalWrite(PGA_POW, LOW);
    powMask &= ~PGA_MASK;
  }
out_pga:

  if ((mask & EXT_MASK) > 0) {
    if (!(powMask & EXT_MASK)) goto out_ext;

    pinMode(EXT_POW, OUTPUT); digitalWrite(EXT_POW, LOW);
    powMask &= ~EXT_MASK;
  }
out_ext:

  return;
}

void setPowerUp(uint8_t mask)
{
  if ((mask & M95_MASK) > 0) {
    if (powMask & M95_MASK) goto out_m95;

    if (m95.powerUp(geocentinela)) powMask |= M95_MASK;
  }
out_m95:

  if ((mask & SD_MASK) > 0) {
    if (powMask & SD_MASK) goto out_sd;

    pinMode(SDX_POW, OUTPUT); digitalWrite(SDX_POW, HIGH);
    delay(200);

    sd = SdFat();
    while(!sd.begin(SD_CS, SPI_FULL_SPEED)) {
      geocentinela.println(PSTR("error: sdcard"));
      delay(LOG_TIMEOUT);
    }
    while (sd.card()->isBusy()); delay(200);
    sd.chvol();

    powMask |= SD_MASK;
  }
out_sd:

  if ((mask & PGA_MASK) > 0) {
    if (powMask & PGA_MASK) goto out_pga;

    pinMode(PGA_POW, OUTPUT); digitalWrite(PGA_POW, HIGH);
    delay(200);

    powMask |= PGA_MASK;
    setPGA(geocentinela.c.getGain());
  }
out_pga:

  if ((mask & EXT_MASK) > 0) {
    if (powMask & EXT_MASK) goto out_ext;

    pinMode(EXT_POW, OUTPUT); digitalWrite(EXT_POW, HIGH);
    delay(200);

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
/*
static boolean gps2rtcSync()
{
  uint32_t fix_age;
  int Year;
  uint8_t Month, Day, Hour, Minute, Second;
  gps.crack_datetime(&Year, &Month, &Day, &Hour, &Minute, &Second, NULL, &fix_age);
  if (fix_age < 500) {
    // set the Time to the latest GPS reading
    setTime(Hour, Minute, Second, Day, Month, Year);
    adjustTime(geocentinela.getTimeZoneOffset());
  } else return false;

  float flat, flon;
  gps.f_get_position(&flat, &flon, &fix_age);
  if (fix_age < 500 and fix_age != TinyGPS::GPS_INVALID_AGE) {
    return true;
  }

  return false;
}
*/

/*
void syncGps()
{
  if (!geocentinela.getGps()) return;
  return;

  // GPS power on
  setPowerUp(EXT_MASK);

  cfgGps();

  // sync
  char c = 0;
  uint8_t timeIni = millis();
  boolean rtcSync = false;
  while (!rtcSync and (millis() - timeIni) <= GPS_RTC_SYNC_TIME and (gcPlayStat & GC_ST_CONFIG) == 0) {
    while (GPS_IO.available() > 0 and !rtcSync) {
      c = GPS_IO.read();
      Serial.write(c);
      if (gps.encode(c)) rtcSync = gps2rtcSync();
    }
  }

  // GPS power off
  setPowerDown(EXT_MASK);
}
*/
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
//--------------------------------------------------
void setPGA(uint8_t gain)
{
  geocentinela.c.setGain(gain);

  if (!(powMask & PGA_MASK)) return;

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
void readCfg()
{
  // se lee la configuracion desde la EEPROM
  while(!geocentinela.read()) {
    while(!geocentinela.write()) {
      geocentinela.println(PSTR("error:readCfg: rw"));
      delay(LOG_TIMEOUT);
      if (geocentinela.read()) break;
    }
  }
  gcCfgStat |= GC_CFG_READ;
}
//--------------------------------------------------
time_t getTeensy3Time()
{
  return RTC_TSR + geocentinela.getTimeZoneOffset();
}
void rtc_seconds_isr(void)
{
  setSyncProvider(getTeensy3Time);
  if (timeStatus() == timeSet) {
    NVIC_DISABLE_IRQ(IRQ_RTC_SECOND);
    RTC_IER &= ~0x10;

    geocentinela.println(PSTR("RTC setSyncProvider!"));
  } else geocentinela.println(PSTR("error: RTC!"));
}
void cfgRTC()
{
  // set the TSIE bit (Time Seconds Interrupt Enable)
  RTC_IER |= 0x10;
  NVIC_ENABLE_IRQ(IRQ_RTC_SECOND);
  // rtc_seconds_isr();
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
void cfgSD()
{
  // se configura la alimentacion
  pinMode(SDX_POW, OUTPUT);
  digitalWrite(SDX_POW, LOW);
  powMask &= ~SD_MASK;

  // se configura los IO
  pinMode(SD_CS, OUTPUT); digitalWrite(SD_CS, LOW);
  pinMode(SD_MOSI, OUTPUT); digitalWrite(SD_MOSI, LOW);
  pinMode(SD_MISO, INPUT);
  pinMode(SD_SCK, OUTPUT); digitalWrite(SD_SCK, LOW);
}
//--------------------------------------------------
boolean cfgM95()
{
  // se configura los IO
  pinMode(XBEE_nSLEEP_ON, INPUT);
  pinMode(XBEE_SLEEP_RQ, INPUT);
  pinMode(XBEE_nRESET, INPUT);
  pinMode(XBEE_nRTS, INPUT);
  pinMode(XBEE_nCTS, INPUT);
  pinMode(XBEE_RX, INPUT);
  pinMode(XBEE_TX, INPUT);

  if (m95.cfg()) {
    if (!m95.powerStatus()) powMask &= ~M95_MASK;
    else powMask |= M95_MASK;

    return true;
  }

  return false;
}
//--------------------------------------------------
void cfgGPS()
{
  // se configura la alimentacion
  pinMode(EXT_POW, OUTPUT); digitalWrite(EXT_POW, LOW);

  // se configura el PPS:
  pinMode(GPS_PPS, INPUT_PULLDOWN);

  // se sincroniza el GPS:
  // syncGps();
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
#if F_BUS == 96000000
  #define ADC_CFG1_16BIT ADC_CFG1_ADIV(2) + ADC_CFG1_ADICLK(1) // 12 MHz
  #define ADC_CFG1_16BIT_CAL ADC_CFG1_ADIV(3) + ADC_CFG1_ADICLK(1) // 6 MHz
#elif F_BUS == 60000000
  #define ADC_CFG1_16BIT ADC_CFG1_ADIV(1) + ADC_CFG1_ADICLK(1) // 15 MHz (overclock)
  #define ADC_CFG1_16BIT_CAL ADC_CFG1_ADIV(3) + ADC_CFG1_ADICLK(1) // 3.75 MHz
#elif F_BUS == 48000000
  #define ADC_CFG1_16BIT  ADC_CFG1_ADIV(1) | ADC_CFG1_ADICLK(1) // 12MHz
  #define ADC_CFG1_16BIT_CAL ADC_CFG1_ADIV(3) | ADC_CFG1_ADICLK(1) // 3MHz
#elif F_BUS == 24000000
  #define ADC_CFG1_16BIT  ADC_CFG1_ADIV(1) | ADC_CFG1_ADICLK(0) // 12MHz
  #define ADC_CFG1_16BIT_CAL ADC_CFG1_ADIV(3) | ADC_CFG1_ADICLK(0) // 3MHz
#else
#error "F_BUS must be 96, 60, 48 or 24 MHz"
#endif

// general configuration
#define GC_ADC_CFG1 0 \
    /* | ADC_CFG1_ADLPC     /* lower power: off, on */ \
    /* | ADC_CFG1_ADIV(1)   /  clock divide: 1, 2, 4, 8 */ \
    /* | ADC_CFG1_ADLSMP    /* sample time: short, long */ \
    | ADC_CFG1_MODE(3)      /* bit resolution mode: 8, 12, 10, 16 */ \
    /* | ADC_CFG1_ADICLK(1) / input clock: bus, bus/2, alternate, asynchronous */ \
    | ADC_CFG1_16BIT        /* set adc_clock, i.e: ADC_CFG1_ADIV and ADC_CFG1_ADICLK */

#define GC_ADC_CFG2 0 \
    | ADC_CFG2_MUXSEL    /* adc mux (see man. pag. 98 Connections/Channel Assignment): ADxxa, ADxxb */ \
    /*| ADC_CFG2_ADACKEN   /* asynchronous clock output: disable, enable */ \
    | ADC_CFG2_ADHSC     /* high speed configuration: normal, high */ \
    | ADC_CFG2_ADLSTS(0) /* long sample time: 20ext, 12ext, 6ext, 2ext */

// control
#define GC_ADC_SC2 0 \
    /* | ADC_SC2_ADTRG                 / trigger select: software, hardware */ \
    /* | ADC_SC2_ACFE                  / compare function: disable, enable */ \
    /* | ADC_SC2_ACFGT                 / compare function greater than: disable, enable */ \
    /* | ADC_SC2_ACREN                 / compare function range: disable, enable */ \
    /* | ADC_SC2_DMAEN                 / DMA enable */ \
    | ADC_SC2_REFSEL(GC_ADC_REFSEL) /* 0->3.3v, 1->1.2v */

#define GC_ADC_SC3 0 \
      /* | ADC_SC3_ADCO    / continuous conversion: disable, enable */ \
      | ADC_SC3_AVGE    /* enable hardware average */ \
      | ADC_SC3_AVGS(3) /* average select: 0->4, 1->8, 2->16, 3->32 4->0 */

#define NAVERAGE 1024 // < 65536
#define NLOOPS 8 // < 65536

void calADC0()
{
  // ADC clock
  SIM_SCGC6 |= SIM_SCGC6_ADC0;

  // general configuration
  ADC0_CFG1 = 0
    | ADC_CFG1_ADLSMP
    | ADC_CFG1_MODE(3)
    | ADC_CFG1_16BIT_CAL;
  ADC0_CFG2 = GC_ADC_CFG2;

  // control
  ADC0_SC2 = GC_ADC_SC2;
  ADC0_SC3 = GC_ADC_SC3;

  uint32_t pMask = powMask;

  PMC_REGSC |= PMC_REGSC_BGBE;
  setPowerUp(PGA_MASK);
  delay(1000);

  // calibration
  ADC0_SC3 |= ADC_SC3_CAL; // begin cal

  uint16_t cal_sum;
  while ((ADC0_SC3 & ADC_SC3_CAL) and (ADC0_SC1A & ADC_SC1_COCO));

  cal_sum = (ADC0_CLPS + ADC0_CLP4 + ADC0_CLP3 + ADC0_CLP2 + ADC0_CLP1 + ADC0_CLP0)/2;
  ADC0_PG = cal_sum | 0x8000;

  cal_sum = (ADC0_CLMS + ADC0_CLM4 + ADC0_CLM3 + ADC0_CLM2 + ADC0_CLM1 + ADC0_CLM0)/2;
  ADC0_MG = cal_sum | 0x8000;

#if 1
  // autozero:
  uint32_t vavrg = 2;
  for (uint16_t j = 0; j < NLOOPS and vavrg > 1; j++) {
    vavrg = 0;
    for (uint16_t i = 0; i < NAVERAGE; i++) {
      ADC0_SC1A = ADC_SC1_ADCH(GC_ADC0_AZ);
      while (!(ADC0_SC1A & ADC_SC1_COCO));
      vavrg += ADC0_RA;
    }
    vavrg /= NAVERAGE;

    boolean ofs_sig = vavrg > (uint32_t)GC_ADC0_AZV;
    if (ofs_sig) {
      vavrg = (vavrg - (uint32_t)GC_ADC0_AZV);
      ADC0_OFS +=  ((int16_t)(vavrg >> 1));
    } else {
      vavrg = ((uint32_t)GC_ADC0_AZV - vavrg);
      ADC0_OFS += -((int16_t)(vavrg >> 1));
    }
  }
#endif

  // Stop conversion
  ADC0_SC1A = ADC_SC1_ADCH(0b11111);

  setPowerDown(~pMask);
  setPowerUp(pMask);
  PMC_REGSC &= ~PMC_REGSC_BGBE;
}

void calADC1()
{
  // ADC clock
  SIM_SCGC3 |= SIM_SCGC3_ADC1;

  // general configuration
  ADC1_CFG1 = 0
    | ADC_CFG1_ADLSMP
    | ADC_CFG1_MODE(3)
    | ADC_CFG1_16BIT_CAL;
  ADC1_CFG2 = GC_ADC_CFG2;

  // control
  ADC1_SC2 = GC_ADC_SC2;
  ADC1_SC3 = GC_ADC_SC3;

  uint32_t pMask = powMask;

  PMC_REGSC |= PMC_REGSC_BGBE;
  setPowerUp(PGA_MASK);

  // calibration
  ADC1_SC3 |= ADC_SC3_CAL; // begin cal

  uint16_t cal_sum;
  while ((ADC1_SC3 & ADC_SC3_CAL) and (ADC1_SC1A & ADC_SC1_COCO));

  cal_sum = (ADC1_CLPS + ADC1_CLP4 + ADC1_CLP3 + ADC1_CLP2 + ADC1_CLP1 + ADC1_CLP0)/2;
  ADC1_PG = cal_sum | 0x8000;

  cal_sum = (ADC1_CLMS + ADC1_CLM4 + ADC1_CLM3 + ADC1_CLM2 + ADC1_CLM1 + ADC1_CLM0)/2;
  ADC1_MG = cal_sum | 0x8000;

#if TRIGGER_BY_SOFTWARE == 0
  // autozero:
  uint32_t vavrg = 2;
  for (uint16_t j = 0; j < NLOOPS and vavrg > 1; j++) {
    vavrg = 0;
    for (uint16_t i = 0; i < NAVERAGE; i++) {
      ADC1_SC1A = ADC_SC1_ADCH(GC_ADC1_AZ);
      while (!(ADC1_SC1A & ADC_SC1_COCO));
      vavrg += ADC1_RA;
    }
    vavrg /= NAVERAGE;
    boolean ofs_sig = vavrg > (uint32_t)GC_ADC1_AZV;
    if (ofs_sig) {
      vavrg = (vavrg - (uint32_t)GC_ADC1_AZV);
      ADC1_OFS +=  ((int16_t)(vavrg >> 1));
    } else {
      vavrg = ((uint32_t)GC_ADC1_AZV - vavrg);
      ADC1_OFS += -((int16_t)(vavrg >> 1));
    }
  }
#endif

  // Stop conversion
  ADC1_SC1A = ADC_SC1_ADCH(0b11111);

  setPowerDown(~pMask);
  setPowerUp(pMask);
  PMC_REGSC &= ~PMC_REGSC_BGBE;
}

void cfgADC0()
{
  // general configuration
  ADC0_CFG1 = GC_ADC_CFG1;
  ADC0_CFG2 = GC_ADC_CFG2;

  // control
  ADC0_SC2 = GC_ADC_SC2 | ADC_SC2_DMAEN; // DMA enable

  uint32_t sc3 = 0; // continuous conversion disable, hardware average disable
  if (geocentinela.c.getHardwareAverage() < 4) {
    sc3 = 0
      //| ADC_SC3_ADCO                 // continuous conversion: disable, enable
      | ADC_SC3_AVGE                 // enable hardware average
      | ADC_SC3_AVGS(geocentinela.c.getHardwareAverage()) // average select: 0->4, 1->8, 2->16, 3->32
    ;
  }
  ADC0_SC3 = sc3;
}

void cfgADC1()
{
  // general configuration
  ADC1_CFG1 = GC_ADC_CFG1;
  ADC1_CFG2 = GC_ADC_CFG2;

  // control
#if TRIGGER_BY_SOFTWARE == 0
  if (0 == geocentinela.c.getTriggerLevel() or 0 == geocentinela.c.getTriggerTime()) {
    ADC1_SC2 = GC_ADC_SC2;

    ADC1_CV1 = 0;
    ADC1_CV2 = 0;
  } else {
    ADC1_SC2 = ADC_SC2_ACFE | ADC_SC2_ACFGT | ADC_SC2_ACREN | ADC_SC2_DMAEN | GC_ADC_SC2;

    ADC1_CV1 = 0x0000 + (geocentinela.c.getTriggerLevel()+1);
    ADC1_CV2 = 0xFFFF - geocentinela.c.getTriggerLevel();
  }
#else
  ADC1_SC2 = GC_ADC_SC2;
#endif
  ADC1_SC3 = GC_ADC_SC3;
}

boolean cfgADC()
{
  if (!(gcCfgStat & GC_CFG_READ)) {
    gcCfgStat &= ~GC_CFG_ADC;
    return false;
  }

  // clear DMA:
  clearDMA();

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

  // ADC0
  calADC0();
  cfgADC0();

  // ADC1
  calADC1();
  cfgADC1();

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
  DMA_TCD2_SADDR = &(adc_config[1]); // ADC0_ch1
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
  DMA_TCD3_SADDR = &(adc_config[9]); ADC1_ch1
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
  uint32_t pMask = powMask;
  setPowerUp(PGA_MASK);

  // Se lee la temperatura
  uint32_t bvalue = 0;
  uint16_t n = 1000; // < 2**16
  for(uint16_t i = 0; i < n; i++) {
    ADC0_SC1A = ADC_SC1_ADCH(GC_ADC_TEMP);
    while(!(ADC0_SC1A & ADC_SC1_COCO));

    bvalue += ADC0_RA;
  }

  // se para la conversion
  ADC0_SC1A = ADC_SC1_ADCH(0b11111); // stop == 0b11111

  // Si estaba apagado el PGA se apaga:
  setPowerDown(~pMask);
  setPowerUp(pMask);

  // se reconecta el DMA
  cfgDMA();

  float mV_value = GC_ADC_VREF*(bvalue/(65536.0*n));
  return 25.0+((719 - mV_value)/1.715);// Temp = 25 - ((mV_temp - mV_temp25)/m)
}
//----------------------------------------------------------------------
float getVBat()
{
  if (!(gcCfgStat & GC_CFG_ADC)) return 0;

  // Se desconecta el DMA
  clearDMA();

  // Se prende el voltage de referencia
  uint32_t pMask = powMask;
  setPowerUp(PGA_MASK);

  // Se lee el votaje de la bateria
  uint32_t bvalue = 0;
  uint16_t n = 1000;
  for(uint16_t i = 0; i < n; i++) {
    ADC0_SC1A = ADC_SC1_ADCH(GC_ADC0_BAT);; // ADC?_BAT
    while(!(ADC0_SC1A & ADC_SC1_COCO));

    bvalue += ADC0_RA;
  }

  // se para la conversion
  ADC0_SC1A = ADC_SC1_ADCH(0b11111); // stop == 0b11111

  // Si estaba apagado el PGA se apaga:
  setPowerDown(~pMask);
  setPowerUp(pMask);

  // se reconecta el DMA
  cfgDMA();

  float mV_value = (GC_ADC_VREF/1000.0)*(bvalue/(65536.0*n));
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
  uint32_t pMask = powMask;
  setPowerDown(ALL_MASK);

  lp.DeepSleep(&lp_cfg);

  setPowerDown(~pMask);
  setPowerUp(pMask);

  return 0;
}

uint32_t sleep_chrono()
{
  if (!(gcCfgStat & GC_CFG_READ)) return 0;

  // reset lp_cfg
  memset(&lp_cfg, 0, sizeof(sleep_block_t));

  // user callback function
  lp_cfg.callback = callbackhandler;

  // OR together different wake sources
  lp_cfg.modules = (GPIO_WAKE | RTCA_WAKE);

  // GPIO alarm wakeup
  lp_cfg.gpio_pin = WAKE_USB;

  // RTC alarm wakeup in seconds:
  lp_cfg.rtc_alarm = geocentinela.c.getTimeBegin();
  uint32_t dseg = geocentinela.c.getTimeEnd();

  if (HIGH == digitalRead(PIN_USB)) return 0;

  if (lp_cfg.rtc_alarm > 0) {
    uint32_t pMask = powMask;
    setPowerDown(ALL_MASK);

    lp.DeepSleep(&lp_cfg);

    setPowerDown(~pMask);
    setPowerUp(pMask);

    if (lp_cfg.wake_source == WAKE_USB) {
      return 0;
    }
  }

  if (HIGH == digitalRead(PIN_USB)) return 0;

  return dseg*(1000000/geocentinela.c.getTickTime());
}

boolean gcSyncEvents(boolean & updated_cfg);
uint32_t sleep_daily()
{
  if (!(gcCfgStat & GC_CFG_READ)) return 0;

  boolean sended = false;

  // reset lp_cfg
  memset(&lp_cfg, 0, sizeof(sleep_block_t));

  // user callback function
  lp_cfg.callback = callbackhandler;

  // OR together different wake sources
  lp_cfg.modules = (GPIO_WAKE | RTCA_WAKE);

  // GPIO alarm wakeup
  lp_cfg.gpio_pin = WAKE_USB;

begin_daily:
  if (HIGH == digitalRead(PIN_USB)) return 0;

  // sync:
  if (geocentinela.getGprs()) {
    boolean updated_cfg = false;
    boolean sync_events = gcSyncEvents(updated_cfg);

    if (updated_cfg) setup();
    if (!sended) sended = sync_events;

    if (HIGH == digitalRead(PIN_USB)) return 0;
  }

  // times in local:
  uint32_t time_begin = geocentinela.c.getTimeBegin();
  uint32_t time_end = geocentinela.c.getTimeEnd();

  // times to UTC
  time_begin += (uint32_t)(SEG_A_DAY - geocentinela.getTimeZoneOffset());
  time_end += (uint32_t)(SEG_A_DAY - geocentinela.getTimeZoneOffset());

  // times to SEG_A_DAY
  time_begin %= SEG_A_DAY;
  time_end %= SEG_A_DAY;

  // set alarm:
  uint32_t time_now = RTC_TSR % SEG_A_DAY;
  uint32_t dseg = 0;

  // RTC alarm wakeup in seconds:
  if (time_begin < time_end) {
    dseg = time_end - time_begin;
    if (time_now <= time_begin) {
      lp_cfg.rtc_alarm = time_begin - time_now;
    } else if (time_now >= time_end) {
      lp_cfg.rtc_alarm = (time_begin + SEG_A_DAY) - time_now;
    } else {
      lp_cfg.rtc_alarm = 0;
      dseg = time_end - time_now;
    }
  } else {
    if (time_now < time_end) {
      lp_cfg.rtc_alarm = 0;
      dseg = time_end - time_now;
    } else if (time_now > time_begin) {
      lp_cfg.rtc_alarm = 0;
      dseg = (time_end + SEG_A_DAY) - time_now;
    } else {
      lp_cfg.rtc_alarm = time_begin - time_now;
      dseg = (time_end + SEG_A_DAY) - time_begin;
    }
  }

  if (0 == lp_cfg.rtc_alarm) return dseg*(1000000.0/geocentinela.c.getTickTime());

  // fix rtc_alarm
  if (geocentinela.getGprs() and !sended) {
    if (lp_cfg.rtc_alarm > geocentinela.c.getSendTriggerTime()) {
      lp_cfg.rtc_alarm = geocentinela.c.getSendTriggerTime();
    }
  }

  if (0 == lp_cfg.rtc_alarm) goto begin_daily;

  // sleep:
  uint32_t pMask = powMask;
  setPowerDown(ALL_MASK);

  lp.DeepSleep(&lp_cfg);

  setPowerDown(~pMask);
  setPowerUp(pMask);

  goto begin_daily;
}
//----------------------------------------------------------------------
boolean syncSensor()
{
  if (!geocentinela.getGprs()) return false;
  if (!(powMask & M95_MASK)) return false;

  uint8_t sync_cnt = 0;
  boolean to_search;

  uint8_t rest_size = 29;
  char rest[rest_size];

try_sensor:
  to_search = true;
  if (sync_cnt >= 3) return false;

  // set default name:
  if (0 == strlen(geocentinela.s.getName())) {
    uint32_t oldTimeStamp = geocentinela.s.getUpdatedAt();

    uint32_t name_size = 9*4;
    char name[name_size];
    snprintf(name, name_size, "%lx.%lx.%lx.%lx", SIM_UIDH, SIM_UIDMH, SIM_UIDML, SIM_UIDL);
    geocentinela.s.setName(name);

    geocentinela.s.setUpdatedAt(oldTimeStamp);
  }

post_sensor:
  if (sync_cnt >= 3) return false;

  if (!to_search) {
    float lng, lat, cta;
    geocentinela.s.getLocation(lng, lat, cta);
    if (0.0 == lng and 0.0 == lat and 0.0 == cta and m95.cellloc(lng, lat)) {
      srandom(micros());
      cta = random(1000, 100000)/1000.0;
      geocentinela.s.setLocation(lng, lat, cta);
    }
  }

  snprintf(rest, rest_size, "POST /sensors"); // len(rest)=14

put_sensor:
  if (sync_cnt >= 3) return false;

  // jsonificando en el buffer
  m95.cleanStringIO();
  geocentinela.s.serialize(m95.getStringIO(), m95.getStringIOLength(),
                           (uint8_t*)adc_ring_buffer, GC_ADC_RING_SIZE_BYTES, to_search);

  // enviando...
  uint16_t stateHTTP = m95.httpJsonREST(geocentinela.getHostName(), rest);

  // guardando...
  if (1 == (stateHTTP & 0x1) and (stateHTTP >> 1) > 0) {
    rSensor s;
    s.set(geocentinela.s);

    Serial.print("0_locat_s:");Serial.println(geocentinela.s.getUpdatedAt());
    Serial.print("0_new_s:");Serial.println(s.getUpdatedAt());

    if (s.deserialize(m95.getStringIO(), (uint8_t*)adc_ring_buffer, GC_ADC_RING_SIZE_BYTES)) {
      if (0 == geocentinela.s.getId()) geocentinela.s.setId(s.getId());

    Serial.print("1_locat_s:");Serial.println(geocentinela.s.getUpdatedAt());
    Serial.print("1_new_s:");Serial.println(s.getUpdatedAt());

      if (!geocentinela.s.updatedAt(s.getUpdatedAt())) {
        geocentinela.s.set(s);
        geocentinela.write();
      } else if (!s.updatedAt(geocentinela.s.getUpdatedAt())) {
        snprintf(rest, rest_size, "PUT /sensors/%lu.json", geocentinela.s.getId()); // len(rest)<=29

        to_search = false;

        // enviando...
        goto put_sensor;
      }
    } else {
      sync_cnt++;

      if (422 == (stateHTTP >> 1)) {
        to_search = false;
        goto post_sensor;
      }

      goto try_sensor;
    }
  } else return false;

  return true;
}
//----------------------------------------------------------------------
boolean syncConfigure(boolean & updated_cfg)
{
  if (!geocentinela.getGprs()) return false;
  if (!(powMask & M95_MASK)) return false;

  uint8_t sync_cnt = 0;
  boolean to_search;

  uint8_t rest_size = 32;
  char rest[rest_size];

try_configure:
  to_search = true;
  if (sync_cnt >= 3) return false;

  // set default name:
  if (0 == strlen(geocentinela.c.getName())) {
    uint32_t oldTimeStamp = geocentinela.c.getUpdatedAt();

    uint32_t name_size = 9*4;
    char name[name_size];
    snprintf(name, name_size, "%lx.%lx.%lx.%lx", SIM_UIDH, SIM_UIDMH, SIM_UIDML, SIM_UIDL);
    geocentinela.c.setName(name);

    geocentinela.c.setUpdatedAt(oldTimeStamp);
  }

post_configure:
  if (sync_cnt >= 3) return false;

  snprintf(rest, rest_size, "POST /configures"); // len(rest)=17

put_configure:
  if (sync_cnt >= 3) return false;

  // jsonificando en el buffer
  m95.cleanStringIO();
  geocentinela.c.serialize(m95.getStringIO(), m95.getStringIOLength(),
                           (uint8_t*)adc_ring_buffer, GC_ADC_RING_SIZE_BYTES, to_search);

  // enviando...
  uint16_t stateHTTP = m95.httpJsonREST(geocentinela.getHostName(), rest);

  // guardando...
  if (1 == (stateHTTP & 0x1) and (stateHTTP >> 1) > 0) {
    rConfigure c;
    c.set(geocentinela.c);

    Serial.print("0_locat_c:");Serial.println(geocentinela.c.getUpdatedAt());
    Serial.print("0_new_c:");Serial.println(c.getUpdatedAt());

    if (c.deserialize(m95.getStringIO(), (uint8_t*)adc_ring_buffer, GC_ADC_RING_SIZE_BYTES)) {
      if (0 == geocentinela.c.getId()) geocentinela.c.setId(c.getId());

    Serial.print("1_locat_c:");Serial.println(geocentinela.c.getUpdatedAt());
    Serial.print("1_new_c:");Serial.println(c.getUpdatedAt());

      if (!geocentinela.c.updatedAt(c.getUpdatedAt())) {
          geocentinela.c.set(c);
          geocentinela.write();
          updated_cfg = true;
      } else if (!c.updatedAt(geocentinela.c.getUpdatedAt())) {
        snprintf(rest, rest_size, "PUT /configures/%lu.json", geocentinela.c.getId()); // len(rest)<=32

        to_search = false;

        // enviando...
        goto put_configure;
      }
    } else {
      sync_cnt++;

      if (422 == (stateHTTP >> 1)) {
        to_search = false;
        goto post_configure;
      }

      goto try_configure;
    }
  } else return false;

  return true;
}
//----------------------------------------------------------------------
boolean syncInstrument(boolean & updated_cfg)
{
  if (!geocentinela.getGprs()) return false;
  if (!(powMask & M95_MASK)) return false;

  uint8_t sync_cnt = 0;
  boolean to_search;

  boolean sync_configure_sensor = false;

  uint8_t rest_size = 33;
  char rest[rest_size];

try_instrument:
  to_search = true;
  if (sync_cnt >= 3) return false;

post_instrument:
  if (sync_cnt >= 3) return false;

  snprintf(rest, rest_size, "POST /instruments"); // len(rest)=18

put_instrument:
  if (sync_cnt >= 3) return false;

  // jsonificando en el buffer
  m95.cleanStringIO();
  geocentinela.serialize(m95.getStringIO(), m95.getStringIOLength(),
                         (uint8_t*)adc_ring_buffer, GC_ADC_RING_SIZE_BYTES, to_search);

  // enviando...
  uint16_t stateHTTP = m95.httpJsonREST(geocentinela.getHostName(), rest);

  // guardando...
  if (1 == (stateHTTP & 0x1) and (stateHTTP >> 1) > 0) {
    rInstrument gc;
    gc.set(geocentinela);

    Serial.print("0_locat_i:");Serial.println(geocentinela.getUpdatedAt());
    Serial.print("0_locat_c:");Serial.println(geocentinela.c.getUpdatedAt());
    Serial.print("0_locat_s:");Serial.println(geocentinela.s.getUpdatedAt());

    Serial.print("0_new_i:");Serial.println(gc.getUpdatedAt());
    Serial.print("0_new_c:");Serial.println(gc.c.getUpdatedAt());
    Serial.print("0_new_s:");Serial.println(gc.s.getUpdatedAt());

    if (gc.deserialize(m95.getStringIO(), (uint8_t*)adc_ring_buffer, GC_ADC_RING_SIZE_BYTES)) {
      if (0 == geocentinela.getId()) geocentinela.setId(gc.getId());

    Serial.print("1_locat_i:");Serial.println(geocentinela.getUpdatedAt());
    Serial.print("1_locat_c:");Serial.println(geocentinela.c.getUpdatedAt());
    Serial.print("1_locat_s:");Serial.println(geocentinela.s.getUpdatedAt());

    Serial.print("1_new_i:");Serial.println(gc.getUpdatedAt());
    Serial.print("1_new_c:");Serial.println(gc.c.getUpdatedAt());
    Serial.print("1_new_s:");Serial.println(gc.s.getUpdatedAt());

      if (!geocentinela.updatedAt(gc.getUpdatedAt())) {
        geocentinela.set(gc);
        geocentinela.write();
        updated_cfg = true;
      } else if (!gc.updatedAt(geocentinela.getUpdatedAt())) {
        snprintf(rest, rest_size, "PUT /instruments/%lu.json", geocentinela.getId()); // len(rest)<=33

        to_search = false;

        // enviando...
        goto put_instrument;
      }
    } else {
      sync_cnt++;

      if (422 == (stateHTTP >> 1)) {
        if (!sync_configure_sensor) {
          if (!syncConfigure(updated_cfg)) return false;
          if (!syncSensor()) return false;
          sync_configure_sensor = true;
        }

        to_search = false;
        goto post_instrument;
      }

      goto try_instrument;
    }
  } else return false;

  if (!sync_configure_sensor) {
    if (!syncConfigure(updated_cfg)) return false;
    if (!syncSensor()) return false;
  }
  return true;
}
//----------------------------------------------------------------------
boolean syncFatFile(SdFile & file)
{
  if (!geocentinela.getGprs()) return false;
  if (!(powMask & M95_MASK) or !(powMask & SD_MASK)) return false;

  if (0 == geocentinela.getId()) return false;

  uint8_t sync_cnt = 0;
  boolean to_search;

  uint8_t rest_size = 34;
  char rest[rest_size];

  rFatFile fatfile;
  fatfile.setInstrumentId(geocentinela.getId());

try_fat_file:
  to_search = true;
  if (sync_cnt >= 3) return false;

  // set:
  fatfile.setFile(file);

post_fat_file:
  if (sync_cnt >= 3) return false;

  snprintf(rest, rest_size, "POST /fat_files"); // len(rest)=16

put_fat_file:
  if (sync_cnt >= 3) return false;

  // jsonificando en el buffer
  m95.cleanStringIO();
  fatfile.serialize(m95.getStringIO(), m95.getStringIOLength(), (uint8_t*)adc_ring_buffer, GC_ADC_RING_SIZE_BYTES, to_search);
  Serial.print("file: ");
  Serial.println(m95.getStringIO());

  // enviando...
  uint16_t stateHTTP = m95.httpJsonREST(geocentinela.getHostName(), rest);

  // guardando...
  if (1 == (stateHTTP & 0x1) and 204 == (stateHTTP >> 1)) return true; // delete! 204 == No Content

  if (1 == (stateHTTP & 0x1) and (stateHTTP >> 1) > 0) {
    rFatFile faux;

    if (faux.deserialize(m95.getStringIO(), (uint8_t*)adc_ring_buffer, GC_ADC_RING_SIZE_BYTES)) {
      if (fatfile.getUpdatedAt() < 2*(faux.getUpdatedAt() >> 1)) { // 2 seconds file datetime granularity
        switch (faux.getAction()) {
          case 0: { // nada
            uint32_t lepoch = faux.getUpdatedAt()+geocentinela.getTimeZoneOffset();

            file.timestamp(T_WRITE | T_ACCESS, year(lepoch), month(lepoch), day(lepoch),
                                               hour(lepoch), minute(lepoch), second(lepoch));
            fatfile.setFile(file);
          } break;
          case 1: { // borrar
            if (file.isOpen()) file.close();

            fatfile.set(faux);
            if (sd.remove(fatfile.getName())) { // se debe borrar en el servidor
              snprintf(rest, rest_size, "DELETE /fat_files/%lu.json", fatfile.getId()); // len(rest)<=34

              to_search = false;

              sync_cnt++;
              goto put_fat_file;
            } else {
              fatfile.setAction(2);
              snprintf(rest, rest_size, "PUT /fat_files/%lu.json", fatfile.getId()); // len(rest)<=31

              to_search = false;

              sync_cnt++;
              goto put_fat_file;
            }
          } break;
          case 2: { // reprocesar
            // no se hace nada por el momento...
          } break;
          case 3: { // send
            // no se hace nada por el momento...
          } break;
        }
      } else if (fatfile.getUpdatedAt() > 2*(faux.getUpdatedAt() >> 1)) { // 2 seconds file datetime granularity
        snprintf(rest, rest_size, "PUT /fat_files/%lu.json", faux.getId()); // len(rest)<=31

        to_search = false;

        // enviando...
        sync_cnt++;
        goto put_fat_file;
      }
    } else {
      sync_cnt++;

      if (422 == (stateHTTP >> 1)) {
        to_search = false;
        goto post_fat_file;
      }

      goto try_fat_file;
    }
  } else return false;

  return true;
}
//----------------------------------------------------------------------
uint32_t time2Number(uint32_t secondsTime, uint32_t tickTime)
{
  return (secondsTime*1000000)/tickTime;
}
//----------------------------------------------------------------------
#define HEADER_BITS 17
#define HEADER_VREF 18
#define HEADER_TZOFFSET 20
#define HEADER_SENSITIVITY 24
#define HEADER_GAIN 40
#define HEADER_TICK_TIME 42
#define HEADER_TRIGGER_TIME 56
#define HEADER_RTC 72
#define HEADER_NCONF 76

#define TAIL_NRCONF 12

boolean gcSaveSyncEvents()
{
  if (0 == geocentinela.c.getTriggerLevel() or 0 == geocentinela.c.getTriggerTime()) return false;

  const uint32_t pMask = powMask;
  setPowerUp(SD_MASK);
  if (!(powMask & SD_MASK)) goto fail;

  snprintf(&filename[FILENAME_MAX_LENGH-3], 4, PSTR("TRG"));
  if (!tfile.open(filename, O_READ)) goto fail;

  snprintf(&filename[FILENAME_MAX_LENGH-3], 4, PSTR("XYZ"));
  if (!file.open(filename, O_READ)) goto fail;

  if (tfile.fileSize() > 0) {
    uint8_t header[FILE_HEAD];
    if (file.read(header, sizeof(header)) != sizeof(header)) goto fail;

    uint8_t tail[FILE_TAIL];
    if (!file.seekEnd(-FILE_TAIL) or file.read(tail, sizeof(tail)) != sizeof(tail)) goto fail;

    uint8_t fBits = header[HEADER_BITS];// factor
    uint16_t fVref = *((uint16_t*)&header[HEADER_VREF]);// factor
    float fSensitivity = *((float*)&header[HEADER_SENSITIVITY]);// factor
    uint8_t fGain = header[HEADER_GAIN];// factor

    float factor = (1000*fSensitivity)*(pow(2,fBits+fGain)/fVref);

    uint32_t fTickTime = *((uint32_t*)&header[HEADER_TICK_TIME]);// fTriggerTimeNumber
    uint32_t fTriggerTime = *((uint32_t*)&header[HEADER_TRIGGER_TIME]);// fTriggerTimeNumber

    uint32_t fTriggerTimeNumber = time2Number(fTriggerTime, fTickTime);
    
    uint32_t fRTCini = *((uint32_t*)&header[HEADER_RTC]);
    int32_t fTZOffset = *((int32_t*)&header[HEADER_TZOFFSET]);
    uint32_t fRTClocal = fRTCini + fTZOffset;

    uint32_t fNConf = *((uint32_t*)&header[HEADER_NCONF]);
    uint32_t fNRConf = *((uint32_t*)&tail[TAIL_NRCONF]);
    fNRConf = fNConf - fNRConf;

    // open sync.log file:
    uint32_t ts_flags = T_WRITE | T_ACCESS;
    if (!sd.exists(FILENAME_SYNC_LOG)) ts_flags |= T_CREATE;

    if (!synclog.open(FILENAME_SYNC_LOG, O_CREAT | O_APPEND | O_WRITE)) goto fail;
    synclog.timestamp(ts_flags, year(), month(), day(), hour(), minute(), second());

    uint8_t sync_event[SYNC_EVENT_SIZE];
    memset(sync_event, 0, SYNC_EVENT_SIZE);
    memcpy(&sync_event[SYNC_EVENT_FNAME], filename, FILENAME_MAX_LENGH);

    uint32_t trg = 0;
    uint32_t nsamples = 0;
    while (tfile.read(&trg, sizeof(uint32_t)) == sizeof(uint32_t)) {
      trg = fNConf - trg;
      file.seekSet(FILE_HEAD+3*sizeof(uint16_t)*trg);

      if (trg+fTriggerTimeNumber <= fNRConf) nsamples = fTriggerTimeNumber;
      else if (trg <= fNRConf) nsamples = fNRConf-trg;
      else nsamples = 0;

      uint32_t vmax = 0;
      uint32_t imax = 0;
      Sample sample;
      for (uint32_t i = 0; i < nsamples; i++) {
        if (file.read((uint8_t*)&sample, sizeof(Sample)) != sizeof(Sample)) goto fail;

        uint32_t
        value  = (uint32_t)(((int32_t)(sample.x-GC_ADC0_AZV))*((int32_t)(sample.x-GC_ADC0_AZV)));
        value += (uint32_t)(((int32_t)(sample.y-GC_ADC0_AZV))*((int32_t)(sample.y-GC_ADC0_AZV)));
        value += (uint32_t)(((int32_t)(sample.z-GC_ADC0_AZV))*((int32_t)(sample.z-GC_ADC0_AZV)));

        if (value > vmax) {
          vmax = value;
          imax = i;
        }
      }

      *((uint32_t*)&sync_event[SYNC_EVENT_RTC]) = fRTClocal+(uint32_t)(((uint64_t)(trg+imax)*fTickTime)/1000000);
      *((uint32_t*)&sync_event[SYNC_EVENT_MICRO]) = (uint32_t)(((uint64_t)(trg+imax)*fTickTime)%1000000);
      *((float*)&sync_event[SYNC_EVENT_PPV]) = (float)(sqrt(vmax)/factor);
      *((uint32_t*)&sync_event[SYNC_EVENT_INDEX]) = trg;
      *((uint32_t*)&sync_event[SYNC_EVENT_NSAMPLES]) = nsamples;

      synclog.write(sync_event, sizeof(sync_event));
    }

    // close sync.log file:
    synclog.close();
  } else goto fail;

  if (file.isOpen()) file.close();
  if (tfile.isOpen()) tfile.close();
  if (synclog.isOpen()) synclog.close();

  setPowerDown(~pMask);
  setPowerUp(pMask);
  return true;

fail:
  if (file.isOpen()) file.close();
  if (tfile.isOpen()) tfile.close();
  if (synclog.isOpen()) synclog.close();

  setPowerDown(~pMask);
  setPowerUp(pMask);
  return false;
}
//----------------------------------------------------------------------
boolean getFileCacheId(char* const& id, size_t const& id_size)
{
  StaticSharedJsonBuffer jsonBuffer((uint8_t*)adc_ring_buffer, GC_ADC_RING_SIZE_BYTES);
  JsonObject& object = jsonBuffer.parseObject(m95.getStringIO());

  if (!object.success()) return false;
  if (!object["id"].success()) return false;

  memset(id, 0, id_size);
  strncpy(id, object["id"], id_size);

  return true;
}
//----------------------------------------------------------------------
boolean syncEventPpv(uint8_t sync_event[SYNC_EVENT_SIZE])
{
  if (!geocentinela.getGprs()) return false;
  if (!(powMask & M95_MASK)) return false;

  uint8_t sync_cnt = 0;
  boolean to_search;

  uint8_t rest_size = 28;
  char rest[rest_size];

  rEvent event;

try_event:
  to_search = true;
  if (sync_cnt >= 3) return false;

  event.read(sync_event, geocentinela);

post_event:
  if (sync_cnt >= 3) return false;

  snprintf(rest, rest_size, "POST /events"); // len(rest)=13

put_event:
  if (sync_cnt >= 3) return false;

  // jsonificando en el buffer
  m95.cleanStringIO();
  event.serialize(m95.getStringIO(), m95.getStringIOLength(),
                  (uint8_t*)adc_ring_buffer, GC_ADC_RING_SIZE_BYTES, to_search);

  // enviando...
  uint16_t stateHTTP = m95.httpJsonREST(geocentinela.getHostName(), rest);

  // guardando...
  if (1 == (stateHTTP & 0x1) and (stateHTTP >> 1) > 0) {
    rEvent e;
    if (e.deserialize(m95.getStringIO(), (uint8_t*)adc_ring_buffer, GC_ADC_RING_SIZE_BYTES)) {
      if (0 == event.getId()) event.setId(e.getId());

      if (!event.updatedAt(e.getUpdatedAt())) {
        event.set(e);
        event.write(sync_event);
      } else if (!e.updatedAt(event.getUpdatedAt())) {
        snprintf(rest, rest_size, "PUT /events/%lu.json", event.getId()); // len(rest)<=28

        to_search = false;

        // enviando...
        goto put_event;
      }
    } else {
      sync_cnt++;

      if (422 == (stateHTTP >> 1)) {
        to_search = false;
        goto post_event;
      }

      goto try_event;
    }
  } else return false;

  return true;
}
//----------------------------------------------------------------------
boolean syncEventFileCache(uint8_t sync_event[SYNC_EVENT_SIZE])
{
  if (!geocentinela.getGprs()) return false;
  if (!(powMask & M95_MASK)) return false;

  uint8_t sync_cnt = 0;
  boolean to_search;

  uint8_t rest_size = 33;
  char rest[rest_size];

  rEventFile event_file;

try_event_file:
  to_search = true;
  if (sync_cnt >= 3) return false;

  event_file.read(sync_event);

post_event_file:
  if (sync_cnt >= 3) return false;

  snprintf(rest, rest_size, "POST /event_files"); // len(rest)=18

put_event_file:
  if (sync_cnt >= 3) return false;

  // jsonificando en el buffer
  m95.cleanStringIO();
  event_file.serialize(m95.getStringIO(), m95.getStringIOLength(),
                       (uint8_t*)adc_ring_buffer, GC_ADC_RING_SIZE_BYTES, to_search);

  // enviando...
  uint16_t stateHTTP = m95.httpJsonREST(geocentinela.getHostName(), rest);

  // guardando...
  if (1 == (stateHTTP & 0x1) and (stateHTTP >> 1) > 0) {
    rEventFile ef;
    if (ef.deserialize(m95.getStringIO(), (uint8_t*)adc_ring_buffer, GC_ADC_RING_SIZE_BYTES)) {
      if (0 == event_file.getId()) event_file.setId(ef.getId());

      if (!event_file.updatedAt(ef.getUpdatedAt())) {
        event_file.set(ef);
        event_file.write(sync_event);
      } else if (!ef.updatedAt(event_file.getUpdatedAt())) {
        snprintf(rest, rest_size, "PUT /event_files/%lu.json", event_file.getId()); // len(rest)<=33

        to_search = false;

        // enviando...
        goto put_event_file;
      }
    } else {
      sync_cnt++;

      if (422 == (stateHTTP >> 1)) {
        to_search = false;
        goto post_event_file;
      }

      goto try_event_file;
    }
  } else return false;

  return true;
}
//----------------------------------------------------------------------
// 0b01 -> sync event_ppv
// 0b10 -> sync event_data
uint8_t syncEventFile(uint8_t sync_event[SYNC_EVENT_SIZE])
{
  if (!geocentinela.getGprs()) return 0b00;
  if (!(powMask & M95_MASK) or !(powMask & SD_MASK)) return 0b00;

  rEventFile event_file;
  event_file.read(sync_event);

  uint8_t sync_resp = 0;
  if (0 == event_file.getEventId()) {
    if (syncEventPpv(sync_event)) {
      sync_resp |= 0b01;
      event_file.read(sync_event); // reload!
    } else goto return_resp;
  } else sync_resp |= 0b01;

  if (0 != event_file.getId()) {
    sync_resp |= 0b10;

    goto return_resp;
  }

  if (strlen(event_file.getFileCacheId()) == (FILE_ID_SIZE-1)) {
    if (syncEventFileCache(sync_event)) sync_resp |= 0b10;

    goto return_resp;
  }

  // Se codifica el archivo del evento
  char fname[FILENAME_MAX_LENGH+1];
  memset(fname, 0, FILENAME_MAX_LENGH+1);
  memcpy(fname, &sync_event[SYNC_EVENT_FNAME], FILENAME_MAX_LENGH);

  if (!file.open(fname, O_READ)) goto return_resp;
  if (!cfile.open(FILENAME_CACHE_EVF, O_CREAT | O_TRUNC | O_WRITE)) goto return_resp;
  cfile.timestamp(T_CREATE | T_WRITE | T_ACCESS, year(), month(), day(), hour(), minute(), second());

  uint8_t header[FILE_HEAD];
  if (file.read(header, sizeof(header)) != sizeof(header)) goto return_resp;

  uint32_t index, rtc, micro, tickTime, nsamples;

  index = *((uint32_t*)&sync_event[SYNC_EVENT_INDEX]);
  rtc = *((uint32_t*)&header[HEADER_RTC]) +
          (uint32_t)(((uint64_t)index*tickTime)/1000000);
  micro = (uint32_t)(((uint64_t)index*tickTime)%1000000);
  tickTime = *((uint32_t*)&header[HEADER_TICK_TIME]);
  nsamples = *((uint32_t*)&sync_event[SYNC_EVENT_NSAMPLES]);

  // write header:
  cfile.write(header[HEADER_BITS]);
  cfile.write((uint8_t*)&header[HEADER_VREF], sizeof(uint16_t));
  cfile.write(header[HEADER_GAIN]);
  cfile.write((uint8_t*)&header[HEADER_SENSITIVITY], sizeof(float));
  cfile.write((uint8_t*)&header[HEADER_TZOFFSET], sizeof(int32_t));
  cfile.write((uint8_t*)&rtc, sizeof(uint32_t));
  cfile.write((uint8_t*)&micro, sizeof(uint32_t));
  cfile.write((uint8_t*)&tickTime, sizeof(uint32_t));
  cfile.write((uint8_t*)&nsamples, sizeof(uint32_t));

  // write data:
  if (index > nsamples/2) index -= nsamples/2;
  else index = 0;

  if (!file.seekSet(FILE_HEAD+index*sizeof(Sample))) goto return_resp;

  Sample * samples;
  uint32_t samples_size;

  Delta * deltas;
  uint32_t deltas_size;

  samples = (Sample *)adc_ring_buffer;
  samples_size = GC_ADC_RING_SIZE_BYTES/sizeof(Sample);

  deltas = (Delta *)sd_buffer;
  deltas_size = GC_SD_BUFFER_SIZE_BYTES/sizeof(Delta);

  dCodec * dco;
  dco = new dCodec(samples, samples_size, deltas, deltas_size);
  dco->encodeFile(file, cfile, nsamples);
  delete dco;

  // close files
  if (file.isOpen()) file.close();
  if (cfile.isOpen()) cfile.close();
  // archivo del evento codificado!

  // send file:
  if (!cfile.open(FILENAME_CACHE_EVF, O_READ)) goto return_resp;

  uint32_t stateHTTP;
  stateHTTP = m95.httpRESTFile(geocentinela.getHostName(), "POST /attachments/cache", cfile);

  if (cfile.isOpen()) cfile.close();

  if (1 != (stateHTTP & 0x1) or 200 != (stateHTTP >> 1)) goto return_resp; // 200 == OK

  if (!getFileCacheId((char*)&sync_event[SYNC_EVENT_FILE_CACHE_ID], FILE_ID_SIZE-1)) goto return_resp;

  if (syncEventFileCache(sync_event)) sync_resp |= 0b10;

return_resp:
  if (file.isOpen()) file.close();
  if (cfile.isOpen()) cfile.close();

  return sync_resp;
}
//----------------------------------------------------------------------
boolean syncSysLog();
boolean gcSyncEvents(boolean & updated_cfg)
{
  rMessage mSysLog;
  mSysLog.setInstrumentId(geocentinela.getId());

  /*
   * Power On
   * 
   */
  const uint32_t pMask = powMask;

  // power on SD
  setPowerUp(SD_MASK);
  if (!(powMask & SD_MASK)) goto fail;

  // power on m95
  setPowerUp(M95_MASK);
  if (!(powMask & M95_MASK)) goto fail;

  // open syslog
  uint32_t ts_flags;
  ts_flags = T_WRITE | T_ACCESS;
  if (!sd.exists(FILENAME_SYS_LOG)) ts_flags |= T_CREATE;

  if (!syslog.open(FILENAME_SYS_LOG, O_CREAT | O_APPEND | O_WRITE)) goto fail;
  syslog.timestamp(ts_flags, year(), month(), day(), hour(), minute(), second());

  /*
   * sync RTC with NTP
   * 
   */
  int32_t dtSR;
  int16_t dtPR;
  double tdelay;
  if (!m95.ntpSyncUDP(dtSR, dtPR, tdelay)) {
    switch (m95.call(geocentinela.getPhoneWarning())) {
      case 1: // NO OK
      case 4: // NO ANSWER
      case 5: // NO CARRIER
      case 6: { // NO DIALTONE
        delay(10000);
        m95.call(geocentinela.getPhoneWarning());
      } break;
      case 2: // WRONG NUMBER
      case 3: // BUSY
      default: { // CALL
      } break;
    }
    goto fail;
  }

  mSysLog.setDateNow();
  mSysLog.setFrom("ntpsync");

  char msg[SMS_SIZE];
  snprintf(msg, SMS_SIZE,
           "dtSR: %+011ld dtPR: %+06d delay: %+5e offset: %+5e",
           dtSR, dtPR, tdelay, dtSR + dtPR/32768.0);
  mSysLog.setSMS(msg);

  // save NTP syslog
  m95.cleanStringIO();
  mSysLog.serialize(m95.getStringIO(), m95.getStringIOLength(), (uint8_t*)adc_ring_buffer, GC_ADC_RING_SIZE_BYTES, false);
  syslog.println(m95.getStringIO());

  /* 
   *  sync instrument
   *   
   */
  updated_cfg = false;
  if (!syncInstrument(updated_cfg)) {
    switch (m95.call(geocentinela.getPhoneWarning())) {
      case 1: // NO OK
      case 4: // NO ANSWER
      case 5: // NO CARRIER
      case 6: { // NO DIALTONE
        delay(10000);
        m95.call(geocentinela.getPhoneWarning());
      } break;
      case 2: // WRONG NUMBER
      case 3: // BUSY
      default: { // CALL
      } break;
    }
    goto fail;
  }

  mSysLog.setDateNow();
  mSysLog.setFrom("instrument");

  if (updated_cfg) {
    mSysLog.setSMS("sync instrument updated");
  } else {
    mSysLog.setSMS("sync instrument no updated");
  }

  // save sync instrument syslog
  m95.cleanStringIO();
  mSysLog.serialize(m95.getStringIO(), m95.getStringIOLength(), (uint8_t*)adc_ring_buffer, GC_ADC_RING_SIZE_BYTES, false);
  syslog.println(m95.getStringIO());

  /* 
   *  sync sd fat_files
   *  
   */
  boolean sync_sd_fat_files;
  sync_sd_fat_files = true;

  uint32_t file_pos;
  file_pos = 0;
  while (true) {
    sd.vwd()->seekSet(file_pos);
    if (!file.openNext(sd.vwd(), O_WRITE)) break;
    file_pos = sd.vwd()->curPosition();

    if (!syncFatFile(file)) sync_sd_fat_files = false;
    if (file.isOpen()) file.close();
  }

  mSysLog.setDateNow();
  mSysLog.setFrom("instrument");

  if (sync_sd_fat_files) mSysLog.setSMS("sdsync ok");
  else mSysLog.setSMS("sdsync fail");

  // save sync sd fat_files
  m95.cleanStringIO();
  mSysLog.serialize(m95.getStringIO(), m95.getStringIOLength(), (uint8_t*)adc_ring_buffer, GC_ADC_RING_SIZE_BYTES, false);
  syslog.println(m95.getStringIO());

  /*
   * Sync SYS.LOG
   * 
   */
  // close syslog
  if (!syslog.close()) goto fail;

  // sync syslog
  syncSysLog();

  /*
   * sync events and events data
   * 
   */
  boolean sync_event_ok;
  sync_event_ok = true;

  boolean sync_event_data_ok;
  sync_event_data_ok = true;

  boolean sync_only_events;
  sync_only_events = true;
  
  // check sync.log file is ok and open it
synclogOpen:
  if (!synclog.open(FILENAME_SYNC_LOG, O_READ)) {
    if (sd.exists(FILENAME_SYNC_LOG)) goto fail;

    if (sync_only_events) {
      if (!synclog.open(FILENAME_SYNC_LOG, O_CREAT | O_TRUNC | O_WRITE)) goto fail;
      synclog.timestamp(T_CREATE | T_WRITE | T_ACCESS, year(), month(), day(), hour(), minute(), second());
      if (!synclog.close()) goto fail;
    } else goto ok;

    goto synclogOpen;
  }

  // Si no hay nada que sincronizar
  if (synclog.fileSize() == 0) goto ok;

  // check sync.aux file is ok and open it
  if (!syncaux.open(FILENAME_SYNC_AUX, O_CREAT | O_TRUNC | O_WRITE)) goto fail;
  syncaux.timestamp(T_CREATE | T_WRITE | T_ACCESS, year(), month(), day(), hour(), minute(), second());

  uint8_t sync_event[SYNC_EVENT_SIZE];

  // send PPVs or data!
  while (synclog.read(sync_event, sizeof(sync_event)) == sizeof(sync_event)) {
    if (sync_only_events) { // primero se envían sólo los ppv
      if (!syncEventPpv(sync_event)) {
        sync_event_ok = false;
      }
      syncaux.write(sync_event, sizeof(sync_event));
    } else { // luego se envia la data
      if (!(syncEventFile(sync_event) & 0b10)) {
        sync_event_data_ok = false;
        syncaux.write(sync_event, sizeof(sync_event));
      }
    }
  }

  if (!synclog.close()) goto fail;
  if (!syncaux.close()) goto fail;

  if (!sd.remove(FILENAME_SYNC_LOG)) goto fail;
  if (!sd.rename(FILENAME_SYNC_AUX, FILENAME_SYNC_LOG)) goto fail;

  if (sync_only_events) {
    sync_only_events = false;
    goto synclogOpen;
  }

  setPowerDown(~pMask);
  setPowerUp(pMask);

  return sync_event_ok && sync_event_data_ok;

ok:
  if (synclog.isOpen()) synclog.close();
  if (syncaux.isOpen()) syncaux.close();
  if (sd.exists(FILENAME_SYNC_AUX)) sd.remove(FILENAME_SYNC_AUX);

  setPowerDown(~pMask);
  setPowerUp(pMask);

  return true;

fail:
  if (synclog.isOpen()) synclog.close();
  if (syncaux.isOpen()) syncaux.close();
  if (sd.exists(FILENAME_SYNC_AUX)) sd.remove(FILENAME_SYNC_AUX);

  setPowerDown(~pMask);
  setPowerUp(pMask);

  return false;
}
//----------------------------------------------------------------------
boolean readline(SdFile & file, char * line, uint32_t max_size)
{
  for (uint32_t i = 0; i < max_size; i++) {
    line[i] = file.read();
    if ('\n' == line[i]) {
      line[i] = '\0';
      return true;
    }
  }
  line[max_size-1] = '\0';

  return false;
}
//----------------------------------------------------------------------
boolean syncSysLog()
{
  if (!geocentinela.getGprs()) return false;
  if (!(powMask & M95_MASK) or !(powMask & SD_MASK)) return false;

  /*
   * load files
   * 
   */
  // sys.log file is ok
  if (!syslog.open(FILENAME_SYS_LOG, O_READ)) {
    if (sd.exists(FILENAME_SYS_LOG)) goto fail;
    else goto ok;
  }

  // Si no hay nada que sincronizar
  if (syslog.fileSize() == 0) goto ok;

  // sys.aux file is ok
  if (!sysaux.open(FILENAME_SYS_AUX, O_CREAT | O_TRUNC | O_WRITE)) goto fail;
  sysaux.timestamp(T_CREATE | T_WRITE | T_ACCESS, year(), month(), day(), hour(), minute(), second());

  /*
   * Sync
   * 
   */
  boolean sync_ok;
  sync_ok = true;

  m95.cleanStringIO();
  while (readline(syslog, m95.getStringIO(), m95.getStringIOLength())) {
    uint8_t msg_str_size = strlen(m95.getStringIO());

    char msg_str[msg_str_size];
    memcpy(msg_str, m95.getStringIO(), msg_str_size);

    uint16_t stateHTTP = m95.httpJsonREST(geocentinela.getHostName(), "POST /messages");

    rMessage msg;
    if (1 != (stateHTTP & 0x1) or 201 != (stateHTTP >> 1) or // 201 == Created
        !msg.deserialize(m95.getStringIO(), (uint8_t*)adc_ring_buffer, GC_ADC_RING_SIZE_BYTES)) {
      sync_ok = false;
      sysaux.print(msg_str);
    }

    m95.cleanStringIO();
  }

  if (!syslog.close()) goto fail;
  if (!sysaux.close()) goto fail;

  if (!sd.remove(FILENAME_SYS_LOG)) goto fail;
  if (!sd.rename(FILENAME_SYS_AUX, FILENAME_SYS_LOG)) goto fail;

  return sync_ok;

ok:
  if (syslog.isOpen()) syslog.close();
  if (sysaux.isOpen()) sysaux.close();
  if (sd.exists(FILENAME_SYS_AUX)) sd.remove(FILENAME_SYS_AUX);

  return true;

fail:
  if (syslog.isOpen()) syslog.close();
  if (sysaux.isOpen()) sysaux.close();
  if (sd.exists(FILENAME_SYS_AUX)) sd.remove(FILENAME_SYS_AUX);

  return false;
}
//----------------------------------------------------------------------
boolean files_close()
{
  if (!(gcPlayStat & GC_ST_FILE_OPEN)) {
    setPowerDown(PGA_MASK|SD_MASK);
    return true;
  }

  boolean close_file = true;
  boolean close_tfile = true;

  if (file.isOpen() and !file.close()) {
    geocentinela.println(PSTR("error:stop: close file!"));
    close_file = false;
  }

  if (tfile.isOpen() and !tfile.close()) {
    geocentinela.println(PSTR("error:stop: close tfile!"));
    close_tfile = false;
  }

  gcPlayStat &= ~GC_ST_FILE_OPEN;
  setPowerDown(PGA_MASK|SD_MASK);
  return (close_file and close_tfile);
}

boolean gcStop()
{
  if (!(gcPlayStat & GC_ST_STOP)) return false;
  gcPlayStat &= ~GC_ST_STOP;

  if (!(gcPlayStat & GC_ST_FILE_OPEN)) {
    setPowerDown(PGA_MASK|SD_MASK);
    return true;
  }

  // stop adc_play
  adc_play.end();
  adc_rtc_stop = RTC_TSR;

  int8_t wadc = 3;
  while (ADC0_SC1A != ADC_SC1_ADCH(0b11111) and wadc-- > 0) {
    geocentinela.println(PSTR("warning:gc_stop: waiting for the ADC!"));
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
  tfile.write(trigger_list, trigger_list_head * sizeof(uint32_t));

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
  file.timestamp(T_WRITE | T_ACCESS, year(), month(), day(), hour(), minute(), second());
  tfile.timestamp(T_WRITE | T_ACCESS, year(), month(), day(), hour(), minute(), second());

  boolean aux = files_close();

  gcSaveSyncEvents();

  return aux;
}

boolean file_cfg()
{
  uint32_t pMask = powMask;
  setPowerUp(SD_MASK);
  if (!(powMask & SD_MASK)) {
    setPowerDown(~pMask);
    setPowerUp(pMask);
    return false;
  }

  uint8_t n = 0;

  // create a new file
try_file:
  if (n++ > 10) goto out_try_file;

  delay(1000);

  snprintf(filename, FILENAME_MAX_LENGH+1, PSTR("%08lX.XYZ"), Teensy3Clock.get());
  snprintf(&filename[FILENAME_MAX_LENGH-3], 4, PSTR("XYZ"));

  if (file.open(filename, O_CREAT | O_EXCL | O_TRUNC | O_WRITE)) {
      file.timestamp(T_CREATE | T_WRITE | T_ACCESS, year(), month(), day(), hour(), minute(), second());

      snprintf(&filename[FILENAME_MAX_LENGH-3], 4, PSTR("TRG"));

      if (tfile.open(filename, O_CREAT | O_TRUNC | O_WRITE)) {
        tfile.timestamp(T_CREATE | T_WRITE | T_ACCESS, year(), month(), day(), hour(), minute(), second());
      } else {
        file.timestamp(T_CREATE | T_WRITE | T_ACCESS, 1973, 9, 11, hour(), minute(), second());
        file.close();

        snprintf(&filename[FILENAME_MAX_LENGH-3], 4, PSTR("XYZ"));

        goto try_file;
      }
  } else goto try_file;

out_try_file:
  if (!file.isOpen() or !tfile.isOpen()) {
    geocentinela.println(PSTR("file_cfg:open files failed!"));

    if (file.isOpen()) {
      file.timestamp(T_CREATE | T_WRITE | T_ACCESS, 1980, month(), day(), hour(), minute(), second());
      file.close();
    }

    if (tfile.isOpen()) {
      tfile.timestamp(T_CREATE | T_WRITE | T_ACCESS, 1980, month(), day(), hour(), minute(), second());
      tfile.close();
    }

    gcPlayStat &= ~GC_ST_FILE_OPEN;

    setPowerDown(~pMask);
    setPowerUp(pMask);
    return false;
  } else {
    // write uint8_t file format
    file.write((uint8_t)FILE_FORMAT); // +1 = 1

    // write uint32_t SIM_UIDH
    file.write((uint8_t*)&SIM_UIDH, sizeof(uint32_t)); // +4 = 5

    // write uint32_t SIM_UIDMH
    file.write((uint8_t*)&SIM_UIDMH, sizeof(uint32_t)); // +4 = 9

    // write uint32_t SIM_UIDML
    file.write((uint8_t*)&SIM_UIDML, sizeof(uint32_t)); // +4 = 13

    // write uint32_t SIM_UIDL
    file.write((uint8_t*)&SIM_UIDL, sizeof(uint32_t)); // +4 = 17

    // write uint8_t GC_ADC_BITS:
    file.write((uint8_t)GC_ADC_BITS); // +1 = 18

    // write uint16_t GC_ADC_VREF in mV:
    uint16_t vref_mV = GC_ADC_VREF;
    file.write((uint8_t*)&vref_mV, sizeof(uint16_t)); // +2 = 20

    // write int32_t 
    file.write((uint8_t*)&geocentinela.getTimeZoneOffset(), sizeof(int32_t)); // +4 = 24

    // write float sensitivity:
    file.write((uint8_t*)&geocentinela.s.getSensitivity(), sizeof(float)); // +4 = 28

    float lng, lat, cta;
    geocentinela.s.getLocation(lng, lat, cta);

    // write float latitud:
    file.write((uint8_t*)&lat, sizeof(float)); // +4 = 32

    // write float longitud:
    file.write((uint8_t*)&lng, sizeof(float)); // +4 = 36

    // write float cota:
    file.write((uint8_t*)&cta, sizeof(float)); // +4 = 40

    // write uint8_t gain
    file.write((uint8_t)geocentinela.c.getGain()); // +1 = 41

    // write uint8_t hardware_average:
    file.write((uint8_t)geocentinela.c.getHardwareAverage()); // +1 = 42

    // write uint32_t tick_time:
    file.write((uint8_t*)&geocentinela.c.getTickTime(), sizeof(uint32_t)); // +4 = 46

    // write uint32_t time_begin:
    file.write((uint8_t*)&geocentinela.c.getTimeBegin(), sizeof(uint32_t)); // +4 = 50

    // write uint32_t time_end_seg:
    file.write((uint8_t*)&geocentinela.c.getTimeEnd(), sizeof(uint32_t)); // +4 = 54

    // write uint16_t trigger_level:
    file.write((uint8_t*)&geocentinela.c.getTriggerLevel(), sizeof(uint16_t)); // +2 = 56

    // write uint32_t trigger_time:
    file.write((uint8_t*)&geocentinela.c.getTriggerTime(), sizeof(uint32_t)); // +4 = 60

    // write uint32_t send_trigger_time:
    file.write((uint8_t*)&geocentinela.c.getSendTriggerTime(), sizeof(uint32_t)); // +4 = 64

    // write float vbat:
    float vbat = getVBat();
    file.write((uint8_t*)&vbat, sizeof(float)); // +4 = 68

    // write float temp:
    float temp = getTemp();
    file.write((uint8_t*)&temp, sizeof(float)); // +4 = 72

    gcPlayStat |= GC_ST_FILE_OPEN;

    setPowerDown(~pMask);
    setPowerUp(pMask);
    return true;
  }
}

void adc_play_callback()
{
  adc_play_cnt--;

  if (adc_play_cnt <= adc_play_stop) {
    adc_play_cnt++;
    stop_reading();
    return;
  }

  if (delta+3 >= GC_ADC_RING_SIZE) {
    buffer_errors++;

    uint16_t *adc_value = (uint16_t*)DMA_TCD1_DADDR;
    uint16_t* pos = (uint16_t*)(((uint32_t)(adc_value-1) | adc_ring_buffer_mmin) & adc_ring_buffer_mmax);
    *pos = 0xFFFF;

    return;
  }

  if (ADC0_SC1A != ADC_SC1_ADCH(0b11111)) {
    adc_errors++;

    uint16_t *adc_value = (uint16_t*)DMA_TCD1_DADDR;
    uint16_t* pos = (uint16_t*)(((uint32_t)(adc_value-1) | adc_ring_buffer_mmin) & adc_ring_buffer_mmax);
    *pos = 0xFFFF;

    return;
  }

  delta += 3;

#if TRIGGER_BY_SOFTWARE == 0
#elif TRIGGER_BY_SOFTWARE == 1
  uint16_t *adc_value = (uint16_t*)DMA_TCD1_DADDR;
#endif

  DMA_TCD2_SADDR = &(adc_config[1]); // ADC0_ch1
  ADC0_SC1A = adc_config[0]; // ADC0_ch0

  if (adc_play_cnt_trigger_check > adc_play_cnt) {
#if TRIGGER_BY_SOFTWARE == 0
    if (ADC1_SC1A != ADC_SC1_ADCH(0b11111)) {
      if (adc_play_cnt > trigger_time_cnt)
        adc_play_cnt_trigger_check = adc_play_cnt - trigger_time_cnt;
      else
        adc_play_cnt_trigger_check = 0;

      if (adc_play_cnt_trigger_check > send_trigger_time_cnt)
        adc_play_stop = adc_play_cnt_trigger_check - send_trigger_time_cnt;
      else
        adc_play_stop = 0;

      trigger_list[trigger_list_head++] = adc_play_cnt+2;
      trigger_list_head &= (TRIGGER_LIST_LENGTH-1);
    }

    DMA_TCD3_SADDR = &(adc_config[9]); // ADC1_ch1
    ADC1_SC1A = adc_config[8]; // ADC1_ch0
#elif TRIGGER_BY_SOFTWARE == 1
    uint16_t* x = (uint16_t*)(((uint32_t)(adc_value-3) | adc_ring_buffer_mmin) & adc_ring_buffer_mmax);
    uint16_t* y = (uint16_t*)(((uint32_t)(adc_value-2) | adc_ring_buffer_mmin) & adc_ring_buffer_mmax);
    uint16_t* z = (uint16_t*)(((uint32_t)(adc_value-1) | adc_ring_buffer_mmin) & adc_ring_buffer_mmax);
    if (trigger_min > *x or *x > trigger_max or
        trigger_min > *y or *y > trigger_max or
        trigger_min > *z or *z > trigger_max) {
      if (adc_play_cnt > trigger_time_cnt)
        adc_play_cnt_trigger_check = adc_play_cnt - trigger_time_cnt;
      else
        adc_play_cnt_trigger_check = 0;

      if (adc_play_cnt_trigger_check > send_trigger_time_cnt)
        adc_play_stop = adc_play_cnt_trigger_check - send_trigger_time_cnt;
      else
        adc_play_stop = 0;

      trigger_list[trigger_list_head++] = adc_play_cnt+2;
      trigger_list_head &= (TRIGGER_LIST_LENGTH-1);
    }
#endif
  }
}

boolean gcStart()
{
  uint32_t cfg_all = GC_CFG_READ|GC_CFG_ADC|GC_CFG_DMA;
  if ((gcCfgStat & cfg_all)!=cfg_all or (gcPlayStat & GC_ST_READING)) return false;

  uint32_t pMask = powMask;
  setPowerUp(PGA_MASK|SD_MASK);
  if (!(powMask & PGA_MASK) or !(powMask & SD_MASK)) goto fail;

  // reset
  delta = 0; tail = 0; sd_head = 0;
  adc_errors = 0; buffer_errors = 0;
  adc_rtc_stop = 0;
  adc_play_stop = 0;
  adc_play_cnt_trigger_check = adc_play_cnt-1;

  // trigger
#if TRIGGER_BY_SOFTWARE == 0
#elif TRIGGER_BY_SOFTWARE == 1
  trigger_min = 0x0000 + (geocentinela.c.getTriggerLevel()+1);
  trigger_max = 0xFFFF - geocentinela.c.getTriggerLevel();
#endif
  trigger_list_head = 0;
  trigger_time_cnt = time2Number(geocentinela.c.getTriggerTime(), geocentinela.c.getTickTime());
  send_trigger_time_cnt = time2Number(geocentinela.c.getSendTriggerTime(), geocentinela.c.getTickTime());

  // default configures:
  if (2 != geocentinela.c.getTimeType()) send_trigger_time_cnt = adc_play_cnt;
  if (0 == geocentinela.c.getTriggerLevel() or 0 == geocentinela.c.getTriggerTime()) adc_play_cnt_trigger_check = 0;

  // configure file
  if (file_cfg()) { // => gcPlayStat += GC_ST_FILE_OPEN
    // read rtc time
    uint32_t rtc_time = RTC_TSR+2;

    // write uint32_t rtc_time_sec:
    sd_buffer[sd_head++] = ((uint16_t*)&rtc_time)[0];
    sd_buffer[sd_head++] = ((uint16_t*)&rtc_time)[1]; // +4 = 76

    // write uint32_t adc_play_cnt:
    sd_buffer[sd_head++] = ((uint16_t*)&adc_play_cnt)[0];
    sd_buffer[sd_head++] = ((uint16_t*)&adc_play_cnt)[1]; // +4 = 80

    // write uint32_t adc_play error
    sd_buffer[sd_head++] = 0;
    sd_buffer[sd_head++] = 0; // +4 = 84

    while (RTC_TSR < rtc_time);
    if (!adc_play.begin(adc_play_callback, geocentinela.c.getTickTime())) {
      adc_play.end();

      sd_buffer[sd_head-1] = 0xEEE1;

      files_close();
      goto fail;
    }

    gcPlayStat |= GC_ST_READING;
    return true;
  }

fail:
  setPowerDown(~pMask);
  setPowerUp(pMask);
  return false;
}
//-----------------------------------------------------------------------
void printDigits(int digits)
{
  // utility function for digital clock display: prints preceding leading 0
  if(digits < 10)
    Serial.print(F("0"));
  Serial.print(digits);
}

void digitalClockDisplay()
{
  // digital clock display of the time:
  Serial.print(year());
  Serial.print(F("-"));
  printDigits(month());
  Serial.print(F("-"));
  printDigits(day());
  Serial.print(F(" "));
  printDigits(hour());
  Serial.print(F(":"));
  printDigits(minute());
  Serial.print(F(":"));
  printDigits(second());
  Serial.print(F(" "));
  //Serial.println();
  //Serial.print(F(" "));
  //Serial.print(now() % SEG_A_DAY);
  //Serial.print(F(":"));
  Serial.print(RTC_TSR % SEG_A_DAY);
  Serial.print(F(" "));
  Serial.println(((int32_t)now()-(int32_t)Teensy3Clock.get())/3600.0);
}

#define EPOCH_1980_UTC 315532800
