void gc_print(const char *log_string);
void gc_println(const char *log_string);
//-------------------------------
gcCFG gc_cfg(PSTR("CNT.CFG"), gc_println);
//-------------------------------
TEENSY3_LP lp = TEENSY3_LP();
sleep_block_t* lp_cfg; // sleep configuration
//-------------------------------
#define GC_ADC_A0 5
#define GC_ADC_A1 14
#define GC_ADC_A2 8
#define GC_ADC_A3 9
#define GC_ADC_A4 13
#define GC_ADC_A5 12
#define GC_ADC_A6 6
#define GC_ADC_A7 7
#define GC_ADC_A8 15
#define GC_ADC_A9 4
#define GC_ADC_A10 0
#define GC_ADC_A11 19
#define GC_ADC_A12 3
//-------------------------------
uint16_t* adc_ring_buffer = NULL;

uint32_t adc_config[6] = {
  ADC_SC1_ADCH(GC_ADC_A3),
  ADC_SC1_ADCH(GC_ADC_A6),
  ADC_SC1_ADCH(GC_ADC_A9),
  ADC_SC1_ADCH(31), // stop=0b11111=31
  ADC_SC1_ADCH(GC_ADC_A11), // read battery state
  ADC_SC1_ADCH(31), // stop=0b11111=31
};
//-------------------------------
#define ADC_BATT ADC_SC1_ADCH(GC_ADC_A11)
//-------------------------------
#define SD_CS SS
#define SD_MOSI 11
SdFat sd;
SdFile file;
uint16_t* sd_buffer;
//-------------------------------
IntervalTimer adc_play;
//-------------------------------
volatile uint16_t delta = 0;
volatile uint16_t tail = 0;
volatile uint16_t sd_head = 0;

volatile uint32_t adc_errors = 0;
volatile uint32_t buffer_errors = 0;

volatile int32_t adc_play_cnt = 0;
volatile uint32_t adc_rtc_stop = 0;
//-------------------------------
volatile uint8_t gcPlayStat = 0;// GeoCentinela Play Status
#define GC_ST_SLEEP   0x01 // GeoCentinela is sleeping
#define GC_ST_READING 0x02 // GeoCentinela is running
#define GC_ST_STOP    0x04 // GeoCentinela is stoping
#define GC_ST_CONFIG  0x08
#define GC_ST_PLAY    0x10

volatile uint8_t gcCfgStat = 0;// GeoCentinela Config Status
#define GC_CFG_ADC    0x01 // ADC OK
#define GC_CFG_DMA    0x02 // DMA OK
#define GC_CFG_RBUFF  0x04 // Buffer OK
#define GC_CFG_RADC   0x08 // Reconfigure ADC
#define GC_CFG_RDMA   0x10 // Reconfigure DMA
#define GC_CFG_RCFG   0x18 // Reconfiguration OK
//-------------------------------
#define FILE_FORMAT 0x02
#define FILENAME_MAX_LENGH 11
#define PIN_USB 30
#define WAKE_USB PIN_30
//-------------------------------
#define DIGITAL_POW 14
#define ANALOG1_POW 22
#define ANALOG2_POW 15
#define DIGITAL_MASK 0b100
#define ANALOG1_MASK 0b010
#define ANALOG2_MASK 0b001
#define POWER_UP_MASK 0b1000
//-------------------------------
#define GAIN_CS 21
#define GAIN_A0 19
#define GAIN_A1 16
#define GAIN_A2 18
//-------------------------------
TinyGPS gps;
#define HOUR_OFFSET -4
#define GPS_RTC_SYNC_TIME 1*60*1000 // 5min
#define GPS Serial1
//-------------------------------
#define LOG_TIMEOUT 1000
#define SEG_A_DAY 86400
//-------------------------------
void stop_reading();

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

void setPowerUp(uint8_t mask)
{
  if ((mask & DIGITAL_MASK) > 0) {
    digitalWrite(SD_CS, HIGH);
    digitalWrite(DIGITAL_POW, HIGH);
    delay(1000);
    if (!sd.chdir(1)) {
      while(!sd.begin(SD_CS, SPI_FULL_SPEED)) {
        gc_println(PSTR("error: sdcard!"));
        delay(LOG_TIMEOUT);
      }
    }
    setSyncProvider(getTeensy3Time);
  }

  if ((mask & ANALOG1_MASK) > 0)
    digitalWrite(ANALOG1_POW, HIGH);

  if ((mask & ANALOG2_MASK) > 0)
    digitalWrite(ANALOG2_POW, HIGH);
}

void setPowerDown(uint8_t mask)
{
  if ((mask & DIGITAL_MASK) > 0) {
    digitalWrite(SD_CS, LOW);
    pinMode(SD_MOSI, OUTPUT);
    digitalWrite(SD_MOSI, LOW);
    digitalWrite(DIGITAL_POW, LOW);
  }

  if ((mask & ANALOG1_MASK) > 0)
    digitalWrite(ANALOG1_POW, LOW);

  if ((mask & ANALOG2_MASK) > 0)
    digitalWrite(ANALOG2_POW, LOW);
}

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

void cfgGps()
{
  GPS.begin(4800);

  GPS.println(PSTR("$PSRF103,0,0,0,1*24"));
  GPS.println(PSTR("$PSRF103,1,0,0,1*25"));
  GPS.println(PSTR("$PSRF103,2,0,0,1*26"));
  GPS.println(PSTR("$PSRF103,3,0,0,1*27"));
  GPS.println(PSTR("$PSRF103,4,0,0,1*20"));
  GPS.println(PSTR("$PSRF103,5,0,0,1*21"));
  GPS.println(PSTR("$PSRF103,6,0,0,1*22"));
  GPS.println(PSTR("$PSRF103,8,0,0,1*2C"));

  //GPS.println(PSTR("$PSRF103,0,0,1,1*25"));
  //GPS.println(PSTR("$PSRF103,1,0,1,1*26"));
  //GPS.println(PSTR("$PSRF103,2,0,1,1*27"));
  //GPS.println(PSTR("$PSRF103,3,0,1,1*28"));
  GPS.println(PSTR("$PSRF103,4,0,1,1*21"));
  //GPS.println(PSTR("$PSRF103,5,0,1,1*22"));
  //GPS.println(PSTR("$PSRF103,6,0,1,1*23"));
  //GPS.println(PSTR("$PSRF103,8,0,1,1*2D"));
}

boolean gps2rtcSync()
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
  setPowerUp(ANALOG2_MASK);
  delay(1000);

  cfgGps();

  // sync
  char c = 0;
  uint8_t timeIni = millis();
  boolean rtcSync = false;
  while (!rtcSync && (millis() - timeIni) <= GPS_RTC_SYNC_TIME && (gcPlayStat & GC_ST_CONFIG) == 0) {
    while (GPS.available() > 0 && !rtcSync) {
      c = GPS.read();
      Serial.write(c);
      if (gps.encode(c)) rtcSync = gps2rtcSync();
    }
  }

  // GPS power off
  setPowerDown(ANALOG2_MASK);
  delay(10000);
}

void cfgPow()
{
  pinMode(DIGITAL_POW, OUTPUT);
  pinMode(ANALOG1_POW, OUTPUT);
  pinMode(ANALOG2_POW, OUTPUT);

  setPowerDown(DIGITAL_MASK | ANALOG1_MASK | ANALOG2_MASK);
}

void cfgGain(uint8_t gain)
{
  pinMode(GAIN_CS, OUTPUT);

  pinMode(GAIN_A0, OUTPUT);
  pinMode(GAIN_A1, OUTPUT);
  pinMode(GAIN_A2, OUTPUT);

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
  digitalWrite(GAIN_CS, LOW);
}

void cfgAdc()
{
  // ADC clock
  SIM_SCGC6 |= SIM_SCGC6_ADC0;

  // general configuration
  ADC0_CFG1 = 0
    //| ADC_CFG1_ADLPC     // lower power: off, on
    | ADC_CFG1_ADIV(1)   // clock divide: 1, 2, 4, 8
    //| ADC_CFG1_ADLSMP    // sample time: short, long
    | ADC_CFG1_MODE(3)   // conversion mode: 8, 12, 10, 16
#if F_BUS == 24000000
    | ADC_CFG1_ADICLK(0) // input clock: bus, bus/2, alternate, asynchronous
#elif F_BUS == 48000000
    | ADC_CFG1_ADICLK(1) // input clock: bus, bus/2, alternate, asynchronous
#endif
  ;

  ADC0_CFG2 = 0
    | ADC_CFG2_MUXSEL    // adc mux (see pag. 96): ADxxa, ADxxb
    //| ADC_CG2_ADACKEN    // asynchronous clock output: disable, enable
    | ADC_CFG2_ADHSC     // high speed configuration: normal, high
    | ADC_CFG2_ADLSTS(0) // long sample time: 20ext, 12ext, 6ext, 2ext
  ;

  // control
  ADC0_SC2 = 0
    //| ADC_SC2_ADTRG     // trigger select: software, hardware
    //| ADC_SC2_ACFE      // compare function: disable, enable
    //| ADC_SC2_ACFGT     // compare function greater than: disable, enable
    //| ADC_SC2_ACREN     // compare function range: disable, enable
    | ADC_SC2_DMAEN     // DMA enable
    | ADC_SC2_REFSEL(1) // 0->3.3v, 1->1.2v
  ;

  if (gc_cfg.average < 4) {
    ADC0_SC3 = 0
      //| ADC_SC3_ADCO                 // continuous conversion: disable, enable
      | ADC_SC3_AVGE                 // enable hardware average
      | ADC_SC3_AVGS(gc_cfg.average) // average select: 0->4, 1->8, 2->16, 3->32
    ;
  } else {
    ADC0_SC3 = 0; // continuous conversion disable, hardware average disable
  }

  // calibration
  ADC0_SC3 |= ADC_SC3_CAL; // begin cal

  uint16_t cal_sum;
  while (ADC0_SC3 & ADC_SC3_CAL);

  cal_sum = (ADC0_CLPS + ADC0_CLP4 + ADC0_CLP3 + ADC0_CLP2 + ADC0_CLP1 + ADC0_CLP0)/2;
  ADC0_PG = cal_sum | 0x8000;

  cal_sum = (ADC0_CLMS + ADC0_CLM4 + ADC0_CLM3 + ADC0_CLM2 + ADC0_CLM1 + ADC0_CLM0)/2;
  ADC0_MG = cal_sum | 0x8000;

  // Stop conversion
  ADC0_SC1A = ADC_SC1_ADCH(0b11111);

  gcCfgStat |= GC_CFG_ADC;
}

void cfgDma()
{
  uint32_t mod = 1+log2(gc_cfg.adc_buffer_size);

  // enable DMA and DMAMUX clock
  SIM_SCGC7 |= SIM_SCGC7_DMA;
  SIM_SCGC6 |= SIM_SCGC6_DMAMUX;

  // Clear before configure DMA channel
  DMAMUX0_CHCFG1 = 0;
  DMA_TCD1_CSR = 0;
  DMA_TCD2_CSR = 0;

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
  //  | DMA_TCD_CSR_MAJORLINKCH(2) // major loop channel to channel linking set$
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

  gcCfgStat |= GC_CFG_DMA;
  gcCfgStat |= GC_CFG_RDMA;
}

boolean rcfgBuff()
{
  if (adc_ring_buffer) { free(adc_ring_buffer); adc_ring_buffer = NULL; }
  if (sd_buffer) { free(sd_buffer); sd_buffer = NULL; }
  if (lp_cfg) { free(lp_cfg); lp_cfg = NULL; }

  adc_ring_buffer = (uint16_t*) memalign(gc_cfg.adc_buffer_size_bytes,
                                         gc_cfg.adc_buffer_size_bytes);
  sd_buffer = (uint16_t*) calloc(gc_cfg.sd_buffer_size, sizeof(uint16_t));
  lp_cfg = (sleep_block_t*) calloc(1, sizeof(sleep_block_t));

  memset(adc_ring_buffer, 0, gc_cfg.adc_buffer_size_bytes);

  if (adc_ring_buffer && sd_buffer && lp_cfg) {
    gcCfgStat |= GC_CFG_RBUFF;
    return true;
  } else {
    if (adc_ring_buffer) { free(adc_ring_buffer); adc_ring_buffer = NULL; }
    if (sd_buffer) { free(sd_buffer); sd_buffer = NULL; }
    if (lp_cfg) { free(lp_cfg); lp_cfg = NULL; }
    return false;
  }
}

void rcfgAdc()
{
  if (!(gcCfgStat & GC_CFG_ADC)
   || !(gcCfgStat & GC_CFG_DMA)
   || !(gcCfgStat & GC_CFG_RBUFF)) return;

  gcCfgStat |= GC_CFG_RADC;
}

void rcfg()
{
  // reconfigure Buffer
  while (!rcfgBuff()) {
    gc_println(PSTR("error: reconfigure buffer!"));
    delay(LOG_TIMEOUT);
  }

  // reconfigure ADC
  rcfgAdc();

  // reconfigure DMA
  cfgDma();
}
//----------------------------------------------------------------------
float getVBat()
{
  // general configuration
  ADC0_CFG1 = 0
    | ADC_CFG1_ADLPC     // lower power: off, on
    | ADC_CFG1_ADIV(3)   // clock divide: 1, 2, 4, 8
    | ADC_CFG1_ADLSMP    // sample time: short, long
    | ADC_CFG1_MODE(3)   // conversion mode: 8, 12, 10, 16
#if F_BUS == 24000000
    | ADC_CFG1_ADICLK(0) // input clock: bus, bus/2, alternate, asynchronous
#elif F_BUS == 48000000
    | ADC_CFG1_ADICLK(1) // input clock: bus, bus/2, alternate, asynchronous
#endif
  ;

  ADC0_CFG2 = 0
    | ADC_CFG2_MUXSEL    // adc mux (see pag. 96): ADxxa, ADxxb
    //| ADC_CG2_ADACKEN    // asynchronous clock output: disable, enable
    | ADC_CFG2_ADHSC     // high speed configuration: normal, high
    | ADC_CFG2_ADLSTS(0) // long sample time: 20ext, 12ext, 6ext, 2ext
  ;

  // control
  ADC0_SC2 = 0
    //| ADC_SC2_ADTRG     // trigger select: software, hardware
    //| ADC_SC2_ACFE      // compare function: disable, enable
    //| ADC_SC2_ACFGT     // compare function greater than: disable, enable
    //| ADC_SC2_ACREN     // compare function range: disable, enable
    //| ADC_SC2_DMAEN     // DMA enable
    | ADC_SC2_REFSEL(1) // 0->3.3v, 1->1.2v
  ;

  ADC0_SC3 = 0
    //| ADC_SC3_ADCO    // continuous conversion: disable, enable
    | ADC_SC3_AVGE    // enable hardware average
    | ADC_SC3_AVGS(3) // average select: 0->4, 1->8, 2->16, 3->32
  ;

  // calibration
  ADC0_SC3 |= ADC_SC3_CAL; // begin cal

  uint16_t cal_sum;
  while (ADC0_SC3 & ADC_SC3_CAL);

  cal_sum = (ADC0_CLPS + ADC0_CLP4 + ADC0_CLP3 + ADC0_CLP2 + ADC0_CLP1 + ADC0_CLP0)/2;
  ADC0_PG = cal_sum | 0x8000;

  cal_sum = (ADC0_CLMS + ADC0_CLM4 + ADC0_CLM3 + ADC0_CLM2 + ADC0_CLM1 + ADC0_CLM0)/2;
  ADC0_MG = cal_sum | 0x8000;

  // Stop conversion
  ADC0_SC1A = ADC_SC1_ADCH(0b11111);

  uint32_t bvalue = 0;
  uint16_t n = 1000;
  for(uint16_t i = 0; i < n; i++) {
    ADC0_SC1A = adc_config[4];
    while(!(ADC0_SC1A & ADC_SC1_COCO));
    bvalue += ADC0_RA;
  }
  bvalue /= n;

  // configure adc
  cfgAdc();

  // configure DMA
  cfgDma();

  // reconfigure
  rcfg();

  float value = bvalue*1.195/pow(2,16);
  return 21*value;
}
//----------------------------------------------------------------------
// User callback handler
void callbackhandler() {
  setSyncProvider(getTeensy3Time);
}

uint32_t deep_sleep()
{
  // reset lp_cfg
  memset(lp_cfg, 0, sizeof(sleep_block_t));

  // OR together different wake sources
  lp_cfg->modules = GPIO_WAKE;

  // GPIO alarm wakeup
  lp_cfg->gpio_pin = WAKE_USB;

  // user callback function
  lp_cfg->callback = callbackhandler;

  // sleep
  lp.DeepSleep(lp_cfg);

  return 0;
}

uint32_t sleep_chrono()
{
  // reset lp_cfg
  memset(lp_cfg, 0, sizeof(sleep_block_t));

  // OR together different wake sources
  lp_cfg->modules = (GPIO_WAKE | RTCA_WAKE);

  // GPIO alarm wakeup
  lp_cfg->gpio_pin = WAKE_USB;

  // RTC alarm wakeup in seconds:
  lp_cfg->rtc_alarm = gc_cfg.time_begin_seg;
  uint32_t dseg = gc_cfg.time_end_seg;

  // user callback function
  lp_cfg->callback = callbackhandler;

  // sleep
  if (lp_cfg->rtc_alarm > 0) {
    lp.DeepSleep(lp_cfg);

    if (lp_cfg->wake_source == WAKE_USB) {
      return 0;
    } else {
      syncGps();
    }
  }

  return dseg*(1000000/gc_cfg.tick_time_useg);
}

uint32_t sleep_daily()
{
  // reset lp_cfg
  memset(lp_cfg, 0, sizeof(sleep_block_t));

  // OR together different wake sources
  lp_cfg->modules = (GPIO_WAKE | RTCA_WAKE);

  // GPIO alarm wakeup
  lp_cfg->gpio_pin = WAKE_USB;

  uint32_t time_n = Teensy3Clock.get() % SEG_A_DAY;
  uint32_t dseg = 0;

  // RTC alarm wakeup in seconds:
  if (gc_cfg.time_begin_seg < gc_cfg.time_end_seg) {
    dseg = gc_cfg.time_end_seg - gc_cfg.time_begin_seg;
    if (time_n < gc_cfg.time_begin_seg) {
      lp_cfg->rtc_alarm = gc_cfg.time_begin_seg - time_n;
    } else if (time_n > gc_cfg.time_end_seg) {
      lp_cfg->rtc_alarm = (gc_cfg.time_begin_seg + SEG_A_DAY) - time_n;
    } else {
      lp_cfg->rtc_alarm = 0;
      dseg = gc_cfg.time_end_seg - time_n;
    }
  } else {
    if (time_n <= gc_cfg.time_end_seg) {
      lp_cfg->rtc_alarm = 0;
      dseg = gc_cfg.time_end_seg - time_n;
    } else if (time_n >= gc_cfg.time_begin_seg) {
      lp_cfg->rtc_alarm = 0;
      dseg = (gc_cfg.time_end_seg + SEG_A_DAY) - time_n;
    } else {
      lp_cfg->rtc_alarm = gc_cfg.time_begin_seg - time_n;
      dseg = (SEG_A_DAY - gc_cfg.time_begin_seg) + gc_cfg.time_end_seg;
    }
  }

  // user callback function
  lp_cfg->callback = callbackhandler;

  // sleep
  if (lp_cfg->rtc_alarm > 0) {
    lp.DeepSleep(lp_cfg);

    if (lp_cfg->wake_source == WAKE_USB) {
      return 0;
    } else { // GPS rtc sync first!
      // sync RTC with the GPS
      syncGps();
    }
  }

  return dseg*(1000000/gc_cfg.tick_time_useg);
}

boolean gc_stop()
{
  if (!(gcPlayStat & GC_ST_STOP)) return false;

  // stop adc_play
  adc_play.end();

  int8_t wadc = 3;
  while (ADC0_SC1A != adc_config[3] && wadc-- > 0) {
    gc_println(PSTR("warning:gc_stop: waiting for the ADC!"));
    delay(LOG_TIMEOUT);
  }

  // save tail
  while (delta > 0) {
    if (sd_head == gc_cfg.sd_buffer_size) {
      file.write(sd_buffer, gc_cfg.sd_buffer_size_bytes);
      sd_head = 0;
    }

    sd_buffer[sd_head++] = adc_ring_buffer[tail++];

    tail &= gc_cfg.adc_buffer_hash;
    delta--;
  }
  file.write(sd_buffer, sd_head * sizeof(uint16_t));

  // save errors
  file.write((uint8_t*)&buffer_errors, sizeof(uint32_t));
  file.write((uint8_t*)&adc_errors, sizeof(uint32_t));

  // save adc_rtc_stop
  file.write((uint8_t*)&adc_rtc_stop, sizeof(uint32_t));

  // save act_play_cnt
  file.write((int32_t*)&adc_play_cnt, sizeof(int32_t));
  adc_play_cnt = 0;

  // write float vbat:
  float vbat = getVBat();
  file.write((uint8_t*)&vbat, sizeof(float));

  // file timestamp
  file.timestamp(T_WRITE, year(), month(), day(), hour(), minute(), second());
  file.timestamp(T_ACCESS, year(), month(), day(), hour(), minute(), second());

  if (file.writeError) {
    gc_println(PSTR("error:stop: write file!"));
    return false;
  }

  // close file
  if (!file.close()) {
    gc_println(PSTR("error:stop: close file!"));
    return false;
  }

  gcPlayStat &= ~GC_ST_STOP;

  return true;
}

boolean file_cfg()
{
  // create a new file
  char name[FILENAME_MAX_LENGH+1];
  strcpy_P(name, PSTR("CNT0000.CNT"));
  for (uint16_t n = 0; n < 10000; n++) {
    name[3] = '0' + n/1000;
    name[4] = '0' + (n%1000)/100;
    name[5] = '0' + (n%100)/10;
    name[6] = '0' + n%10;
    if (file.open(name, O_CREAT | O_EXCL | O_TRUNC | O_WRITE)) {
      file.timestamp(T_CREATE, year(), month(), day(), hour(), minute(), second());
      file.timestamp(T_WRITE, year(), month(), day(), hour(), minute(), second());
      file.timestamp(T_ACCESS, year(), month(), day(), hour(), minute(), second());
      break;
    }
  }

  if (!file.isOpen()) {
    gc_println(PSTR("log:file open failed"));
    file.timestamp(T_CREATE, 1980, month(), day(), hour(), minute(), second());
    file.timestamp(T_WRITE, 1980, month(), day(), hour(), minute(), second());
    file.timestamp(T_ACCESS, 1980, month(), day(), hour(), minute(), second());
    return false;
  } else {
    // write uint8_t file format
    file.write((uint8_t)FILE_FORMAT);

    // write uint8_t ((gain << 4)| average):
    file.write((gc_cfg.gain << 4) | gc_cfg.average);

    // write uint32_t tick_time_useg:
    file.write((uint8_t*)&gc_cfg.tick_time_useg, sizeof(uint32_t));

    // write uint8_t time_type:
    file.write(gc_cfg.time_type);

    // write uint32_t time_begin_seg:
    file.write((uint8_t*)&gc_cfg.time_begin_seg, sizeof(uint32_t));

    // write uint32_t time_end_seg:
    file.write((uint8_t*)&gc_cfg.time_end_seg, sizeof(uint32_t));

    // write float vbat:
    float vbat = getVBat();
    file.write((uint8_t*)&vbat, sizeof(float));

    return true;
  }
}

void adc_play_callback() {
  if (adc_play_cnt-- <= 0) {
    stop_reading();
    return;
  }

  if (ADC0_SC1A != adc_config[3]) {
    adc_errors++;
    return;
  }

  if (delta+3 >= gc_cfg.adc_buffer_size) {
    buffer_errors++;
    return;
  }

  delta += 3;

  DMA_TCD2_SADDR = &(adc_config[1]);
  ADC0_SC1A = adc_config[0];
}

boolean gc_start()
{
  if (gcPlayStat & GC_ST_READING) return false;

  // restart
  delta = 0; tail = 0; sd_head = 0;
  adc_errors = 0; buffer_errors = 0;
  adc_rtc_stop = 0;
  setPowerUp(ANALOG1_MASK); delay(200);

  // configure file
  if (file_cfg()) {
    // read rtc time
    uint32_t rtc_time = Teensy3Clock.get();

    // write uint32_t rtc_time_sec:
    sd_buffer[sd_head++] = ((uint16_t*)&rtc_time)[0];
    sd_buffer[sd_head++] = ((uint16_t*)&rtc_time)[1];

    // write uint32_t adc_play_cnt:
    sd_buffer[sd_head++] = ((uint16_t*)&adc_play_cnt)[0];
    sd_buffer[sd_head++] = ((uint16_t*)&adc_play_cnt)[1];

    // configure adc_play
    sd_buffer[sd_head++] = 0;
    sd_buffer[sd_head++] = 0;
    if (!adc_play.begin(adc_play_callback, gc_cfg.tick_time_useg)) {
      sd_buffer[sd_head-1] = 61153;
      gc_println(PSTR("error:start: adc_play!"));
      setPowerDown(ANALOG1_MASK);
      return false;
    }

    gcPlayStat |= GC_ST_READING;
    return true;
  } else {
    gc_println(PSTR("error:start: file!"));
    setPowerDown(ANALOG1_MASK);
    return false;
  }
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
  if (!first) Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

void digitalClockDisplay()
{
  // digital clock display of the time
  printDigits(hour(), true);
  printDigits(minute(), false);
  printDigits(second(), false);
  Serial.print(" ");
  Serial.print(day());
  Serial.print("/");
  Serial.print(month());
  Serial.print("/");
  Serial.print(year());
  Serial.print(" : ");
  Serial.println(Teensy3Clock.get() % SEG_A_DAY);
}

