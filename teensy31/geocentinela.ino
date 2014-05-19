#include <IntervalTimer.h>
#include <Time.h>

#include <SdFat.h>
#include <LowPower_Teensy3.h>

#include <malloc.h> // needed to mamalign and calloc
#include <cmath> // needed to log2

#include "gcCFG.h"
//-------------------------------
gcCFG gc_cfg(PSTR("CNT.CFG"), gc_println);
//-------------------------------
TEENSY3_LP lp = TEENSY3_LP();
sleep_block_t* lp_cfg; // sleep configuration
//-------------------------------
uint16_t* adc_ring_buffer = NULL;
uint32_t* adc_config = NULL;

SdFat sd;
SdFile file;
uint16_t* sd_buffer;

IntervalTimer adc_play;
//-------------------------------
volatile uint16_t delta = 0;
volatile uint16_t tail = 0;
volatile uint16_t sd_head = 0;

volatile uint32_t adc_errors = 0;
volatile uint32_t buffer_errors = 0;

volatile int32_t adc_play_cnt = 0;
volatile uint32_t adc_rtc_stop = 0;

volatile uint32_t time = millis();
//-------------------------------
volatile uint8_t gc_st = 0;     // GeoCentinela Status
#define GC_ST_SLEEP        0x01 // GeoCentinela is sleeping
#define GC_ST_ADC          0x02 // ADC OK
#define GC_ST_DMA          0x04 // DMA OK
#define GC_ST_RBUFF        0x08 // Buffer OK
#define GC_ST_RADC         0x10 // Reconfigure ADC
#define GC_ST_RDMA         0x20 // Reconfigure DMA
#define GC_ST_RCFG         0x38 // Reconfiguration OK
#define GC_ST_RUN          0x40 // GeoCentinela is running
#define GC_ST_STOP         0x80 // GeoCentinela is stoping
//-------------------------------
#define FILE_FORMAT 0x01
#define FILENAME_MAX_LENGH 11
#define USB_PIN 9
#define USB_WAKE PIN_30
//-------------------------------
#define POW0 5 // 0
#define POW1 6 // 2
#define POW2 7 // 4
#define POW3 8 // 6
//-------------------------------
#define GAIN0 3 // 1
#define GAIN1 0 // 3
#define GAIN2 1 // 5
#define GAIN3 2 // 7
//-------------------------------
void power_cfg()
{
  pinMode(POW0, OUTPUT);
  pinMode(POW1, OUTPUT);
  pinMode(POW2, OUTPUT);
  pinMode(POW3, INPUT);

  digitalWrite(POW0, LOW);
  digitalWrite(POW1, LOW);
  digitalWrite(POW2, LOW);
}

void gain_cfg(uint8_t gain)
{
  pinMode(GAIN0, OUTPUT);

  pinMode(GAIN3, OUTPUT);
  pinMode(GAIN2, OUTPUT);
  pinMode(GAIN1, OUTPUT);

  digitalWrite(GAIN0, LOW);
  switch (gain) {
  case 0:
    digitalWrite(GAIN3, LOW);
    digitalWrite(GAIN2, LOW);
    digitalWrite(GAIN1, LOW);
    break;
  case 1:
    digitalWrite(GAIN3, HIGH);
    digitalWrite(GAIN2, LOW);
    digitalWrite(GAIN1, LOW);
    break;
  case 2:
    digitalWrite(GAIN3, LOW);
    digitalWrite(GAIN2, HIGH);
    digitalWrite(GAIN1, LOW);
    break;
  case 3:
    digitalWrite(GAIN3, HIGH);
    digitalWrite(GAIN2, HIGH);
    digitalWrite(GAIN1, LOW);
    break;
  case 4:
    digitalWrite(GAIN3, LOW);
    digitalWrite(GAIN2, LOW);
    digitalWrite(GAIN1, HIGH);
    break;
  case 5:
    digitalWrite(GAIN3, HIGH);
    digitalWrite(GAIN2, LOW);
    digitalWrite(GAIN1, HIGH);
    break;
  case 6:
    digitalWrite(GAIN3, LOW);
    digitalWrite(GAIN2, HIGH);
    digitalWrite(GAIN1, HIGH);
    break;
  case 7:
    digitalWrite(GAIN3, HIGH);
    digitalWrite(GAIN2, HIGH);
    digitalWrite(GAIN1, HIGH);
    break;
  default:
    break;
  }
  digitalWrite(GAIN0, LOW);
}

void adc_cfg()
{
  // ADC clock
  SIM_SCGC6 |= SIM_SCGC6_ADC0;

  // general configuration
  ADC0_CFG1 = 0
    | ADC_CFG1_ADLPC     // lower power: off, on
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

  gc_st |= GC_ST_ADC;
}

void dma_cfg()
{
  // enable DMAMUX clock
  SIM_SCGC6 |= SIM_SCGC6_DMAMUX;

  // to connect ADC with DMA
  DMA_SERQ |= 1; // enable channel 1 requests

  // channels priority
  DMA_CR |= DMA_CR_ERCA; // enable round robin scheduling
  //DMA_DCHPRI0 = 0;
  //DMA_DCHPRI1 = 1;
  //DMA_DCHPRI2 = 2;
  //DMA_DCHPRI3 = 3;

  // configure the DMAMUX so that the ADC DMA request triggers DMA channel 1
  DMAMUX0_CHCFG1 &= ~DMAMUX_ENABLE;
  DMAMUX0_CHCFG1 = DMAMUX_ENABLE | DMAMUX_SOURCE_ADC0;

  gc_st |= GC_ST_DMA;
}

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

bool rtc_cfg()
{
  // set the Time library to use Teensy 3.0's RTC to keep time
  setSyncProvider(getTeensy3Time);

  return timeStatus() == timeSet;
}

bool buff_rcfg()
{
  if (adc_ring_buffer) { free(adc_ring_buffer); adc_ring_buffer = NULL; }
  if (adc_config) { free(adc_config); adc_config = NULL; }
  if (sd_buffer) { free(sd_buffer); sd_buffer = NULL; }
  if (lp_cfg) { free(lp_cfg); lp_cfg = NULL; }

  adc_ring_buffer = (uint16_t*) memalign(gc_cfg.adc_buffer_size_bytes,
                                         gc_cfg.adc_buffer_size_bytes);
  adc_config = (uint32_t*) calloc(4, sizeof(uint32_t));
  sd_buffer = (uint16_t*) calloc(gc_cfg.sd_buffer_size, sizeof(uint16_t));
  lp_cfg = (sleep_block_t*) calloc(1, sizeof(sleep_block_t));

  memset(adc_ring_buffer, 0, gc_cfg.adc_buffer_size_bytes);

  if (adc_ring_buffer && adc_config && sd_buffer && lp_cfg) {
    gc_st |= GC_ST_RBUFF;
    return true;
  } else {
    if (adc_ring_buffer) { free(adc_ring_buffer); adc_ring_buffer = NULL; }
    if (adc_config) { free(adc_config); adc_config = NULL; }
    if (sd_buffer) { free(sd_buffer); sd_buffer = NULL; }
    if (lp_cfg) { free(lp_cfg); lp_cfg = NULL; }
    return false;
  }
}

void adc_rcfg()
{
  if (!(gc_st & GC_ST_ADC)
   || !(gc_st & GC_ST_DMA)
   || !(gc_st & GC_ST_RBUFF)) return;

  // channels config:
  // A0=5, A1=14, A2=8, A3=9, A4=13, A5=12, A6=6, A7=7, A8=15, A9=4
  adc_config[0] = ADC_SC1_ADCH(15);
  adc_config[1] = ADC_SC1_ADCH(7);
  adc_config[2] = ADC_SC1_ADCH(6);
  adc_config[3] = ADC_SC1_ADCH(31); // stop=31

  gc_st |= GC_ST_RADC;
}

void dma_rcfg()
{
  if (!(gc_st & GC_ST_ADC)
   || !(gc_st & GC_ST_DMA)
   || !(gc_st & GC_ST_RBUFF)
   || !(gc_st & GC_ST_RADC)) return;

  uint32_t mod = 1+log2(gc_cfg.adc_buffer_size);

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

  gc_st |= GC_ST_RDMA;
}

void rcfg()
{
  // reconfigure Buffer
  if (buff_rcfg()) {
    // reconfigure ADC
    adc_rcfg();

    // reconfigure DMA
    dma_rcfg();
  } else {
    while (true) {
      gc_println(PSTR("error: buffer!"));
      delay(1000);
    }
  }
}

void pow_s()
{
  digitalWrite(POW2, HIGH);

  while (!(bool)digitalRead(POW3)) {
    gc_println(PSTR("power: waiting!"));
    delay(500);
  }

  digitalWrite(POW0, HIGH);
  delay(2);
  digitalWrite(POW1, HIGH);
}

void pow_l()
{
  digitalWrite(POW1, LOW);
  delay(2);
  digitalWrite(POW0, LOW);
  delay(2);
  digitalWrite(POW2, LOW);
}

void gc_power(uint8_t cmd)
{
  switch(cmd) {
    case 'l': {
      pow_l();
    } break;
    case 's': {
      pow_s();
    } break;
  }
}

void setup()
{
  // configure POW
  power_cfg();

  // initialize file system
  if (!sd.begin(SS, SPI_FULL_SPEED)) {
    while (true) {
      gc_println(PSTR("error: sdcard!"));
      delay(1000);
    }
  }

  // read configuration from sdcard
  if (!gc_cfg.read()) {
    if (!gc_cfg.write()) {
      while (true) {
        gc_println(PSTR("error: cfg/rw!"));
        delay(1000);
      }
    }
  }

  // configure RTC
  if (!rtc_cfg()) {
    while (true) {
      gc_println(PSTR("error: rtc!"));
      delay(1000);
    }
  }

  // configure ADC
  adc_cfg();

  // configure DMA
  dma_cfg();

  // reconfigure (buffer?)
  rcfg();

  // power switching
  pow_s();

  // gain configuration
  gain_cfg(gc_cfg.gain);

  // GPIO alarm wakeup
  pinMode(USB_PIN, INPUT_PULLUP);
}

void deep_sleep()
{
  // reset lp_cfg
  memset(lp_cfg, 0, sizeof(sleep_block_t));

  // OR together different wake sources
  lp_cfg->modules = GPIO_WAKE;

  // GPIO alarm wakeup
  lp_cfg->gpio_pin = USB_WAKE;

  // sleep
  lp.DeepSleep(lp_cfg);
  gc_st &= ~GC_ST_SLEEP;
}

uint32_t sleep_chrono()
{
  // reset lp_cfg
  memset(lp_cfg, 0, sizeof(sleep_block_t));

  // OR together different wake sources
  if (gc_st & GC_ST_SLEEP) {
    lp_cfg->modules = (GPIO_WAKE | RTCA_WAKE);

    // GPIO alarm wakeup
    lp_cfg->gpio_pin = USB_WAKE;
  } else {
    lp_cfg->modules = RTCA_WAKE;
    gc_st |= GC_ST_SLEEP;
  }

  // RTC alarm wakeup in seconds:
  lp_cfg->rtc_alarm = gc_cfg.time_begin_seg;
  uint32_t dseg = gc_cfg.time_end_seg;

  // sleep
  if (lp_cfg->rtc_alarm > 0) {
    lp.DeepSleep(lp_cfg);

    if (lp_cfg->wake_source == USB_WAKE) {
      gc_st &= ~GC_ST_SLEEP;
      return 0;
    }
  }

  return dseg*(1000000/gc_cfg.tick_time_useg);
}

uint32_t sleep_daily()
{
  // reset lp_cfg
  memset(lp_cfg, 0, sizeof(sleep_block_t));

  // OR together different wake sources
  if (gc_st & GC_ST_SLEEP) {
    lp_cfg->modules = (GPIO_WAKE | RTCA_WAKE);

    // GPIO alarm wakeup
    lp_cfg->gpio_pin = USB_WAKE;
  } else {
    lp_cfg->modules = RTCA_WAKE;
    gc_st |= GC_ST_SLEEP;
  }

  uint32_t time_n = Teensy3Clock.get() % 86400;
  uint32_t dseg = 0;

  // RTC alarm wakeup in seconds:
  if (gc_cfg.time_begin_seg < gc_cfg.time_end_seg) {
    dseg = gc_cfg.time_end_seg - gc_cfg.time_begin_seg;
    if (time_n < gc_cfg.time_begin_seg) {
      lp_cfg->rtc_alarm = gc_cfg.time_begin_seg - time_n;
    } else if (time_n > gc_cfg.time_end_seg) {
      lp_cfg->rtc_alarm = (gc_cfg.time_begin_seg + 86400) - time_n;
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
      dseg = (gc_cfg.time_end_seg + 86400) - time_n;
    } else {
      lp_cfg->rtc_alarm = gc_cfg.time_begin_seg - time_n;
      dseg = (86400 - gc_cfg.time_begin_seg) + gc_cfg.time_end_seg;
    }
  }

  // sleep
  if (lp_cfg->rtc_alarm > 0) {
    lp.DeepSleep(lp_cfg);

    if (lp_cfg->wake_source == USB_WAKE) {
      gc_st &= ~GC_ST_SLEEP;
      return 0;
    }
  }

  return dseg*(1000000/gc_cfg.tick_time_useg);
}

bool gc_stop()
{
  if (!(gc_st & GC_ST_STOP)) return false;

  // stop adc_play
  adc_play.end();

  int8_t wadc = 3;
  while (ADC0_SC1A != adc_config[3] && wadc-- > 0) {
    gc_println(PSTR("warning:gc_stop: waiting for the ADC!"));
    delay(1000);
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

  if (file.writeError) {
    gc_println(PSTR("error:stop: write file!"));
    return false;
  }

  // close file
  if (!file.close()) {
    gc_println(PSTR("error:stop: close file!"));
    return false;
  }

  gc_st &= ~GC_ST_STOP;

  return true;
}

bool file_cfg()
{
  // create a new file
  char name[FILENAME_MAX_LENGH+1];
  strcpy_P(name, PSTR("CNT0000.CNT"));
  for (uint8_t n = 0; n < 10000; n++) {
    name[3] = '0' + n/1000;
    name[4] = '0' + (n%1000)/100;
    name[5] = '0' + (n%100)/10;
    name[6] = '0' + n%10;
    if (file.open(name, O_CREAT | O_EXCL | O_TRUNC | O_WRITE)) break;
  }

  if (!file.isOpen()) {
    gc_println(PSTR("log:file open failed"));
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

    return true;
  }
}

void adc_play_callback() {
  if (adc_play_cnt-- <= 0) {
    if (adc_rtc_stop == 0) adc_rtc_stop = Teensy3Clock.get();

    gc_st &= ~GC_ST_RUN;
    gc_st |= GC_ST_STOP;

    adc_play_cnt = 0;
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

bool gc_start()
{
  if (gc_st & GC_ST_RUN) return false;

  // restart
  delta = 0; tail = 0; sd_head = 0;
  adc_errors = 0; buffer_errors = 0;
  adc_rtc_stop = 0;

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
      return false;
    }

    gc_st |= GC_ST_RUN;
    return true;
  } else {
    gc_println(PSTR("error:start: file!"));
    return false;
  }
}

void loop()
{
  while (gc_st & GC_ST_RUN) {
    for (; delta > 3; delta--) {
      if (sd_head == gc_cfg.sd_buffer_size) {
        file.write(sd_buffer, gc_cfg.sd_buffer_size_bytes);
        sd_head = 0;
      }

      sd_buffer[sd_head++] = adc_ring_buffer[tail++];

      tail &= gc_cfg.adc_buffer_hash;
    }

    if (file.writeError) {
      gc_println(PSTR("error:loop: file write!"));

      if (adc_rtc_stop == 0) adc_rtc_stop = Teensy3Clock.get();

      gc_st &= ~GC_ST_RUN;
      gc_st |= GC_ST_STOP;

      adc_play_cnt = 0;
    }
  }

  if (gc_st & GC_ST_STOP) {
    if (gc_stop()) {
      gc_println(PSTR("Stopped!"));
      uint8_t end[2] = { 'e', 'c' };
      gc_cmd(end, 2);
    } else {
      gc_println(PSTR("Stopped?"));
    }

    if (buffer_errors) gc_println(PSTR("error:loop: buffer!"));
    if (adc_errors) gc_println(PSTR("error:loop: adc!"));

    switch (gc_cfg.time_type) {
      case 0: { // chrono
        deep_sleep();
      } break;
      case 1: { // daily
        adc_play_cnt = sleep_daily();
        if (adc_play_cnt != 0 && gc_start()) gc_println(PSTR("Started!"));
        else adc_play_cnt = 0;
      } break;
    }
  }

  while (!(gc_st & (GC_ST_RUN | GC_ST_STOP))) {
    uint8_t cmd = 0;
    uint32_t length = 0;

    if (HIGH == digitalRead(USB_PIN)) {
      if ((boolean)Serial.available()) {
        cmd = Serial.read();
        length = (uint32_t)Serial.available();
      }

      time = millis();
    } else if (gc_cfg.time_type == 1 && time + 10000 <= millis()) {
      cmd = 'n';
    }

    if (cmd > 0) {
      uint8_t message[FILENAME_MAX_LENGH+1];
      SdFile fileData;

      bool write_cfg = false;
      switch (cmd) {
        case 'j': {
          uint32_t tj = (uint32_t)Serial.parseInt() % 86400;
          gc_cfg.time_begin_seg = tj;

          write_cfg = true;
        } break;
        case 'k': {
          uint32_t tk = (uint32_t)Serial.parseInt() % 86400;
          gc_cfg.time_end_seg = tk;

          write_cfg = true;
        } break;
        case 'n': { // start
          switch (gc_cfg.time_type) {
            case 0: {
              adc_play_cnt = sleep_chrono();
            } break;
            case 1: {
              adc_play_cnt = sleep_daily();
            } break;
          }
          if (adc_play_cnt != 0 && gc_start()) gc_println(PSTR("Started!"));
        } break;
        case 'l': { // ls command
          uint8_t head[1] = { 'l' };
          gc_cmd(head, 1);

          sd.ls(LS_R);
          gc_println(PSTR("list with cmd!"));
        } break;
        case 'g': { // get a file
          for (cmd = 0; cmd <= FILENAME_MAX_LENGH && cmd < length; cmd++)
            message[cmd] = Serial.read();
          message[cmd] = '\0';

          if (cmd > 0) {
            if (!fileData.open((char*)message, O_READ)) {
              gc_println(PSTR("error:loop:conf:g: open file data!"));
            } else {
              uint8_t head[2] = { 'f', cmd };
              gc_cmd(head, 2);

              Serial.write(message, cmd);
              int16_t data;
              while ((data = fileData.read()) >= 0) Serial.write((uint8_t)data);

              uint8_t eof_cmd[2] = { 'g', cmd };
              gc_cmd(eof_cmd, 2);

              gc_println(PSTR("Getting file with cmd!"));
            }
            gc_println((char*)message);
          } else {
            gc_println(PSTR("error:loop:conf:g: empty filename!"));
          }
        } break;
        case 'r': { // remove file
          for (cmd = 0; cmd <= FILENAME_MAX_LENGH && cmd < length; cmd++)
            message[cmd] = Serial.read();
          message[cmd] = '\0';

          if (cmd > 0) {
            if (!fileData.open((char*)message, O_WRITE)) {
              gc_println(PSTR("error:loop:conf:r: open file data!"));
            } else {
              if (!fileData.remove()) {
                gc_println(PSTR("error:loop:conf:r: remove file data!"));
              } else {
                gc_println(PSTR("Remove file with cmd!"));
              }
            }
            gc_println((char*)message);
          }
        } break;
        case 's': {
          if (length > 0) {
            cmd = Serial.read();

            switch (cmd) {
              case 'o': {
                if (length > 1) {
                  cmd = Serial.read();
                  gc_cfg.set_gain(cmd);
                  gain_cfg(cmd);

                  write_cfg = true;
                }
              } break;
              case 'g': {
                uint8_t settings[20] = { 's',
                                         (gc_cfg.gain << 4) | gc_cfg.average,
                                         ((uint8_t*)&gc_cfg.tick_time_useg)[0],
                                         ((uint8_t*)&gc_cfg.tick_time_useg)[1],
                                         ((uint8_t*)&gc_cfg.tick_time_useg)[2],
                                         ((uint8_t*)&gc_cfg.tick_time_useg)[3],
                                         gc_cfg.time_type,
                                         ((uint8_t*)&gc_cfg.time_begin_seg)[0],
                                         ((uint8_t*)&gc_cfg.time_begin_seg)[1],
                                         ((uint8_t*)&gc_cfg.time_begin_seg)[2],
                                         ((uint8_t*)&gc_cfg.time_begin_seg)[3],
                                         ((uint8_t*)&gc_cfg.time_end_seg)[0],
                                         ((uint8_t*)&gc_cfg.time_end_seg)[1],
                                         ((uint8_t*)&gc_cfg.time_end_seg)[2],
                                         ((uint8_t*)&gc_cfg.time_end_seg)[3],
                                         ((uint8_t*)&gc_cfg.adc_buffer_size)[0],
                                         ((uint8_t*)&gc_cfg.adc_buffer_size)[1],
                                         ((uint8_t*)&gc_cfg.sd_buffer_size)[0],
                                         ((uint8_t*)&gc_cfg.sd_buffer_size)[1],
                };
                gc_cmd(settings, 20);
              } break;
              case 'p': {
                gc_println(PSTR("Settings:"));
                gc_cfg.print();
                digitalClockDisplay();
              } break;
              case 'm': {
                if (length > 1) {
                  cmd = Serial.read();
                  gc_cfg.set_average(cmd);
                  write_cfg = true;
                }
              } break;
              case 's': {
                if (length > 4) {
                  uint32_t tick_time_useg;
                  ((uint8_t*)&tick_time_useg)[0] = Serial.read();
                  ((uint8_t*)&tick_time_useg)[1] = Serial.read();
                  ((uint8_t*)&tick_time_useg)[2] = Serial.read();
                  ((uint8_t*)&tick_time_useg)[3] = Serial.read();
                  gc_cfg.set_tick_time_useg(tick_time_useg);
                  write_cfg = true;
                }
              } break;
              case 't': {
                if (length > 1) {
                  cmd = Serial.read();
                  gc_cfg.set_time_type(cmd);
                  write_cfg = true;
                }
              } break;
              case 'u': {
                if (length > 4) {
                  uint32_t time_begin_seg;
                  ((uint8_t*)&time_begin_seg)[0] = Serial.read();
                  ((uint8_t*)&time_begin_seg)[1] = Serial.read();
                  ((uint8_t*)&time_begin_seg)[2] = Serial.read();
                  ((uint8_t*)&time_begin_seg)[3] = Serial.read();
                  gc_cfg.set_time_begin(time_begin_seg);
                  write_cfg = true;
                }
              } break;
              case 'v': {
                if (length > 4) {
                  uint32_t time_end_seg;
                  ((uint8_t*)&time_end_seg)[0] = Serial.read();
                  ((uint8_t*)&time_end_seg)[1] = Serial.read();
                  ((uint8_t*)&time_end_seg)[2] = Serial.read();
                  ((uint8_t*)&time_end_seg)[3] = Serial.read();
                  gc_cfg.set_time_end(time_end_seg);
                  write_cfg = true;
                }
              } break;
              case 'a': {
                if (length > 2) {
                  uint16_t adc_buffer_size;
                  ((uint8_t*)&adc_buffer_size)[0] = Serial.read();
                  ((uint8_t*)&adc_buffer_size)[1] = Serial.read();
                  gc_cfg.set_adc_buffer_size(adc_buffer_size);
                  write_cfg = true;
                }
              } break;
              case 'b': {
                if (length > 2) {
                  uint16_t sd_buffer_size;
                  ((uint8_t*)&sd_buffer_size)[0] = Serial.read();
                  ((uint8_t*)&sd_buffer_size)[1] = Serial.read();
                  gc_cfg.set_sd_buffer_size(sd_buffer_size);
                  write_cfg = true;
                }
              } break;
              case 'r': {
                time_t pctime = (time_t)(Serial.parseInt() + 1);
                if (pctime != 0) {
                  delay(10);
                  Teensy3Clock.set(pctime); // set the RTC
                  setTime(pctime);
                }
              } break;
              case 'w': {
                if (length > 1) {
                  cmd = Serial.read();
                  gc_power(cmd);
                }
              } break;
              default: {
                gc_println(PSTR("bad set!"));
              } break;
            }
          } else {
            gc_println(PSTR("empty set!"));
          }
        } break;
        default: {
          gc_println(PSTR("bad cmd!"));
        } break;
      }

      if (write_cfg) {
        gc_cfg.write();

        // configure adc
        adc_cfg();

        // configure DMA
        dma_cfg();

        // reconfigure
        rcfg();

        gc_cfg.print();
        digitalClockDisplay();
      }
    }
  }
}

void gc_cmd(uint8_t* cmd, uint8_t n)
{
  uint8_t head[9] = { '\xaa', '\xaa', '\xaa',
                      '\xff', '\xff', '\xff',
                      '\x00', '\x00', '\x00'};
  Serial.write(head, 9);
  Serial.write(cmd, n);
}

void gc_println(const char *log_string)
{
  uint8_t cmd[1] = { 't' };
  gc_cmd(cmd, 1);

  Serial.println(log_string);
}

void gc_print(const char *log_string)
{
  uint8_t cmd[1] = { 't' };
  gc_cmd(cmd, 1);

  Serial.print(log_string);
}

void printDigits(int digits)
{
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

void digitalClockDisplay()
{
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year());
  Serial.println();
}
