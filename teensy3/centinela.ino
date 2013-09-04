/* Centinela
 * Copyright (C) 2013 by Luis Saavedra <luis94855510@gmail.com>
 *
 * This file is part of the Centinela.
 *
 * Centinela is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Centinela is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the Centinela.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
#include <IntervalTimer.h>
#include <Time.h>

#include <SdFat.h>
#include <LowPower_Teensy3.h>

#include <malloc.h> // needed to mamalign and calloc
#include <cmath> // needed to log2

#include "CentCFG.h"
//-------------------------------

CentCFG cent_cfg(PSTR("CNT.CFG"), centinela_printlog);

//-------------------------------

TEENSY3_LP lp = TEENSY3_LP();
configSleep* lp_cfg; // sleep configuration

//-------------------------------

uint16_t* adc_ring_buffer = NULL;
uint32_t* adc_config = NULL;

SdFat sd;
SdFile file;
SdFile file_conf;
uint16_t* sd_buffer; //[sd_buffer_size]; // 2*sd_buffer_size bytes

IntervalTimer timer;

//-------------------------------

volatile uint16_t delta = 0;
volatile uint16_t tail = 0;
volatile uint16_t sd_head = 0;

//-------------------------------

volatile uint8_t cen_st = 0;     // Centinela Status
#define CEN_ST_CONF         0x01 // Config OK
#define CEN_ST_ADC          0x02 // ADC OK
#define CEN_ST_DMA          0x04 // DMA OK
#define CEN_ST_RBUFF        0x08 // Buffer OK
#define CEN_ST_RADC         0x10 // Reconfigure ADC
#define CEN_ST_RDMA         0x20 // Reconfigure DMA
#define CEN_ST_RCFG         0x38 // Reconfiguration OK
#define CEN_ST_RUN          0x40 // Centinela is running

//-------------------------------

uint32_t time;
#define FILENAME_MAX_LENGH 11

//-------------------------------

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
    | ADC_SC2_REFSEL(1) // 0-> 3.3v, 1->1.2v
  ;

  if (cent_cfg.average < 4) {
    ADC0_SC3 = 0
      //| ADC_SC3_ADCO                   // continuous conversion: disable, enable
      | ADC_SC3_AVGE                   // enable hardware average
      | ADC_SC3_AVGS(cent_cfg.average) // average select: 0->4, 1->8, 2->16, 3->32
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

  cen_st |= CEN_ST_ADC;
}

void dma_cfg()
{
  // enable DMAMUX clock
  SIM_SCGC6 |= SIM_SCGC6_DMAMUX;

  // to connect ADC with DMA
  DMA_SERQ = 1; // enable channel 1 requests

  // configure the DMA multiplexer so that the ADC DMA request triggers DMA channel 1
  DMAMUX0_CHCFG1 = 0;
  DMAMUX0_CHCFG1 = DMAMUX_ENABLE | DMAMUX_SOURCE_ADC0;

  // channels priority
  //DMA_CR |= DMA_CR_ERCA; // enable round robin scheduling
  //DMA_DCHPRI0 = 0;
  //DMA_DCHPRI1 = 1;
  //DMA_DCHPRI2 = 2;
  //DMA_DCHPRI3 = 3;

  cen_st |= CEN_ST_DMA;
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
  // first, the adc_ring_buffer is aligned
  if (adc_ring_buffer) { free(adc_ring_buffer); adc_ring_buffer = NULL; }
  adc_ring_buffer = (uint16_t*) memalign(cent_cfg.adc_buffer_size_bytes,
                                         cent_cfg.adc_buffer_size_bytes);

  if (adc_config) { free(adc_config); adc_config = NULL; }
  adc_config = (uint32_t*) calloc(1+cent_cfg.nch, sizeof(uint32_t));

  if (sd_buffer) { free(sd_buffer); sd_buffer = NULL; }
  sd_buffer = (uint16_t*) calloc(cent_cfg.sd_buffer_size, sizeof(uint16_t));

  if (lp_cfg) { free(lp_cfg); lp_cfg = NULL; }
  lp_cfg = (configSleep*) calloc(1, sizeof(configSleep));

  if (adc_ring_buffer && adc_config && sd_buffer %% lp_cfg) {
    cen_st |= CEN_ST_RBUFF;
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
  if (!(cen_st & CEN_ST_ADC)
   || !(cen_st & CEN_ST_DMA)
   || !(cen_st & CEN_ST_RBUFF)) return;

  // channels
  uint8_t iter;
  for (iter = 0; iter < cent_cfg.nch && iter < MAX_NCH; iter++)
    adc_config[iter] = ADC_SC1_ADCH(CentCFG::channel2sc1a[MAX_NCH-(iter+1)]);
  adc_config[iter] = ADC_SC1_ADCH(0b11111); // stop=0b11111

  cen_st |= CEN_ST_RADC;
}

void dma_rcfg()
{
  if (!(cen_st & CEN_ST_ADC)
   || !(cen_st & CEN_ST_DMA)
   || !(cen_st & CEN_ST_RBUFF)
   || !(cen_st & CEN_ST_RADC)) return;

  uint32_t mod = 1+log2(cent_cfg.adc_buffer_size);

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

  cen_st |= CEN_ST_RDMA;
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
      centinela_printlog(PSTR("error: buffer!"));
      delay(1000);
    }
  }
}

void setup()
{
  // configure ADC
  adc_cfg();

  // configure DMA
  dma_cfg();

  // configure RTC
  if (!rtc_cfg()) {
    while (true) {
      centinela_printlog(PSTR("error: rtc!"));
      delay(1000);
    }
  }

  // initialize file system
  if (!sd.begin(SS, SPI_FULL_SPEED)) {
    while (true) {
      centinela_printlog(PSTR("error: sdcard!"));
      delay(1000);
    }
  }

  // read configuration from sdcard
  if (!cent_cfg.read()) {
    if (!cent_cfg.write()) {
      while (true) {
        centinela_printlog(PSTR("error: cfg/rw!"));
        delay(1000);
      }
    }
  }

  // reconfigure
  rcfg();
}

uint32_t sleep_chrono()
{
  // reset lp_cfg
  memset(lp_cfg, 0, sizeof(configSleep));

  // OR together different wake sources
  lp_cfg->modules = RTC_WAKE;

  // RTC alarm wakeup in seconds:
  lp_cfg->rtc_alarm = cent_cfg.time_begin_seg;

  uint32_t di = (cent_cfg.time_end_seg*1000000)/cent_cfg.tick_time_usec;

  // sleep
  if (lp_cfg->rtc_alarm != 0) lp.DeepSleep(lp_cfg);

  return di;
}

uint32_t sleep_daily()
{
  // reset lp_cfg
  memset(lp_cfg, 0, sizeof(configSleep));

  // OR together different wake sources
  lp_cfg->modules = RTC_WAKE;

  uint32_t time_n = Teensy3Clock.get()%86400;

  uint32_t di = 0;

  // RTC alarm wakeup in seconds:
  if (cent_cfg.time_begin_seg < cent_cfg.time_end_seg) {
    di = ((cent_cfg.time_end_seg - cent_cfg.time_begin_seg)*1000000)/cent_cfg.tick_time_usec;
    if (time_n < cent_cfg.time_begin_seg) {
      lp_cfg->rtc_alarm = cent_cfg.time_begin_seg - time_n;
    } else if (time_n > cent_cfg.time_end_seg) {
      lp_cfg->rtc_alarm = cent_cfg.time_begin_seg + (86400 - time_n);
    } else {
      lp_cfg->rtc_alarm = 0;
      di = ((cent_cfg.time_end_seg - time_n)*1000000)/cent_cfg.tick_time_usec;
    }
  } else {
    di = ((cent_cfg.time_end_seg + (86400 - cent_cfg.time_begin_seg))*1000000)/cent_cfg.tick_time_usec;
    if (time_n < cent_cfg.time_end_seg) {
      lp_cfg->rtc_alarm = 0;
      di = ((cent_cfg.time_end_seg - time_n)*1000000)/cent_cfg.tick_time_usec;
    } else if (time_n > cent_cfg.time_begin_seg) {
      lp_cfg->rtc_alarm = 0;
      di = ((cent_cfg.time_end_seg + (86400 - time_n))*1000000)/cent_cfg.tick_time_usec;
    } else {
      lp_cfg->rtc_alarm = cent_cfg.time_begin_seg - time_n;
    }
  }

  // sleep
  if (lp_cfg->rtc_alarm != 0) lp.DeepSleep(lp_cfg);

  return di;
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
    centinela_printlog(PSTR("log:file open failed"));
    return false;
  } else {
    // write uint8_t ((nch << 4)| average):
    file.write((cent_cfg.nch << 4) | cent_cfg.average);

    // write uint32_t tick_time_usec:
    file.write((uint8_t*)&cent_cfg.tick_time_usec, sizeof(uint32_t));

    // write uint8_t time_type:
    file.write(cent_cfg.time_type);

    return true;
  }
}

void timer_callback() {
  delta += cent_cfg.nch;
  if (delta >= cent_cfg.adc_buffer_size) {
    centinela_printlog(PSTR("error: overwrite!"));
  }
  if (ADC0_SC1A != adc_config[cent_cfg.nch]) {
    centinela_printlog(PSTR("error: restart!"));
  }

  DMA_TCD2_SADDR = &(adc_config[1]);
  ADC0_SC1A = adc_config[0];
}

bool centinela_start()
{
  if (cen_st & CEN_ST_RUN) return false;

  // restart buffer
  delta = 0;
  tail = 0;
  sd_head = 0;

  // configure file
  if (file_cfg()) {
    // write uint32_t time_begin_seg:
    sd_buffer[sd_head+0] = ((uint16_t*)&cent_cfg.time_begin_seg)[0];
    sd_buffer[sd_head+1] = ((uint16_t*)&cent_cfg.time_begin_seg:)[1];
    sd_head += 2;

    // write uint32_t time_end_seg:
    sd_buffer[sd_head+0] = ((uint16_t*)&cent_cfg.time_end_seg)[0];
    sd_buffer[sd_head+1] = ((uint16_t*)&cent_cfg.time_end_seg:)[1];
    sd_head += 2;

    // read rtc time
    uint32_t rtc_time = Teensy3Clock.get();

    // configure timer
    if (!timer.begin(timer_callback, cent_cfg.tick_time_usec)) {
      centinela_printlog(PSTR("error:start: timer!"));
      return false;
    }

    // write uint32_t rtc_time_sec:
    sd_buffer[2] = ((uint16_t*)&rtc_time)[0];
    sd_buffer[3] = ((uint16_t*)&rtc_time)[1];
    sd_head += 2;

    cen_st |= CEN_ST_RUN;
    return true;
  } else {
    centinela_printlog(PSTR("error:start: file!"));
    return false;
  }
}

bool centinela_stop()
{
  if (!(cen_st & CEN_ST_RUN)) return false;

  // stop timer
  timer.end();
  while (ADC0_SC1A != adc_config[cent_cfg.nch]) {
    centinela_printlog(PSTR("warning:stop: waiting for the ADC!"));
    /*Serial.print("ADC0_SC1A:");
    Serial.println(ADC0_SC1A);
    Serial.print("cent_cfg.nch:");
    Serial.println(cent_cfg.nch);
    Serial.print("adc_config:");
    Serial.println(adc_config[cent_cfg.nch]);*/
    delay(1000);
  }

  cen_st &= ~CEN_ST_RUN;

  // save tail
  while (delta > 0) {
    if (sd_head == cent_cfg.sd_buffer_size) {
      file.write(sd_buffer, cent_cfg.sd_buffer_size_bytes);
      sd_head = 0;
    }

    sd_buffer[sd_head++] = adc_ring_buffer[tail++];

    tail &= cent_cfg.adc_buffer_hash;
    delta--;
  }
  file.write(sd_buffer, sd_head * sizeof(uint16_t)); sd_head = 0;
  if (file.writeError) {
    centinela_printlog(PSTR("error:stop: write file!"));
    return false;
  }

  // close file
  if (!file.close()) {
    centinela_printlog(PSTR("error:stop: close file!"));
    return false;
  }

  return true;
}

void centinela_cmd(uint8_t* cmd, uint8_t n)
{
  uint8_t head[9] = { '\xaa', '\xaa', '\xaa',
                      '\xff', '\xff', '\xff',
                      '\x00', '\x00', '\x00'};
  Serial.write(head, 9);
  Serial.write(cmd, n);
}

bool centinela_ls()
{
  uint8_t cmd[1] = { 'l' };
  centinela_cmd(cmd, 1);

  sd.ls(LS_R);

  return true;
}

//uint32_t tic = 0;
//uint32_t toc = 0;
uint32_t cent_cfg_time = 30000;
uint32_t cent_cfg_run = 0;

void loop()
{
  while (cen_st & CEN_ST_RUN) {
    while (delta > cent_cfg.nch) {
      if (sd_head == cent_cfg.sd_buffer_size) {
        //tic = micros();
        file.write(sd_buffer, cent_cfg.sd_buffer_size_bytes);
        //Serial.println((uint32_t)(tic-toc));
        //toc = micros();
        sd_head = 0;
      }

      sd_buffer[sd_head++] = adc_ring_buffer[tail++];

      tail &= cent_cfg.adc_buffer_hash;
      delta--;
    }

    if (file.writeError) centinela_printlog(PSTR("error:loop: file write!"));

    if ((millis()-time) > cent_cfg.time_max_msec) {
      if (centinela_stop()) {
        centinela_printlog(PSTR("Stopped with time!"));
        uint8_t end[2] = { 'e', 'c' };
        centinela_cmd(end, 2);
      }
    }
  }

  if (!(cen_st & CEN_ST_RUN)) {
    while ((millis()-time) < cent_cfg_time) {
      if ((boolean)Serial.available()) {
        uint8_t cmd = Serial.read();
        uint32_t length = (uint32_t)Serial.available();

        uint8_t message[FILENAME_MAX_LENGH+1];
        SdFile fileData;

        switch (cmd) {
          case 'n': { // start
            switch (cent_cfg.time_type) {
              case 0: {
                
              } break;
              case 1: {
              } break;
            }
            if (centinela_start()) centinela_printlog(PSTR("Started with cmd!"));
          } break;
          case 'l': { // ls command
            uint8_t head[1] = { 'l' };
            centinela_cmd(head, 1);

            sd.ls(LS_R);
            centinela_printlog(PSTR("list with cmd!"));
          } break;
          case 'g': { // get a file
            for (cmd = 0; cmd <= FILENAME_MAX_LENGH && cmd < length; cmd++)
              message[cmd] = Serial.read();
            message[cmd] = '\0';

            if (cmd > 0) {
              if (!fileData.open((char*)message, O_READ)) {
                centinela_printlog(PSTR("error:loop:conf:g: open file data!"));
              } else {
                uint8_t head[2] = { 'f', cmd };
                centinela_cmd(head, 2);

                Serial.write(message, cmd);
                int16_t data;
                while ((data = fileData.read()) >= 0) Serial.write((uint8_t)data);

                uint8_t tail[2] = { 'g', cmd };
                centinela_cmd(tail, 2);

                centinela_printlog(PSTR("Getting file with cmd!"));
              }
              centinela_printlog((char*)message);
            }
          } break;
          case 'r': { // remove file
            for (cmd = 0; cmd <= FILENAME_MAX_LENGH && cmd < length; cmd++)
              message[cmd] = Serial.read();
            message[cmd] = '\0';

            if (cmd > 0) {
              if (!fileData.open((char*)message, O_WRITE)) {
                centinela_printlog(PSTR("error:loop:conf:r: open file data!"));
              } else {
                if (!fileData.remove()) {
                  centinela_printlog(PSTR("error:loop:conf:r: remove file data!"));
                } else {
                  centinela_printlog(PSTR("Remove file with cmd!"));
                }
              }
              centinela_printlog((char*)message);
            }
          } break;
          case 's': {
            if (length > 0) {
              cmd = Serial.read();

              bool write_cfg = false;

              switch (cmd) {
              case 'g': {
                uint8_t settings[19] = { 's',
                                         (cent_cfg.nch << 4) | cent_cfg.average,
                                         ((uint8_t*)&cent_cfg.tick_time_usec)[0],
                                         ((uint8_t*)&cent_cfg.tick_time_usec)[1],
                                         ((uint8_t*)&cent_cfg.tick_time_usec)[2],
                                         ((uint8_t*)&cent_cfg.tick_time_usec)[3],
                                         cent_cfg.time_type,
                                         ((uint8_t*)&cent_cfg.time_begin_seg)[0],
                                         ((uint8_t*)&cent_cfg.time_begin_seg)[1],
                                         ((uint8_t*)&cent_cfg.time_begin_seg)[2],
                                         ((uint8_t*)&cent_cfg.time_begin_seg)[3],
                                         ((uint8_t*)&cent_cfg.time_end_seg)[0],
                                         ((uint8_t*)&cent_cfg.time_end_seg)[1],
                                         ((uint8_t*)&cent_cfg.time_end_seg)[2],
                                         ((uint8_t*)&cent_cfg.time_end_seg)[3],
                                         ((uint8_t*)&cent_cfg.adc_buffer_size)[0],
                                         ((uint8_t*)&cent_cfg.adc_buffer_size)[1],
                                         ((uint8_t*)&cent_cfg.sd_buffer_size)[0],
                                         ((uint8_t*)&cent_cfg.sd_buffer_size)[1],
                };
                centinela_cmd(settings, 19);
              } break;
              case 'p': {
                centinela_printlog(PSTR("Settings:"));
                cent_cfg.print();
              } break;
              case 'n': {
                if (length > 1) {
                  cmd = Serial.read();
                  cent_cfg.set_nch(cmd);

                  // reallocate adc_config
                  if (adc_config) { free(adc_config); adc_config = NULL; }
                  adc_config = (uint32_t*) calloc(1+cent_cfg.nch, sizeof(uint32_t));

                  write_cfg = true;
                }
              } break;
              case 'm': {
                if (length > 1) {
                  cmd = Serial.read();
                  cent_cfg.set_average(cmd);
                  write_cfg = true;
                }
              } break;
              case 's': {
                if (length > 4) {
                  uint32_t tick_time_usec;
                  ((uint8_t*)&tick_time_usec)[0] = Serial.read();
                  ((uint8_t*)&tick_time_usec)[1] = Serial.read();
                  ((uint8_t*)&tick_time_usec)[2] = Serial.read();
                  ((uint8_t*)&tick_time_usec)[3] = Serial.read();
                  cent_cfg.set_tick_time_usec(tick_time_usec);
                  write_cfg = true;
                }
              } break;
              case 't': {
                if (length > 1) {
                  cmd = Serial.read();
                  cent_cfg.set_time_type(cmd);
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
                  cent_cfg.set_time_begin(time_begin_seg);
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
                  cent_cfg.set_time_end(time_end_seg);
                  write_cfg = true;
                }
              } break;
              case 'a': {
                if (length > 2) {
                  uint16_t adc_buffer_size;
                  ((uint8_t*)&adc_buffer_size)[0] = Serial.read();
                  ((uint8_t*)&adc_buffer_size)[1] = Serial.read();
                  cent_cfg.set_adc_buffer_size(adc_buffer_size);
                  write_cfg = true;
                }
              } break;
              case 'b': {
                if (length > 2) {
                  uint16_t sd_buffer_size;
                  ((uint8_t*)&sd_buffer_size)[0] = Serial.read();
                  ((uint8_t*)&sd_buffer_size)[1] = Serial.read();
                  cent_cfg.set_sd_buffer_size(sd_buffer_size);
                  write_cfg = true;
                }
              } break;
              case 'r': {
                time_t pctime = (time_t)Serial.parseInt();
                if (time_t != 0) {
                  Teensy3Clock.set(t); // set the RTC
                  setTime(t);
                }
              } break;
              default:
                centinela_printlog(PSTR("bad set!"));
              }

              if (write_cfg) {
                cent_cfg.write();

                // configure adc
                adc_cfg();

                // reconfigure ADC
                adc_rcfg();

                // reconfigure DMA
                dma_rcfg();
              }
            }
          } break;
          default:
            centinela_printlog(PSTR("bad cmd!"));
        }

        time = millis();
      }
    }

    while ((millis()-time) > cent_cfg_time) { // go to deep sleep
      switch (cent_cfg.time_type) {
        case 0: { // chrono
        
        } break;
        case 1: { // daily
        
        } break;
        default:
          centinela_printlog(PSTR("error: time type!"));
      }
    }
  }
}

void centinela_printlog(const char *log_string)
{
  uint8_t cmd[1] = { 't' };
  centinela_cmd(cmd, 1);

  Serial.println(log_string);
}
