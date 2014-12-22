#include <IntervalTimer.h>

#include <TinyGPS.h>
#include <Time.h>

#include <SPI.h>
#include <SdFat.h>

#include <LowPower_Teensy3.h>

#include <malloc.h> // needed to memalign and calloc
#include <cmath>    // needed to log2

#include "gcCFG.h"
#include "gcAll.h"

void start_reading()
{
  cli();
  if (LOW == digitalRead(PIN_USB)) {
    attachInterrupt(PIN_USB, stop_reading, RISING);
    gcPlayStat |= GC_ST_PLAY;
    gcPlayStat |= GC_ST_SLEEP;
  }
  sei();
}

void stop_reading()
{
  cli();
  setPowerDown(ANALOG1_MASK);
  if (adc_rtc_stop == 0) adc_rtc_stop = Teensy3Clock.get();

  gcPlayStat &= ~GC_ST_SLEEP;
  gcPlayStat &= ~GC_ST_READING;
  gcPlayStat |= GC_ST_STOP;
  adc_play_cnt = 0;
  sei();

  if (HIGH == digitalRead(PIN_USB)) {
    attachInterrupt(PIN_USB, start_reading, FALLING);
    gcPlayStat |= GC_ST_CONFIG;
  } 
  else gcPlayStat &= ~GC_ST_CONFIG;
}

void setup()
{
  // configure POW
  cfgPow();

  // digital power up
  setPowerUp(DIGITAL_MASK); delay(200);

  // USB wakeup
  pinMode(PIN_USB, INPUT);
  *portConfigRegister(PIN_USB) = PORT_PCR_MUX(1) | PORT_PCR_PE;

  if (HIGH == digitalRead(PIN_USB)) {
    attachInterrupt(PIN_USB, start_reading, FALLING);
    gcPlayStat |= GC_ST_CONFIG;
    gcPlayStat &= ~GC_ST_SLEEP;
  }
  else {
    start_reading();
  }

  // read configuration from sdcard
  while(!gc_cfg.read()) {
    while(!gc_cfg.write()) {
      gc_println(PSTR("error: cfg/rw!"));
      delay(LOG_TIMEOUT);
    }
  }

  // checking RTC
  while (timeStatus() != timeSet) {
    gc_println(PSTR("error: rtc!"));
    delay(LOG_TIMEOUT);
    setSyncProvider(getTeensy3Time);
  }

  // configure ADC
  cfgAdc();

  // configure DMA
  cfgDma();

  // reconfigure (buffer?)
  rcfg();

  // gain configuration
  cfgGain(gc_cfg.gain);

  // sync gps
  //syncGps();
}

void loop()
{
  if (LOW == digitalRead(PIN_USB)) {
    while (gcPlayStat & GC_ST_READING) {
      while (delta > 3) {
        if (sd_head == gc_cfg.sd_buffer_size) {
          file.write(sd_buffer, gc_cfg.sd_buffer_size_bytes);
          sd_head = 0;
        }

        sd_buffer[sd_head++] = adc_ring_buffer[tail++];

        tail &= gc_cfg.adc_buffer_hash;

        cli();
        delta--;
        sei();
      }

      if (file.writeError) {
        gc_println(PSTR("error:loop: file write!"));
        stop_reading();
      }
    }

    if (gcPlayStat & GC_ST_STOP) {
      if (gc_stop()) {
        gc_println(PSTR("Stopped!"));
        uint8_t end[2] = { 
          'e', 'c'         };
        gc_cmd(end, 2);
      } 
      else {
        gc_println(PSTR("Stopped?"));
      }

      if (buffer_errors) gc_println(PSTR("error:loop: buffer!"));
      if (adc_errors) gc_println(PSTR("error:loop: adc!"));
    }

    if (gcPlayStat & GC_ST_PLAY) {
      switch (gc_cfg.time_type) {
      case 0: 
        {
          if (gcPlayStat & GC_ST_CONFIG) adc_play_cnt = sleep_chrono();
          else adc_play_cnt = deep_sleep();
        } 
        break;
      case 1: 
        {
          adc_play_cnt = sleep_daily();
        } 
        break;
      }

      if (adc_play_cnt != 0 && gc_start()) {
        gcPlayStat &= ~GC_ST_CONFIG;
        gc_println(PSTR("Started!"));
      } 
      else {
        if (0 == adc_play_cnt) {

        } 
        else {
          stop_reading();
          /*
  uint32_t t = Teensy3Clock.get() % SEG_A_DAY;
           while (!Serial.available()) {
           uint32_t s = Teensy3Clock.get() % SEG_A_DAY;
           Serial.print("no:");
           Serial.print(s-t);
           Serial.print(":");
           Serial.println(adc_play_cnt);
           delay(1000);
           }
           while(Serial.available()) Serial.read();
           */
        }
      }
    }
  } 
  else {
    uint8_t cmd = 0;
    uint32_t length = 0;

    if ((boolean)Serial.available()) {
      cmd = Serial.read();
      length = (uint32_t)Serial.available();
    }

    if (cmd > 0) {
      uint8_t message[FILENAME_MAX_LENGH+1];
      SdFile fileData;

      bool write_cfg = false;
      switch (cmd) {
      case 'j': 
        {
          uint32_t tj = (uint32_t)Serial.parseInt();
          gc_cfg.set_time_begin(tj % SEG_A_DAY);

          write_cfg = true;
        } 
        break;
      case 'k': 
        {
          uint32_t tk = (uint32_t)Serial.parseInt() % SEG_A_DAY;
          gc_cfg.set_time_end(tk % SEG_A_DAY);

          write_cfg = true;
        } 
        break;
      case 'l': 
        { // ls command
          uint8_t head[1] = { 
            'l'           };
          gc_cmd(head, 1);

          sd.ls(LS_R);
          gc_println(PSTR("list with cmd!"));
        } 
        break;
      case 'g': 
        { // get a file
          for (cmd = 0; cmd <= FILENAME_MAX_LENGH && cmd < length; cmd++)
            message[cmd] = Serial.read();
          message[cmd] = '\0';

          if (cmd > 0) {
            if (!fileData.open((char*)message, O_READ)) {
              gc_println(PSTR("error:loop:conf:g: open file data!"));
            } 
            else {
              uint8_t head[2] = { 
                'f', cmd               };
              gc_cmd(head, 2);

              Serial.write(message, cmd);
              int16_t data;
              while ((data = fileData.read()) >= 0) Serial.write((uint8_t)data);

              uint8_t eof_cmd[2] = { 
                'g', cmd               };
              gc_cmd(eof_cmd, 2);

              gc_println(PSTR("Getting file with cmd!"));
            }
            gc_println((char*)message);
          } 
          else {
            gc_println(PSTR("error:loop:conf:g: empty filename!"));
          }
        } 
        break;
      case 'r': 
        { // remove file
          for (cmd = 0; cmd <= FILENAME_MAX_LENGH && cmd < length; cmd++)
            message[cmd] = Serial.read();
          message[cmd] = '\0';

          if (cmd > 0) {
            if (!fileData.open((char*)message, O_WRITE)) {
              gc_println(PSTR("error:loop:conf:r: open file data!"));
            } 
            else {
              if (!fileData.remove()) {
                gc_println(PSTR("error:loop:conf:r: remove file data!"));
              } 
              else {
                gc_println(PSTR("Remove file with cmd!"));
              }
            }
            gc_print((char*)message);
          }
        } 
        break;
      case 's': 
        {
          if (length > 0) {
            cmd = Serial.read();

            switch (cmd) {
            case 'o': 
              {
                if (length > 1) {
                  cmd = Serial.read();
                  gc_cfg.set_gain(cmd);
                  cfgGain(cmd);

                  write_cfg = true;
                }
              } 
              break;
            case 'g': 
              {
                uint8_t settings[20] = { 
                  's',
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
              } 
              break;
            case 'p': 
              {
                gc_println(PSTR("Settings:"));
                gc_cfg.print();
                digitalClockDisplay();
              } 
              break;
            case 'm': 
              {
                if (length > 1) {
                  cmd = Serial.read();
                  gc_cfg.set_average(cmd);
                  write_cfg = true;
                }
              } 
              break;
            case 's': 
              {
                if (length > 4) {
                  uint32_t tick_time_useg;
                  ((uint8_t*)&tick_time_useg)[0] = Serial.read();
                  ((uint8_t*)&tick_time_useg)[1] = Serial.read();
                  ((uint8_t*)&tick_time_useg)[2] = Serial.read();
                  ((uint8_t*)&tick_time_useg)[3] = Serial.read();
                  gc_cfg.set_tick_time_useg(tick_time_useg);
                  write_cfg = true;
                }
              } 
              break;
            case 't': 
              {
                if (length > 1) {
                  cmd = Serial.read();
                  gc_cfg.set_time_type(cmd);
                  write_cfg = true;
                }
              } 
              break;
            case 'u': 
              {
                if (length > 4) {
                  uint32_t time_begin_seg;
                  ((uint8_t*)&time_begin_seg)[0] = Serial.read();
                  ((uint8_t*)&time_begin_seg)[1] = Serial.read();
                  ((uint8_t*)&time_begin_seg)[2] = Serial.read();
                  ((uint8_t*)&time_begin_seg)[3] = Serial.read();
                  gc_cfg.set_time_begin(time_begin_seg % SEG_A_DAY);
                  write_cfg = true;
                }
              } 
              break;
            case 'v': 
              {
                if (length > 4) {
                  uint32_t time_end_seg;
                  ((uint8_t*)&time_end_seg)[0] = Serial.read();
                  ((uint8_t*)&time_end_seg)[1] = Serial.read();
                  ((uint8_t*)&time_end_seg)[2] = Serial.read();
                  ((uint8_t*)&time_end_seg)[3] = Serial.read();
                  gc_cfg.set_time_end(time_end_seg % SEG_A_DAY);
                  write_cfg = true;
                }
              } 
              break;
            case 'z': 
              {
                uint32_t z = Teensy3Clock.get();
                uint32_t u = (z+60*10) % SEG_A_DAY;
                uint32_t v = (z+60*20) % SEG_A_DAY;
                gc_cfg.set_time_begin(u);
                gc_cfg.set_time_end(v);
                write_cfg = true;
              } 
              break;
            case 'a': 
              {
                if (length > 2) {
                  uint16_t adc_buffer_size;
                  ((uint8_t*)&adc_buffer_size)[0] = Serial.read();
                  ((uint8_t*)&adc_buffer_size)[1] = Serial.read();
                  gc_cfg.set_adc_buffer_size(adc_buffer_size);
                  write_cfg = true;
                }
              } 
              break;
            case 'b': 
              {
                if (length > 2) {
                  uint16_t sd_buffer_size;
                  ((uint8_t*)&sd_buffer_size)[0] = Serial.read();
                  ((uint8_t*)&sd_buffer_size)[1] = Serial.read();
                  gc_cfg.set_sd_buffer_size(sd_buffer_size);
                  write_cfg = true;
                }
              } 
              break;
            case 'r': 
              {
                time_t pctime = (time_t)(Serial.parseInt());
                if (pctime != 0) {
                  delay(10);
                  Teensy3Clock.set(pctime); // set the RTC
                  setTime(pctime);
                }
              } 
              break;
            case 'w': 
              {
                if (length > 1) {
                  cmd = (uint8_t)Serial.read();
                  if ((cmd & POWER_UP_MASK) > 0) setPowerUp(cmd);
                  else setPowerDown(cmd);
                }
              } 
              break;
            case 'y': 
              {
                syncGps();
              } 
              break;
            default: 
              {
                gc_println(PSTR("bad set!"));
              } 
              break;
            }
          } 
          else {
            gc_println(PSTR("empty set!"));
          }
        } 
        break;
      default: 
        {
          gc_println(PSTR("bad cmd!"));
        } 
        break;
      }

      if (write_cfg) {
        gc_cfg.write();

        // configure adc
        cfgAdc();

        // configure DMA
        cfgDma();

        // reconfigure
        rcfg();

        gc_cfg.print();
        digitalClockDisplay();
      }
    }
  }
}
