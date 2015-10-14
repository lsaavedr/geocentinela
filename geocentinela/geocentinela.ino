#include <XBeeNG.h>

#include <IntervalTimer.h>

#include <TinyGPS.h>
#include <Time.h>

#include <SdFat.h>

#include <LowPower_Teensy3.h>

#include "gcAll.h"

void start_reading()
{
  noInterrupts();
  if (LOW == digitalRead(PIN_USB)) {
    attachInterrupt(PIN_USB, stop_reading, RISING);
    gcPlayStat = GC_ST_PLAY;
  }
  interrupts();
}

void stop_reading()
{
  noInterrupts();
  gcPlayStat = GC_ST_STOP;
  interrupts();

  if (HIGH == digitalRead(PIN_USB)) {
    attachInterrupt(PIN_USB, start_reading, FALLING);
    gcPlayStat |= GC_ST_CONFIG;
  } else gcPlayStat &= ~GC_ST_CONFIG;
}

void setup()
{
  // se configura el RTC
  cfgRTC();

  // se configura el PGA
  cfgPGA();

  // se configura la SD y el XBEE
  cfgSDX();

  // se configura el GPS
  cfgGPS();

  // configure ADC
  cfgADC();

  // configure DMA
  cfgDMA();

  // USB wakeup
  pinMode(PIN_USB, INPUT);
  *portConfigRegister(PIN_USB) = PORT_PCR_MUX(1) | PORT_PCR_PE; // INPUT_PULLDOWN
  delay(100);

  if (HIGH == digitalRead(PIN_USB)) {
    attachInterrupt(PIN_USB, start_reading, FALLING);
    gcPlayStat |= GC_ST_CONFIG;
  } else {
    start_reading();
  }
}

void loop()
{
  while (LOW == digitalRead(PIN_USB)) {
    if (gcPlayStat & GC_ST_PLAY) {
      gcPlayStat &= ~GC_ST_PLAY;

      switch (gc_cfg.time_type) {
        case 0: {
          adc_play_cnt = sleep_chrono();
        } break;
        case 1: {
          adc_play_cnt = sleep_daily();
        } break;
      }

      if (adc_play_cnt != 0 && gcStart()) {
        gc_println(PSTR("Started!"));
      } else {
        stop_reading();
      }
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
      }

      if (file.getWriteError()) {
        gc_println(PSTR("error:loop: file write!"));
        stop_reading();
      }
    }

    if (gcPlayStat & GC_ST_STOP) {
      if (gcStop()) {
        gc_println(PSTR("Stopped!"));
      } else {
        gc_println(PSTR("Stopped error!"));
      }

      if (buffer_errors) gc_println(PSTR("error:loop: buffer!"));
      if (adc_errors) gc_println(PSTR("error:loop: adc!"));

      switch (gc_cfg.time_type) {
        case 0: {
          gcPlayStat &= ~GC_ST_PLAY;
          attachInterrupt(PIN_USB, start_reading, FALLING);
          deep_sleep();
          gcPlayStat |= GC_ST_CONFIG;
        } break;
        case 1: {
          gcPlayStat |= GC_ST_PLAY;
        } break;
      }
    }
  }

  uint8_t cmd;
  uint32_t lcmd;
  while (HIGH == digitalRead(PIN_USB)) { // comands:
    cmd = 0;
    lcmd = 0;

    if (Serial.available()) {
      cmd = Serial.read();
      lcmd = (uint32_t)Serial.available();
    }

    if (cmd > 0) {
      uint8_t* message = (uint8_t*)sd_buffer;
      SdFile fileData;

      bool write_cfg = false;
      switch (cmd) {
        case 'n': {
          gc_println(PSTR("Date:"));
          digitalClockDisplay();
        } break;
        case 'j': {
          uint32_t tj = (uint32_t)Serial.parseInt();
          gc_cfg.set_time_begin(tj % SEG_A_DAY);
          write_cfg = true;
        } break;
        case 'k': {
          uint32_t tk = (uint32_t)Serial.parseInt() % SEG_A_DAY;
          gc_cfg.set_time_end(tk % SEG_A_DAY);
          write_cfg = true;
        } break;
        case 'l': { // ls command
          uint8_t head[1] = { 'l' };
          gc_cmd(head, 1);

          setPowerUp(SD_MASK);
          sd.ls(LS_R);
          setPowerDown(SD_MASK);

          gc_println(PSTR("list with cmd!"));
        } break;
        case 'g': { // get a file
          for (cmd = 0; cmd <= FILENAME_MAX_LENGH && cmd < lcmd; cmd++)
            message[cmd] = Serial.read();
          message[cmd] = '\0';

          if (cmd > 0) {
            setPowerUp(SD_MASK);
            if (!fileData.open((char*)message, O_READ)) {
              gc_println(PSTR("error:api:g: open file data!"));
            } else {
              uint8_t head[2] = { 'f', cmd };
              gc_cmd(head, 2);
              Serial.write(message, cmd);

              uint16_t n;
              while ((n = fileData.read(message, GC_SD_BUFFER_SIZE_BYTES)) > 0)
                Serial.write(message, n);

              uint8_t eof_cmd[2] = { 'g', cmd };
              gc_cmd(eof_cmd, 2);
              gc_println(PSTR("Getting file with cmd!"));
            }
            setPowerDown(SD_MASK);
            gc_println((char*)message);
          } else {
            gc_println(PSTR("error:loop:conf:g: empty filename!"));
          }
        } break;
        case 'r': { // remove file
          for (cmd = 0; cmd <= FILENAME_MAX_LENGH && cmd < lcmd; cmd++)
            message[cmd] = Serial.read();
          message[cmd] = '\0';

          if (cmd > 0) {
            setPowerUp(SD_MASK);
            if (!fileData.open((char*)message, O_WRITE)) {
              gc_println(PSTR("error:loop:conf:r: open file data!"));
            } else {
              if (!fileData.remove()) {
                gc_println(PSTR("error:loop:conf:r: remove file data!"));
              } else {
                gc_println(PSTR("Remove file with cmd!"));
              }
            }
            setPowerDown(SD_MASK);
            gc_print((char*)message);
          }
        } break;
        case 's': {
          if (lcmd > 0) {
            cmd = Serial.read();

            switch (cmd) {
              case 'o': {
                if (lcmd > 1) {
                  cmd = Serial.read();
                  gc_cfg.set_gain(cmd);
                  setPGA(cmd);

                  write_cfg = true;
                }
              } break;
              case 'g': {
                uint8_t settings[21] = { 
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
                  (uint8_t)gc_cfg.gps,
                };
                gc_cmd(settings, 21);
              } break;
              case 'p': {
                gc_println(PSTR("Settings:"));
                gc_cfg.print();
                digitalClockDisplay();
                Serial.print(F("Battery: "));
                Serial.print(getVBat());
                Serial.println(F("V"));
                Serial.print(F("Temp: "));
                Serial.print(getTemp());
                Serial.println(F("C"));
              } break;
              case 'm': {
                if (lcmd > 1) {
                  cmd = Serial.read();
                  gc_cfg.set_average(cmd);
                  write_cfg = true;
                }
              } break;
              case 's': {
                if (lcmd > 4) {
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
                if (lcmd > 1) {
                  cmd = Serial.read();
                  gc_cfg.set_time_type(cmd);
                  write_cfg = true;
                }
              } break;
              case 'u': {
                if (lcmd > 4) {
                  uint32_t time_begin_seg;
                  ((uint8_t*)&time_begin_seg)[0] = Serial.read();
                  ((uint8_t*)&time_begin_seg)[1] = Serial.read();
                  ((uint8_t*)&time_begin_seg)[2] = Serial.read();
                  ((uint8_t*)&time_begin_seg)[3] = Serial.read();
                  gc_cfg.set_time_begin(time_begin_seg % SEG_A_DAY);
                  write_cfg = true;
                }
              } break;
              case 'v': {
                if (lcmd > 4) {
                  uint32_t time_end_seg;
                  ((uint8_t*)&time_end_seg)[0] = Serial.read();
                  ((uint8_t*)&time_end_seg)[1] = Serial.read();
                  ((uint8_t*)&time_end_seg)[2] = Serial.read();
                  ((uint8_t*)&time_end_seg)[3] = Serial.read();
                  gc_cfg.set_time_end(time_end_seg % SEG_A_DAY);
                  write_cfg = true;
                }
              } break;
              case 'z': {
                uint32_t z = Teensy3Clock.get();
                uint32_t u = (z+60*10) % SEG_A_DAY;
                uint32_t v = (z+60*20) % SEG_A_DAY;
                gc_cfg.set_time_type(1);
                gc_cfg.set_time_begin(u);
                gc_cfg.set_time_end(v);
                write_cfg = true;
              } break;
              case 'r': {
                time_t pctime = (time_t)(Serial.parseInt());
                if (pctime != 0) {
                  Teensy3Clock.set(pctime); // set the RTC
                  setTime(pctime);
                }
              } break;
              case 'w': {
                if (lcmd > 1) {
                  cmd = (uint8_t)Serial.read();
                  if ((cmd & POWER_UP_MASK) > 0) setPowerUp(cmd);
                  else setPowerDown(cmd);
                }
              } break;
              case 'y': {
                syncGps();
              } break;
              case 'x': {
                if (lcmd > 1) {
                  cmd = Serial.read();
                  gc_cfg.set_gps(cmd);
                  write_cfg = true;
                }
              } break;
              case 'q': {
                float vbat = getVBat();
                gc_println(PSTR("Battery:"));
                Serial.print(vbat);
                Serial.println(F("V"));
              } break;
              default: {
                gc_println(PSTR("bad set!"));
              } break;
            }
          } else {
            gc_println(PSTR("empty set!"));
          }
        } break;
        case 'i': {
          if (lcmd > 0) {
            cmd = Serial.read();

            switch (cmd) {
              case 't': {
                gc_print(PSTR("HID:"));
                Serial.print(SIM_UIDH);
                Serial.print(F("."));
                Serial.print(SIM_UIDMH);
                Serial.print(F("."));
                Serial.print(SIM_UIDML);
                Serial.print(F("."));
                Serial.println(SIM_UIDL);
              } break;
              case 'p': {
                uint8_t ip[17] = { 
                  'i',
                  ((uint8_t*)&SIM_UIDH)[0],
                  ((uint8_t*)&SIM_UIDH)[1],
                  ((uint8_t*)&SIM_UIDH)[2],
                  ((uint8_t*)&SIM_UIDH)[3],
                  ((uint8_t*)&SIM_UIDMH)[0],
                  ((uint8_t*)&SIM_UIDMH)[1],
                  ((uint8_t*)&SIM_UIDMH)[2],
                  ((uint8_t*)&SIM_UIDMH)[3],
                  ((uint8_t*)&SIM_UIDML)[0],
                  ((uint8_t*)&SIM_UIDML)[1],
                  ((uint8_t*)&SIM_UIDML)[2],
                  ((uint8_t*)&SIM_UIDML)[3],
                  ((uint8_t*)&SIM_UIDL)[0],
                  ((uint8_t*)&SIM_UIDL)[1],
                  ((uint8_t*)&SIM_UIDL)[2],
                  ((uint8_t*)&SIM_UIDL)[3],
                };
                gc_cmd(ip, 17);
              } break;
              default: {
              } break;
            }
          }
        } break;
        case 'x': {
          if (xbeeBD == 0) break;

          setPowerUp(XBEE_MASK);
          while (true) {
            while (Serial.available()) {
              cmd = Serial.read();
              if (cmd == 'x') goto xout;
              XBEE_IO.write(cmd);
            }
            while (XBEE_IO.available()) {
              cmd = XBEE_IO.read();
              Serial.write(cmd);
            }
          }
xout:
          setPowerDown(XBEE_MASK);
        } break;
        default: {
          gc_println(PSTR("bad cmd!"));
        } break;
      }

      if (write_cfg) {
        setPowerUp(SD_MASK);
        gc_cfg.write();
        setPowerDown(SD_MASK);

        // configure adc
        cfgADC();

        // configure DMA
        cfgDMA();

        gc_cfg.print();
        digitalClockDisplay();
      }
    }
  }
}
