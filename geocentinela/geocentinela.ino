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
    gcPlayStat |= GC_ST_PLAY;

    gcPlayStat &= ~GC_ST_CONFIG;
    setPowerDown(SD_MASK);
  }
  interrupts();
}

void stop_reading()
{
  noInterrupts();
  gcPlayStat &= ~GC_ST_READING;
  gcPlayStat |= GC_ST_STOP;

  if (HIGH == digitalRead(PIN_USB)) {
    attachInterrupt(PIN_USB, start_reading, FALLING);

    gcPlayStat |= GC_ST_CONFIG;
    setPowerUp(SD_MASK);
  } else gcPlayStat &= ~GC_ST_CONFIG;
  interrupts();
}

void setup()
{
  // pines no utilizados:
  pinMode(24, INPUT); //digitalWrite(24, LOW);
  pinMode(25, INPUT); //digitalWrite(25, LOW);
  pinMode(26, INPUT); //digitalWrite(26, LOW);
  pinMode(27, INPUT); //digitalWrite(27, LOW);
  pinMode(28, INPUT); //digitalWrite(28, LOW);
  pinMode(29, INPUT); //digitalWrite(29, LOW);
  //pinMode(30, INPUT); //digitalWrite(30, LOW); PIN_USB
  pinMode(31, INPUT); //digitalWrite(31, LOW);
  pinMode(32, INPUT); //digitalWrite(32, LOW);
  pinMode(33, INPUT); //digitalWrite(33, LOW);

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
    setPowerUp(SD_MASK);
  } else {
    start_reading();
  }
}

void loop()
{
  while (LOW == digitalRead(PIN_USB)) {
    if (gcPlayStat & GC_ST_PLAY) {
      gcPlayStat &= ~GC_ST_PLAY;

      adc_play_cnt = 0;
      switch (gc_cfg.time_type) {
        case 0: {
          adc_play_cnt = 1+sleep_chrono();
        } break;
        case 1: {
          adc_play_cnt = 1+sleep_daily();
        } break;
      }

      if (adc_play_cnt == 1) {
        stop_reading();
        gcPlayStat &= ~GC_ST_STOP;
      } else if (!gcStart()) {
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

        if (quake_head == QUAKE_LIST_LENGTH) {
          qfile.write(quake_list, QUAKE_LIST_LENGTH_BYTES);
          quake_head = 0;
        }
      }

      if (file.getWriteError()) {
        stop_reading();
      }
    }

    if (gcPlayStat & GC_ST_STOP) {
      if ((gcPlayStat & GC_ST_CONFIG) == 0) {
        gcStop();
        switch (gc_cfg.time_type) {
          case 0: {
            detachInterrupt(PIN_USB);
            deep_sleep(); // wake only if PIN_USB are HIGH:
            attachInterrupt(PIN_USB, start_reading, FALLING);
            gcPlayStat |= GC_ST_CONFIG;
            setPowerUp(SD_MASK);
          } break;
          case 1: {
            gcPlayStat |= GC_ST_PLAY;
          } break;
        }
      } else {
        gcStop();
        setPowerUp(SD_MASK);
      }
    }
  }

  uint8_t cmd;
  uint32_t lcmd;
  while (HIGH == digitalRead(PIN_USB)) {
    cmd = 0;
    lcmd = (uint32_t)Serial.available();

    if (lcmd > 0) {
      cmd = Serial.read();
    }

    if (lcmd > 0) {
      uint8_t* message = (uint8_t*)sd_buffer;

      boolean write_cfg = false;
      boolean reload_adc_dma = false;

      switch (cmd) {
        case 'd': {
          gc_print(PSTR("Date: "));
          digitalClockDisplay();
          gc_cmd('r');
        } break;
        case 'b': {
          float vbat = getVBat();
          gc_print(PSTR("Battery[V]: "));
          Serial.println(vbat);
          gc_cmd('r');
        } break;
        case 't': {
          float temp = getTemp();
          gc_print(PSTR("Temp[C]: "));
          Serial.println(temp);
          gc_cmd('r');
        } break;
        case 'l': { // ls command
          uint32_t powMaskOld = powMask;
          setPowerUp(SD_MASK);
          gc_cmd('l');
          sd.ls(LS_DATE | LS_SIZE);
          gc_cmd('r');
          if ((powMaskOld & SD_MASK) == 0) setPowerDown(SD_MASK);
        } break;
        case 'g': { // get a file
          cmd = 0;
          uint8_t* filename = (uint8_t*)adc_ring_buffer;
          if (lcmd > 1) {
            for (; cmd < FILENAME_MAX_LENGH && cmd < (lcmd-1); cmd++)
              filename[cmd] = Serial.read();
          }
          filename[cmd] = '\0';

          if (cmd > 0) {
            uint32_t powMaskOld = powMask;
            setPowerUp(SD_MASK);
            SdFile fileData;
            if (!fileData.open((char*)filename, O_READ)) {
              gc_print(PSTR("error:loop:g: "));
              Serial.println((char*)filename);
              gc_cmd('r');
            } else {
              uint8_t head[2] = { 'f', cmd };
              uint8_t eof_cmd[2] = { 'g', cmd };

              gc_cmd('r');
              gc_cmd(head, 2);
              Serial.write(filename, cmd);

              uint16_t n;
              while ((n = fileData.read(message, GC_SD_BUFFER_SIZE_BYTES)) > 0)
                Serial.write(message, n);
              gc_cmd(eof_cmd, 2);
              gc_cmd('r');
            }
            if ((powMaskOld & SD_MASK) == 0) setPowerDown(SD_MASK);
          } else {
            gc_println(PSTR("error:loop:g: empty filename!"));
            gc_cmd('r');
          }
        } break;
        case 'r': { // remove file
          cmd = 0;
          if (lcmd > 1) {
            for (; cmd < FILENAME_MAX_LENGH && cmd < (lcmd-1); cmd++)
              message[cmd] = Serial.read();
          }
          message[cmd] = '\0';
          Serial.println((char*)message);

          if (cmd > 0) {
            uint32_t powMaskOld = powMask;
            setPowerUp(SD_MASK);
            if (!sd.remove((char*)message)) {
              gc_println(PSTR("error:loop:r: remove file data!"));
              gc_cmd('r');
            }
            if ((powMaskOld & SD_MASK) == 0) setPowerDown(SD_MASK);
            gc_print(PSTR("rm "));
            Serial.println((char*)message);
            gc_cmd('r');
          } else {
            gc_println(PSTR("error:loop:r: empty filename!"));
            gc_cmd('r');
          }
        } break;
        case 's': {
          if (lcmd >= 2) {
            cmd = Serial.read();

            switch (cmd) {
              case 'f': {
                if (lcmd >= 6) {
                  float sensitivity;
                  ((uint8_t*)&sensitivity)[0] = Serial.read();
                  ((uint8_t*)&sensitivity)[1] = Serial.read();
                  ((uint8_t*)&sensitivity)[2] = Serial.read();
                  ((uint8_t*)&sensitivity)[3] = Serial.read();
                  Serial.print("s:");
                  Serial.println(sensitivity);
                  gc_cfg.set_sensitivity(sensitivity);

                  write_cfg = true;
                }
              } break;
              case 'j': {
                if (lcmd >= 3) {
                  uint32_t tj = (uint32_t)Serial.parseInt();
                  gc_cfg.set_time_begin(tj % SEG_A_DAY);

                  write_cfg = true;
                }
              } break;
              case 'k': {
                if (lcmd >= 3) {
                  uint32_t tk = (uint32_t)Serial.parseInt() % SEG_A_DAY;
                  gc_cfg.set_time_end(tk % SEG_A_DAY);

                  write_cfg = true;
                }
              } break;
              case 'l': {
                if (lcmd >= 3) {
                  uint16_t trigger_level = (uint16_t)Serial.parseInt();
                  gc_cfg.set_trigger_level(trigger_level);

                  write_cfg = true;

#if TRIGGER_BY_SOFTWARE == 0
                  reload_adc_dma = true;
#endif
                }
              } break;
              case 'd': {
                if (lcmd >= 3) {
                  uint32_t trigger_time_seg = (uint32_t)Serial.parseInt();
                  gc_cfg.set_trigger_time_seg(trigger_time_seg);

                  write_cfg = true;

#if TRIGGER_BY_SOFTWARE == 0
                  reload_adc_dma = true;
#endif
                }
              } break;
              case 'o': {
                if (lcmd >= 3) {
                  cmd = Serial.read();
                  setPGA(cmd);

                  write_cfg = true;
                }
              } break;
              case 'g': {
                uint8_t settings[26] = { 's',
                  ((uint8_t*)&gc_cfg.sensitivity)[0],
                  ((uint8_t*)&gc_cfg.sensitivity)[1],
                  ((uint8_t*)&gc_cfg.sensitivity)[2],
                  ((uint8_t*)&gc_cfg.sensitivity)[3],
                  (uint8_t)((gc_cfg.gain << 4) | gc_cfg.average),
                  ((uint8_t*)&gc_cfg.tick_time_useg)[0],
                  ((uint8_t*)&gc_cfg.tick_time_useg)[1],
                  ((uint8_t*)&gc_cfg.tick_time_useg)[2],
                  ((uint8_t*)&gc_cfg.tick_time_useg)[3],
                  (uint8_t)gc_cfg.time_type,
                  ((uint8_t*)&gc_cfg.time_begin_seg)[0],
                  ((uint8_t*)&gc_cfg.time_begin_seg)[1],
                  ((uint8_t*)&gc_cfg.time_begin_seg)[2],
                  ((uint8_t*)&gc_cfg.time_begin_seg)[3],
                  ((uint8_t*)&gc_cfg.time_end_seg)[0],
                  ((uint8_t*)&gc_cfg.time_end_seg)[1],
                  ((uint8_t*)&gc_cfg.time_end_seg)[2],
                  ((uint8_t*)&gc_cfg.time_end_seg)[3],
                  (uint8_t)gc_cfg.gps,
                  ((uint8_t*)&gc_cfg.trigger_level)[0],
                  ((uint8_t*)&gc_cfg.trigger_level)[1],
                  ((uint8_t*)&gc_cfg.trigger_time_number)[0],
                  ((uint8_t*)&gc_cfg.trigger_time_number)[1],
                  ((uint8_t*)&gc_cfg.trigger_time_number)[2],
                  ((uint8_t*)&gc_cfg.trigger_time_number)[3],
                };
                gc_cmd(settings, 26);
              } break;
              case 'p': {
                float vbat = getVBat();
                float temp = getTemp();

                gc_println(PSTR("Settings:"));
                gc_cfg.print();
                digitalClockDisplay();
                Serial.print(F("Battery: "));
                Serial.print(vbat);
                Serial.println(F("V"));
                Serial.print(F("Temp: "));
                Serial.print(temp);
                Serial.println(F("C"));
              } break;
              case 'm': {
                if (lcmd >= 3) {
                  cmd = Serial.read();
                  gc_cfg.set_average(cmd);

                  write_cfg = true;
                  reload_adc_dma = true;
                }
              } break;
              case 's': {
                if (lcmd >= 6) {
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
                if (lcmd >= 3) {
                  cmd = Serial.read();
                  gc_cfg.set_time_type(cmd);

                  write_cfg = true;
                }
              } break;
              case 'u': {
                if (lcmd >= 6) {
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
                if (lcmd >= 6) {
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
                if (lcmd >= 3) {
                  time_t pctime = (time_t)(Serial.parseInt());
                  if (pctime != 0) {
                    Teensy3Clock.set(pctime); // set the RTC
                    setTime(pctime);
                  }
                }
              } break;
              case 'w': {
                if (lcmd >= 3) {
                  cmd = (uint8_t)Serial.read();
                  if ((cmd & POWER_UP_MASK) > 0) setPowerUp(cmd);
                  else setPowerDown(cmd);
                }
              } break;
              case 'y': {
                syncGps();
              } break;
              case 'x': {
                if (lcmd >= 3) {
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
          if (lcmd >= 2) {
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
        case 'p': {
          Serial.println(powMask, HEX);
        } break;
        case 'x': {
          if (xbeeBD == 0) break;

          uint32_t powMaskOld = powMask;
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
          if ((powMaskOld & XBEE_MASK) == 0) setPowerDown(XBEE_MASK);
        } break;
        default: {
          gc_println(PSTR("bad cmd!"));
        } break;
      }

      if (write_cfg) {
        uint32_t powMaskOld = powMask;
        setPowerUp(SD_MASK);
        gc_cfg.write();
        if ((powMaskOld & SD_MASK) == 0) setPowerDown(SD_MASK);
      }

      if (reload_adc_dma) {
        // configure adc
        cfgADC();

        // configure DMA
        cfgDMA();
      }
    }
  }
}
