//*
#include <IntervalTimer.h>

#include <TinyGPS.h>
#include <TimeLib.h>

#include <SdFat.h>

  #include <EEPROM.h>
  #include <ArduinoJson.h>
  #include "gcCFG.h"

  #include <LowPower_Teensy3.h>
#include "gcAll.h"

#include "dcodec.h"

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
  gcPlayStat &= ~GC_ST_READING;
  gcPlayStat |= GC_ST_STOP;

  if (HIGH == digitalRead(PIN_USB)) {
    attachInterrupt(PIN_USB, start_reading, FALLING);
    gcPlayStat |= GC_ST_CONFIG;
  }
  interrupts();
}

void setup()
{
  Serial.begin(9600);
  uint32_t t_ini = millis();
  while (!Serial and millis() <= (t_ini + 2000));

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

  // USB wakeup
  pinMode(PIN_USB, INPUT_PULLDOWN);
  detachInterrupt(PIN_USB);

  delay(100);
  if (HIGH == digitalRead(PIN_USB)) {
    stop_reading();
  } else {
    start_reading();
  }

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

void loop()
{
  while (LOW == digitalRead(PIN_USB)) {
    if (gcPlayStat & GC_ST_PLAY) {
      gcPlayStat &= ~GC_ST_PLAY;

      setPowerDown(SD_MASK);

      adc_play_cnt = 0;

      switch (geocentinela.c.getTimeType()) {
        case 0: {
          adc_play_cnt = 1+sleep_chrono();
        } break;
        case 1:
        case 2: {
          adc_play_cnt = 1+sleep_daily();
        } break;
      }

      if (adc_play_cnt <= 1 or !gcStart()) {
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
      if (!(gcPlayStat & GC_ST_CONFIG)) {
        switch (geocentinela.c.getTimeType()) {
          case 0: {
            detachInterrupt(PIN_USB);
            deep_sleep(); // wake only if PIN_USB are HIGH:
            stop_reading();
          } break;
          case 1:
          case 2: {
            start_reading();
          } break;
        }
      }
      gcPlayStat &= ~GC_ST_STOP;
    }
  }

  uint8_t cmd;
  uint32_t lcmd;
  while (HIGH == digitalRead(PIN_USB)) {
    if (gcPlayStat & GC_ST_CONFIG) {
      gcPlayStat = ~GC_ST_CONFIG;
    }

    lcmd = (uint32_t) Serial.available();
    if (lcmd > 0) {
      cmd = Serial.read();

      switch (cmd) {
        case 'd': {
          geocentinela.print(PSTR("Date: "));
          digitalClockDisplay();
        } break;
        case 'b': {
          float vbat = getVBat();
          geocentinela.print(PSTR("Battery[V]: "));
          Serial.println(vbat);
        } break;
        case 't': {
          float temp = getTemp();
          geocentinela.print(PSTR("Temp[C]: "));
          Serial.println(temp);
        } break;
        case 'l': { // ls command
          uint32_t pMask = powMask;
          setPowerUp(SD_MASK);
          if (!(powMask & SD_MASK)) break;

          geocentinela.gcCmd('l');
          sd.ls(LS_DATE | LS_SIZE);

          setPowerDown(~pMask);
          setPowerUp(pMask);
        } break;
        case 'g': { // get a file
          cmd = 0;
          uint8_t* filename = (uint8_t*) adc_ring_buffer;
          if (lcmd > 1) {
            for (; cmd < FILENAME_MAX_LENGH and cmd < (lcmd-1); cmd++)
              filename[cmd] = Serial.read();
          }
          filename[cmd] = '\0';

          if (cmd > 0) {
            uint32_t pMask = powMask;
            setPowerUp(SD_MASK);
            if (!(powMask & SD_MASK)) break;

            SdFile fileData;
            if (!fileData.open((char*) filename, O_READ)) {
              geocentinela.print(PSTR("error:loop:g: "));
              Serial.println((char*) filename);
            } else {
              uint8_t head[2] = { 'f', cmd };
              geocentinela.gcCmd(head, 2);

              Serial.print((char*) filename);

              uint16_t n;
              while ((n = fileData.read(m95.getStringIO(), m95.getStringIOLength())) > 0)
                Serial.write(m95.getStringIO(), n);

              uint8_t eof_cmd[2] = { 'g', cmd };
              geocentinela.gcCmd(eof_cmd, 2);
            }

            setPowerDown(~pMask);
            setPowerUp(pMask);
          } else {
            geocentinela.println(PSTR("error:loop:g: empty filename!"));
          }
        } break;
        case 'r': { // remove file
          cmd = 0;
          uint8_t* filename = (uint8_t*) adc_ring_buffer;
          if (lcmd > 1) {
            for (; cmd < FILENAME_MAX_LENGH and cmd < (lcmd-1); cmd++)
              filename[cmd] = Serial.read();
          }
          filename[cmd] = '\0';
          Serial.println((char*) filename);

          if (cmd > 0) {
            uint32_t pMask = powMask;
            setPowerUp(SD_MASK);
            if (!(powMask & SD_MASK)) break;

            if (!sd.remove((char*) filename)) {
              geocentinela.println(PSTR("error:loop:r: remove file data!"));
            }
            setPowerDown(~pMask);
            setPowerUp(pMask);

            geocentinela.print(PSTR("rm "));
            Serial.println((char*) filename);
          } else {
            geocentinela.println(PSTR("error:loop:r: empty filename!"));
          }
        } break;
        case 's': {
          if (lcmd >= 2) {
            cmd = Serial.read();

            switch (cmd) {
              case 'o': {
                if (lcmd >= 3) {
                  cmd = Serial.read();
                  setPGA(cmd);

                  geocentinela.write();
                }
              } break;
              case 'r': {
                if (lcmd >= 3) {
                  time_t pctime = (time_t) Serial.parseInt();
                  if (pctime != 0) {
                    Teensy3Clock.set(pctime); // set the RTC in UTC
                    setSyncProvider(getTeensy3Time);
                  }
                }
              } break;
              case 'w': {
                if (lcmd >= 3) {
                  cmd = (uint8_t) Serial.read();
                  if ((cmd & POWER_UP_MASK) > 0) {
                    setPowerUp(cmd);
                  } else {
                    setPowerDown(cmd);
                  }
                }
              } break;
              case 'y': {
                // syncGps();
              } break;
              case 'x': {
                if (lcmd >= 3) {
                  cmd = Serial.read();
                  geocentinela.setGps(cmd);

                  geocentinela.write();
                }
              } break;
              default: {
                geocentinela.println(PSTR("bad set!"));
              } break;
            }
          } else {
            geocentinela.println(PSTR("empty set!"));
          }
        } break;
        case 'j': { // json
          if (lcmd >= 3) {
            cmd = Serial.read();

            rMessage mlog;
            mlog.setInstrumentId(geocentinela.getId());
            mlog.setDateNow();
            mlog.setFrom("error");
            boolean send_mlog = false;

            switch (cmd) {
              case 's': {
                cmd = Serial.read();

                Serial.setTimeout(30);
                uint16_t index = Serial.readBytes(m95.getStringIO(), m95.getStringIOLength());
                if (index < m95.getStringIOLength()) m95.getStringIO()[index] = 0;
                else m95.getStringIO()[m95.getStringIOLength() - 1] = 0;

                switch (cmd) {
                  case 'r': {
                    setup();
                  } break;
                  case 'i': {
                    if (geocentinela.deserialize(m95.getStringIO(), (uint8_t*)adc_ring_buffer, GC_ADC_RING_SIZE_BYTES, false)) {
                      geocentinela.touch();
                      geocentinela.write();

                      mlog.setFrom("ok");
                      mlog.setSMS("jsi");
                      send_mlog = true;
                    }
                  } break;
                  case 'c': {
                    if (geocentinela.c.deserialize(m95.getStringIO(), (uint8_t*)adc_ring_buffer, GC_ADC_RING_SIZE_BYTES, false)) {
                      geocentinela.c.touch();
                      geocentinela.write();

                      mlog.setFrom("ok");
                      mlog.setSMS("jsc");
                      send_mlog = true;
                    }
                  } break;
                  case 's': {
                    if (geocentinela.s.deserialize(m95.getStringIO(), (uint8_t*)adc_ring_buffer, GC_ADC_RING_SIZE_BYTES, false)) {
                      geocentinela.s.touch();
                      geocentinela.write();

                      mlog.setFrom("ok");
                      mlog.setSMS("jss");
                      send_mlog = true;
                    }
                  } break;
                  case 'f': {
                    rFatFile flog;
                    if (flog.deserialize(m95.getStringIO(), (uint8_t*)adc_ring_buffer, GC_ADC_RING_SIZE_BYTES, false)) {
                      if (flog.getInstrumentId() != geocentinela.getId()) {
                        mlog.setSMS(PSTR("jsf: bad file instrument!"));
                        send_mlog = true;
                        break;
                      }

                      switch (flog.getAction()) {
                        case 1: { // remove
                          uint32_t pMask = powMask;
                          setPowerUp(SD_MASK);

                          if (!sd.exists(flog.getName())) {
                            mlog.setSMS(PSTR("jsf: try to remove a non existent file!"));
                          } else if (!sd.remove(flog.getName()) && !sd.rmdir(flog.getName())) {
                            mlog.setSMS(PSTR("jsf: unable to delete file or folder!"));
                          } else {
                            mlog.setFrom("ok");

                            char msg[SMS_SIZE];
                            snprintf(msg, SMS_SIZE, "jsf: rm %s", flog.getName());
                            mlog.setSMS(msg);
                          }
                          send_mlog = true;

                          setPowerDown(~pMask);
                          setPowerUp(pMask);
                        } break;
                        case 2: { // request
                          uint32_t pMask = powMask;
                          setPowerUp(SD_MASK);

                          if (sd.exists(flog.getName())) {
                            SdFile fileData;
                            if (fileData.open(flog.getName(), O_READ)) {
                              flog.setAction(3); // send

                              m95.cleanStringIO();
                              flog.serialize(m95.getStringIO(), m95.getStringIOLength(),
                                (uint8_t*)adc_ring_buffer, GC_ADC_RING_SIZE_BYTES, false);

                              geocentinela.printCmd('j', m95.getStringIO());
                              geocentinela.gcCmd('f'); // bof

                              uint16_t n;
                              while ((n = fileData.read(m95.getStringIO(), m95.getStringIOLength())) > 0)
                                Serial.write(m95.getStringIO(), n);

                              geocentinela.gcCmd('g'); // eof

                              mlog.setFrom("ok");
                              char msg[SMS_SIZE];
                              snprintf(msg, SMS_SIZE, "jsf: get %s", flog.getName());
                              mlog.setSMS(msg);
                            } else {
                              mlog.setSMS(PSTR("jsf: open file fail!"));
                            }
                          } else {
                            mlog.setSMS(PSTR("jsf: try to request a non existent file!"));
                          }
                          send_mlog = true;

                          setPowerDown(~pMask);
                          setPowerUp(pMask);
                        } break;
                        default: {
                          mlog.setSMS(PSTR("jsf: bad file action!"));
                          send_mlog = true;
                        } break;
                      }
                    }
                  } break;
                  default: {
                    mlog.setSMS(PSTR("js?: bad json set!"));
                    send_mlog = true;
                  } break;
                }
              } break;
              case 'g': {
                cmd = Serial.read();

                switch (cmd) {
                  case 'i': {
                    // jsonificando en el buffer
                    m95.cleanStringIO();
                    geocentinela.serialize(m95.getStringIO(), m95.getStringIOLength(), (uint8_t*)adc_ring_buffer, GC_ADC_RING_SIZE_BYTES, false);
                    geocentinela.printCmd('j', m95.getStringIO());
                  } break;
                  case 'c': {
                    // jsonificando en el buffer
                    m95.cleanStringIO();
                    geocentinela.c.serialize(m95.getStringIO(), m95.getStringIOLength(), (uint8_t*)adc_ring_buffer, GC_ADC_RING_SIZE_BYTES, false);
                    geocentinela.printCmd('j', m95.getStringIO());
                  } break;
                  case 's': {
                    // jsonificando en el buffer
                    m95.cleanStringIO();
                    geocentinela.s.serialize(m95.getStringIO(), m95.getStringIOLength(), (uint8_t*)adc_ring_buffer, GC_ADC_RING_SIZE_BYTES, false);
                    geocentinela.printCmd('j', m95.getStringIO());
                  } break;
                  case 'l': {
                    uint32_t pMask = powMask;
                    setPowerUp(SD_MASK);
                    if (!(powMask & SD_MASK)) break;

                    rFatFile flog;
                    flog.setInstrumentId(geocentinela.getId());

                    boolean empty_dir = true;
                    sd.vwd()->rewind();
                    while (file.openNext(sd.vwd(), O_READ)) {
                      flog.setFile(file);

                      m95.cleanStringIO();
                      flog.serialize(m95.getStringIO(), m95.getStringIOLength(), (uint8_t*)adc_ring_buffer, GC_ADC_RING_SIZE_BYTES, false);
                      geocentinela.printCmd('j', m95.getStringIO());

                      file.close();
                      empty_dir = false;
                    }

                    if (empty_dir) {
                      flog.setName("");

                      m95.cleanStringIO();
                      flog.serialize(m95.getStringIO(), m95.getStringIOLength(), (uint8_t*)adc_ring_buffer, GC_ADC_RING_SIZE_BYTES, false);
                      geocentinela.printCmd('j', m95.getStringIO());                      
                    }

                    setPowerDown(~pMask);
                    setPowerUp(pMask);
                  } break;
                  default: {
                    mlog.setSMS(PSTR("jg?: bad json get!"));
                    send_mlog = true;
                  }
                }
              } break;
              case 'r': {
                if (!Serial.available()) {
                  mlog.setSMS(PSTR("jr?: bad json command message!"));

                  send_mlog = true;
                  break;
                }
                cmd = Serial.read();

                Serial.setTimeout(30);
                uint16_t index = Serial.readBytes(m95.getStringIO(), m95.getStringIOLength());
                if (index < m95.getStringIOLength()) m95.getStringIO()[index] = 0;
                else m95.getStringIO()[m95.getStringIOLength() - 1] = 0;

                rMessage msg;
                if (msg.deserialize(m95.getStringIO(), (uint8_t*)adc_ring_buffer, GC_ADC_RING_SIZE_BYTES, false)) {

                  uint16_t sms_index = 0;

                  uint32_t pMask = powMask;
                  switch (cmd) {
                    case 'e': { // echo
                      mlog.setFrom(PSTR("GeoCentinela"));

                      sms_index = 0;
                      sms_index += snprintf(m95.getStringLog() + sms_index, (m95.getStringLogLength() - sms_index) - 1,
                        PSTR("%s"), msg.getSMS());

                      m95.getStringLog()[sms_index] = 0;
                      mlog.setSMS(m95.getStringLog());
                      send_mlog = true;
                    } break;
                    case 'c': { // Call Phone
                      geocentinela.println(PSTR("Power on GPRS..."));
                      setPowerUp(M95_MASK);
                      if (!(powMask & M95_MASK)) {
                        mlog.setSMS(PSTR("jrc: Can not power on GPRS!"));
                        send_mlog = true;
                        break;
                      }

                      mlog.setFrom(msg.getFrom());

                      uint8_t resp_call = m95.call(msg.getSMS());
                      uint8_t resp_csq = m95.csq(10);

                      sms_index = 0;
                      switch (resp_call) {
                        case 1: {
                          sms_index += snprintf(m95.getStringLog() + sms_index, (m95.getStringLogLength() - sms_index) - 1,
                            PSTR("Error: NO OK\n  RSSI:%u\n  RXQUAL:%u"), resp_csq & 0x1F, resp_csq >> 5);
                        } break;
                        case 2: {
                          sms_index += snprintf(m95.getStringLog() + sms_index, (m95.getStringLogLength() - sms_index) - 1,
                            PSTR("Warning: WRONG NUMBER\n  RSSI:%u\n  RXQUAL:%u"), resp_csq & 0x1F, resp_csq >> 5);
                        } break;
                        case 3: {
                          sms_index += snprintf(m95.getStringLog() + sms_index, (m95.getStringLogLength() - sms_index) - 1,
                            PSTR("Ok: BUSY\n  RSSI:%u\n  RXQUAL:%u"), resp_csq & 0x1F, resp_csq >> 5);
                        } break;
                        case 4: {
                          sms_index += snprintf(m95.getStringLog() + sms_index, (m95.getStringLogLength() - sms_index) - 1,
                            PSTR("Ok: NO ANSWER\n  RSSI:%u\n  RXQUAL:%u"), resp_csq & 0x1F, resp_csq >> 5);
                        } break;
                        case 5: {
                          sms_index += snprintf(m95.getStringLog() + sms_index, (m95.getStringLogLength() - sms_index) - 1,
                            PSTR("Warning: NO CARRIER\n  RSSI:%u\n  RXQUAL:%u"), resp_csq & 0x1F, resp_csq >> 5);
                        } break;
                        case 6: {
                          sms_index += snprintf(m95.getStringLog() + sms_index, (m95.getStringLogLength() - sms_index) - 1,
                            PSTR("Error: NO DIALTONE\n  RSSI:%u\n  RXQUAL:%u"), resp_csq & 0x1F, resp_csq >> 5);
                        } break;
                        default: {
                          sms_index += snprintf(m95.getStringLog() + sms_index, (m95.getStringLogLength() - sms_index) - 1,
                            PSTR("Ok: CALL\n  RSSI:%u\n  RXQUAL:%u"), resp_csq & 0x1F, resp_csq >> 5);
                        }
                      }

                      m95.getStringLog()[sms_index] = 0;
                      mlog.setSMS(m95.getStringLog());
                      send_mlog = true;
                    } break;
                    case 'h': { // Ping HostName
                      geocentinela.println(PSTR("Power on GPRS..."));
                      setPowerUp(M95_MASK);
                      if (!(powMask & M95_MASK)) {
                        mlog.setSMS(PSTR("jrh: Can not power on GPRS!"));
                        send_mlog = true;
                        break;
                      }

                      mlog.setFrom(msg.getFrom());

                      m95.cleanStringIO();
                      uint16_t stateHTTP = m95.httpREST(msg.getSMS(), PSTR("GET /"), PSTR("text/html; charset=utf-8"));

                      // send header
                      sms_index = 0;
                      if ((stateHTTP >> 1) > 0) {
                        char* subStrResponse = strstr(m95.getStringLog(), "HTTP");
                        subStrResponse = strstr(subStrResponse, "\n") + 1;
                        sms_index = subStrResponse - m95.getStringLog();
                      } else {
                        sms_index += snprintf(m95.getStringLog() + sms_index, (m95.getStringLogLength() - sms_index) - 1,
                          PSTR("Server Not Found\n"));
                      }
                      m95.getStringLog()[sms_index] = 0;
                      mlog.setSMS(m95.getStringLog());

                      uint8_t resp_csq = m95.csq(10);

                      sms_index = 0;
                      sms_index += snprintf(m95.getStringLog() + sms_index, (m95.getStringLogLength() - sms_index) - 1,
                        PSTR("%s\n  RSSI:%u\n  RXQUAL:%u"), mlog.getSMS(), resp_csq & 0x1F, resp_csq >> 5);

                      m95.getStringLog()[sms_index] = 0;
                      mlog.setSMS(m95.getStringLog());
                      send_mlog = true;
                    } break;
                    case 's': {
                      geocentinela.println(PSTR("Power on GPRS..."));
                      setPowerUp(M95_MASK);
                      if (!(powMask & M95_MASK)) {
                        mlog.setSMS(PSTR("jrs: Can not power on GPRS!"));
                        send_mlog = true;
                        break;
                      }

                      mlog.setFrom(msg.getFrom());

                      boolean updated_cfg = false;

                      if (syncInstrument(updated_cfg)) {
                        if (updated_cfg) {
                          mlog.setSMS(PSTR("Sync Successful and Updated"));
                        } else {
                          mlog.setSMS(PSTR("Sync Successful"));
                        }
                      } else {
                        mlog.setSMS(PSTR("Cannot Sync"));
                      }

                      if (updated_cfg) {
                        geocentinela.write();
                        setup();
                      }

                      send_mlog = true;
                    } break;
                    case 'f': {
                      geocentinela.println(PSTR("Power on GPRS..."));
                      setPowerUp(M95_MASK);
                      if (!(powMask & M95_MASK)) {
                        mlog.setSMS(PSTR("jrf: Can not power on GPRS!"));
                        send_mlog = true;
                        break;
                      }

                      geocentinela.println(PSTR("Power on SD..."));
                      setPowerUp(SD_MASK);
                      if (!(powMask & SD_MASK)) {
                        mlog.setSMS(PSTR("jrf: Can not power on SD!"));
                        send_mlog = true;
                        break;
                      }

                      mlog.setFrom(msg.getFrom());

                      boolean sync_sd_fat_files = true;

                      uint32_t file_pos;
                      file_pos = 0;
                      while (true) {
                        Serial.println("sync file...");

                        sd.vwd()->seekSet(file_pos);
                        if (!file.openNext(sd.vwd(), O_WRITE)) break;
                          file_pos = sd.vwd()->curPosition();

                        if (!syncFatFile(file)) sync_sd_fat_files = false;
                        if (file.isOpen()) file.close();
                      }

                      if (sync_sd_fat_files) mlog.setSMS(PSTR("sdsync ok"));
                      else mlog.setSMS(PSTR("sdsync fail!"));

                      send_mlog = true;
                    } break;
                    case 'm': { // MTP Sync
                      time_t rtctime = (time_t) strtol(msg.getSMS(), NULL, 10);

                      mlog.setFrom(msg.getFrom());

                      if (rtctime != 0) {
                        Teensy3Clock.set(rtctime); // set the RTC in UTC
                        setSyncProvider(getTeensy3Time);

                        m95.cleanStringLog();
                        uint16_t sms_index = 0;

                        sms_index += snprintf(m95.getStringLog() + sms_index, (m95.getStringLogLength() - sms_index) - 1,
                          PSTR("%lu"), Teensy3Clock.get());
  
                        mlog.setSMS(m95.getStringLog());
                      } else {
                        mlog.setSMS(PSTR("jrm: Cannot set with a RTC null!"));
                      }
                      send_mlog = true;
                    } break;
                    case 'n': { // NTP Sync
                      geocentinela.println(PSTR("Power on GPRS..."));
                      setPowerUp(M95_MASK);
                      if (!(powMask & M95_MASK)) {
                        mlog.setSMS(PSTR("jrn: Can not power on GPRS!"));
                        send_mlog = true;
                        break;
                      }

                      int32_t dtsr = 0;
                      int16_t dtpr = 0;
                      double lag = 0;

                      m95.ntpSyncUDP(dtsr, dtpr, lag);
                      setSyncProvider(getTeensy3Time);

                      m95.cleanStringLog();
                      uint16_t sms_index = 0;

                      mlog.setFrom(msg.getFrom());
                      if (lag < 1.5) {
                        sms_index += snprintf(m95.getStringLog() + sms_index, (m95.getStringLogLength() - sms_index) - 1,
                          PSTR("dtSR:%ld\tdtPR:%d\tLAG:%6.3f\noffset:%6.3f"), dtsr, dtpr, lag, dtsr + dtpr/32768.0);
                        mlog.setSMS(m95.getStringLog());
                      } else {
                        mlog.setSMS(PSTR("Network Lag Detected"));
                      }
                      send_mlog = true;
                    } break;
                    case 'C': { // Temperatura C°
                      float temp = getTemp();

                      m95.cleanStringLog();
                      uint16_t sms_index = 0;

                      sms_index += snprintf(m95.getStringLog() + sms_index, (m95.getStringLogLength() - sms_index) - 1, PSTR("%6.3f[C°]"), temp);

                      mlog.setFrom(msg.getFrom());
                      mlog.setSMS(m95.getStringLog());
                      send_mlog = true;
                    } break;
                    case 'v': { // Voltaje
                      float vbat = getVBat();

                      m95.cleanStringLog();
                      uint16_t sms_index = 0;

                      sms_index += snprintf(m95.getStringLog() + sms_index, (m95.getStringLogLength() - sms_index) - 1, PSTR("%6.3f[V]"), vbat);

                      mlog.setFrom(msg.getFrom());
                      mlog.setSMS(m95.getStringLog());
                      send_mlog = true;
                    } break;
                    case 't': { // Tiempo
                      m95.cleanStringLog();
                      uint16_t sms_index = 0;

                      sms_index += snprintf(m95.getStringLog() + sms_index, (m95.getStringLogLength() - sms_index) - 1, PSTR("%lu"), Teensy3Clock.get());

                      mlog.setFrom(msg.getFrom());
                      mlog.setSMS(m95.getStringLog());
                      send_mlog = true;
                    } break;
                    case 'b': { // Benchmark
                      geocentinela.println(PSTR("Power On SD..."));
                      setPowerUp(SD_MASK);
                      if (!(powMask & SD_MASK)) {
                        mlog.setSMS(PSTR("jrb: Can not power on SD!"));
                        send_mlog = true;

                        break;
                      }

                      m95.cleanStringLog();
                      uint16_t sms_index = 0;

                      switch(sd.card()->type()) {
                        case SD_CARD_TYPE_SD1: {
                          sms_index += snprintf(m95.getStringLog() + sms_index, (m95.getStringLogLength() - sms_index) - 1, PSTR("SD1_"));
                        } break;
                        case SD_CARD_TYPE_SD2: {
                          sms_index += snprintf(m95.getStringLog() + sms_index, (m95.getStringLogLength() - sms_index) - 1, PSTR("SD2_"));
                        } break;
                        case SD_CARD_TYPE_SDHC: {
                           sms_index += snprintf(m95.getStringLog() + sms_index, (m95.getStringLogLength() - sms_index) - 1, PSTR("SDHC_"));
                        } break;
                        default: {
                          sms_index += snprintf(m95.getStringLog() + sms_index, (m95.getStringLogLength() - sms_index) - 1, PSTR("SD?_"));
                        }
                      }

                      sms_index += snprintf(m95.getStringLog() + sms_index, (m95.getStringLogLength() - sms_index) - 1, PSTR("%d"), sd.vol()->fatType());

                      uint32_t volumesize = sd.vol()->blocksPerCluster();
                      volumesize *= sd.vol()->clusterCount();
                      volumesize /= 2; // Kbytes
                      volumesize /= 1024; // Mbytes
                      sms_index += snprintf(m95.getStringLog() + sms_index, (m95.getStringLogLength() - sms_index) - 1, PSTR(" %luMB\n"), volumesize);

                      // File size in MB where MB = 1,048,576 bytes.
                      const uint32_t FILE_SIZE_MB = 10;

                      // File size in bytes.
                      const uint32_t FILE_SIZE = 1048576UL*FILE_SIZE_MB;

                      // number of test
                      const uint8_t TEST_COUNT = 3;

                      SdFile bfile;
                      // open or create file - truncate existing file.
                      if (!bfile.open(PSTR("bench.dat"), O_CREAT | O_TRUNC | O_RDWR)) {
                        sms_index += snprintf(m95.getStringLog() + sms_index, (m95.getStringLogLength() - sms_index) - 1, PSTR("Open file failed!"));

                        mlog.setFrom(PSTR("SD Error"));
                        mlog.setSMS(m95.getStringLog());
                        send_mlog = true;

                        goto out_jrb;
                      }

                      // fill buf with known data
                      for (uint16_t i = 0; i < (m95.getStringIOLength()-2); i++) {
                        m95.getStringIO()[i] = 'A' + (i % 26);
                      }
                      m95.getStringIO()[m95.getStringIOLength()-2] = '\r';
                      m95.getStringIO()[m95.getStringIOLength()-1] = '\n';

                      sms_index += snprintf(m95.getStringLog() + sms_index, (m95.getStringLogLength() - sms_index) - 1, PSTR("Speed\t Min \t Max \t Avrg\n"));
                      for (uint8_t nTest = 0; nTest < TEST_COUNT; nTest++) {
                        bfile.truncate(0);

                        uint32_t maxLatency = 0;
                        uint32_t minLatency = 9999999;
                        uint32_t totalLatency = 0;

                        uint32_t n = FILE_SIZE/m95.getStringIOLength();

                        uint32_t t = millis();
                        for (uint32_t i = 0; i < n; i++) {
                          uint32_t m = micros();

                          if (bfile.write(m95.getStringIO(), m95.getStringIOLength()) != m95.getStringIOLength()) {
                            sms_index += snprintf(m95.getStringLog() + sms_index, (m95.getStringLogLength() - sms_index) - 1, PSTR("Write file failed!"));

                            mlog.setFrom(PSTR("SD Error"));
                            mlog.setSMS(m95.getStringLog());
                            send_mlog = true;

                            bfile.close();
                            goto out_jrb;
                          }

                          m = micros() - m;
                          if (maxLatency < m) {
                            maxLatency = m;
                          }
                          if (minLatency > m) {
                            minLatency = m;
                          }
                          totalLatency += m;
                        }
                        bfile.sync();
                        t = millis() - t;

                        float s = bfile.fileSize()/t; // file size in B time in millis
                        s *= 8; // file size in Bits time in millis
                        s /= 1.024; // file size in KBits time in s
                        s /= 1.024; // file size in MBits time in Ks
                        s /= 1000; // file size in MBits time in s

                        // s *= 7.62939453125; // file size in MBits time in Ks
                        // s /= 1000; // time in s 

                        sms_index += snprintf(m95.getStringLog() + sms_index, (m95.getStringLogLength() - sms_index) - 1,
                          PSTR("%.2f\t%.2f\t%.2f\t%.2f\n"), s,
                          (8*m95.getStringIOLength()/(maxLatency*1.048576)),
                          (8*m95.getStringIOLength()/(minLatency*1.048576)),
                          n*(8*m95.getStringLogLength()/(totalLatency*1.048576)));
                      }
                      bfile.close();

                      sms_index += snprintf(m95.getStringLog() + sms_index, (m95.getStringLogLength() - sms_index) - 1,
                        PSTR("FS:%luMB BS:%uB\n"), FILE_SIZE_MB, m95.getStringIOLength());

                      mlog.setFrom(msg.getFrom());
                      mlog.setSMS(m95.getStringLog());
                      send_mlog = true;
out_jrb:
                      if (sd.exists(PSTR("bench.dat"))) sd.remove(PSTR("bench.dat"));
                    } break;
                    case 'w': {
                      Serial.print("powMask"); Serial.println(powMask);
                      Serial.print("pMask"); Serial.println(pMask);
                      uint32_t wa = 4294967295;
                      Serial.println(wa, HEX);
                      setPowerDown(wa);
                    }
                    default: {
                      mlog.setSMS(PSTR("jr?: bad json com!"));
                      send_mlog = true;
                    }
                  }

                  setPowerDown(~pMask);
                  setPowerUp(pMask);
                } else {
                  mlog.setSMS(PSTR("jr?: bad json command message!"));
                  send_mlog = true;
                }
              } break;
              default: {
                mlog.setSMS(PSTR("j?X: bad json nor get or set!"));
                send_mlog = true;
              } break;
            }

            if (send_mlog) {
              m95.cleanStringIO();
              mlog.serialize(m95.getStringIO(), m95.getStringIOLength(),
                (uint8_t*) adc_ring_buffer, GC_ADC_RING_SIZE_BYTES, false);
              geocentinela.printCmd('j', m95.getStringIO());
            }
          } else {
            geocentinela.println(PSTR("empty json request!"));
          }
        } break;
        case 'p': {
          Serial.println(powMask, HEX);
        } break;
        case 'R': {
          while (true) {
            Serial3.write(PSTR("AT+CCLK?\r"));
            m95.waitOfReaction(300);

            if (101 == m95.getResp()) {
              char *pstr = strchr(m95.getStringLog(),'\"');
              pstr[21] = 0;
              Serial.println(pstr+1);
            }

            if (Serial.available()) {
              if (Serial.read() == '@') goto out_R;
            }
          }
out_R:
          setPowerDown(M95_MASK);
          Serial.println("out print quectel RTC");
        } break;
        case 'B': {
          Serial.print("egin quectel comunication");
          setPowerUp(M95_MASK);
          while (true) {
            while (Serial.available()) {
              uint8_t cmd_s = Serial.read();
              if (cmd_s == '@') goto out_B;
              Serial3.write(cmd_s);
            }
            while (Serial3.available()) Serial.write(Serial3.read());
          }
out_B:
          Serial.println("ut quectel comunication...");
        } break;
        case 'E': {
          SdFile in;
          SdFile out;
          if (in.open("GC000000.XYZ", O_READ) and out.open("GC000000.DCO", O_CREAT | O_WRITE)) {
            // head:
            uint8_t head_size = 62;
            uint8_t head[head_size];
            in.read(head, head_size);
            out.write(head, head_size);

            // data:
            Sample * samples = (Sample *)adc_ring_buffer;
            uint32_t samples_size = GC_ADC_RING_SIZE_BYTES/sizeof(Sample);

            Delta * deltas = (Delta *)sd_buffer;
            uint32_t deltas_size = GC_SD_BUFFER_SIZE_BYTES/sizeof(Delta);

            dCodec dco(samples, samples_size, deltas, deltas_size);
            Serial.println(in.fileSize());
            Serial.println(Teensy3Clock.get());
            dco.encodeFile(in, out, 1+(in.fileSize()/sizeof(Sample)));
            Serial.println(Teensy3Clock.get());

            in.close();
            out.close();
          } else {
            Serial.println("io error!");
          }
        } break;
        case 'D': {
          SdFile in;
          SdFile out;
          if (in.open("GC000000.DCO", O_READ) and out.open("GC000000.NEW", O_CREAT | O_WRITE)) {
            // head:
            uint8_t head_size = 62;
            uint8_t head[head_size];
            in.read(head, head_size);
            out.write(head, head_size);

            // data:
            Sample * samples = (Sample *)adc_ring_buffer;
            uint32_t samples_size = GC_ADC_RING_SIZE_BYTES/sizeof(Sample);

            Delta * deltas = (Delta *)sd_buffer;
            uint32_t deltas_size = GC_SD_BUFFER_SIZE_BYTES/sizeof(Delta);

            dCodec dco(samples, samples_size, deltas, deltas_size);
            Serial.println(in.fileSize());
            Serial.println(Teensy3Clock.get());
            dco.decodeFile(in, out);
            Serial.println(Teensy3Clock.get());

            in.close();
            out.close();
          } else {
            Serial.println("io error!");
          }
        } break;
        case 'L': {
          Serial.print("filename:");
          if (gcSaveSyncEvents()) Serial.print("1:");
          else Serial.print("0:");
          Serial.println(filename);
        } break;
        case 'S': {
          boolean updated_cfg = false;
          if (gcSyncEvents(updated_cfg)) {
            geocentinela.print("updated_cfg:");
            if (updated_cfg) geocentinela.print("true ");
            else geocentinela.print("false ");
            geocentinela.print("send OK:");
            geocentinela.println(filename);
          } else {
            geocentinela.print("updated_cfg:");
            if (updated_cfg) geocentinela.print("true ");
            else geocentinela.print("false ");
            geocentinela.print("send not OK:");
            geocentinela.println(filename);
          }
        } break;
        case 'F': {
          uint32_t pMask = powMask;
          setPowerUp(M95_MASK | SD_MASK);
          if (!(powMask & M95_MASK) or !(powMask & SD_MASK)) break;

          if (!cfile.open(FILENAME_SYS_LOG, O_READ)) {
            Serial.println("File Error");
            setPowerDown(~pMask);
            setPowerUp(pMask);
            break;
          }

          uint16_t stateHTTP = m95.httpRESTFile(geocentinela.getHostName(), "POST /attachments/cache", cfile);
          cfile.close();

          Serial.print("stateHTTP:");
          Serial.println(stateHTTP >> 1);
          Serial.print("header-------------------begin:");
          Serial.println(strlen(m95.getStringLog()));
          Serial.println(m95.getStringLog());
          Serial.println("header-------------------end");
          Serial.print("body-------------------begin:");
          Serial.println(strlen(m95.getStringIO()));
          Serial.println(m95.getStringIO());
          Serial.println("body-------------------end");

          setPowerDown(~pMask);
          setPowerUp(pMask);
        } break;
        case 'M': {
          //boolean wa = true;
          //geocentinela.setGps(wa);
          //geocentinela.setGps(false);
          boolean updated_cfg = false;
          if (syncInstrument(updated_cfg)) {
            Serial.println("sync ok!");
          } else {
            m95.callWarning(geocentinela);
            setPowerDown(M95_MASK);
            Serial.println("sync fail!");
          }

          geocentinela.write();
        } break;
        case 'N': {
          uint32_t pMask = powMask;
          setPowerUp(M95_MASK | SD_MASK);
          if (!(powMask & M95_MASK) or !(powMask & SD_MASK)) break;
          
          uint32_t ts_flags = T_WRITE | T_ACCESS;
          if (!sd.exists(FILENAME_SYS_LOG)) ts_flags |= T_CREATE;

          if (!syslog.open(FILENAME_SYS_LOG, O_WRITE | O_APPEND | O_CREAT)) break;
          syslog.timestamp(ts_flags, year(), month(), day(), hour(), minute(), second());

          int32_t dtsr = 0;
          int16_t dtpr = 0;
          double tdelay = 0;
          m95.ntpSyncUDP(dtsr, dtpr, tdelay);
          Serial.print("dtsr: "); Serial.println(dtsr);
          Serial.print("dtpr: "); Serial.println(dtpr);
          Serial.print("delay: "); Serial.println(tdelay);
          Serial.print("offset: "); Serial.println(dtsr + dtpr/32768.0);
          
          setSyncProvider(getTeensy3Time);

          rMessage mlog;
          mlog.setInstrumentId(geocentinela.getId());
          mlog.setDateNow();
          mlog.setFrom("ntpsync");

          char msg[SMS_SIZE];
          snprintf(msg, SMS_SIZE,
                   "dtsr: %+011ld dtpr: %+06d delay: %+5e offset: %+5e",
                   dtsr, dtpr, tdelay, dtsr + dtpr/32768.0);
          mlog.setSMS(msg);

          m95.cleanStringIO();
          mlog.serialize(m95.getStringIO(), m95.getStringIOLength(), (uint8_t*)adc_ring_buffer, GC_ADC_RING_SIZE_BYTES, false);
          syslog.println(m95.getStringIO());
          syslog.timestamp(T_WRITE | T_ACCESS, year(), month(), day(), hour(), minute(), second());

          if (syslog.isOpen()) syslog.close();

          setPowerDown(~pMask);
          setPowerUp(pMask);
        } break;
        case 'O': {
          uint32_t pMask = powMask;
          setPowerUp(M95_MASK | SD_MASK);
          if (!(powMask & M95_MASK) or !(powMask & SD_MASK)) break;

          // sync sd fat_files
          uint32_t file_pos = 0;
          while (true) {
            sd.vwd()->seekSet(file_pos);
            if (!file.openNext(sd.vwd(), O_WRITE)) break;
            file_pos = sd.vwd()->curPosition();

            if (!syncFatFile(file)) Serial.println("sync fat_file fail!");
            else Serial.println("sync fat_file ok!");

            if (file.isOpen()) file.close();
          }

          setPowerDown(~pMask);
          setPowerUp(pMask);
        } break;
        case 'P': {
          uint32_t pMask = powMask;
          setPowerUp(M95_MASK | SD_MASK);
          if (!(powMask & M95_MASK) or !(powMask & SD_MASK)) break;

          if (syncSysLog()) {
            Serial.println("sync ok!");
          } else {
            Serial.println("sync fail!");
          }

          setPowerDown(~pMask);
          setPowerUp(pMask);
        } break;
        case 'Q': {
          Serial.println("::::::::::::::::::::::::::::::::::::::");
          Serial.println(geocentinela.getId());
          Serial.println(geocentinela.getUpdatedAt());
          Serial.println(geocentinela.c.getId());
          Serial.println(geocentinela.c.getUpdatedAt());
          Serial.println(geocentinela.s.getId());
          Serial.println(geocentinela.s.getUpdatedAt());
          Serial.println("::::::::::::::::::::::::::::::::::::::");
        } break;
        default: {
          delay(500);
          Serial.begin(9600);
          delay(500);
          Serial.flush();
          while (Serial.available()) Serial.read();

          geocentinela.println(PSTR("bad cmd!"));
        } break;
      }
    }
  }
}
/**/
