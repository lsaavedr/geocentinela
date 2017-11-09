#ifndef GC_CFG_H
#define GC_CFG_H

#if defined(ARDUINO) and ARDUINO >= 100
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif
#include <EEPROM.h>
#include <ArduinoJson.h>
#include <TimeLib.h>
#include <SdFat.h>
#include <math.h>       /* round */

#define CFG_VERSION 7

#define MAX_GAIN 7 // 2**7 == 128
#define MAX_HARDWARE_AVERAGE 4 // 0->4, 1->8, 2->16, 3->32, 4->0
#define MIN_TICK_TIME 100 // 10kSPS
#define MAX_TIME_TYPE 2
#define SEG_A_DAY 86400
#define MAX_TRIGGER_LEVEL (0x7FFF-1) // ~1.25v
#define MAX_TRIGGER_TIME 60 // 1min
#define MAX_TIME_ZONE_OFFSET 43200 // 12hrs
#define MIN_TIME_ZONE_OFFSET -43200 // -12hrs
#define MAX_FAT_FILE_ACTION 3

#define HOST_SIZE 33
#define PHONE_SIZE 13
#define DATE_SIZE 23
#define NAME_SIZE 36
#define DIGIT_SIZE 15
#define FILE_ID_SIZE 61
#define SMS_SIZE 161

//#define NTP_N 5
//#define NTP_NAME_SIZE 30

#define SYNC_EVENT_SIZE 100
#define SYNC_EVENT_ID 0
#define SYNC_EVENT_RTC 4
#define SYNC_EVENT_MICRO 8
#define SYNC_EVENT_PPV 12
#define SYNC_EVENT_FNAME 16
#define SYNC_EVENT_INDEX 28
#define SYNC_EVENT_NSAMPLES 32
#define SYNC_EVENT_FILE_ID 36
#define SYNC_EVENT_FILE_CACHE_ID 40

class rObject {
public:
  void set(rObject const& o)
  {
    this->setId(o.id);
    this->setUpdatedAt(o.updated_at);
  }
  void setId(uint32_t const& id)
  {
    if (this->id != id) {
      this->id = id;
    }
  }
  void setUpdatedAt(uint32_t const& updated_at)
  {
    this->updated_at = updated_at;
  }
  void touch()
  {
    this->updated_at = Teensy3Clock.get();
  }

  void serialize(char* const& json, size_t const& maxSize, uint8_t* const& buffer, size_t const& buffer_size, boolean const& to_search);
  boolean deserialize(char const* const& json, uint8_t* const& buffer, size_t const& buffer_size, boolean checkValues);
  boolean deserialize(char const* const& json, uint8_t* const& buffer, size_t const& buffer_size)
  {
    return deserialize(json, buffer, buffer_size, true);
  }

  uint32_t const& getId() const { return this->id; }
  uint32_t const& getUpdatedAt() const { return this->updated_at; }

  boolean updatedAt(uint32_t const& updated_at) { return this->updated_at >= updated_at; }
  void show(uint8_t* const& buffer, size_t const& buffer_size);

protected:
  uint32_t id = 0;
  uint32_t updated_at = 0;

  // deben ser implementadas:
  virtual void fixValues();
  virtual boolean checkJsonValues(JsonObject& object);
  virtual void setJsonValues(JsonObject& object);
  virtual void builtJson(JsonObject& root, boolean const& to_search);
};

class rSensor : public rObject {
public:
  void set(rSensor const& s)
  {
    this->rObject::set(s);

    strncpy(this->name, s.name, NAME_SIZE);

    this->sensitivity = s.sensitivity;

    this->lng = s.lng;
    this->lat = s.lat;
    this->cta = s.cta;

    fixValues();
  }

  void setName(char const* const& name)
  {
    if (strcmp(this->name, name) != 0) {
      strncpy(this->name, name, NAME_SIZE);
      this->updated_at = Teensy3Clock.get();
    }
    fixValues();
  }

  void setSensitivity(float const& sensitivity)
  {
    if (this->sensitivity != sensitivity) {
      this->sensitivity = sensitivity;
      this->updated_at = Teensy3Clock.get();
    }
    fixValues();
  }

  void setLocation(const float& lng, const float& lat, const float& cta)
  {
    if (this->lng != lng or this->lat != lat or this->cta != cta) {
      this->lng = lng;
      this->lat = lat;
      this->cta = cta;
      this->updated_at = Teensy3Clock.get();
    }
  }

  char const* getName() { return this->name; }
  float const& getSensitivity() { return this->sensitivity; }
  void getLocation(float& lng, float& lat, float& cta)
  {
    lng = this->lng;
    lat = this->lat;
    cta = this->cta;
  }

  // deben ser implementadas:
  void fixValues();
private:
  char name[NAME_SIZE] = "";
  
  float sensitivity = 1.0; // V/[mm/s]

  float lng = 0.0;
  float lat = 0.0;
  float cta = 0.0;

  // deben ser implementadas:
  //void fixValues();
  boolean checkJsonValues(JsonObject& sensor);
  void setJsonValues(JsonObject& sensor);
  void builtJson(JsonObject& root, boolean const& to_search);
};

class rConfigure : public rObject {
public:
  void set(rConfigure const& c)
  {
    this->rObject::set(c);

    strncpy(this->name, c.name, NAME_SIZE);

    this->gain = c.gain;
    this->hardware_average = c.hardware_average;
    this->tick_time = c.tick_time;

    this->time_type = c.time_type;
    this->time_begin = c.time_begin;
    this->time_end = c.time_end;

    this->trigger_level = c.trigger_level;
    this->trigger_time = c.trigger_time;
    this->send_trigger_time = c.send_trigger_time;

    fixValues();
  }

  void setName(char const* const& name)
  {
    if (strcmp(this->name, name) != 0) {
      strncpy(this->name, name, NAME_SIZE);
      this->updated_at = Teensy3Clock.get();
    }
    fixValues();
  }

  void setGain(uint8_t const& gain)
  {
    if (this->gain != gain) {
      this->gain = gain;
      this->updated_at = Teensy3Clock.get();
    }
    fixValues();
  }
  void setHardwareAverage(uint8_t const& hardware_average)
  {
    if (this->hardware_average != hardware_average) {
      this->hardware_average = hardware_average;
      this->updated_at = Teensy3Clock.get();
    }
    fixValues();
  }
  void setTickTime(uint32_t const& tick_time)
  {
    if (this->tick_time != tick_time) {
      this->tick_time = tick_time;
      this->updated_at = Teensy3Clock.get();
    }
    fixValues();
  }

  void setTimeType(uint8_t const& time_type)
  {
    if (this->time_type != time_type) {
      this->time_type = time_type;
      this->updated_at = Teensy3Clock.get();
    }
    fixValues();
  }
  void setTimeBegin(uint32_t const& time_begin)
  {
    if (this->time_begin != time_begin) {
      this->time_begin = time_begin;
      this->updated_at = Teensy3Clock.get();
    }
    fixValues();
  }
  void setTimeEnd(uint32_t const& time_end)
  {
    if (this->time_end != time_end) {
      this->time_end = time_end;
      this->updated_at = Teensy3Clock.get();
    }
    fixValues();
  }

  void setTriggerLevel(uint16_t const& trigger_level)
  {
    if (this->trigger_level != trigger_level) {
      this->trigger_level = trigger_level;
      this->updated_at = Teensy3Clock.get();
    }
    fixValues();
  }
  void setTriggerTime(uint32_t const& trigger_time)
  {
    if (this->trigger_time != trigger_time) {
      this->trigger_time = trigger_time;
      this->updated_at = Teensy3Clock.get();
    }
    fixValues();
  }
  void setSendTriggerTime(uint32_t const& send_trigger_time)
  {
    if (this->send_trigger_time != send_trigger_time) {
      this->send_trigger_time = send_trigger_time;
      this->updated_at = Teensy3Clock.get();
    }
    fixValues();
  }

  char const* getName() { return this->name; }
  uint8_t const& getGain() { return this->gain; }
  uint8_t const& getHardwareAverage() { return this->hardware_average; }
  uint32_t const& getTickTime() { return this->tick_time; }
  uint8_t const& getTimeType() { return this->time_type; }
  uint32_t const& getTimeBegin() { return this->time_begin; }
  uint32_t const& getTimeEnd() { return this->time_end; }
  uint16_t const& getTriggerLevel() { return this->trigger_level; }
  uint32_t const& getTriggerTime() { return this->trigger_time; }
  uint32_t const& getSendTriggerTime() { return this->send_trigger_time; }

  // deben ser implementadas:
  void fixValues();

private:
  char name[NAME_SIZE] = "";

  uint8_t gain = 0; //  0->1, 1->2, 2->4, 3->8, 4->16, 5->32, 6->64, 7->128
  uint8_t hardware_average = 3; //  0->4, 1->8, 2->16, 3->32
  uint32_t tick_time = 250; // 4ksps

  uint8_t time_type = 0; // testing
  uint32_t time_begin = 5;
  uint32_t time_end = 10;

  uint16_t trigger_level = 0; // 0->min_value->100% 0x7FFF->max_value+1->0%
  uint32_t trigger_time = 0; // 0 segundos < 86400
  uint32_t send_trigger_time = 3600; // 3600 segundos < 86400

  // deben ser implementadas:
  //void fixValues();
  boolean checkJsonValues(JsonObject& configure);
  void setJsonValues(JsonObject& configure);
  void builtJson(JsonObject& root, boolean const& to_search);
};

class rInstrument : public rObject {
public:
  void set(rInstrument const& i)
  {
    this->rObject::set(i);

    this->s.set(i.s);
    this->c.set(i.c);

    strncpy(this->host_name, i.host_name, HOST_SIZE);
    strncpy(this->phone_warning, i.phone_warning, PHONE_SIZE);

    strncpy(this->time_zone, i.time_zone, NAME_SIZE);
    this->time_zone_offset = i.time_zone_offset;

    this->gprs = i.gprs;
    this->gps = i.gps;

    fixValues();
  }
  void setUpdatedAt(uint32_t const& updated_at)
  {
    this->rObject::setUpdatedAt(updated_at);

    this->s.setUpdatedAt(updated_at);
    this->c.setUpdatedAt(updated_at);
  }

  void setHostName(char const* const& host_name)
  {
    if (strcmp(this->host_name, host_name) != 0) {
      strncpy(this->host_name, host_name, HOST_SIZE);
      this->updated_at = Teensy3Clock.get();
    }
    fixValues();
  }
  void setPhoneWarning(char const* const& phone_warning)
  {
    if (strcmp(this->phone_warning, phone_warning) != 0) {
      strncpy(this->phone_warning, phone_warning, PHONE_SIZE);
      this->updated_at = Teensy3Clock.get();
    }
    fixValues();
  }
  void setTimeZone(char const* const& time_zone)
  {
    if (strcmp(this->time_zone, time_zone) != 0) {
      strncpy(this->time_zone, time_zone, NAME_SIZE);
      this->updated_at = Teensy3Clock.get();
    }
    fixValues();
  }
  void setTimeZoneOffset(int32_t const& time_zone_offset)
  {
    if (this->time_zone_offset != time_zone_offset) {
      this->time_zone_offset = time_zone_offset;
      this->updated_at = Teensy3Clock.get();
    }
    fixValues();
  }
  void setGprs(boolean const& gprs)
  {
    if (this->gprs != gprs) {
      this->gprs = gprs;
      this->updated_at = Teensy3Clock.get();
    }
  }
  void setGps(boolean const& gps)
  {
    if (this->gps != gps) {
      this->gps = gps;
      this->updated_at = Teensy3Clock.get();
    }
  }

  char const* getHostName() { return this->host_name; }
  char const* getPhoneWarning() { return this->phone_warning; }
  char const* getTimeZone() { return this->time_zone; }
  int32_t const& getTimeZoneOffset() { return this->time_zone_offset; }
  boolean const& getGprs() { return this->gprs; }
  boolean const& getGps() { return this->gps; }

// otras:
  boolean read();
  boolean write();

  void show();
  void show(uint8_t* const& buffer, size_t const& buffer_size); // caso especial

  size_t gcCmd(uint8_t const* const& cmd, uint8_t const& n)
  {
    uint8_t head[9] = { '\xaa', '\xaa', '\xaa',
                        '\xff', '\xff', '\xff',
                        '\x00', '\x00', '\x00' };
    size_t aux = Serial.write(head, 9);
    return aux + Serial.write(cmd, n);
  }

  size_t gcCmd(uint8_t const& cmd)
  {
    return gcCmd(&cmd, 1);
  }

  size_t printCmd(char const& cmd, char const* const& log_string)
  {
    size_t aux = gcCmd(cmd);
    return aux + Serial.print(log_string);
  }
  
  size_t print(char const* const& log_string)
  {
    size_t aux = gcCmd('t');
    return aux + Serial.print(log_string);
  }

  size_t println(char const* const& log_string)
  {
    size_t aux = print(log_string);
    return aux + Serial.println();
  }

public:
  // sensor
  rSensor s;

  // configure
  rConfigure c;

private:
  char host_name[HOST_SIZE] = PSTR("centinela.timining.cl");
  char phone_warning[PHONE_SIZE] = PSTR("+56993118748");

  char time_zone[NAME_SIZE] = PSTR("Santiago");
  int32_t time_zone_offset = -14400; // -4hrs

  boolean gprs = true;
  boolean gps = false;
/*
  char ntp_servers[NTP_N][NTP_NAME_SIZE] = {
    PSTR("3.cl.pool.ntp.org"),            // ntp0
    PSTR("0.south-america.pool.ntp.org"), // ntp1
    PSTR("1.south-america.pool.ntp.org"), // ntp2
    PSTR("2.south-america.pool.ntp.org"), // ntp3
    PSTR("3.south-america.pool.ntp.org")  // ntp4
  };
*/
  // deben ser implementadas:
  void fixValues();
  boolean checkJsonValues(JsonObject& instrument);
  void setJsonValues(JsonObject& instrument);
  void builtJson(JsonObject& root, boolean const& to_search);
};

class rFatFile : public rObject {
public:
  void set(rFatFile const& ff)
  {
    this->rObject::set(ff);

    this->instrument_id = ff.instrument_id;
    this->action = ff.action;

    strncpy(this->name, ff.name, NAME_SIZE);

    this->size = ff.size;
    this->ts = ff.ts;

    fixValues();
  }

  void setInstrumentId(uint32_t const& instrument_id)
  {
    if (this->instrument_id != instrument_id) {
      this->instrument_id = instrument_id;
    }
  }

  void setAction(uint8_t const& action)
  {
    if (this->action != action) {
      this->action = action;
      this->updated_at = Teensy3Clock.get();
    }
    fixValues();
  }

  void setName(char const* const& name)
  {
    if (strcmp(this->name, name) != 0) {
      strncpy(this->name, name, NAME_SIZE);
      this->updated_at = Teensy3Clock.get();
    }
    fixValues();
  }

  void setFile(SdFile &file)
  {
    if (file.getName(this->name, NAME_SIZE)) {
      if (file.isDir()) {
        // Indicate a directory
        uint8_t len = strlen(this->name);
        if (len < NAME_SIZE-1) {
          this->name[len++] = '/';
          this->name[len] = '\0';
        }
      }

      this->size = file.fileSize();
      this->ts = getFatFileCreationEpoch(file);
      this->updated_at = getFatFileLastWriteEpoch(file);
    }
    fixValues();
  }

  uint32_t getInstrumentId() { return this->instrument_id; }
  uint8_t getAction() { return this->action; }
  char const* getName() { return this->name; }
  uint32_t getTimeStamp() { return this->ts; }

private:
  uint32_t instrument_id;

  uint8_t action = 0; // 0->nada, 1->borrar, 2->procesar, 3->???

  char name[NAME_SIZE];
  uint32_t size = 0;
  uint32_t ts = 0;

  uint32_t getFatFileCreationEpoch(SdFile &file)
  {
    dir_t dir;
    if (!file.dirEntry(&dir)) return 0;

    tmElements_t tm;
    tm.Year = FAT_YEAR(dir.creationDate) - 1970;
    tm.Month = FAT_MONTH(dir.creationDate);
    tm.Day = FAT_DAY(dir.creationDate);
    tm.Hour = FAT_HOUR(dir.creationTime);
    tm.Minute = FAT_MINUTE(dir.creationTime);
    tm.Second = FAT_SECOND(dir.creationTime);
    tm.Wday = 0; /// ???

    int64_t tzoffset = (int32_t)now()-(int32_t)Teensy3Clock.get();
    tzoffset = (int64_t)round(tzoffset*1.0/900); // 900s = 15min
    tzoffset *= 900;

    int64_t epoch = (int64_t)makeTime(tm);

    if (epoch > tzoffset) epoch -= tzoffset;
    else return 0;

    return (uint32_t)epoch;
  }

  uint32_t getFatFileLastWriteEpoch(SdFile &file)
  {
    dir_t dir;
    if (!file.dirEntry(&dir)) return 0;

    tmElements_t tm;
    tm.Year = FAT_YEAR(dir.lastWriteDate) - 1970;
    tm.Month = FAT_MONTH(dir.lastWriteDate);
    tm.Day = FAT_DAY(dir.lastWriteDate);
    tm.Hour = FAT_HOUR(dir.lastWriteTime);
    tm.Minute = FAT_MINUTE(dir.lastWriteTime);
    tm.Second = FAT_SECOND(dir.lastWriteTime);
    tm.Wday = 0; /// ???

    int64_t tzoffset = (int32_t)now()-(int32_t)Teensy3Clock.get();
    tzoffset = (int64_t)round(tzoffset*1.0/900); // 900s = 15min
    tzoffset *= 900;

    int64_t epoch = (int64_t)makeTime(tm);

    if (epoch > tzoffset) epoch -= tzoffset;
    else return 0;

    return (uint32_t)epoch;
  }

  // deben ser implementadas:
  void fixValues();
  boolean checkJsonValues(JsonObject& instrument);
  void setJsonValues(JsonObject& instrument);
  void builtJson(JsonObject& root, boolean const& to_search);
};

class rEvent : public rObject {
public:
  void read(uint8_t sync_event[SYNC_EVENT_SIZE], rInstrument const& i)
  {
    this->id = *((uint32_t*)&sync_event[SYNC_EVENT_ID]);

    this->ppv = *((float*)&sync_event[SYNC_EVENT_PPV]);
    this->rtc = *((uint32_t*)&sync_event[SYNC_EVENT_RTC]);
    this->event_file_id = *((uint32_t*)&sync_event[SYNC_EVENT_FILE_ID]);
    this->sensor_id = i.s.getId();

    fixValues();
  }
  void write(uint8_t sync_event[SYNC_EVENT_SIZE])
  {
    *((uint32_t*)&sync_event[SYNC_EVENT_ID]) = this->id;

    *((float*)&sync_event[SYNC_EVENT_PPV]) = this->ppv;
    *((uint32_t*)&sync_event[SYNC_EVENT_RTC]) = this->rtc;
    *((uint32_t*)&sync_event[SYNC_EVENT_FILE_ID]) = this->event_file_id;
  }

  void set(rEvent const& e)
  {
    rObject::set(e);

    this->ppv = e.ppv;
    this->rtc = e.rtc;
    this->event_file_id = e.event_file_id;
    this->sensor_id = e.sensor_id;
  }

  void setPPV(float const& ppv)
  {
    if (this->ppv != ppv) {
      this->ppv = ppv;
    }
    fixValues();
  }
  void setRtc(uint32_t const& rtc)
  {
    if (this->rtc != rtc) {
      this->rtc = rtc;
    }
    fixValues();
  }
  void setEventFileId(uint32_t const& event_file_id)
  {
    if (this->event_file_id != event_file_id) {
      this->event_file_id = event_file_id;
    }
    fixValues();
  }
  void setSensorId(uint32_t const& sensor_id)
  {
    if (this->sensor_id != sensor_id) {
      this->sensor_id = sensor_id;
    }
    fixValues();
  }

  float getPPV() { return this->ppv; }
  uint32_t getRtc() { return this->rtc; }
  uint32_t getEventFileId() { return this->event_file_id; }
  uint32_t getSensorId() { return this->sensor_id; }

private:
  float ppv = 0;
  uint32_t rtc = 0;

  uint32_t event_file_id = 0;

  uint32_t sensor_id = 0;

  // deben ser implementadas:
  void fixValues();
  boolean checkJsonValues(JsonObject& instrument);
  void setJsonValues(JsonObject& instrument);
  void builtJson(JsonObject& root, boolean const& to_search);
};

class rEventFile : public rObject {
public:
  void set(rEventFile const& ef)
  {
    this->rObject::set(ef);

    this->event_id = ef.event_id;
    strncpy(this->file_cache_id, ef.file_cache_id, FILE_ID_SIZE);

    fixValues();
  }

  void read(uint8_t sync_event[SYNC_EVENT_SIZE])
  {
    this->id = *((uint32_t*)&sync_event[SYNC_EVENT_FILE_ID]);

    this->event_id = *((uint32_t*)&sync_event[SYNC_EVENT_ID]);

    strncpy(this->file_cache_id, (char*)&sync_event[SYNC_EVENT_FILE_CACHE_ID], FILE_ID_SIZE);

    fixValues();
  }
  void write(uint8_t sync_event[SYNC_EVENT_SIZE])
  {
    *((uint32_t*)&sync_event[SYNC_EVENT_FILE_ID]) = this->id;

    *((uint32_t*)&sync_event[SYNC_EVENT_ID]) = this->event_id;

    strncpy((char*)&sync_event[SYNC_EVENT_FILE_CACHE_ID], this->file_cache_id, FILE_ID_SIZE);
  }
/*
  void setEventId(uint32_t const& event_id)
  {
    if (this->event_id != event_id) {
      this->event_id = event_id;
    }
    fixValues();
  }
  void setFileCacheId(char const* const& file_cache_id)
  {
    if (strcmp(this->file_cache_id, file_cache_id) != 0) {
      strncpy(this->file_cache_id, file_cache_id, FILE_ID_SIZE);
      this->updated_at = Teensy3Clock.get();
    }
    fixValues();
  }
*/
  uint32_t getEventId() { return this->event_id; }
  char const* getFileCacheId() { return this->file_cache_id; }
private:
  uint32_t event_id;
  char file_cache_id[FILE_ID_SIZE] = "";

  // deben ser implementadas:
  void fixValues();
  boolean checkJsonValues(JsonObject& instrument);
  void setJsonValues(JsonObject& instrument);
  void builtJson(JsonObject& root, boolean const& to_search);
};

class rMessage : public rObject {
public:
  void setInstrumentId(uint32_t const& instrument_id)
  {
    if (this->instrument_id != instrument_id) {
      this->instrument_id = instrument_id;
    }
  }
  void setFrom(char const* const& from)
  {
    if (strcmp(this->from, from) != 0) {
      strncpy(this->from, from, PHONE_SIZE);
      this->updated_at = Teensy3Clock.get();
    }
    fixValues();
  }
  void setDateNow()
  {
    snprintf(this->date, DATE_SIZE,
             "%04d/%02d/%02d %02d:%02d:%02d%+03ld",
             year(), month(), day(), hour(), minute(), second(),
             ((int32_t)now()-(int32_t)Teensy3Clock.get())/900);
    this->updated_at = Teensy3Clock.get();
    fixValues();
  }
  void setDate(char const* const& date)
  {
    if (strcmp(this->date, date) != 0) {
      strncpy(this->date, date, DATE_SIZE);
      this->updated_at = Teensy3Clock.get();
    }
    fixValues();
  }
  void setSMS(char const* const& sms)
  {
    if (strcmp(this->sms, sms) != 0) {
      strncpy(this->sms, sms, SMS_SIZE);
      this->updated_at = Teensy3Clock.get();
    }
    fixValues();
  }

  uint32_t getInstrumentId() { return this->instrument_id; }
  char const* getFrom() { return this->from; }
  char const* getDate() { return this->date; }
  char const* getSMS() { return this->sms; }

private:
  uint32_t instrument_id;

  char from[PHONE_SIZE];
  char date[DATE_SIZE];
  char sms[SMS_SIZE];

  // deben ser implementadas:
  void fixValues();
  boolean checkJsonValues(JsonObject& instrument);
  void setJsonValues(JsonObject& instrument);
  void builtJson(JsonObject& root, boolean const& to_search);
};

#endif // GC_CFG_H
