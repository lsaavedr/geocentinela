#include "gcCFG.h"

void rObject::serialize(char* const& json, size_t const& maxSize, uint8_t* const& buffer, size_t const& buffer_size, boolean const& to_search)
{
  StaticSharedJsonBuffer jsonBuffer(buffer, buffer_size);
  JsonObject& root = jsonBuffer.createObject();

  builtJson(root, to_search);

  root.printTo(json, maxSize);
}

boolean rObject::deserialize(char const* const& json, uint8_t* const& buffer, size_t const& buffer_size, boolean checkValues)
{
  StaticSharedJsonBuffer jsonBuffer(buffer, buffer_size);
  JsonObject& object = jsonBuffer.parseObject(json);

  if (!object.success()) return false;
  if (checkValues) {
    if (!object["id"].success()) return false;
    if (!object["updated_at"].success()) return false;

    if (!checkJsonValues(object)) return false;
  }

  if (object["id"].success()) this->id = object["id"];
  if (object["updated_at"].success()) this->updated_at = object["updated_at"];
  setJsonValues(object);

  fixValues();
  return true;
}

void rObject::show(uint8_t* const& buffer, size_t const& buffer_size)
{
  StaticSharedJsonBuffer jsonBuffer(buffer, buffer_size);
  JsonObject& root = jsonBuffer.createObject();

  builtJson(root, false);

  root.prettyPrintTo(Serial);
}
//------------------------------------------------------------------------------------------------
void rSensor::fixValues()
{
  name[NAME_SIZE-1] = '\0';

  if (0.0 >= sensitivity) this->sensitivity = 1.0;
}

void rConfigure::fixValues()
{
  name[NAME_SIZE-1] = '\0';

  if (MAX_GAIN < this->gain) this->gain %= MAX_GAIN;
  if (MAX_HARDWARE_AVERAGE < this->hardware_average) this->hardware_average %= (MAX_HARDWARE_AVERAGE+1);
  if (MIN_TICK_TIME > this->tick_time) this->tick_time = MIN_TICK_TIME;
  if (MAX_TIME_TYPE < this->time_type) this->time_type %= (MAX_TIME_TYPE+1);
  if (SEG_A_DAY < this->time_begin) this->time_begin %= SEG_A_DAY;
  if (SEG_A_DAY < this->time_end) this->time_end %= SEG_A_DAY;
  if (MAX_TRIGGER_LEVEL < this->trigger_level) this->trigger_level = MAX_TRIGGER_LEVEL;
  if (MAX_TRIGGER_TIME < this->trigger_time) this->trigger_time %= MAX_TRIGGER_TIME;
  if (SEG_A_DAY < this->send_trigger_time) this->send_trigger_time %= SEG_A_DAY;
}

void rInstrument::fixValues()
{
  s.fixValues();
  c.fixValues();

  host_name[HOST_SIZE-1] = '\0';
  phone_warning[PHONE_SIZE-1] = '\0';
  time_zone[NAME_SIZE-1] = '\0';

  if (MAX_TIME_ZONE_OFFSET < this->time_zone_offset) this->time_zone_offset = MAX_TIME_ZONE_OFFSET;
  if (MIN_TIME_ZONE_OFFSET > this->time_zone_offset) this->time_zone_offset = MIN_TIME_ZONE_OFFSET;
}

void rFatFile::fixValues()
{
  name[NAME_SIZE-1] = '\0';

  if (MAX_FAT_FILE_ACTION < this->action) this->action %= (MAX_FAT_FILE_ACTION+1);
}

void rEvent::fixValues()
{
  if (0.0 > ppv) this->ppv = 0;
}

void rEventFile::fixValues()
{
  this->file_cache_id[FILE_ID_SIZE-1] = '\0';
}

void rMessage::fixValues()
{
  from[PHONE_SIZE-1] = '\0';
  date[DATE_SIZE-1] = '\0';
  sms[SMS_SIZE-1] = '\0';
}

void rSensor::builtJson(JsonObject& root, boolean const& to_search)
{
  JsonObject& sensor = root.createNestedObject("sensor");

  if (0 != this->id) sensor["id"] = this->id;
  else sensor["name"] = this->name;

  if (!to_search) {
    if (0 != this->id) sensor["name"] = this->name;

    sensor["sensitivity"] = double_with_n_digits(this->sensitivity, DIGIT_SIZE);
    sensor["lng"] = double_with_n_digits(this->lng, DIGIT_SIZE);
    sensor["lat"] = double_with_n_digits(this->lat, DIGIT_SIZE);
    sensor["cta"] = double_with_n_digits(this->cta, DIGIT_SIZE);
  }
}

void rConfigure::builtJson(JsonObject& root, boolean const& to_search)
{
  JsonObject& configure = root.createNestedObject("configure");

  if (0 != this->id) configure["id"] = this->id;
  else configure["name"] = this->name;

  if (!to_search) {
    if (0 != this->id) configure["name"] = this->name;

    configure["gain"] = this->gain;
    configure["hardware_average"] = this->hardware_average;
    configure["tick_time"] = this->tick_time;
    configure["time_type"] = this->time_type;
    configure["time_begin"] = this->time_begin;
    configure["time_end"] = this->time_end;
    configure["trigger_level"] = this->trigger_level;
    configure["trigger_time"] = this->trigger_time;
    configure["send_trigger_time"] = this->send_trigger_time;
  }
}

void rInstrument::builtJson(JsonObject& root, boolean const& to_search)
{
  JsonObject& instrument = root.createNestedObject("instrument");

  instrument["hid"] = SIM_UIDH;
  instrument["hmid"] = SIM_UIDMH;
  instrument["lmid"] = SIM_UIDML;
  instrument["lid"] = SIM_UIDL;

  if (!to_search) {
    if (0 != this->id) instrument["id"] = this->id;

    instrument["host_name"] = this->host_name;
    instrument["phone_warning"] = this->phone_warning;

    instrument["time_zone"] = this->time_zone;
    instrument["time_zone_offset"] = this->time_zone_offset;

    instrument["gprs"] = this->gprs;
    instrument["gps"] = this->gps;

    uint32_t sid = s.getId();
    if (sid != 0) instrument["sensor_id"] = sid;

    uint32_t cid = c.getId();
    if (cid != 0) instrument["configure_id"] = cid;
  }
}

void rFatFile::builtJson(JsonObject& root, boolean const& to_search)
{
  JsonObject& fat_file = root.createNestedObject("fat_file");

  if (0 != this->instrument_id) fat_file["instrument_id"] = this->instrument_id;
  fat_file["name"] = this->name;

  if (!to_search) {
    fat_file["ts"] = this->ts;
    fat_file["action"] = this->action;
    fat_file["size"] = this->size;
  }
}

void rEvent::builtJson(JsonObject& root, boolean const& to_search)
{
  JsonObject& event = root.createNestedObject("event");

  if (0 != this->id) event["id"] = this->id;

  event["ppv"] = this->ppv;
  event["rtc"] = this->rtc;
  if (0 != this->sensor_id) event["sensor_id"] = this->sensor_id;
}
void rEventFile::builtJson(JsonObject& root, boolean const& to_search)
{
  JsonObject& event_file = root.createNestedObject("event_file");

  if (0 != this->id) event_file["id"] = this->id;
  if (0 != this->event_id) event_file["event_id"] = this->event_id;
  if (strlen(this->file_cache_id) > 0) event_file["file_cache_id"] = this->file_cache_id;
}

void rMessage::builtJson(JsonObject& root, boolean const& to_search)
{
  JsonObject& message = root.createNestedObject("message");

  if (0 != this->id) message["id"] = this->id;

  if (0 != this->instrument_id) message["instrument_id"] = this->instrument_id;
  message["from"] = this->from;
  message["date"] = this->date;
  message["sms"] = this->sms;
}

boolean rSensor::checkJsonValues(JsonObject& sensor)
{
  if (!sensor["name"].success()) return false;
  if (!sensor["sensitivity"].success()) return false;
  if (!sensor["lng"].success()) return false;
  if (!sensor["lat"].success()) return false;
  if (!sensor["cta"].success()) return false;

  return true;
}

boolean rConfigure::checkJsonValues(JsonObject& configure)
{
  if (!configure["name"].success()) return false;
  if (!configure["gain"].success()) return false;
  if (!configure["hardware_average"].success()) return false;
  if (!configure["tick_time"].success()) return false;
  if (!configure["time_type"].success()) return false;
  if (!configure["time_begin"].success()) return false;
  if (!configure["time_end"].success()) return false;
  if (!configure["trigger_level"].success()) return false;
  if (!configure["trigger_time"].success()) return false;
  if (!configure["send_trigger_time"].success()) return false;

  return true;
}

boolean rInstrument::checkJsonValues(JsonObject& instrument)
{
  if (!instrument["host_name"].success()) return false;
  if (!instrument["phone_warning"].success()) return false;
  if (!instrument["time_zone"].success()) return false;
  if (!instrument["time_zone_offset"].success()) return false;
  if (!instrument["sensor_id"].success()) return false;
  if (!instrument["configure_id"].success()) return false;
  if (!instrument["gprs"].success()) return false;
  if (!instrument["gps"].success()) return false;

  return true;
}

boolean rFatFile::checkJsonValues(JsonObject& fat_file)
{
  if (!fat_file["instrument_id"].success()) return false;

  if (!fat_file["action"].success()) return false;

  if (!fat_file["name"].success()) return false;
  if (!fat_file["size"].success()) return false;
  if (!fat_file["ts"].success()) return false;

  return true;
}

boolean rEvent::checkJsonValues(JsonObject& event)
{
  if (!event["ppv"].success()) return false;
  if (!event["rtc"].success()) return false;
  if (!event["sensor_id"].success()) return false;

  return true;
}

boolean rEventFile::checkJsonValues(JsonObject& event_file)
{
  if (!event_file["event_id"].success()) return false;

  return true;
}

boolean rMessage::checkJsonValues(JsonObject& message)
{
  if (!message["instrument_id"].success()) return false;
  if (!message["from"].success()) return false;
  if (!message["date"].success()) return false;
  if (!message["sms"].success()) return false;

  return true;
}

void rSensor::setJsonValues(JsonObject& sensor)
{
  if (sensor["name"].success()) strncpy(this->name, sensor["name"], NAME_SIZE);
  if (sensor["sensitivity"].success()) this->sensitivity = sensor["sensitivity"];
  if (sensor["lng"].success()) this->lng = sensor["lng"];
  if (sensor["lat"].success()) this->lat = sensor["lat"];
  if (sensor["cta"].success()) this->cta = sensor["cta"];
}

void rConfigure::setJsonValues(JsonObject& configure)
{
  if (configure["name"].success()) strncpy(this->name, configure["name"], NAME_SIZE);
  if (configure["gain"].success()) this->gain = configure["gain"];
  if (configure["hardware_average"].success()) this->hardware_average = configure["hardware_average"];
  if (configure["tick_time"].success()) this->tick_time = configure["tick_time"];
  if (configure["time_type"].success()) this->time_type = configure["time_type"];
  if (configure["time_begin"].success()) this->time_begin = configure["time_begin"];
  if (configure["time_end"].success()) this->time_end = configure["time_end"];
  if (configure["trigger_level"].success()) this->trigger_level = configure["trigger_level"];
  if (configure["trigger_time"].success()) this->trigger_time = configure["trigger_time"];
  if (configure["send_trigger_time"].success()) this->send_trigger_time = configure["send_trigger_time"];
}

void rInstrument::setJsonValues(JsonObject& instrument)
{
  if (instrument["host_name"].success()) strncpy(this->host_name, instrument["host_name"], HOST_SIZE);
  if (instrument["phone_warning"].success()) strncpy(this->phone_warning, instrument["phone_warning"], PHONE_SIZE);

  if (instrument["time_zone"].success()) strncpy(this->time_zone, instrument["time_zone"], NAME_SIZE);
  if (instrument["time_zone_offset"].success()) this->time_zone_offset = instrument["time_zone_offset"];

  if (instrument["gprs"].success()) this->gprs = instrument["gprs"];
  if (instrument["gps"].success()) this->gps = instrument["gps"];

  if (instrument["sensor_id"].success()) s.setId(instrument["sensor_id"]);
  if (instrument["configure_id"].success()) c.setId(instrument["configure_id"]);
}

void rFatFile::setJsonValues(JsonObject& fat_file)
{
  if (fat_file["instrument_id"].success()) this->instrument_id = fat_file["instrument_id"];

  if (fat_file["action"].success()) this->action = fat_file["action"];

  if (fat_file["name"].success()) strncpy(this->name, fat_file["name"], NAME_SIZE);
  if (fat_file["size"].success()) this->size = fat_file["size"];
  if (fat_file["ts"].success()) this->ts = fat_file["ts"];
}

void rEvent::setJsonValues(JsonObject& event)
{
  if (event["ppv"].success()) this->ppv = event["ppv"];
  if (event["rtc"].success()) this->rtc = event["rtc"];

  if (event["event_file_id"].success()) this->event_file_id = event["event_file_id"];

  if (event["sensor_id"].success()) this->sensor_id = event["sensor_id"];
}

void rEventFile::setJsonValues(JsonObject& event_file)
{
  if (event_file["event_id"].success()) this->event_id = event_file["event_id"];
}

void rMessage::setJsonValues(JsonObject& message)
{
  if (message["instrument_id"].success()) this->instrument_id = message["instrument_id"];
  if (message["from"].success()) strncpy(this->from, message["from"], PHONE_SIZE);
  if (message["date"].success()) strncpy(this->date, message["date"], DATE_SIZE);
  if (message["sms"].success()) strncpy(this->sms, message["sms"], SMS_SIZE);
}
//------------------------------------------------------------------------------------------------
// otras:
boolean rInstrument::read()
{
  uint32_t address = 0;

  uint8_t cfg_version = 0;
  EEPROM.get(address, cfg_version);
  address += sizeof(cfg_version);

  if (CFG_VERSION != cfg_version) return false;

  // read
#if 1
  rInstrument aux;
  EEPROM.get(address, aux);

  address += sizeof(aux);

  this->set(aux);

  //memcpy(this, &aux, sizeof(*this));
#else
  EEPROM.get(address, (rInstrument &)(*this));

  this->fixValues();
#endif

  return true;
}

boolean rInstrument::write()
{
  uint32_t address = 0;
  EEPROM.write(address++, CFG_VERSION);

  // write:
  EEPROM.put(address, *this);
  address += sizeof(*this);

  return true;
}

void rInstrument::show()
{
  Serial.print(F("version:")); Serial.println(CFG_VERSION);
  Serial.print(F("sname:")); Serial.println(s.getName());
  Serial.print(F("cname:")); Serial.println(c.getName());

  Serial.print(F("sensitivity:")); Serial.println(s.getSensitivity());
  Serial.print(F("gain:")); Serial.println(c.getGain());
  Serial.print(F("average:")); Serial.println(c.getHardwareAverage());
  Serial.print(F("tick_time:")); Serial.println(c.getTickTime());
  Serial.print(F("time_type:")); Serial.println(c.getTimeType());
  Serial.print(F("time_begin:")); Serial.println(c.getTimeBegin());
  Serial.print(F("time_end:")); Serial.println(c.getTimeEnd());
  Serial.print(F("gprs:")); Serial.println(getGprs());
  Serial.print(F("gps:")); Serial.println(getGps());
  Serial.print(F("trigger_level:")); Serial.println(c.getTriggerLevel());
  Serial.print(F("trigger_time:")); Serial.println(c.getTriggerTime());
  Serial.print(F("send_trigger_time:")); Serial.println(c.getSendTriggerTime());
  Serial.print(F("time_zone_offset:")); Serial.println(getTimeZoneOffset());
  Serial.print(F("F_BUS:")); Serial.println(F_BUS);
  Serial.println();
}

void rInstrument::show(uint8_t* const& buffer, size_t const& buffer_size)
{
  Serial.println();
  Serial.print(F("version:")); Serial.println(CFG_VERSION);

  s.show(buffer, buffer_size);
  c.show(buffer, buffer_size);

  StaticSharedJsonBuffer jsonBuffer(buffer, buffer_size);
  JsonObject& root = jsonBuffer.createObject();
  builtJson(root, false);
  root.prettyPrintTo(Serial);
  Serial.println();
  Serial.print(F("F_BUS:")); Serial.println(F_BUS);
}
