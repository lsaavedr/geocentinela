#include <stdio.h>
#include <EEPROM.h>

#include "gcCFG.h"

boolean gcCFG::writeEEPROM()
{
  uint32_t address = 0;
  EEPROM.write(address++, CFG_VERSION);

  // write float sensitivity:
  EEPROM.put(address, sensitivity);
  address += sizeof(float);

  // write uint8_t gain:
  EEPROM.put(address, gain);
  address += sizeof(uint8_t);

  // write uint8_t average:
  EEPROM.put(address, average);
  address += sizeof(uint8_t);

  // write uint32_t tick_time_useg:
  EEPROM.put(address, tick_time_useg);
  address += sizeof(uint32_t);

  // write uint8_t time_type:
  EEPROM.put(address, time_type);
  address += sizeof(uint8_t);

  // write uint32_t time_begin_seg:
  EEPROM.put(address, time_begin_seg);
  address += sizeof(uint32_t);

  // write uint32_t time_end_seg:
  EEPROM.put(address, time_end_seg);
  address += sizeof(uint32_t);

  // write gps:
  EEPROM.put(address, gps);
  address += sizeof(uint8_t);

  // write uint16_t trigger_level:
  EEPROM.put(address, trigger_level);
  address += sizeof(uint16_t);

  // write uint32_t trigger_time_number:
  EEPROM.put(address, trigger_time_number);
  address += sizeof(uint32_t);

  // write uint32_t ppv_send_time:
  EEPROM.put(address, ppv_send_time);
  address += sizeof(uint32_t);

  return true;
}

boolean gcCFG::write()
{
  if (file_cfg.open(file_name, O_CREAT | O_TRUNC | O_WRITE)) {
    file_cfg.timestamp(T_CREATE, year(), month(), day(), hour(), minute(), second());
    file_cfg.timestamp(T_WRITE, year(), month(), day(), hour(), minute(), second());
    file_cfg.timestamp(T_ACCESS, year(), month(), day(), hour(), minute(), second());

    // write float sensitivity:
    file_cfg.write((uint8_t*)&sensitivity, sizeof(float));

    // write uint8_t (gain << 4) | average:
    file_cfg.write((gain << 4) | average);

    // write uint32_t tick_time_useg:
    file_cfg.write((uint8_t*)&tick_time_useg, sizeof(uint32_t));

    // write uint8_t time_type:
    file_cfg.write((uint8_t*)&time_type, sizeof(uint8_t));

    // write uint32_t time_begin_seg:
    file_cfg.write((uint8_t*)&time_begin_seg, sizeof(uint32_t));

    // write uint32_t time_end_seg:
    file_cfg.write((uint8_t*)&time_end_seg, sizeof(uint32_t));

    // write gps:
    file_cfg.write((uint8_t*)&gps, sizeof(uint8_t));

    // write uint16_t trigger_level:
    file_cfg.write((uint8_t*)&trigger_level, sizeof(uint16_t));

    // write uint32_t trigger_time_number:
    file_cfg.write((uint8_t*)&trigger_time_number, sizeof(uint32_t));

    // write uint32_t ppv_send_time:
    file_cfg.write((uint8_t*)&ppv_send_time, sizeof(uint32_t));

    if (!file_cfg.close()) {
      log(PSTR("gcCFG:write: file close error"));
      log(file_name);

      writeEEPROM();
      return false;
    }
  } else {
    log(PSTR("gcCFG:write: file open error"));
    log(file_name);

    writeEEPROM();
    return false;
  }

  return writeEEPROM();
}

boolean gcCFG::readEEPROM()
{
  uint32_t address = 0;

  uint8_t cfg_version = 0;
  EEPROM.get(address, cfg_version);
  address += sizeof(uint8_t);

  if (cfg_version == CFG_VERSION) {
    // read float sensitivity:
    EEPROM.get(address, sensitivity);
    address += sizeof(float);

    // read uint8_t gain:
    EEPROM.get(address, gain);
    address += sizeof(uint8_t);

    // read uint8_t average:
    EEPROM.get(address, average);
    address += sizeof(uint8_t);

    // read uint32_t tick_time_useg:
    EEPROM.get(address, tick_time_useg);
    address += sizeof(uint32_t);

    // read uint8_t time_type:
    EEPROM.get(address, time_type);
    address += sizeof(uint8_t);

    // read uint32_t time_begin_seg:
    EEPROM.get(address, time_begin_seg);
    address += sizeof(uint32_t);

    // read uint32_t time_end_seg:
    EEPROM.get(address, time_end_seg);
    address += sizeof(uint32_t);

    // read uint8_t gps:
    EEPROM.get(address, gps);
    address += sizeof(uint8_t);

    // read uint16_t trigger_level:
    EEPROM.get(address, trigger_level);
    address += sizeof(uint16_t);

    // read uint32_t trigger_time_number:
    EEPROM.get(address, trigger_time_number);
    address += sizeof(uint32_t);

    // read uint32_t ppv_send_time:
    EEPROM.get(address, ppv_send_time);
    address += sizeof(uint32_t);

    return true;
  }

  return false;
}

boolean gcCFG::read()
{
  uint32_t address = 0;
  address += sizeof(uint8_t); // cfg_version

  if (file_cfg.open(file_name, O_READ)) {
    // read float sensitivity:
    if (file_cfg.read(&sensitivity, sizeof(float)) != sizeof(float)) {
      log(PSTR("gcCFG:read: sensitivity"));
      if (!file_cfg.close()) {
        log(PSTR("gcCFG:read: file close error"));
        log(file_name);
      }
      return readEEPROM();
    } else {
      // write float sensitivity:
      EEPROM.put(address, sensitivity);
      address += sizeof(float);
    }

    // read uint8_t gain and average:
    if (file_cfg.read(&gain, sizeof(uint8_t)) != sizeof(uint8_t)) {
      log(PSTR("gcCFG:read: gain"));
      if (!file_cfg.close()) {
        log(PSTR("gcCFG:read: file close error"));
        log(file_name);
      }
      return readEEPROM();
    } else {
      average = gain & 0xf;
      gain = gain >> 4;

      // write uint8_t gain:
      EEPROM.put(address, gain);
      address += sizeof(uint8_t);

      // write uint8_t average:
      EEPROM.put(address, average);
      address += sizeof(uint8_t);
    }

    // read uint32_t tick_time_useg:
    if (file_cfg.read(&tick_time_useg, sizeof(uint32_t)) != sizeof(uint32_t)) {
      log(PSTR("gcCFG:read: tick_time_useg"));
      if (!file_cfg.close()) {
        log(PSTR("gcCFG:read: file close error"));
        log(file_name);
      }
      return readEEPROM();
    } else {
      // write uint32_t tick_time_useg:
      EEPROM.put(address, tick_time_useg);
      address += sizeof(uint32_t);
    }

    // read uint8_t time_type:
    if (file_cfg.read(&time_type, sizeof(uint8_t)) != sizeof(uint8_t)) {
      log(PSTR("gcCFG:read: time_type"));
      if (!file_cfg.close()) {
        log(PSTR("gcCFG:read: file close error"));
        log(file_name);
      }
      return readEEPROM();
    } else {
      // write uint8_t time_type:
      EEPROM.put(address, time_type);
      address += sizeof(uint8_t);
    }

    // read uint32_t time_begin_seg:
    if (file_cfg.read(&time_begin_seg, sizeof(uint32_t)) != sizeof(uint32_t)) {
      log(PSTR("gcCFG:read: time_begin_seg"));
      if (!file_cfg.close()) {
        log(PSTR("gcCFG:read: file close error"));
        log(file_name);
      }
      return readEEPROM();
    } else {
      // write uint32_t time_begin_seg:
      EEPROM.put(address, time_begin_seg);
      address += sizeof(uint32_t);
    }

    // read uint32_t time_end_seg:
    if (file_cfg.read(&time_end_seg, sizeof(uint32_t)) != sizeof(uint32_t)) {
      log(PSTR("gcCFG:read: time_end_seg"));
      if (!file_cfg.close()) {
        log(PSTR("gcCFG:read: file close error"));
        log(file_name);
      }
      return readEEPROM();
    } else {
      // write uint32_t time_end_seg:
      EEPROM.put(address, time_end_seg);
      address += sizeof(uint32_t);
    }

    // read gps:
    if (file_cfg.read(&gps, sizeof(uint8_t)) != sizeof(uint8_t)) {
      log(PSTR("gcCFG:read: gps"));
      if (!file_cfg.close()) {
        log(PSTR("gcCFG:read: file close error"));
        log(file_name);
      }
      return readEEPROM();
    } else {
      // write gps:
      EEPROM.put(address, gps);
      address += sizeof(uint8_t);
    }

    // read uint16_t trigger_level:
    if (file_cfg.read(&trigger_level, sizeof(uint16_t)) != sizeof(uint16_t)) {
      log(PSTR("gcCFG:read: trigger_level"));
      if (!file_cfg.close()) {
        log(PSTR("gcCFG:read: file close error"));
        log(file_name);
      }
      return readEEPROM();
    } else {
      // write uint16_t trigger_level:
      EEPROM.put(address, trigger_level);
      address += sizeof(uint16_t);
    }

    // read uint32_t trigger_time_number:
    if (file_cfg.read(&trigger_time_number, sizeof(uint32_t)) != sizeof(uint32_t)) {
      log(PSTR("gcCFG:read: trigger_time_number"));
      if (!file_cfg.close()) {
        log(PSTR("gcCFG:read: file close error"));
        log(file_name);
      }
      return readEEPROM();
    } else {
      // write uint32_t trigger_time_number:
      EEPROM.put(address, trigger_time_number);
      address += sizeof(uint32_t);
    }

    // read uint32_t ppv_send_time:
    if (file_cfg.read(&ppv_send_time, sizeof(uint32_t)) != sizeof(uint32_t)) {
      log(PSTR("gcCFG:read: ppv_send_time"));
      if (!file_cfg.close()) {
        log(PSTR("gcCFG:read: file close error"));
        log(file_name);
      }
      return readEEPROM();
    } else {
      // write uint32_t ppv_send_time:
      EEPROM.put(address, ppv_send_time);
      address += sizeof(uint32_t);
    }

    if (!file_cfg.close()) {
      log(PSTR("gcCFG:read: file close error"));
      log(file_name);
      return readEEPROM();
    }

    file_cfg.timestamp(T_ACCESS, year(), month(), day(), hour(), minute(), second());
  } else {
    log(PSTR("gcCFG:read: file open error"));
    log(file_name);

    return readEEPROM();
  }

  return writeEEPROM();
}

void gcCFG::print()
{
  Serial.print(F("file_name:")); Serial.println(file_name);
  Serial.print(F("sensitivity:")); Serial.println(sensitivity);
  Serial.print(F("gain:")); Serial.println(gain);
  Serial.print(F("average:")); Serial.println(average);
  Serial.print(F("tick_time_useg:")); Serial.println(tick_time_useg);
  Serial.print(F("time_type:")); Serial.println(time_type);
  Serial.print(F("time_begin_seg:")); Serial.println(time_begin_seg);
  Serial.print(F("time_end_seg:")); Serial.println(time_end_seg);
  Serial.print(F("gps:")); Serial.println(gps);
  Serial.print(F("trigger_level:")); Serial.println(trigger_level);
  Serial.print(F("trigger_time_number:")); Serial.println(trigger_time_number);
  Serial.print(F("ppv_send_time:")); Serial.println(ppv_send_time);
  Serial.print(F("F_BUS:")); Serial.println(F_BUS);
  Serial.println();
}
