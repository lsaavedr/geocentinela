#include <stdio.h>

#include "gcCFG.h"

bool gcCFG::write()
{
  if (file_cfg.open(file_name, O_CREAT | O_TRUNC | O_WRITE)) {
    file_cfg.timestamp(T_CREATE, year(), month(), day(), hour(), minute(), second());
    file_cfg.timestamp(T_WRITE, year(), month(), day(), hour(), minute(), second());
    file_cfg.timestamp(T_ACCESS, year(), month(), day(), hour(), minute(), second());

    // write uint8_t (gain << 4) | average:
    file_cfg.write((gain << 4) | average);

    // write uint32_t tick_time_useg:
    file_cfg.write((uint8_t*)&tick_time_useg, sizeof(uint32_t));

    // write uint8_t time_type:
    file_cfg.write(time_type);

    // write uint32_t time_begin_seg:
    file_cfg.write((uint8_t*)&time_begin_seg, sizeof(uint32_t));

    // write uint32_t time_end_seg:
    file_cfg.write((uint8_t*)&time_end_seg, sizeof(uint32_t));

    // write gps:
    file_cfg.write((uint8_t*)&gps, sizeof(boolean));

    if (!file_cfg.close()) {
      log(PSTR("GeoCentinelaCFG: file close error"));
      return false;
    }
  } else {
    log(PSTR("GeoCentinelaCFG: file open error"));
    log(file_name);
    return false;
  }

  return true;
}

bool gcCFG::read()
{
  if (file_cfg.open(file_name, O_READ)) {
    // read uint8_t gain and average:
    if (file_cfg.read(&gain, sizeof(uint8_t)) != sizeof(uint8_t)) {
      log(PSTR("GeoCentinelaCFG: gain"));
      if (!file_cfg.close()) {
        log(PSTR("GeoCentinelaCFG: file close error"));
        log(file_name);
      }
      return false;
    } else {
      average = gain & 0xf;
      gain = gain >> 4;
    }

    // read uint32_t tick_time_useg:
    if (file_cfg.read(&tick_time_useg, sizeof(uint32_t)) != sizeof(uint32_t)) {
      log(PSTR("GeoCentinelaCFG: tick_time_useg"));
      if (!file_cfg.close()) {
        log(PSTR("GeoCentinelaCFG: file close error"));
        log(file_name);
      }
      return false;
    }

    // read uint8_t time_type:
    if (file_cfg.read(&time_type, sizeof(uint8_t)) != sizeof(uint8_t)) {
      log(PSTR("GeoCentinelaCFG: time_type"));
      if (!file_cfg.close()) {
        log(PSTR("GeoCentinelaCFG: file close error"));
        log(file_name);
      }
      return false;
    }

    // read uint32_t time_begin_seg:
    if (file_cfg.read(&time_begin_seg, sizeof(uint32_t)) != sizeof(uint32_t)) {
      log(PSTR("GeoCentinelaCFG: time_begin_seg"));
      if (!file_cfg.close()) {
        log(PSTR("GeoCentinelaCFG: file close error"));
        log(file_name);
      }
      return false;
    }

    // read uint32_t time_end_seg:
    if (file_cfg.read(&time_end_seg, sizeof(uint32_t)) != sizeof(uint32_t)) {
      log(PSTR("GeoCentinelaCFG: time_end_seg"));
      if (!file_cfg.close()) {
        log(PSTR("GeoCentinelaCFG: file close error"));
        log(file_name);
      }
      return false;
    }

    // read gps:
    if (file_cfg.read(&gps, sizeof(boolean)) != sizeof(boolean)) {
      log(PSTR("GeoCentinelaCFG: gps"));
      if (!file_cfg.close()) {
        log(PSTR("GeoCentinelaCFG: file close error"));
        log(file_name);
      }
      return false;
    }

    if (!file_cfg.close()) {
      log(PSTR("GeoCentinelaCFG: file close error"));
      log(file_name);
      return false;
    }

    file_cfg.timestamp(T_ACCESS, year(), month(), day(), hour(), minute(), second());
  } else {
    log(PSTR("GeoCentinelaCFG: file open error"));
    log(file_name);
    return false;
  }

  return true;
}

void gcCFG::print()
{
  Serial.print(PSTR("file_name:")); Serial.println(file_name);
  Serial.print(PSTR("gain:")); Serial.println(gain);
  Serial.print(PSTR("average:")); Serial.println(average);
  Serial.print(PSTR("tick_time_useg:")); Serial.println(tick_time_useg);
  Serial.print(PSTR("time_type:")); Serial.println(time_type);
  Serial.print(PSTR("time_begin_seg:")); Serial.println(time_begin_seg);
  Serial.print(PSTR("time_end_seg:")); Serial.println(time_end_seg);
  Serial.print(PSTR("gps:")); Serial.println(gps);
  Serial.print(PSTR("F_BUS:")); Serial.println(F_BUS);
  Serial.println();
}

