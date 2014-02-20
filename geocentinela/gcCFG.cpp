#include <stdio.h>

#include "gcCFG.h"

bool gcCFG::write()
{
  if (file_cfg.open(file_name, O_CREAT | O_TRUNC | O_WRITE)) {
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

    // write uint16_t adc_buffer_size:
    file_cfg.write((uint8_t*)&adc_buffer_size, sizeof(uint16_t));

    // write uint16_t sd_buffer_size:
    file_cfg.write((uint8_t*)&sd_buffer_size, sizeof(uint16_t));

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
    if (file_cfg.read(&gain, 1) != 1) {
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
    if (file_cfg.read(&tick_time_useg, 4) != 4) {
      log(PSTR("GeoCentinelaCFG: tick_time_useg"));
      if (!file_cfg.close()) {
        log(PSTR("GeoCentinelaCFG: file close error"));
        log(file_name);
      }
      return false;
    }

    // read uint8_t time_type:
    if (file_cfg.read(&time_type, 1) != 1) {
      log(PSTR("GeoCentinelaCFG: time_type"));
      if (!file_cfg.close()) {
        log(PSTR("GeoCentinelaCFG: file close error"));
        log(file_name);
      }
      return false;
    }

    // read uint32_t time_begin_seg:
    if (file_cfg.read(&time_begin_seg, 4) != 4) {
      log(PSTR("GeoCentinelaCFG: time_begin_seg"));
      if (!file_cfg.close()) {
        log(PSTR("GeoCentinelaCFG: file close error"));
        log(file_name);
      }
      return false;
    }

    // read uint32_t time_end_seg:
    if (file_cfg.read(&time_end_seg, 4) != 4) {
      log(PSTR("GeoCentinelaCFG: time_end_seg"));
      if (!file_cfg.close()) {
        log(PSTR("GeoCentinelaCFG: file close error"));
        log(file_name);
      }
      return false;
    }

    // read uint16_t adc_buffer_size:
    if (file_cfg.read(&adc_buffer_size, 2) != 2) {
      log(PSTR("GeoCentinelaCFG: adc_buffer_size"));
      if (!file_cfg.close()) {
        log(PSTR("GeoCentinelaCFG: file close error"));
        log(file_name);
      }
      return false;
    }

    // read uint16_t sd_buffer_size:
    if (file_cfg.read(&sd_buffer_size, 2) != 2) {
      log(PSTR("GeoCentinelaCFG: sd_buffer_size"));
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
  Serial.print(PSTR("adc_buffer_size:")); Serial.println(adc_buffer_size);
  Serial.print(PSTR("sd_buffer_size:")); Serial.println(sd_buffer_size);
  Serial.print(PSTR("F_BUS:")); Serial.println(F_BUS);
  Serial.println();
}
