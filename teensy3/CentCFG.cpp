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
#include <stdio.h>

#include "CentCFG.h"

//const uint8_t CentCFG::channel2sc1a[MAX_NCH] = { 5, 14, 8, 9, 13, 12, 6, 7, 15, 4 };
const uint8_t CentCFG::channel2sc1a[MAX_NCH] = { 5, 14, 8, 9, 13, 12, 15, 6, 7, 4 };

bool CentCFG::write()
{
  if (file_cfg.open(file_name, O_CREAT | O_TRUNC | O_WRITE)) {
    // write uint8_t (nch << 4) | average:
    file_cfg.write((nch << 4) | average);

    // write uint32_t tick_time_usec:
    file_cfg.write((uint8_t*)&tick_time_usec, sizeof(uint32_t));

    // write uint32_t time_max_msec:
    file_cfg.write((uint8_t*)&time_max_msec, sizeof(uint32_t));

    // write uint16_t adc_buffer_size:
    file_cfg.write((uint8_t*)&adc_buffer_size, sizeof(uint16_t));

    // write uint16_t sd_buffer_size:
    file_cfg.write((uint8_t*)&sd_buffer_size, sizeof(uint16_t));

    if (!file_cfg.close()) {
      log(PSTR("CentCFG: file close error"));
      return false;
    }
  } else {
    log(PSTR("CentCFG: file open error"));
    log(file_name);
    return false;
  }

  return true;
}

bool CentCFG::read()
{
  if (file_cfg.open(file_name, O_READ)) {
    // read uint8_t nch:
    if (file_cfg.read(&nch, 1) != 1) {
      log(PSTR("CentCFG: nch"));
      if (!file_cfg.close()) {
        log(PSTR("CentCFG: file close error"));
        log(file_name);
      }
      return false;
    } else {
      average = nch & 0xf;
      nch = nch >> 4;
    }

    // read uint32_t tick_time_usec:
    if (file_cfg.read(&tick_time_usec, 4) != 4) {
      log(PSTR("CentCFG: tick_time_usec"));
      if (!file_cfg.close()) {
        log(PSTR("CentCFG: file close error"));
        log(file_name);
      }
      return false;
    }

    // read uint32_t time_max_msec:
    if (file_cfg.read(&time_max_msec, 4) != 4) {
      log(PSTR("CentCFG: time_max_msec"));
      if (!file_cfg.close()) {
        log(PSTR("CentCFG: file close error"));
        log(file_name);
      }
      return false;
    }

    // read uint16_t adc_buffer_size:
    if (file_cfg.read(&adc_buffer_size, 2) != 2) {
      log(PSTR("CentCFG: adc_buffer_size"));
      if (!file_cfg.close()) {
        log(PSTR("CentCFG: file close error"));
        log(file_name);
      }
      return false;
    }

    // read uint16_t sd_buffer_size:
    if (file_cfg.read(&sd_buffer_size, 2) != 2) {
      log(PSTR("CentCFG: sd_buffer_size"));
      if (!file_cfg.close()) {
        log(PSTR("CentCFG: file close error"));
        log(file_name);
      }
      return false;
    }

    if (!file_cfg.close()) {
      log(PSTR("CentCFG: file close error"));
      log(file_name);
      return false;
    }
  } else {
    log(PSTR("CentCFG: file open error"));
    log(file_name);
    return false;
  }

  return true;
}

void CentCFG::print()
{
  Serial.print(PSTR("file_name:")); Serial.println(file_name);
  Serial.print(PSTR("nch:")); Serial.println(nch);
  Serial.print(PSTR("average:")); Serial.println(average);
  Serial.print(PSTR("tick_time_usec:")); Serial.println(tick_time_usec);
  Serial.print(PSTR("time_max_msec:")); Serial.println(time_max_msec);
  Serial.print(PSTR("adc_buffer_size:")); Serial.println(adc_buffer_size);
  Serial.print(PSTR("sd_buffer_size:")); Serial.println(sd_buffer_size);
  Serial.print(PSTR("F_BUS:")); Serial.println(F_BUS);
  Serial.println();
}
