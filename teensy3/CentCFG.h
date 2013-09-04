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
#include <SdFat.h>

#ifndef CENT_CFG_H
#define CENT_CFG_H

#define FILE_NAME_SIZE 13

#define MAX_NCH 10
#define MIN_TICK_TIME_USEC 100

class CentCFG {
public:
  static const uint8_t channel2sc1a[MAX_NCH]; // = { 5, 14, 8, 9, 13, 12, 6, 7, 15, 4 };

  uint8_t nch = 3;
  uint8_t average = 2; //  0->4, 1->8, 2->16, 3->32
  uint32_t tick_time_usec = 125; // 8ksps

  uint8_t time_type = 0;
  uint32_t time_begin_seg = 30;
  uint32_t time_end_seg = 20;

  uint16_t adc_buffer_size = 1024; // 1024*2
  uint16_t adc_buffer_size_bytes = adc_buffer_size * sizeof(uint16_t);
  uint16_t adc_buffer_hash = adc_buffer_size - 1;
  uint16_t sd_buffer_size = 2048; // 2048*2
  uint16_t sd_buffer_size_bytes = sd_buffer_size * sizeof(uint16_t);

  CentCFG(const char* file_name, void (*log)(const char*)) {
    strcpy_P(this->file_name, file_name);
    this->log = log;
  }

  bool write();
  bool read();
  void print();

  void set_adc_buffer_size(uint16_t adc_buffer_size)
  {
    this->adc_buffer_size = adc_buffer_size;
    this->adc_buffer_hash = adc_buffer_size - 1;
    this->adc_buffer_size_bytes = adc_buffer_size * sizeof(uint16_t);
  }

  void set_average(uint8_t average) {
    switch (average) {
    case 4:
      this->average = 0;
      break;
    case 8:
      this->average = 1;
      break;
    case 16:
      this->average = 2;
      break;
    case 32:
      this->average = 3;
      break;
    default:
      this->average = 4;
      break;
    }
  }

  void set_nch(uint8_t nch)
  {
    if (0 < nch && nch < MAX_NCH) this->nch = nch;
  }

  void set_tick_time_usec(uint16_t tick_time_usec)
  {
    if (tick_time_usec >= MIN_TICK_TIME_USEC) this->tick_time_usec = tick_time_usec;
  }

  void set_time_type(uint8_t time_type)
  {
    this->time_type = time_type;
  }

  void set_time_begin(uint32_t time_begin_seg)
  {
    this->time_begin_seg = time_begin_seg;
  }

  void set_time_end(uint32_t time_end_seg)
  {
    this->time_end_seg = time_end_seg;
  }

  void set_sd_buffer_size(uint16_t sd_buffer_size)
  {
    this->sd_buffer_size = sd_buffer_size;
    this->sd_buffer_size_bytes = sd_buffer_size * sizeof(uint16_t);
  }

private:
  SdFile file_cfg;
  char file_name[FILE_NAME_SIZE];

  void (*log)(const char*);
};

#endif // CENT_CFG_H
