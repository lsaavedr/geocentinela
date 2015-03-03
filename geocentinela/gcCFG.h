#include <SdFat.h>
#include <Time.h>

#ifndef GC_CFG_H
#define GC_CFG_H

#define FILE_NAME_SIZE 13

#define MIN_TICK_TIME_USEG 100

class gcCFG {
public:
  uint8_t gain = 0; //  0->1, 1->2, 2->4, 3->8, 4->16, 5->32, 6->6, 7->16
  uint8_t average = 3; //  0->4, 1->8, 2->16, 3->32
  uint32_t tick_time_useg = 250; // 4ksps

  uint8_t time_type = 0; // testing
  uint32_t time_begin_seg = 5;
  uint32_t time_end_seg = 10;

  uint16_t adc_buffer_size = 2048; // 2048*2
  uint16_t adc_buffer_size_bytes = adc_buffer_size * sizeof(uint16_t);
  uint16_t adc_buffer_hash = adc_buffer_size - 1;

  uint16_t sd_buffer_size = 4096; // 4096*2
  uint16_t sd_buffer_size_bytes = sd_buffer_size * sizeof(uint16_t);

  boolean gps = true;

  static void gcCmd(uint8_t* cmd, uint8_t n)
  {
    uint8_t head[9] = { '\xaa', '\xaa', '\xaa',
                        '\xff', '\xff', '\xff',
                        '\x00', '\x00', '\x00'};
    Serial.write(head, 9);
    Serial.write(cmd, n);
  }

  static void gcPrintln(const char *log_string)
  {
    uint8_t cmd[1] = { 't' };
    gcCmd(cmd, 1);

    Serial.println(log_string);
  }

  gcCFG(const char* file_name, void (*log)(const char*))
  {
    strcpy_P(this->file_name, file_name);
    this->log = log;
  }

  gcCFG(const char* file_name)
  {
    strcpy_P(this->file_name, file_name);
    this->log = gcPrintln;
  }

  void toggle_gps() {
    gps = !gps;
  }

  bool write();
  bool read();
  void print();

  void set_gain(uint8_t gain)
  {
    if (gain < 8) this->gain = gain;
  }

  void set_average(uint8_t average)
  {
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

  void set_tick_time_useg(uint16_t tick_time_useg)
  {
    if (tick_time_useg >= MIN_TICK_TIME_USEG) this->tick_time_useg = tick_time_useg;
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

  void set_adc_buffer_size(uint16_t adc_buffer_size)
  {
    this->adc_buffer_size = adc_buffer_size;
    this->adc_buffer_hash = adc_buffer_size - 1;
    this->adc_buffer_size_bytes = adc_buffer_size * sizeof(uint16_t);
  }

private:
  SdFile file_cfg;
  char file_name[FILE_NAME_SIZE];

  void (*log)(const char*);
};

#endif // GC_CFG_H
