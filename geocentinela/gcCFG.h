#include <SdFat.h>
#include <Time.h>

#ifndef GC_CFG_H
#define GC_CFG_H

#define CFG_VERSION 4
#define FILE_NAME_SIZE 13

#define MIN_TICK_TIME_USEG 100

#define MAX_TRIGGER_LEVEL 0x7FFF // 1.25v
#define MAX_TRIGGER_TIME_SEG 0x3C // 60seg

class gcCFG {
public:
  float sensitivity = 1.0; // V/mm/s

  uint8_t gain = 0;              //  0->1, 1->2, 2->4, 3->8, 4->16, 5->32, 6->64, 7->128
  uint8_t average = 3;           //  0->4, 1->8, 2->16, 3->32
  uint32_t tick_time_useg = 250; // 4ksps

  uint8_t time_type = 0; // testing
  uint32_t time_begin_seg = 5;
  uint32_t time_end_seg = 10;

  uint8_t gps = false;

  uint16_t trigger_level = 0; // 100%
  uint32_t trigger_time_number = 0; // 0 segundos, 4000 == 1 segundo

  uint32_t ppv_send_time = 3600; // < 86400

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

  void set_ppv_send_time(uint32_t ppv_send_time)
  {
    if (ppv_send_time >= 86400) return;

    this->ppv_send_time = ppv_send_time;
  }

  void toggle_gps()
  {
    gps = !gps;
  }

  void set_gps(uint8_t gps)
  {
    this->gps = gps > 0;
  }

  boolean write();
  boolean read();
  void print();

  void set_sensitivity(float sensitivity)
  {
    if (sensitivity <= 0) return;

    this->sensitivity = sensitivity;
  }

  void set_gain(uint8_t gain)
  {
    if (gain < 8) this->gain = gain;
  }

  void set_average(uint8_t average)
  {
    switch (average) {
      case 4: {
        this->average = 0;
      } break;
      case 8: {
        this->average = 1;
      } break;
      case 16: {
        this->average = 2;
      } break;
      case 32: {
        this->average = 3;
      } break;
      default: {
        this->average = 4;
      } break;
    }
  }

  void set_tick_time_useg(uint16_t tick_time_useg)
  {
    if (tick_time_useg >= MIN_TICK_TIME_USEG) {
      this->trigger_time_number *= this->tick_time_useg;
      this->trigger_time_number /= tick_time_useg;

      this->tick_time_useg = tick_time_useg;
    }
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

  void set_trigger_level(uint16_t trigger_level)
  {
    if (trigger_level <= MAX_TRIGGER_LEVEL)
      this->trigger_level = trigger_level;
    else
      this->trigger_level = MAX_TRIGGER_LEVEL;
  }

  void set_trigger_time_seg(uint32_t trigger_time_seg)
  {
    if (trigger_time_seg <= 0) trigger_time_seg = 0;
    if (trigger_time_seg > MAX_TRIGGER_TIME_SEG) trigger_time_seg = MAX_TRIGGER_TIME_SEG;

    this->trigger_time_number = (trigger_time_seg*1000000)/tick_time_useg;
  }

private:
  SdFile file_cfg;
  char file_name[FILE_NAME_SIZE];

  void (*log)(const char*);

  boolean readEEPROM();
  boolean writeEEPROM();
};

#endif // GC_CFG_H
