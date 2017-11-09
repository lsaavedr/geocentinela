#ifndef M95_H
#define M95_H

#if defined(ARDUINO) and ARDUINO >= 100
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif
#include <SdFat.h>

#include "gcCFG.h"
//--------------------------------------------------
#define M95_RX 8
#define M95_TX 7
#define M95_W1 21
#define M95_BD 115200
//--------------------------------------------------
#define T1900 2208988800UL

#define SWAP_32(x) ( ((x)<<24 & 0xFF000000UL) | \
                     ((x)<< 8 & 0x00FF0000UL) | \
                     ((x)>> 8 & 0x0000FF00UL) | \
                     ((x)>>24 & 0x000000FFUL) )
#define SWAP_16(x) ( ((x)<< 8 & 0xFF00UL) | \
                     ((x)>> 8 & 0x00FFUL) )

#define htonl(x) ( ((x)<<24 & 0xFF000000UL) | \
                   ((x)<< 8 & 0x00FF0000UL) | \
                   ((x)>> 8 & 0x0000FF00UL) | \
                   ((x)>>24 & 0x000000FFUL) )
#define ntohl(x) htonl(x)

#define NTPN 5
const char ntp_servers[NTPN][29] = {
  PSTR("3.cl.pool.ntp.org"),
  PSTR("0.south-america.pool.ntp.org"),
  PSTR("1.south-america.pool.ntp.org"),
  PSTR("2.south-america.pool.ntp.org"),
  PSTR("3.south-america.pool.ntp.org")
};
#define NTP_NPKGS 8
#define NTP_MAXDIST 1.5
//--------------------------------------------------
#define M95_RESP_POWER_DOWN 1000
#define M95_RESP_RDY 1001
#define M95_RESP_CFUN_1 1002
#define M95_RESP_CFUN_X 1003
#define M95_RESP_CPIN_READY 1004
#define M95_RESP_CPIN_X 1005
#define M95_RESP_CALL_READY 1006
#define M95_RESP_CREG_X_1 1007
#define M95_RESP_CREG_X 1008

#define M95_RESP_COPS_CHILE_ENTEL 24
#define M95_RESP_COPS_CHILE_CLARO 25
#define M95_RESP_COPS_CHILE_MOVISTAR 26
#define M95_RESP_COPS_PERU_MOVISTAR 126
#define M95_RESP_COPS_PERU_CLARO 125
#define M95_RESP_COPS_PERU_ENTEL 124
#define M95_RESP_COPS_PERU_BITEL 127

#define M95_RESP_CGREG_X_1 1009
#define M95_RESP_CGREG_X 1010
#define M95_RESP_QNSTATUS_0 1011
#define M95_RESP_QNSTATUS_X 1012
#define M95_RESP_CSQ 1013

#define M95_RESP_CONNECT_OK 2000
#define M95_RESP_CONNECT_X 2001
#define M95_RESP_ALREADY_CONNECT 2002
#define M95_RESP_CLOSE_OK 2003
#define M95_RESP_PROMPT 2004
#define M95_RESP_SEND_OK 2005
#define M95_RESP_SEND_X 2006
#define M95_RESP_QIRD 2007
#define M95_RESP_QIRDI 2008
#define M95_RESP_QISACK 2009

#define M95_RESP_QCELLLOC 3000

#define M95_RESP_CPMS 4000
#define M95_RESP_CMGR 4001
#define M95_RESP_CMTI 4002

#define M95_RESP_ERROR 5001
#define M95_RESP_CLOSED 5002

#define M95_RESP_BUSY 5003
#define M95_RESP_NO_CARRIER 5004
#define M95_RESP_NO_ANSWER 5005
#define M95_RESP_NO_DIALTONE 5006

#define M95_RESP_OK 1
//--------------------------------------------------

#define SEND_DATA_MAX_SIZE 1460

class M95 {
public:
  M95(HardwareSerial& M95_IO, char* M95_string, uint16_t M95_string_size, char* M95_data, uint16_t M95_data_size):
  M95_IO(M95_IO),
  M95_string(M95_string), M95_string_size(M95_string_size),
  M95_data(M95_data), M95_data_size(M95_data_size) {}

  uint16_t const& getResp() { return resp; }
  char* const& getStringLog() { return M95_string; }
  uint16_t const& getStringLogLength() { return M95_string_size; }
  void cleanStringLog() { memset(M95_string, 0, M95_string_size); }

  char* const& getStringIO() { return M95_data; }
  uint16_t const& getStringIOLength() { return M95_data_size; }
  void cleanStringIO() { memset(M95_data, 0, M95_data_size); }

  void waitOfReaction(uint32_t const& timeout);

  boolean cfg();
  boolean powerStatus() { return isOn; }
  boolean powerUp(rInstrument& instrument);
  boolean powerDown();

  uint8_t csq(uint8_t niter);
  uint8_t call(char const* const& phone);
  boolean callWarning(rInstrument& instrument);
  boolean ntpSyncUDP(int32_t & dtsr, int16_t & dtpr, double & tdelay);
  boolean cellloc(float& longitude, float& latitude);

  uint16_t httpREST(char const* const& url, char const* const& rest, char const* const& content_type) {
    return httpREST(url, rest, content_type, true);
  }

  uint32_t httpJsonREST(char const* const& url, char const* const& rest) {
      return httpJsonREST(url, rest, true);
  }

  uint32_t httpRESTFile(char const* const& url, char const* const& rest, SdFile & ifile) {
      return httpRESTFile(url, rest, ifile, true);
  }

  boolean syncSMS(rInstrument& instrument);
  void clearSerialLine();
private:
  HardwareSerial& M95_IO;

  char* M95_string = 0;
  uint16_t M95_string_size = 0;
  char* M95_data = 0;
  uint16_t M95_data_size = 0;
  uint16_t resp = 0;

  boolean isOk = false;
  boolean isOn = false;

  uint16_t getResp(char const* const& respStr);

  uint16_t httpREST(char const* const& url, char const* const& rest, char const* const& content_type, boolean checkOn);

  uint32_t httpJsonREST(char const* const& url, char const* const& rest, boolean checkOn) {
    return httpREST(url, rest, PSTR("application/json; charset=utf-8"), checkOn);
  }
  uint32_t httpRESTFile(char const* const& url, char const* const& rest, SdFile & ifile, boolean checkOn);

  boolean qiclose(boolean checkOn);
  void strtoASCII(char * str);

  boolean sendData(uint16_t data_size);
  uint16_t extractResponse(boolean checkOn);
};

#endif // M95_H
