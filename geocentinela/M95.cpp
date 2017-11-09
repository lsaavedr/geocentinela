#include "M95.h"

#define DEBUG_IO Serial

void M95::waitOfReaction(uint32_t const& timeout)
{
  uint16_t index = 0;
  uint8_t inByte = 0;
  char WS = 0;

  M95_string[0] = 0;

  //----- wait of the first character for "timeout" ms
  M95_IO.setTimeout(timeout);
  inByte = M95_IO.readBytes(&WS, 1);

  //----- wait of further characters until a pause of 30 ms occures
  while(inByte > 0 and index < M95_string_size)
  {
    M95_string[index++] = WS;
    M95_string[index] = 0;

    M95_IO.setTimeout(30);
    inByte = M95_IO.readBytes(&WS, 1);
  }

#ifdef DEBUG_IO
  DEBUG_IO.println(M95_string);
#endif

  resp = getResp(M95_string);
}

uint16_t M95::getResp(char const* const& respStr)
{
    //----- analyse the reaction of the mobile module
  if(strstr(respStr, "NORMAL POWER DOWN"))     { return M95_RESP_POWER_DOWN; }
  if(strstr(respStr, "RDY"))                   { return M95_RESP_RDY; }
  if(strstr(respStr, "+CFUN: 1"))              { return M95_RESP_CFUN_1; }
  if(strstr(respStr, "+CFUN: "))               { return M95_RESP_CFUN_X; }
  if(strstr(respStr, "+CPIN: READY"))          { return M95_RESP_CPIN_READY; }
  if(strstr(respStr, "+CPIN: "))               { return M95_RESP_CPIN_X; }
  if(strstr(respStr, "Call Ready"))            { return M95_RESP_CALL_READY; }
  if(strstr(respStr, "+CREG: ")) {
    if(strstr(respStr, ": 0,1"))               { return M95_RESP_CREG_X_1; }
    if(strstr(respStr, ": 1,1"))               { return M95_RESP_CREG_X_1; }
    if(strstr(respStr, ": 2,1"))               { return M95_RESP_CREG_X_1; }
                                                  { return M95_RESP_CREG_X; }
  }
  if(strstr(respStr, "+COPS:")) {
    if(strstr(respStr, "\"73001\""))           { return M95_RESP_COPS_CHILE_ENTEL; }
    if(strstr(respStr, "\"73010\""))           { return M95_RESP_COPS_CHILE_ENTEL; }
    if(strstr(respStr, "\"73002\""))           { return M95_RESP_COPS_CHILE_MOVISTAR; }
    if(strstr(respStr, "\"73003\""))           { return M95_RESP_COPS_CHILE_CLARO; } // Chile: Claro/Smartcom/Telmex
    if(strstr(respStr, "\"71606\""))           { return M95_RESP_COPS_PERU_MOVISTAR; }
    if(strstr(respStr, "\"71610\""))           { return M95_RESP_COPS_PERU_CLARO; } // Peru: Claro/Telmex
    if(strstr(respStr, "\"71617\""))           { return M95_RESP_COPS_PERU_ENTEL; }
    if(strstr(respStr, "\"71615\""))           { return M95_RESP_COPS_PERU_BITEL; }

    if(strstr(respStr, "ENTEL PCS"))           { return M95_RESP_COPS_CHILE_ENTEL; }
    if(strstr(respStr, "CLARO CHILE"))         { return M95_RESP_COPS_CHILE_CLARO; }
    if(strstr(respStr, "Movistar Peru"))       { return M95_RESP_COPS_PERU_MOVISTAR; }
  }
  if(strstr(respStr, "+CGREG: ")) {
    if(strstr(respStr, ": 0,1"))               { return M95_RESP_CGREG_X_1; }
    if(strstr(respStr, ": 1,1"))               { return M95_RESP_CGREG_X_1; }
    if(strstr(respStr, ": 2,1"))               { return M95_RESP_CGREG_X_1; }
                                                  { return M95_RESP_CGREG_X; }
  }
  if(strstr(respStr, "+CSQ: "))                { return M95_RESP_CSQ; }

  if(strstr(respStr, "+QNSTATUS: 0"))          { return M95_RESP_QNSTATUS_0; }
  if(strstr(respStr, "+QNSTATUS: "))           { return M95_RESP_QNSTATUS_X; }

  if(strstr(respStr, "CONNECT OK"))            { return M95_RESP_CONNECT_OK; }
  if(strstr(respStr, "CONNECT "))              { return M95_RESP_CONNECT_X; }
  if(strstr(respStr, "ALREADY CONNECT"))       { return M95_RESP_ALREADY_CONNECT; }
  if(strstr(respStr, "CLOSE OK"))              { return M95_RESP_CLOSE_OK; }
  if(strstr(respStr, "SEND OK"))               { return M95_RESP_SEND_OK; }
  if(strstr(respStr, "SEND "))                 { return M95_RESP_SEND_X; }

  if(strstr(respStr, "+QIRD:"))                { return M95_RESP_QIRD; }

  if(strstr(respStr, "CLOSED\r\n"))            { return M95_RESP_CLOSED; }

  if(strstr(respStr, "+QIRDI:"))               { return M95_RESP_QIRDI; }
  if(strstr(respStr, "+QISACK: "))             { return M95_RESP_QISACK; }

  if(strstr(respStr, "\n> "))                  { return M95_RESP_PROMPT; }

  if(strstr(respStr, "+QCELLLOC: "))           { return M95_RESP_QCELLLOC; }

  if(strstr(respStr, "+CPMS:"))                { return M95_RESP_CPMS; }
  if(strstr(respStr, "+CMGR:"))                { return M95_RESP_CMGR; }
  if(strstr(respStr, "+CMTI:"))                { return M95_RESP_CMTI; }

  if(strstr(respStr, "SIM PIN\r\n"))           { return 2; }
  if(strstr(respStr, "READY\r\n"))             { return 3; }
  if(strstr(respStr, "0,1\r\n"))               { return 4; }
  if(strstr(respStr, "0,5\r\n"))               { return 4; }
  if(strstr(respStr, "+CGATT: 1\r\n"))         { return 7; }
  if(strstr(respStr, "IP INITIAL\r\n"))        { return 8; }
  if(strstr(respStr, "IP STATUS\r\n"))         { return 8; }
  if(strstr(respStr, "IP CLOSE\r\n"))          { return 8; }
  if(strstr(respStr, "RING\r\n"))              { return 11; }
  if(strstr(respStr, "+QPING:"))               { return 12; }
  if(strstr(respStr, "OK\r\n\r\nCONNECT\r\n")) { return 14; }
  if(strstr(respStr, "+QSMTPBODY:"))           { return 15; }
  if(strstr(respStr, "+QSMTPPUT: 0"))          { return 16; }
  if(strstr(respStr, ":0\r\n"))                { return 17; }
  if(strstr(respStr, "+QFTPGET:"))             { return 18; }

  if(strstr(respStr, "\r\nCONNECT\r\n"))       { return 27; }

  if(strstr(respStr, "+QNTP: 0"))              { return 100; }
  if(strstr(respStr, "+CCLK:"))                { return 101; }

  if(strstr(respStr, "OK\r\n"))                { return M95_RESP_OK; }
  if(strstr(respStr, "ERROR\r\n"))             { return M95_RESP_ERROR; }

  if(strstr(respStr, "BUSY\r\n"))              { return M95_RESP_BUSY; }
  if(strstr(respStr, "NO CARRIER\r\n"))        { return M95_RESP_NO_CARRIER; }
  if(strstr(respStr, "NO ANSWER\r\n"))         { return M95_RESP_NO_ANSWER; }
  if(strstr(respStr, "NO DIALTONE\r\n"))       { return M95_RESP_NO_DIALTONE; }

  return 0;
}

boolean M95::cfg()
{
  if (isOk) {
    pinMode(M95_W1, OUTPUT);
    digitalWrite(M95_W1, HIGH);

    M95_IO.begin(M95_BD);

    return true;
  }

  pinMode(M95_W1, OUTPUT);
  digitalWrite(M95_W1, HIGH);

  uint8_t rst_cnt = 0;
  uint8_t com_cnt = 0;
  boolean comOk = false;

m95_rst:
  if (com_cnt++ >= 3) {
    if (rst_cnt++ >= 1) return isOk;

    digitalWrite(M95_W1, LOW);
    delay(1000);

    Serial.print("reset serial BD:");
    for (uint16_t bd = 1200; bd <= 115200; bd*=2) {
      M95_IO.begin(bd);

      // clear serial line
      while (M95_IO.available()) M95_IO.read();

      // set bd to M95_BD
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
#define IPR_SET PSTR("AT+IPR=" STR(M95_BD) "\r")
      M95_IO.write(IPR_SET);
      while (M95_IO.available()) M95_IO.read();

      M95_IO.write("AT&W\r");
      waitOfReaction(300);
      if (M95_RESP_OK == resp) {
        Serial.print("reset serial BD:");
        Serial.println(bd);

        M95_IO.begin(M95_BD);
        delay(3000);
        M95_IO.write("AT\r");
        waitOfReaction(300);

        com_cnt = 0;
        goto m95_rst;
      }
    }

    return isOk;
  }

  M95_IO.begin(M95_BD);

  // reset M95_IO
  digitalWrite(M95_W1, HIGH);
  delay(1000);

  // clear serial line
  while (M95_IO.available()) M95_IO.read();

  digitalWrite(M95_W1, LOW);
  delay(1000);
  waitOfReaction(4000);
  if (0 == resp) waitOfReaction(4000);
  if (0 == resp) goto m95_rst;

  digitalWrite(M95_W1, HIGH);
  if (resp == M95_RESP_RDY) {
    M95_IO.write("AT+QPOWD=1\r");
    waitOfReaction(300);
    comOk = true;
  }
  if (resp == M95_RESP_POWER_DOWN) {
    comOk = true;
    isOk = true;
    isOn = false;
    M95_IO.end();
  }

  if (comOk and !isOk) goto m95_rst;

  return isOk;
}

boolean M95::powerDown()
{
  if (!isOk) return false;
  if (!isOn) return true;

  uint8_t com_cnt = 0;
  boolean comOk = false;

m95_rst:
  if (com_cnt++ >= 3) return !isOn;

  digitalWrite(M95_W1, HIGH);
  M95_IO.begin(M95_BD);
  delay(1000);

  // clear serial line
  clearSerialLine();

  digitalWrite(M95_W1, LOW);
  delay(1000);
  waitOfReaction(4000);
  if (0 == resp) waitOfReaction(4000);
  if (0 == resp) goto m95_rst;
  digitalWrite(M95_W1, HIGH);

  if (resp == M95_RESP_RDY) {
    M95_IO.write("AT+QPOWD=1\r");
    waitOfReaction(300);
    comOk = true;
  }
  if (resp == M95_RESP_POWER_DOWN) {
    comOk = true;
    isOn = false;
    M95_IO.end();
  }

  if (isOn and comOk) goto m95_rst;

  return !isOn;
}

boolean M95::powerUp(rInstrument& instrument)
{
  if (!isOk) return false;
  if (isOn) return true;

  uint8_t try_on_cnt = 0;

m95_rst:
  if (try_on_cnt++ >= 3) goto m95_fail;

  digitalWrite(M95_W1, HIGH);
  M95_IO.begin(M95_BD);
  delay(1000);

  // clear serial line
  clearSerialLine();

  digitalWrite(M95_W1, LOW);

m95_wait_on:
  if (try_on_cnt > 3) goto m95_fail;

  delay(1000);
  waitOfReaction(4000);
  if (0 == resp) waitOfReaction(4000);
  if (0 == resp) goto m95_rst;
  digitalWrite(M95_W1, HIGH);

  if (M95_RESP_POWER_DOWN == resp) {
    delay(550);
    goto m95_rst;
  }

  if (M95_RESP_RDY != resp) {
m95_fail:
    if (M95_RESP_RDY != resp) {
      if (try_on_cnt >= 3) goto m95_call_fail;
      else goto m95_end_fail;
    }

    waitOfReaction(15000);
    if (M95_RESP_CFUN_1 != resp) goto m95_end_fail;

    waitOfReaction(5000);
    if (M95_RESP_CPIN_READY != resp) goto m95_end_fail;

    waitOfReaction(5000);
    if (M95_RESP_CALL_READY != resp) goto m95_end_fail;

m95_call_fail:
    // call emergency!!! jajajaja
    callWarning(instrument);

m95_end_fail:
    isOn = true;
    powerDown();

    return false;
  } else {
    waitOfReaction(15000);
    if (M95_RESP_CFUN_1 != resp) {
      M95_IO.write("AT+CFUN=1,1\r");
      waitOfReaction(15000);
      if (0 == resp) waitOfReaction(15000);
      if (M95_RESP_OK == resp) {
        try_on_cnt++;
        goto m95_wait_on;
      } else goto m95_rst;
    }

    waitOfReaction(5000);
    if (M95_RESP_CPIN_READY != resp) goto m95_rst;

    waitOfReaction(5000);
    if (M95_RESP_CALL_READY != resp) goto m95_rst;

    // make sure the GSM network is registered successfully
    M95_IO.write("AT+CREG=0\r");
    waitOfReaction(300);
    if (M95_RESP_OK != resp) goto m95_rst;

    for (uint8_t i = 0; i < 6; i++) {
      // check the network
      M95_IO.write("AT+CREG?\r");
      waitOfReaction(300);
      if (M95_RESP_CREG_X_1 == resp) break;

      delay(5000);
    }
    if (M95_RESP_CREG_X_1 != resp) goto m95_rst;

    // mute changes in GSM network...
    M95_IO.write("AT+CREG=0\r");
    waitOfReaction(300);
    if (M95_RESP_OK != resp) goto m95_rst;

    // Set the context 0 as the foreground context
    M95_IO.write("AT+QIFGCNT=0\r");
    waitOfReaction(300);
    if (M95_RESP_OK != resp) goto m95_rst;

    // Set APN for the current context
    M95_IO.write("AT+COPS?\r");
    waitOfReaction(75000);

    switch (resp) {
      case M95_RESP_COPS_CHILE_ENTEL: { // APN Entel-Chile
        M95_IO.write("AT+QICSGP=1,\"bam.entelpcs.cl\",\"entelpcs\",\"entelpcs\"\r");
      } break;
      case M95_RESP_COPS_PERU_ENTEL: { // APN Entel-Peru
        M95_IO.write("AT+QICSGP=1,\"entel.pe\"\r");
      } break;
      case M95_RESP_COPS_CHILE_CLARO: { // APN Claro-Chile
        M95_IO.write("AT+QICSGP=1,\"bam.clarochile.cl\",\"clarochile\",\"clarochile\"\r");
      } break;
      case M95_RESP_COPS_PERU_CLARO: { // APN Claro-Peru
        M95_IO.write("AT+QICSGP=1,\"claro.pe\",\"claro\",\"claro\"\r");
      } break;
      case M95_RESP_COPS_CHILE_MOVISTAR: { // APN Movistar-Chile
        M95_IO.write("AT+QICSGP=1,\"wap.tmovil.cl\",\"wap\",\"wap\"\r");
      } break;
      case M95_RESP_COPS_PERU_MOVISTAR: { // APN Movistar-Peru
        M95_IO.write("AT+QICSGP=1,\"movistar.pe\",\"movistar@datos\",\"movistar\"\r");
      } break;
      case M95_RESP_COPS_PERU_BITEL: { // APN Bitel-Peru
        M95_IO.write("AT+QICSGP=1,\"bitel.pe\"\r");
      } break;
      default: {
        if (try_on_cnt++ < 3) goto m95_rst;
        else goto m95_fail;
      }
    }
    waitOfReaction(300);
    if (M95_RESP_OK != resp) goto m95_rst;

    // make sure the GPRS network is registered successfully
    M95_IO.write("AT+CGREG=2\r");
    waitOfReaction(300);
    if (M95_RESP_OK != resp) goto m95_rst;

    for (uint8_t i = 0; i < 6; i++) {
      // check the network
      M95_IO.write("AT+CGREG?\r");
      waitOfReaction(300);
      if (M95_RESP_CGREG_X_1 == resp) break;

      delay(5000);
    }
    if (M95_RESP_CGREG_X_1 != resp) goto m95_rst;

    // mute changes in GPRS network...
    M95_IO.write("AT+CGREG=0\r");
    waitOfReaction(300);
    if (M95_RESP_OK != resp) goto m95_rst;

    // query GSM network status
    M95_IO.write("AT+QNSTATUS\r");
    waitOfReaction(300);
    if (M95_RESP_QNSTATUS_0 != resp) goto m95_rst;

    // Set server address to the domain name server format
    M95_IO.write("AT+QIDNSIP=1\r");
    waitOfReaction(300);
    if (M95_RESP_OK != resp) goto m95_rst;

    // Set mode: when receiving the data
    M95_IO.write("AT+QINDI=2\r");
    waitOfReaction(300);
    if (M95_RESP_OK != resp) goto m95_rst;

    // Set echo off mode
    M95_IO.write("AT+QISDE=0\r");
    waitOfReaction(300);
    if (M95_RESP_OK != resp) goto m95_rst;

    // Set mode:
    M95_IO.write("AT+QINDRI=0\r");
    waitOfReaction(300);
    if (M95_RESP_OK != resp) goto m95_rst;

    // set SMS text message format
    M95_IO.write("AT+CMGF=1\r");
    waitOfReaction(300);
    if (M95_RESP_OK != resp) goto m95_rst;

    if (!syncSMS(instrument)) goto m95_rst;

    // Set Call Forwarding...
    if (strlen(instrument.getPhoneWarning()) > 3) {
      if(instrument.getPhoneWarning()[0] == '+')
        M95_IO.write("AT+CCFC=0,3,\"");
      else
        M95_IO.write("AT+CCFC=0,3,\"+");

      M95_IO.print(instrument.getPhoneWarning());
      M95_IO.write("\"\r");

      waitOfReaction(30000);
      if (0 == resp) waitOfReaction(30000);
      //if (M95_RESP_OK != resp) goto m95_rst;
    }

    // Set mode: no sms, no call
    M95_IO.write("AT+QREFUSECS=1,1\r");
    waitOfReaction(300);
    if (M95_RESP_OK != resp) goto m95_rst;

    // Set mode:
    M95_IO.write("AT+QIURC=0\r");
    waitOfReaction(300);
    if (M95_RESP_OK != resp) goto m95_rst;

    // Set mode:
    M95_IO.write("AT+QEXTUNSOL=\"SQ\",0\r");
    waitOfReaction(300);
    if (M95_RESP_OK != resp) goto m95_rst;

    // Set mode:
    M95_IO.write("AT+QEXTUNSOL=\"FN\",0\r");
    waitOfReaction(300);
    if (M95_RESP_OK != resp) goto m95_rst;

    // Set mode:
    M95_IO.write("AT+QEXTUNSOL=\"MW\",0\r");
    waitOfReaction(300);
    if (M95_RESP_OK != resp) goto m95_rst;

    // Set mode:
    M95_IO.write("AT+QEXTUNSOL=\"UR\",0\r");
    waitOfReaction(300);
    if (M95_RESP_OK != resp) goto m95_rst;

    // Set mode:
    M95_IO.write("AT+QEXTUNSOL=\"BC\",0\r");
    waitOfReaction(300);
    if (M95_RESP_OK != resp) goto m95_rst;

    // Set mode:
    M95_IO.write("AT+QEXTUNSOL=\"BM\",0\r");
    waitOfReaction(300);
    if (M95_RESP_OK != resp) goto m95_rst;

    // Set mode:
    M95_IO.write("AT+QEXTUNSOL=\"SM\",0\r");
    waitOfReaction(300);
    if (M95_RESP_OK != resp) goto m95_rst;

    // Set mode:
    M95_IO.write("AT+QEXTUNSOL=\"CC\",0\r");
    waitOfReaction(300);
    if (M95_RESP_OK != resp) goto m95_rst;

    // Set mode:
    M95_IO.write("AT+QEXTUNSOL=\"CN\",0\r");
    waitOfReaction(300);
    if (M95_RESP_OK != resp) goto m95_rst;
  }

  isOn = true;
  return true;
}

uint8_t M95::call(char const* const& phone)
{
  if (strlen(phone) > 3) {
    M95_IO.write("ATD");
    M95_IO.print(phone);
    M95_IO.write(";\r");

    waitOfReaction(2000);
    if (M95_RESP_OK != resp) return 1;

    waitOfReaction(120000);
    if (M95_RESP_BUSY == resp) return 3;
    if (M95_RESP_NO_ANSWER == resp) return 4;
    if (M95_RESP_NO_CARRIER == resp) return 5;
    if (M95_RESP_NO_DIALTONE == resp) return 6;

    return 0;
  }

  return 2;  
}

uint8_t M95::csq(uint8_t niter)
{
  if (niter == 0) return 0;

  uint16_t rssi = 0;
  uint16_t ber = 0;
  uint8_t n = 0;
  for (uint8_t i = 0; i < niter; i++) {
    M95_IO.write("AT+CSQ\r");

    waitOfReaction(300);
    if (M95_RESP_CSQ == resp) {
      char* subStrResponse = strstr(M95_string, "+CSQ:") + 5;
      uint8_t rssi_i = (uint8_t) strtol(subStrResponse, &subStrResponse, 10);
      uint8_t ber_i = (uint8_t) strtol(subStrResponse+1, &subStrResponse, 10);

      if (rssi_i < 32 and ber_i < 8) {
        rssi += rssi_i;
        ber += ber_i;
        n++;
      }
    }
  }

  if (0 == n) return 0;

  rssi /= n; // < 32 5bits
  ber /= n; // < 8 3bits
  return ((ber << 5) | rssi);
}

boolean M95::callWarning(rInstrument& instrument)
{
  uint8_t call_cnt = 0;

call:
  if (call_cnt++ >= 3) return false;
  // call emergency!!! jajajaja
  uint8_t rcall = call(instrument.getPhoneWarning());

  switch (rcall) {
    case 0: {
      return true;
    } break;
    case 1: {
      goto call;
    } break;
    default: {
      return false;
    }
  }
}

boolean M95::ntpSyncUDP(int32_t & dtsr, int16_t & dtpr, double & tdelay)
{
  if (!isOn) return false;

  int32_t delta_tsr[NTPN];
  int16_t delta_tpr[NTPN];

  double delays[NTPN];

  for (uint8_t i = 0; i < NTPN; i++) {
    // init delays
    delta_tsr[i] = 0;
    delta_tpr[i] = 0;

    delays[i] = 10;

    // clear serial line
    clearSerialLine();

    // establish UDP session
    memset(M95_string, 0, M95_string_size);
    snprintf(M95_string, M95_string_size-1,
             "AT+QIOPEN=\"UDP\",\"%s\",123\r", ntp_servers[i]);
    M95_IO.write(M95_string);
    waitOfReaction(300);

    if (M95_RESP_OK != resp) {
qiclose:
      if (qiclose(true)) continue;

      break;
    }

    waitOfReaction(90000);
    if (M95_RESP_CONNECT_X == resp) continue;
    if (M95_RESP_CONNECT_OK != resp) goto qiclose;

    uint8_t pkgo[48];
    memset(pkgo, 0, 48);
    pkgo[0] = 0b00100011;  // li:2 vn:3 mode:3
    pkgo[1] = 0x00;        // stratum
    pkgo[2] = 0x06;        // poll
    pkgo[3] = 0xf1;        // precision -15 = log2(eps) 32.768kHz
    // 4-5-6-7             // root delay
    // 8-9-10-11           // root dispersion
    pkgo[12]= 0x58;
    pkgo[13]= 0x47;
    pkgo[14]= 0x43;
    pkgo[15]= 0x58;        // reference id (0=stratum)
    // 16-17-18-19
    // 20-21-22-23         // reference timestamp
    // 24-25-26-27
    // 28-29-30-31         // origin timestamp
    // 32-33-34-35
    // 36-37-38-39         // receive timestamp
    // 40-41-42-43
    // 44-45-46-47         // transmit timestamp

    uint8_t npkgs = 0;

sendpkg:
    M95_IO.write("AT+QISEND=48\r");
    waitOfReaction(300);
    for (uint8_t iter = 0; 0 == resp and iter < 5; iter++) waitOfReaction(300); //why 0?
    if (M95_RESP_PROMPT != resp) goto qiclose;

    uint32_t t1 = RTC_TSR + T1900;
    uint32_t t1_f = (RTC_TPR & 0x7fff) << 17;
    if (t1_f == 0) t1 = RTC_TSR + T1900;

    *((uint32_t*)&pkgo[40]) = htonl(t1);
    *((uint32_t*)&pkgo[44]) = htonl(t1_f);

    M95_IO.write(pkgo, 48);
    waitOfReaction(300);
    for (uint8_t iter = 0; 0 == resp and iter < 5; iter++) waitOfReaction(300); //why 0?
    if (M95_RESP_SEND_OK != resp) goto qiclose;

    waitOfReaction(10000);
    if (M95_RESP_QIRDI != resp) goto qiclose;

    uint32_t t4 = RTC_TSR + T1900;
    uint32_t t4_f = (RTC_TPR & 0x7fff) << 17;
    if (t4_f == 0) t4 = RTC_TSR + T1900;

    M95_IO.write("AT+QIRD=0,1,0,48\r");
    waitOfReaction(300);
    if (M95_RESP_QIRD != resp) goto qiclose;

    *((uint32_t*)&pkgo[32]) = htonl(t4);
    *((uint32_t*)&pkgo[36]) = htonl(t4_f);

    char* pkg_str = strstr(M95_string, "UDP") + 4;
    uint32_t pkg_len = (uint32_t)strtol(pkg_str, &pkg_str, 10);
    pkg_len = min(48u, pkg_len);

    uint8_t pkgi[48];
    memcpy(pkgi, pkg_str+2, pkg_len);

    // sanity test:
    if (pkgi[1] <= 0 or  15 <= pkgi[1]) goto qiclose; // stratum error 0==KoD

    if (!memcmp(&pkgo[24], &pkgi[40], 8)) goto qiclose; // T0 == T3
    memcpy(&pkgo[24], &pkgi[40], 8);

    if (memcmp(&pkgo[40], &pkgi[24], 8)) goto qiclose; // T1_client == T1_server
    memset(&pkgo[40], 0, 8);

    if (0b11000000 == (pkgi[0] & 0b11000000)) goto qiclose; // li==3 (reloj no sincronizado)

    if (0b00100000 != (pkgi[0] & 0b00111000)) goto qiclose; // mode!=4 (no es un servidor)

#ifdef DEBUG_IO
    DEBUG_IO.print("li: "); DEBUG_IO.println((pkgi[0] & 0b11000000) >> 6);
    DEBUG_IO.print("vn: "); DEBUG_IO.println((pkgi[0] & 0b00111000) >> 3);
    DEBUG_IO.print("mode: "); DEBUG_IO.println((pkgi[0] & 0b00000111) >> 0);
    DEBUG_IO.print("stratum: "); DEBUG_IO.println(pkgi[1], HEX);
    DEBUG_IO.print("ppoll: "); DEBUG_IO.println(pkgi[2], HEX);
    DEBUG_IO.print("precision: "); DEBUG_IO.println((int8_t)pkgi[3]);
    DEBUG_IO.print("rdelay: "); DEBUG_IO.println(ntohl(*((uint32_t*)&pkgi[4])));
    DEBUG_IO.print("rdispersion: "); DEBUG_IO.println(ntohl(*((uint32_t*)&pkgi[8])));
    DEBUG_IO.print("refID: "); DEBUG_IO.println(*((uint32_t*)&pkgi[12]), HEX);
#endif

    uint32_t t2, t2_f;
    t2 = ntohl(*((uint32_t*)&pkgi[32]));
    t2_f = ntohl(*((uint32_t*)&pkgi[36]));

    uint32_t t3, t3_f;
    t3 = ntohl(*((uint32_t*)&pkgi[40]));
    t3_f = ntohl(*((uint32_t*)&pkgi[44]));

#ifdef DEBUG_IO
    DEBUG_IO.print("t1: "); DEBUG_IO.print(t1); DEBUG_IO.print("."); DEBUG_IO.println(t1_f);
    DEBUG_IO.print("t2: "); DEBUG_IO.print(t2); DEBUG_IO.print("."); DEBUG_IO.println(t2_f);
    DEBUG_IO.print("t3: "); DEBUG_IO.print(t3); DEBUG_IO.print("."); DEBUG_IO.println(t3_f);
    DEBUG_IO.print("t4: "); DEBUG_IO.print(t4); DEBUG_IO.print("."); DEBUG_IO.println(t4_f);
#endif

    int64_t theta = (((int64_t)t2-(int64_t)t1)+((int64_t)t3-(int64_t)t4));
    int64_t theta_f = (((int64_t)t2_f-(int64_t)t1_f)+((int64_t)t3_f-(int64_t)t4_f))/2;

    if (theta & 0b1) {// theta!=0 and theta=2n+1
      if (theta > 0) theta_f += 0x80000000; // +0.5 seg
      else theta_f -= 0x80000000; // -0.5 seg
    }
    theta /= 2;

    int64_t delta_client = (t4-t1);
    int64_t delta_client_t = (t4_f - t1_f);
    double deltaClient = delta_client;
    deltaClient += delta_client_t/4294967296.0;

    int64_t delta_server = (t3-t2);
    int64_t delta_server_t = (t3_f - t2_f);
    double deltaServer = delta_server;
    deltaServer += delta_server_t/4294967296.0;

    double delta = deltaClient - deltaServer;

    if (delta < delays[i] and -2147483648 <= theta and theta <= 2147483647) {
      delta_tsr[i] = (int32_t)theta;
      delta_tpr[i] = (int16_t)(theta_f >> 17);

      delays[i] = delta;
    }

    if (npkgs++ < NTP_NPKGS) goto sendpkg;

    goto qiclose;
  }

  tdelay = NTP_MAXDIST;
  for (uint32_t i = 0; i < NTPN; i++) {

#ifdef DEBUG_IO
    DEBUG_IO.print("dtsr: "); DEBUG_IO.println(delta_tsr[i]);
    DEBUG_IO.print("dtpr: "); DEBUG_IO.println(delta_tpr[i]);
    DEBUG_IO.print("delay: "); DEBUG_IO.println(delays[i]);
#endif

    if (delays[i] < tdelay) {
      dtsr = delta_tsr[i];
      dtpr = delta_tpr[i];
      tdelay = delays[i];
    }
  }
  if (tdelay >= NTP_MAXDIST) return false;

  if (dtpr > 0) {
    while (RTC_TPR > 0);
    RTC_SR = 0;
    RTC_TPR += dtpr;
    RTC_TSR += dtsr;
    RTC_SR = RTC_SR_TCE;
  } else if (dtpr < 0) {
#if 0 // 0->13bits 1->12bits
  #define TPR_DIV 0x1000 // 0x1000 0x2000
  #define TPR_RST 0x0fff // 0x0fff 0x1fff
#else
  #define TPR_DIV 0x2000 // 0x1000 0x2000
  #define TPR_RST 0x1fff // 0x0fff 0x1fff
#endif
#define TPR_MSK (TPR_DIV | TPR_RST) // 12bits> 13bits>
    uint32_t dtpr_div = (-dtpr) / TPR_DIV;
    uint32_t dtpr_rst = (-dtpr) & TPR_RST;

    while ((RTC_TPR & TPR_MSK) > 0);
    while ((RTC_TPR & TPR_MSK) < dtpr_rst);
    RTC_SR = 0;
    RTC_TPR -= dtpr_rst;
    RTC_TSR += dtsr;
    RTC_SR = RTC_SR_TCE;

    for (uint32_t i = 0; i < dtpr_div; i++) {
      while ((RTC_TPR & TPR_MSK) > 0);
      while ((RTC_TPR & TPR_MSK) < TPR_DIV);
      RTC_SR = 0;
      RTC_TPR -= TPR_DIV;
      RTC_SR = RTC_SR_TCE;
    }
  }

  return true;
}

boolean M95::cellloc(float& longitude, float& latitude)
{
  if (!isOn) return false;

  char* respStr;

  uint8_t try_count;
  try_count = 0;
try_loc:

  // clear serial line
  clearSerialLine();

  // get the current location
  M95_IO.write("AT+QCELLLOC=1\r");
  waitOfReaction(300);
  for (uint8_t iter = 0; 0 == resp and iter < 5; iter++) waitOfReaction(300); //why 0?
  if (M95_RESP_QCELLLOC != resp) waitOfReaction(120000);
  if (M95_RESP_QCELLLOC != resp and try_count++ < 3) goto try_loc;
  if (M95_RESP_QCELLLOC != resp) return false;

  respStr = strstr(M95_string, "+QCELLLOC: ");
  longitude = strtod(&respStr[10], &respStr);
  latitude = strtod(&respStr[1], &respStr);

  return true;
}

boolean M95::qiclose(boolean checkOn)
{
  if (checkOn and !isOn) return false;

  // clear serial line
  clearSerialLine();

  if (M95_RESP_CLOSED == resp) return true;

  M95_IO.write("AT+QICLOSE\r");
  waitOfReaction(300);
  if (M95_RESP_CLOSE_OK != resp) return false;

  return true;
}

boolean M95::sendData(uint16_t data_size)
{
prompt:
  memset(M95_string, 0, 17);
  snprintf(M95_string, 16, "AT+QISEND=%u\r", data_size);

  M95_IO.write(M95_string);
  waitOfReaction(300);
  for (uint8_t iter = 0; 0 == resp and iter < 5; iter++) waitOfReaction(300); //why 0?
  if (M95_RESP_ERROR == resp) {
    delay(1460);
    goto prompt;
  }
  if (M95_RESP_PROMPT != resp) return false;

  M95_IO.write((uint8_t*)M95_data, data_size); // send data!
  waitOfReaction(300);
  for (uint8_t iter = 0; 0 == resp and iter < 5; iter++) waitOfReaction(300); //why 0?
  if (M95_RESP_SEND_OK != resp) return false;

  return true;
}

uint16_t M95::extractResponse(boolean checkOn)
{
  uint16_t stateHTTP = 0;

  if (M95_RESP_QIRDI != resp) waitOfReaction(75000);
  if (M95_RESP_QIRDI != resp) {
qiclose:
#ifdef DEBUG_IO
    DEBUG_IO.println("-------------------------");
    DEBUG_IO.println(M95_string);
    DEBUG_IO.println("-------------------------");
    DEBUG_IO.println(resp);
    DEBUG_IO.println("-------------------------");
#endif

    qiclose(checkOn);
    return 0 | (stateHTTP << 1);
  }

  M95_IO.write("AT+QIRD=0,1,0,1024\r");
  waitOfReaction(300);
  if (M95_RESP_QIRD != resp) goto qiclose;

  // get response
  char* subStrResponse = strstr(M95_string, "TCP,") + 4;
  uint16_t response_size = (uint16_t) strtol(subStrResponse, &subStrResponse, 10);
  subStrResponse += 2;
  subStrResponse[response_size] = '\0';// remove tail

  // codigo de estado
  char* subStrState = strstr(subStrResponse, "HTTP/1.") + 9;
  stateHTTP = (uint16_t)strtol(subStrState, &subStrState, 10);

  // se extrae el contenido
  char* subStrContent = strstr(subStrResponse, "Content-Length:") + 15;
  uint16_t contentLength = 0;
  if ((uint32_t)subStrContent > 15)
    contentLength = (uint16_t)strtol(subStrContent, &subStrContent, 10);
  subStrContent = strstr(subStrResponse, "\r\n\r\n") + 4;

  contentLength = min(contentLength, M95_data_size-1);

  M95_data[0] = '\0';
  if ((uint32_t)subStrContent > 4) {
    // head:
    uint32_t head_size = ((subStrContent-4)-subStrResponse)+1; // +1 == '\0'
    char head[head_size];
    strncpy(head, subStrResponse, head_size-1);
    head[head_size-1] = '\0';

    // content:
    uint32_t calcLength = response_size-((head_size-1)+4);
    uint16_t data_index = 0;

    while ((data_index + calcLength) < contentLength) {
      strncpy(M95_data+data_index, subStrContent, calcLength);
      data_index += calcLength;
      M95_data[data_index] = '\0';

      resp = getResp(subStrResponse+response_size+1);

      uint8_t qird_tryes = 0;
at_qird:

      if (M95_RESP_CLOSED == resp) {
        strncpy(M95_string, head, head_size);
        return 1 | (stateHTTP << 1);
      }

      M95_IO.write("AT+QIRD=0,1,0,1024\r");
      waitOfReaction(300);

      if (M95_RESP_CLOSED == resp) {
        strncpy(M95_string, head, head_size);
        return 1 | (stateHTTP << 1);
      }
      if (M95_RESP_OK == resp) {
        if (qird_tryes++ > 75) {
          qiclose(checkOn);
          strncpy(M95_string, head, head_size);
          return 1 | (stateHTTP << 1);
        }
        waitOfReaction(700);

        goto at_qird;
      }
      if (M95_RESP_QIRD != resp) goto qiclose;

      // get response
      subStrResponse = strstr(M95_string, "TCP,") + 4;
      response_size = (uint16_t)strtol(subStrResponse, &subStrResponse, 10);
      subStrResponse += 2;
      subStrResponse[response_size] = '\0';// remove tail

      subStrContent = subStrResponse;
      calcLength = response_size;
    }
    if (data_index < contentLength) {
      calcLength = min(calcLength, (uint32_t)(contentLength-data_index));
      strncpy(M95_data+data_index, subStrContent, calcLength);
      data_index += calcLength;
      M95_data[data_index] = '\0';

      resp = getResp(subStrResponse+response_size+1);

      if (M95_RESP_CLOSED == resp) {
        strncpy(M95_string, head, head_size);
        return 1 | (stateHTTP << 1);
      }
    }

    qiclose(checkOn);
    strncpy(M95_string, head, head_size);
  } else {
    uint32_t head_size = response_size+1; // +1 == '\0'
    char head[head_size];
    strncpy(head, subStrResponse, head_size-1);
    head[head_size-1] = '\0';
    
    qiclose(checkOn);
    strncpy(M95_string, head, head_size);
  }

  return 1 | (stateHTTP << 1);
}

uint32_t M95::httpRESTFile(char const* const& url, char const* const& rest, SdFile & ifile, boolean checkOn)
{
  if (checkOn and !isOn) return 0;

  uint16_t send_data_max_size = min(SEND_DATA_MAX_SIZE, M95_data_size);

  // clear serial line
  clearSerialLine();

  // establish TCP session
  cleanStringLog();
  snprintf(M95_string, M95_string_size-1, "AT+QIOPEN=\"TCP\",\"%s\",80\r", url);
  M95_IO.write(M95_string);
  waitOfReaction(300);

  if (M95_RESP_OK != resp) {
qiclose:
#ifdef DEBUG_IO
    DEBUG_IO.println("-------------------------");
    DEBUG_IO.println(M95_string);
    DEBUG_IO.println("-------------------------");
    DEBUG_IO.println(resp);
    DEBUG_IO.println("-------------------------");
#endif

    qiclose(checkOn);

    return 0;
  }
  waitOfReaction(90000);
  if (M95_RESP_CONNECT_OK != resp) goto qiclose;

  // formatos:
  char const boundary[] = "abcdefghijklmnopqrstuvwxyz";
  char const header[] =
    "%s HTTP/1.1\r\n"
    "User-Agent: GeoCentinela\r\n"
    "Host: %s\r\n"
    "Content-Type: multipart/form-data; boundary=%s\r\n"
    "Content-Length: %lu\r\n\r\n";
    // rest url boundary data_length : 11+26+8+46+20 + (<=33 + ~21 + <=4)+26
  char const body[] =
    "--%s\r\n"
    "Content-Disposition: form-data; name=\"file\"; filename=\"FILE.EVF\"\r\n"
    "Content-Type: application/octet-stream\r\n\r\n";
    // boundary : 4+66+42 + 26
  char const tail[] =
    "\r\n--%s--\r\n";// boundary : 8 + 26

  // data_length:
  uint32_t content_length = ifile.fileSize();

  // add body length
  cleanStringLog();
  snprintf(M95_string, M95_string_size-1, body, boundary);
  content_length += strlen(M95_string);

  // add tail length
  cleanStringLog();
  snprintf(M95_string, M95_string_size-1, tail, boundary);
  content_length += strlen(M95_string);

  cleanStringIO();
  snprintf(M95_data, send_data_max_size, header, rest, url, boundary, content_length);
  uint16_t data_length = strlen(M95_data);

  snprintf(M95_data+data_length, send_data_max_size-data_length, body, boundary);
  data_length = strlen(M95_data);

  // header+body
  if (!sendData(data_length)) goto qiclose;

  data_length = ifile.read((uint8_t*)M95_data, send_data_max_size);
  while (data_length > 0) {
    // data_length <= send_data_max_size
    if (!sendData(data_length)) goto qiclose;

    data_length = ifile.read((uint8_t*)M95_data, send_data_max_size);
  }

  cleanStringIO();
  snprintf(M95_data, send_data_max_size, tail, boundary);
  data_length = strlen(M95_data);

  // tail
  if (!sendData(data_length)) goto qiclose;

  return extractResponse(checkOn);
}

uint16_t M95::httpREST(char const* const& url, char const* const& rest, char const* const& content_type,boolean checkOn)
{
  if (checkOn and !isOn) return 0;

  uint16_t send_data_max_size = min(SEND_DATA_MAX_SIZE, M95_data_size);

  // clear serial line
  clearSerialLine();

  // establish TCP session
  cleanStringLog();
  snprintf(M95_string, M95_string_size-1, "AT+QIOPEN=\"TCP\",\"%s\",80\r", url);
  M95_IO.write(M95_string);
  waitOfReaction(300);

  if (M95_RESP_OK != getResp()) {
qiclose:
#ifdef DEBUG_IO
    DEBUG_IO.println("-------------------------");
    DEBUG_IO.println(M95_string);
    DEBUG_IO.println("-------------------------");
    DEBUG_IO.println(resp);
    DEBUG_IO.println("-------------------------");
#endif

    qiclose(checkOn);

    return 0;
  }
  waitOfReaction(90000);
  if (M95_RESP_CONNECT_OK != getResp()) goto qiclose;

  // Se respalda el StringIO:
  cleanStringLog();
  strncpy(M95_string, getStringIO(), M95_string_size-1);

  // Formato de peticion HTTP:
  char const format[] =
    "%s HTTP/1.1\r\n"
    "User-Agent: GeoCentinela\r\n"
    "Host: %s\r\n"
    "Content-Type: %s\r\n"
    "Accept: %s\r\n"
    "Content-Length: %d\r\n\r\n%s";// rest url content_type content_type data_length data
  cleanStringIO();
  snprintf(getStringIO(), send_data_max_size, format,
    rest, url, content_type, content_type, strlen(getStringLog()), getStringLog());

  // Send data
  uint16_t data_length = strlen(getStringIO());
  if (!sendData(data_length)) goto qiclose;

  return extractResponse(checkOn);
}

void M95::clearSerialLine()
{
  // clear serial line
  M95_IO.flush();
  while (M95_IO.available()) M95_IO.read(); 
}

boolean M95::syncSMS(rInstrument& instrument)
{
  if (strlen(instrument.getPhoneWarning()) <= 4) return true;
  if (0 == instrument.getId()) return true;

  uint8_t try_cnt = 0;

try_sms:
  if (try_cnt++ >= 3) return false;

  M95_IO.write("AT+CPMS?\r");
  waitOfReaction(300);
  if (M95_RESP_CPMS != resp) return false;

  char* findStr = strstr(M95_string, "\"SM\"") + 4;
  if (4 == (uint32_t)findStr) return false;

  findStr = strstr(findStr, ",") + 1;
  if (1 == (uint32_t)findStr) return false;

  uint8_t sms_unsend = (uint8_t)strtol(findStr, &findStr, 10);

  // receive message!
  waitOfReaction(5000);
  if (M95_RESP_CMTI != resp) waitOfReaction(5000);
  if (M95_RESP_CMTI != resp) waitOfReaction(5000);
  if (M95_RESP_CMTI != resp) waitOfReaction(5000);
  if (M95_RESP_CMTI == resp) {
    while (M95_RESP_CMTI == resp) waitOfReaction(5000);
    goto try_sms;
  }
  if (sms_unsend == 0) return true;

  findStr = strstr(findStr, ",") + 1;
  if (1 == (uint32_t)findStr) return false;
  uint8_t sms_list_size = (uint8_t)strtol(findStr, &findStr, 10);

  for (uint8_t it = 1; it <= sms_list_size; it++) {
    M95_IO.write("AT+CMGR=");
    M95_IO.print(it);
    M95_IO.write("\r");
    waitOfReaction(60000);
    for (uint8_t iter = 0; 0 == resp and iter < 5; iter++) waitOfReaction(60000);

    if (M95_RESP_CMGR == resp) {
      findStr = strstr(M95_string, "+CMGR:") + 6;
      if (6 < (uint32_t)findStr) findStr = strstr(findStr, ",\"") + 2;
      if (2 < (uint32_t)findStr) {
        rMessage msg;
        msg.setInstrumentId(instrument.getId());

        // set from:
        char * endFind = strstr(findStr, "\"");
        if (endFind > 0) endFind[0] = '\0';
        else continue;

        strtoASCII(findStr);
        msg.setFrom(findStr);

        // set date:
        findStr = endFind+1;
        findStr = strstr(findStr, ",\"") + 2;
        if (2 == (uint32_t)findStr) continue;
        findStr = strstr(findStr, ",\"") + 2;
        if (2 == (uint32_t)findStr) continue;
        endFind = strstr(findStr, "\"");
        if (endFind > 0) endFind[0] = '\0';
        else continue;

        strtoASCII(findStr);
        msg.setDate(findStr);

        // set sms:
        findStr = endFind+3;
        endFind = strstr(findStr, "\r\n\r\nOK");
        if (endFind > 0) endFind[0] = '\0';
        else continue;

        strtoASCII(findStr);
        msg.setSMS(findStr);

        // send message:
        uint8_t sync_cnt = 0;

try_send:
        if (sync_cnt++ >= 3) continue;

        cleanStringIO();
        msg.serialize(M95_data, M95_data_size, (uint8_t*)M95_string, M95_string_size, false);
        uint16_t stateHTTP = httpJsonREST(instrument.getHostName(), "POST /messages", false);

        if (!(stateHTTP&0x1) or 201 != (stateHTTP >> 1)) goto try_send;

        // mensaje enviado es mensaje borrando...
        if (msg.deserialize(M95_data, (uint8_t*)M95_string, M95_string_size)) {
          M95_IO.write("AT+CMGD=");
          M95_IO.print(it);
          M95_IO.write("\r");
          waitOfReaction(300);
          if (0 == resp) waitOfReaction(300);
        }
      }
    }
  }
  return true;
}

void M95::strtoASCII(char * str)
{
  uint32_t str_size = strlen(str);
  for (uint32_t i = 0; i < str_size; i++) {
    if (str[i] < 0x20 or str[i] > 0x7F) {
      if (0xC9 == str[i]) str[i] = 0x45;
      else if (0xE9 == str[i]) str[i] = 0x65;
      else if (0x02 == str[i]) str[i] = 0x24;
      else str[i] = 0x3F;
    }
    else if (0x60 == str[i]) str[i] = 0x20;
  }
}
