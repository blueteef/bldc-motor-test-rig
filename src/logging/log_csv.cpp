#include "bldc_rig/logging/log_csv.h"

static fs::File s_csv;

static inline void spiSelectSd(int tftCsPin, int sdCsPin){
  digitalWrite(tftCsPin, HIGH);
  digitalWrite(sdCsPin, LOW);
}
static inline void spiDeselectSd(int tftCsPin, int sdCsPin){
  digitalWrite(sdCsPin, HIGH);
  // Leave both deselected — Adafruit library handles its own TFT CS
}

bool logCsvOpen(const char* path, int tftCsPin, int sdCsPin, LogCsvHeaderWriter headerWriter){
  if(s_csv){
    // already open
    return true;
  }

  spiSelectSd(tftCsPin, sdCsPin);
  s_csv = SD.open(path, FILE_WRITE);
  spiDeselectSd(tftCsPin, sdCsPin);

  if(!s_csv){
    return false;
  }

  if(headerWriter){
    spiSelectSd(tftCsPin, sdCsPin);
    headerWriter(s_csv);
    s_csv.flush();
    spiDeselectSd(tftCsPin, sdCsPin);
  }
  return true;
}

void logCsvClose(int tftCsPin, int sdCsPin){
  if(!s_csv) return;
  spiSelectSd(tftCsPin, sdCsPin);
  s_csv.flush();
  s_csv.close();
  spiDeselectSd(tftCsPin, sdCsPin);
}

bool logCsvIsOpen(){
  return (bool)s_csv;
}

fs::File logCsvFile(){
  return s_csv;
}

void logCsvWriteBytes(const uint8_t* data, size_t n, int tftCsPin, int sdCsPin){
  if(!s_csv || !data || n == 0) return;
  spiSelectSd(tftCsPin, sdCsPin);
  s_csv.write(data, n);
  spiDeselectSd(tftCsPin, sdCsPin);
}

void logCsvMaybeFlush(uint16_t* ctr, uint16_t flushEvery, int tftCsPin, int sdCsPin){
  if(!s_csv || !ctr) return;
  if(flushEvery == 0) flushEvery = 1;
  (*ctr)++;
  if(*ctr >= flushEvery){
    spiSelectSd(tftCsPin, sdCsPin);
    s_csv.flush();
    spiDeselectSd(tftCsPin, sdCsPin);
    *ctr = 0;
  }
}

void logCsvWriteCommentU32(const char* key, uint32_t value, int tftCsPin, int sdCsPin){
  if(!s_csv || !key) return;
  spiSelectSd(tftCsPin, sdCsPin);
  s_csv.print("# ");
  s_csv.print(key);
  s_csv.print("=");
  s_csv.println(value);
  s_csv.flush();
  spiDeselectSd(tftCsPin, sdCsPin);
}
