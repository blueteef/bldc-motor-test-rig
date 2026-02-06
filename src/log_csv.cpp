#include "log_csv.h"

static File s_file;

static void spiSelectSd(int tftCsPin, int sdCsPin){
  digitalWrite(tftCsPin, HIGH);
  digitalWrite(sdCsPin, LOW);
}
static void spiDeselectSd(int tftCsPin, int sdCsPin){
  digitalWrite(sdCsPin, HIGH);
  digitalWrite(tftCsPin, LOW);
}

bool logCsvOpen(const char* filename,
                int tftCsPin,
                int sdCsPin,
                LogHeaderWriter headerWriter)
{
  if(s_file) s_file.close();

  spiSelectSd(tftCsPin, sdCsPin);
  s_file = SD.open(filename, FILE_WRITE);
  spiDeselectSd(tftCsPin, sdCsPin);

  if(!s_file) return false;

  if(headerWriter){
    spiSelectSd(tftCsPin, sdCsPin);
    headerWriter(s_file);
    s_file.flush();
    spiDeselectSd(tftCsPin, sdCsPin);
  }
  return true;
}

void logCsvWriteCommentU32(const char* key,
                           uint32_t value,
                           int tftCsPin,
                           int sdCsPin)
{
  if(!s_file) return;
  spiSelectSd(tftCsPin, sdCsPin);
  s_file.print("# ");
  s_file.print(key);
  s_file.print("=");
  s_file.println(value);
  spiDeselectSd(tftCsPin, sdCsPin);
}

void logCsvFlush(int tftCsPin, int sdCsPin){
  if(!s_file) return;
  spiSelectSd(tftCsPin, sdCsPin);
  s_file.flush();
  spiDeselectSd(tftCsPin, sdCsPin);
}

void logCsvClose(int tftCsPin, int sdCsPin){
  if(!s_file) return;
  spiSelectSd(tftCsPin, sdCsPin);
  s_file.flush();
  s_file.close();
  spiDeselectSd(tftCsPin, sdCsPin);
}

void logCsvWriteBytes(const uint8_t* data, size_t n, int tftCsPin, int sdCsPin){
  if(!s_file || !data || n == 0) return;
  spiSelectSd(tftCsPin, sdCsPin);
  s_file.write(data, n);
  spiDeselectSd(tftCsPin, sdCsPin);
}

void logCsvMaybeFlush(uint16_t* flushCtr,
                      uint16_t flushEveryN,
                      int tftCsPin,
                      int sdCsPin)
{
  if(!s_file || !flushCtr || flushEveryN == 0) return;
  (*flushCtr)++;
  if(*flushCtr >= flushEveryN){
    logCsvFlush(tftCsPin, sdCsPin);
    *flushCtr = 0;
  }
}

bool logCsvIsOpen(){ return (bool)s_file; }
File& logCsvFile(){ return s_file; }
