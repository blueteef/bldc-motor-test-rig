#include "bldc_rig/logging/run_summary.h"

static void spiSelectSd(int tftCsPin, int sdCsPin){
  digitalWrite(tftCsPin, HIGH);
  digitalWrite(sdCsPin, LOW);
}
static void spiDeselectSd(int tftCsPin, int sdCsPin){
  digitalWrite(sdCsPin, HIGH);
  digitalWrite(tftCsPin, HIGH);
}

static bool fileIsEmpty(File& f){
  // If file exists and size==0 => empty
  // PlatformIO SD File has size()
  return (f.size() == 0);
}

bool runSummaryAppendRow(const char* filename,
                         int tftCsPin,
                         int sdCsPin,
                         const char* row,
                         bool ensureHeader,
                         const char* headerLine)
{
  if(!filename || !row) return false;

  spiSelectSd(tftCsPin, sdCsPin);
  File f = SD.open(filename, FILE_APPEND);
  spiDeselectSd(tftCsPin, sdCsPin);
  if(!f) return false;

  if(ensureHeader && headerLine){
    bool empty = false;
    spiSelectSd(tftCsPin, sdCsPin);
    empty = fileIsEmpty(f);
    spiDeselectSd(tftCsPin, sdCsPin);

    if(empty){
      spiSelectSd(tftCsPin, sdCsPin);
      f.println(headerLine);
      spiDeselectSd(tftCsPin, sdCsPin);
    }
  }

  spiSelectSd(tftCsPin, sdCsPin);
  f.print(row);
  if(row[0] && row[strlen(row)-1] != '\n') f.print("\n");
  f.flush();
  f.close();
  spiDeselectSd(tftCsPin, sdCsPin);

  return true;
}
