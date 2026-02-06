#pragma once
#include <Arduino.h>
#include <SD.h>

bool runSummaryAppendRow(const char* filename,
                         int tftCsPin,
                         int sdCsPin,
                         const char* row,
                         bool ensureHeader,
                         const char* headerLine);
