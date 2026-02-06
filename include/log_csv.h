#pragma once
#include <Arduino.h>
#include <SD.h>

// CSV logging helper for ESP32 rigs that share SPI between TFT and SD.
// This module owns a single File handle internally.
//
// All functions that touch SD will:
//  - pull TFT_CS HIGH
//  - pull SD_CS LOW
//  - do IO
//  - pull SD_CS HIGH
//  - pull TFT_CS LOW

using LogCsvHeaderWriter = void (*)(fs::File&);

bool      logCsvOpen(const char* path, int tftCsPin, int sdCsPin, LogCsvHeaderWriter headerWriter);
void      logCsvClose(int tftCsPin, int sdCsPin);
bool      logCsvIsOpen();
fs::File  logCsvFile();               // returns a copy (File is a handle)

void      logCsvWriteBytes(const uint8_t* data, size_t n, int tftCsPin, int sdCsPin);
void      logCsvMaybeFlush(uint16_t* ctr, uint16_t flushEvery, int tftCsPin, int sdCsPin);

void      logCsvWriteCommentU32(const char* key, uint32_t value, int tftCsPin, int sdCsPin);
