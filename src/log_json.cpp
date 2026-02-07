#include "log_json.h"
#include <string.h>

static fs::File gJson;

// Simple JSON writer with correct comma handling (no trailing commas).
// Supports nested objects and simple arrays written via logJsonWriteColumns.
struct JsonCtx {
  bool first;   // true until first element written in this object
};
static JsonCtx gStack[8];
static int gDepth = 0;

static inline void spiSelect(int pin)   { if (pin >= 0) digitalWrite(pin, LOW); }
static inline void spiDeselect(int pin) { if (pin >= 0) digitalWrite(pin, HIGH); }

static inline void ctxPush()
{
  if (gDepth < (int)(sizeof(gStack)/sizeof(gStack[0]))) {
    gStack[gDepth].first = true;
    gDepth++;
  }
}

static inline void ctxPop()
{
  if (gDepth > 0) gDepth--;
}

static inline JsonCtx* ctxCur()
{
  if (gDepth <= 0) return nullptr;
  return &gStack[gDepth - 1];
}

static inline void writeCommaIfNeeded()
{
  JsonCtx* c = ctxCur();
  if (!c) return;
  if (!c->first) gJson.print(",\n");
}

static inline void markWroteElement()
{
  JsonCtx* c = ctxCur();
  if (!c) return;
  c->first = false;
}

static void writeEscaped(const char* s)
{
  gJson.print('"');
  if (!s) { gJson.print('"'); return; }

  for (const char* p = s; *p; ++p)
  {
    const char ch = *p;
    switch (ch)
    {
      case '\\': gJson.print("\\\\"); break;
      case '\"': gJson.print("\\\""); break;
      case '\n': gJson.print("\\n");  break;
      case '\r': gJson.print("\\r");  break;
      case '\t': gJson.print("\\t");  break;
      default:   gJson.print(ch);     break;
    }
  }
  gJson.print('"');
}

static inline void writeKeyPrefix(const char* key)
{
  writeCommaIfNeeded();
  writeEscaped(key);
  gJson.print(": ");
  markWroteElement();
}

bool logJsonOpen(const char* path, int tftCsPin, int sdCsPin)
{
  spiSelect(sdCsPin);
  gJson = SD.open(path, FILE_WRITE);

  if (!gJson)
  {
    spiDeselect(sdCsPin);
    return false;
  }

  // Reset writer state
  gDepth = 0;
  ctxPush();

  gJson.print("{\n");
  gJson.flush();
  spiDeselect(sdCsPin);
  return true;
}

void logJsonClose(int tftCsPin, int sdCsPin)
{
  if (!gJson) return;
  spiSelect(sdCsPin);

  // Close any unclosed objects defensively
  while (gDepth > 1)
  {
    gJson.print("\n}");
    ctxPop();
  }

  gJson.print("\n}\n");
  gJson.flush();
  gJson.close();

  gDepth = 0;
  spiDeselect(sdCsPin);
}

bool logJsonIsOpen() { return (bool)gJson; }
fs::File logJsonFile() { return gJson; }

void logJsonBeginObject(const char* nameOrNull, int tftCsPin, int sdCsPin)
{
  if (!gJson) return;
  spiSelect(sdCsPin);

  if (nameOrNull && nameOrNull[0])
  {
    writeKeyPrefix(nameOrNull);
  }
  else
  {
    // Anonymous object in current context (rare here)
    writeCommaIfNeeded();
    markWroteElement();
  }

  gJson.print("{\n");
  ctxPush();

  spiDeselect(sdCsPin);
}

void logJsonEndObject(bool /*trailingComma*/, int tftCsPin, int sdCsPin)
{
  if (!gJson) return;
  spiSelect(sdCsPin);

  // End current object
  if (gDepth > 1) ctxPop();
  gJson.print("\n}");

  spiDeselect(sdCsPin);
}

void logJsonWriteKeyValueStr(const char* key, const char* value, int tftCsPin, int sdCsPin)
{
  if (!gJson) return;
  spiSelect(sdCsPin);

  writeKeyPrefix(key);
  writeEscaped(value ? value : "");

  spiDeselect(sdCsPin);
}

void logJsonWriteKeyValueU32(const char* key, uint32_t value, int tftCsPin, int sdCsPin)
{
  if (!gJson) return;
  spiSelect(sdCsPin);

  writeKeyPrefix(key);
  gJson.print(value);

  spiDeselect(sdCsPin);
}

void logJsonWriteKeyValueI32(const char* key, int32_t value, int tftCsPin, int sdCsPin)
{
  if (!gJson) return;
  spiSelect(sdCsPin);

  writeKeyPrefix(key);
  gJson.print(value);

  spiDeselect(sdCsPin);
}

void logJsonWriteKeyValueF(const char* key, float value, int decimals, int tftCsPin, int sdCsPin)
{
  if (!gJson) return;
  spiSelect(sdCsPin);

  writeKeyPrefix(key);
  gJson.print(value, decimals);

  spiDeselect(sdCsPin);
}

void logJsonWriteKeyValueBool(const char* key, bool value, int tftCsPin, int sdCsPin)
{
  if (!gJson) return;
  spiSelect(sdCsPin);

  writeKeyPrefix(key);
  gJson.print(value ? "true" : "false");

  spiDeselect(sdCsPin);
}

void logJsonWriteColumns(const char* cols[], size_t n, int tftCsPin, int sdCsPin)
{
  if (!gJson) return;
  spiSelect(sdCsPin);

  writeKeyPrefix("columns");
  gJson.print("[");
  for (size_t i = 0; i < n; i++)
  {
    if (i) gJson.print(",");
    writeEscaped(cols[i]);
  }
  gJson.print("]");

  spiDeselect(sdCsPin);
}

void logJsonCloseWithStopMs(uint32_t stopMs, int tftCsPin, int sdCsPin)
{
  if (!gJson) return;
  spiSelect(sdCsPin);

  // Write stop_ms at root level
  writeKeyPrefix("stop_ms");
  gJson.print(stopMs);

  // Close any unclosed objects defensively
  while (gDepth > 1)
  {
    gJson.print("\n}");
    ctxPop();
  }

  gJson.print("\n}\n");
  gJson.flush();
  gJson.close();

  gDepth = 0;
  spiDeselect(sdCsPin);
}
