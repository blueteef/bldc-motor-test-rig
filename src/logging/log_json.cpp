#include "bldc_rig/logging/log_json.h"
#include <string.h>

static fs::File gJson;

// Simple JSON writer with correct comma handling (no trailing commas).
// Supports nested objects and simple arrays written via logJsonWriteColumns.
struct JsonCtx {
  bool first;   // true until first element written in this object
};
static JsonCtx gStack[8];
static int gDepth = 0;

static int gTftCs = -1;
static int gSdCs  = -1;

static inline void spiSelectSd(int tftCsPin, int sdCsPin) {
  if (tftCsPin >= 0) digitalWrite(tftCsPin, HIGH);
  if (sdCsPin >= 0)  digitalWrite(sdCsPin, LOW);
}
static inline void spiDeselectSd(int tftCsPin, int sdCsPin) {
  if (sdCsPin >= 0)  digitalWrite(sdCsPin, HIGH);
  // Leave both deselected — Adafruit library handles its own TFT CS
}

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



// Defensive finalization: if the last non-whitespace byte is a comma, remove it.
// This guards against partial writes where a comma was emitted but the next key/value
// never made it to the file (e.g., brown-out/reset mid-print).
// Overwrites a trailing comma with whitespace (fs::File has no truncate()).
// Call this RIGHT BEFORE you write the final closing brace of the root object.
static void trimTrailingCommaBeforeClose(fs::File &f)
{
  if (!f) return;

  const size_t n = f.size();
  if (n == 0) return;

  // Walk backwards skipping whitespace. If we hit a comma, overwrite it with a space.
  for (int32_t i = (int32_t)n - 1; i >= 0; --i)
  {
    f.seek((uint32_t)i);
    const int c = f.read();
    if (c < 0) break;

    if (c == ' ' || c == '\t' || c == '\r' || c == '\n')
      continue;

    if (c == ',')
    {
      f.seek((uint32_t)i);
      f.write((uint8_t)' ');  // replace comma with whitespace
      f.flush();
    }
    break; // stop once we've processed the last non-whitespace char
  }
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
  gTftCs = tftCsPin;
  gSdCs  = sdCsPin;

  spiSelectSd(tftCsPin, sdCsPin);
  gJson = SD.open(path, FILE_WRITE);

  if (!gJson)
  {
    spiDeselectSd(tftCsPin, sdCsPin);
    return false;
  }

  // Reset writer state
  gDepth = 0;
  ctxPush();

  gJson.print("{\n");
  gJson.flush();
  spiDeselectSd(tftCsPin, sdCsPin);
  return true;
}

void logJsonClose(int tftCsPin, int sdCsPin)
{
  if (!gJson) return;
  spiSelectSd(tftCsPin, sdCsPin);

  // Close any unclosed objects defensively
  while (gDepth > 1)
  {
    gJson.print("\n}");
    ctxPop();
  }

  trimTrailingCommaBeforeClose(gJson);
  gJson.print("\n}\n");
  gJson.flush();
  gJson.close();

  gDepth = 0;
  spiDeselectSd(tftCsPin, sdCsPin);
}

bool logJsonIsOpen() { return (bool)gJson; }
fs::File logJsonFile() { return gJson; }

void logJsonBeginObject(const char* nameOrNull, int tftCsPin, int sdCsPin)
{
  if (!gJson) return;
  spiSelectSd(tftCsPin, sdCsPin);

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

  spiDeselectSd(tftCsPin, sdCsPin);
}

void logJsonEndObject(bool /*trailingComma*/, int tftCsPin, int sdCsPin)
{
  if (!gJson) return;
  spiSelectSd(tftCsPin, sdCsPin);

  // End current object
  if (gDepth > 1) ctxPop();
  gJson.print("\n}");

  spiDeselectSd(tftCsPin, sdCsPin);
}

void logJsonWriteKeyValueStr(const char* key, const char* value, int tftCsPin, int sdCsPin)
{
  if (!gJson) return;
  spiSelectSd(tftCsPin, sdCsPin);

  writeKeyPrefix(key);
  writeEscaped(value ? value : "");

  spiDeselectSd(tftCsPin, sdCsPin);
}

void logJsonWriteKeyValueU32(const char* key, uint32_t value, int tftCsPin, int sdCsPin)
{
  if (!gJson) return;
  spiSelectSd(tftCsPin, sdCsPin);

  writeKeyPrefix(key);
  gJson.print(value);

  spiDeselectSd(tftCsPin, sdCsPin);
}

void logJsonWriteKeyValueI32(const char* key, int32_t value, int tftCsPin, int sdCsPin)
{
  if (!gJson) return;
  spiSelectSd(tftCsPin, sdCsPin);

  writeKeyPrefix(key);
  gJson.print(value);

  spiDeselectSd(tftCsPin, sdCsPin);
}

void logJsonWriteKeyValueF(const char* key, float value, int decimals, int tftCsPin, int sdCsPin)
{
  if (!gJson) return;
  spiSelectSd(tftCsPin, sdCsPin);

  writeKeyPrefix(key);
  gJson.print(value, decimals);

  spiDeselectSd(tftCsPin, sdCsPin);
}

void logJsonWriteKeyValueBool(const char* key, bool value, int tftCsPin, int sdCsPin)
{
  if (!gJson) return;
  spiSelectSd(tftCsPin, sdCsPin);

  writeKeyPrefix(key);
  gJson.print(value ? "true" : "false");

  spiDeselectSd(tftCsPin, sdCsPin);
}

void logJsonWriteColumns(const char* const* cols, size_t n, int tftCsPin, int sdCsPin)
{
  if (!gJson) return;
  spiSelectSd(tftCsPin, sdCsPin);

  writeKeyPrefix("columns");
  gJson.print("[");
  for (size_t i = 0; i < n; i++)
  {
    if (i) gJson.print(",");
    writeEscaped(cols[i]);
  }
  gJson.print("]");

  spiDeselectSd(tftCsPin, sdCsPin);
}

void logJsonCloseWithStopMs(uint32_t stopMs, int tftCsPin, int sdCsPin)
{
  if (!gJson) return;
  spiSelectSd(tftCsPin, sdCsPin);

  // Write stop_ms at root level
  writeKeyPrefix("stop_ms");
  gJson.print(stopMs);

  // Close any unclosed objects defensively
  while (gDepth > 1)
  {
    gJson.print("\n}");
    ctxPop();
  }

  trimTrailingCommaBeforeClose(gJson);
  gJson.print("\n}\n");
  gJson.flush();
  gJson.close();

  gDepth = 0;
  spiDeselectSd(tftCsPin, sdCsPin);
}
