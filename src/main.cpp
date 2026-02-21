// ============================================================
//  RusEFI Teensy 4.1 Data Logger — main.cpp
// ============================================================
//
//  Boot sequence
//  1. Init SD card (halt + solid LED on failure)
//  2. Start USB host, wait for ECU
//  3. Assert DTR → send 'S' → read firmware signature (text mode)
//  4. Hash signature → look for <XXXXXXXX>.INI on SD card
//     Falls back to DEFAULT.INI if hash file not present
//  5. Parse INI: ochBlockSize + [OutputChannels] channel table
//  6. Open next log file (YYYYMMDD/HHMMSS.msl or LOG001.msl fallback)
//  7. Send 'F' once to activate CRC binary protocol
//  8. Poll ECU with CRC-framed 'O' at 20 Hz; decode blob; write MSL rows
//  9. On USB disconnect: flush/close log, return to step 2
//
//  SD card layout
//  /<XXXXXXXX>.INI   — INI named by djb2 hash of ECU signature
//  /DEFAULT.INI      — fallback for single-tune setups
//  /"Feb 21 2026"/"1201pm Feb 21.msl" — timestamped logs (requires valid RTC)
//  /LOG001.msl …     — sequential fallback if RTC time is invalid
//
//  Serial commands
//  t  — set internal RTC to compile time (use after battery replacement)
//  s  — stop logging; SD accessible via MTP; power-cycle to resume
//
//  LED patterns (pin 13)
//  Slow blink  1 Hz    — waiting for ECU
//  Fast blink  5 Hz    — connecting / handshake
//  Short flash 1 Hz    — logging OK
//  Medium blink 2.5 Hz — stopped, SD on MTP
//  Solid on            — error (SD or INI)
// ============================================================

#include <Arduino.h>
#include <USBHost_t36.h>
#include <SD.h>       // Teensy's SD wraps SdFat (SdFs) — supports exFAT + FS& for MTP
#include <TimeLib.h>  // Teensy 4.1 internal RTC (backed by VBAT)
#ifndef DISABLE_MTP
#include <MTP_Teensy.h>
#endif

// ─── Configuration ──────────────────────────────────────────
static constexpr uint8_t  LED_PIN          = 13;
static constexpr uint16_t MAX_CHANNELS     = 300;
static constexpr uint16_t OCH_BUF_SIZE     = 2948;  // must be >= ochBlockSize
static constexpr uint32_t POLL_INTERVAL_MS = 50;    // 20 Hz
static constexpr uint32_t SYNC_INTERVAL_MS = 1000;  // max data loss on hard power-off

// ─── Channel descriptor ─────────────────────────────────────
enum TypeCode : uint8_t {
    TC_U08 = 0, TC_S08,
    TC_U16,     TC_S16,
    TC_U32,     TC_S32,
    TC_F32,
    TC_UNKNOWN = 0xFF
};

struct Channel {
    char     name[24];
    char     unit[12];
    uint16_t offset;
    TypeCode tc;
    float    mul;
    float    add;
};

struct DLChannel {
    char     label[40];
    uint16_t chanIdx;
    bool     isFloat;
};

// ─── State machine ──────────────────────────────────────────
enum class State : uint8_t {
    WaitDevice, AssertDTR, GetSignature, LoadINI,
    Logging,
    Stopped,   // 's' command — MTP accessible, power-cycle to resume
    ErrorSD, ErrorINI,
};

// ─── USB host ───────────────────────────────────────────────
USBHost             myusb;
USBHub              hub1(myusb);
USBSerial_BigBuffer userial(myusb, 1);

// ─── RTC ────────────────────────────────────────────────────
bool rtcOK = false;   // true when internal RTC holds a valid time (year >= 2024)

// ─── SD ─────────────────────────────────────────────────────
File logFile;

// ─── Channel table ──────────────────────────────────────────
Channel  channels[MAX_CHANNELS];
uint16_t numChannels  = 0;
uint16_t ochBlockSize = 0;
uint8_t  ochBuffer[OCH_BUF_SIZE];

DLChannel dlChannels[MAX_CHANNELS];
uint16_t  numDLChannels = 0;

// ─── State vars ─────────────────────────────────────────────
State    state        = State::WaitDevice;
uint32_t stateEnterMs = 0;
uint32_t lastPollMs   = 0;
uint32_t logStartMs   = 0;
uint32_t lastSyncMs   = 0;
bool     logOpen      = false;
char     signature[64]   = {};
char     iniFilename[13] = {};

// ─────────────────────────────────────────────────────────────
//  LED
// ─────────────────────────────────────────────────────────────
struct LedPattern { uint16_t onMs; uint16_t offMs; };
constexpr LedPattern PAT_WAIT    = {500, 500};
constexpr LedPattern PAT_CONNECT = {100, 100};
constexpr LedPattern PAT_LOG     = { 50, 950};
constexpr LedPattern PAT_MTP     = {200, 200};
constexpr LedPattern PAT_ERROR   = {  0,   0};  // solid on

const LedPattern* ledPat   = &PAT_WAIT;
uint32_t          ledTimer = 0;
bool              ledOn    = false;

static void setLED(const LedPattern* p) { ledPat = p; ledTimer = millis(); }

static void updateLED() {
    if (!ledPat->onMs && !ledPat->offMs) { digitalWrite(LED_PIN, HIGH); return; }
    uint32_t period = ledOn ? ledPat->onMs : ledPat->offMs;
    if ((uint32_t)(millis() - ledTimer) >= period) {
        ledOn = !ledOn;
        digitalWrite(LED_PIN, ledOn ? HIGH : LOW);
        ledTimer = millis();
    }
}

// ─────────────────────────────────────────────────────────────
//  RTC helpers
// ─────────────────────────────────────────────────────────────
static time_t rtcGetTime() { return Teensy3Clock.get(); }

// Parse __DATE__ / __TIME__ compiler macros into a time_t.
static time_t compileTime() {
    const char* months = "JanFebMarAprMayJunJulAugSepOctNovDec";
    char ms[4] = { __DATE__[0], __DATE__[1], __DATE__[2], '\0' };
    tmElements_t tm;
    tm.Month  = (strstr(months, ms) - months) / 3 + 1;
    tm.Day    = atoi(__DATE__ + 4);
    tm.Year   = atoi(__DATE__ + 7) - 1970;
    tm.Hour   = atoi(__TIME__ + 0);
    tm.Minute = atoi(__TIME__ + 3);
    tm.Second = atoi(__TIME__ + 6);
    return makeTime(tm);
}

// ─────────────────────────────────────────────────────────────
//  String helpers
// ─────────────────────────────────────────────────────────────
static void trimRight(char* s) {
    int n = (int)strlen(s);
    while (n > 0 && (s[n-1]==' '||s[n-1]=='\t'||s[n-1]=='\r'||s[n-1]=='\n'))
        s[--n] = '\0';
}

static bool readLine(File& f, char* buf, size_t maxLen) {
    size_t i = 0;
    int c;
    while ((c = f.read()) >= 0) {
        if (c == '\n') break;
        if (c == '\r') continue;
        if (i < maxLen - 1) buf[i++] = (char)c;
    }
    buf[i] = '\0';
    return (i > 0 || c >= 0);
}

// ─────────────────────────────────────────────────────────────
//  INI filename from signature hash
// ─────────────────────────────────────────────────────────────
static uint32_t djb2(const char* s) {
    uint32_t h = 5381;
    while (*s) h = ((h << 5) + h) ^ (uint8_t)*s++;
    return h;
}

static void sigToFilename(const char* sig, char* out, size_t outLen) {
    snprintf(out, outLen, "%08lX.INI", (unsigned long)djb2(sig));
}

// ─────────────────────────────────────────────────────────────
//  INI parser
// ─────────────────────────────────────────────────────────────
static TypeCode strToTC(const char* t) {
    if (!strcmp(t,"U08")||!strcmp(t,"UBYTE")) return TC_U08;
    if (!strcmp(t,"S08")||!strcmp(t,"BYTE"))  return TC_S08;
    if (!strcmp(t,"U16")||!strcmp(t,"UINT"))  return TC_U16;
    if (!strcmp(t,"S16")||!strcmp(t,"INT"))   return TC_S16;
    if (!strcmp(t,"U32")||!strcmp(t,"ULONG")) return TC_U32;
    if (!strcmp(t,"S32")||!strcmp(t,"LONG"))  return TC_S32;
    if (!strcmp(t,"F32")||!strcmp(t,"FLOAT")) return TC_F32;
    return TC_UNKNOWN;
}

static void consumeField(const char*& p, char* out, size_t outLen) {
    while (*p == ' ' || *p == '\t') p++;
    size_t i = 0;
    bool quoted = (*p == '"');
    if (quoted) p++;
    while (*p) {
        if ( quoted && *p == '"') { p++; break; }
        if (!quoted && *p == ',') break;
        if (i < outLen - 1) out[i++] = *p;
        p++;
    }
    out[i] = '\0';
    trimRight(out);
    while (*p == ' ' || *p == '\t') p++;
    if (*p == ',') p++;
}

static bool parseChannelLine(const char* line, Channel& ch) {
    const char* eq = strchr(line, '=');
    if (!eq) return false;

    size_t ni = 0;
    for (const char* p = line; p < eq && ni < sizeof(ch.name)-1; p++)
        if (*p != ' ' && *p != '\t') ch.name[ni++] = *p;
    ch.name[ni] = '\0';
    if (ni == 0) return false;

    const char* p = eq + 1;
    while (*p == ' ' || *p == '\t') p++;
    if (strncmp(p, "scalar", 6) != 0) return false;
    p += 6;
    while (*p == ' ' || *p == '\t') p++;
    if (*p == ',') p++;

    char f[32];
    consumeField(p, f, sizeof(f));
    TypeCode tc = strToTC(f);
    if (tc == TC_UNKNOWN) return false;

    consumeField(p, f, sizeof(f));
    uint16_t offset = (uint16_t)atoi(f);
    if (offset >= OCH_BUF_SIZE) return false;

    consumeField(p, ch.unit, sizeof(ch.unit));
    consumeField(p, f, sizeof(f)); float mul = atof(f);
    consumeField(p, f, sizeof(f)); float add = atof(f);

    ch.offset = offset; ch.tc = tc; ch.mul = mul; ch.add = add;
    return true;
}

static int16_t findChannelByName(const char* name) {
    for (uint16_t i = 0; i < numChannels; i++)
        if (strcmp(channels[i].name, name) == 0) return (int16_t)i;
    return -1;
}

static bool parseINI(const char* filename) {
    Serial.print("[INI] Reading: "); Serial.println(filename);
    File f = SD.open(filename, FILE_READ);
    if (!f) { Serial.println("[INI] File not found!"); return false; }

    numChannels = 0; ochBlockSize = 0; numDLChannels = 0;
    bool inOCH = false, inDL = false;
    char line[256];
    uint16_t lineCount = 0;

    while (readLine(f, line, sizeof(line))) {
        if (++lineCount % 50 == 0) myusb.Task();
        char* sc = strchr(line, ';'); if (sc) *sc = '\0';
        trimRight(line);
        char* lp = line;
        while (*lp == ' ' || *lp == '\t') lp++;
        if (lp != line) memmove(line, lp, strlen(lp) + 1);
        if (line[0] == '\0') continue;

        if (line[0] == '[') {
            inOCH = (strncmp(line, "[OutputChannels]", 16) == 0);
            inDL  = (strncmp(line, "[Datalog]",         9) == 0);
            continue;
        }

        if (ochBlockSize == 0 && strncmp(line, "ochBlockSize", 12) == 0) {
            const char* eq = strchr(line, '=');
            if (eq) ochBlockSize = (uint16_t)atoi(eq + 1);
        }

        if (inOCH && numChannels < MAX_CHANNELS) {
            Channel ch = {};
            if (parseChannelLine(line, ch)) channels[numChannels++] = ch;
        }

        if (inDL && numDLChannels < MAX_CHANNELS && strncmp(line, "entry", 5) == 0) {
            const char* eq = strchr(line, '=');
            if (eq) {
                const char* p = eq + 1;
                char name[24] = {}, lbl[40] = {}, typeStr[8] = {};
                consumeField(p, name,    sizeof(name));
                consumeField(p, lbl,     sizeof(lbl));
                consumeField(p, typeStr, sizeof(typeStr));
                int16_t idx = findChannelByName(name);
                if (idx >= 0) {
                    DLChannel& dl = dlChannels[numDLChannels++];
                    strncpy(dl.label, lbl, sizeof(dl.label) - 1);
                    dl.label[sizeof(dl.label) - 1] = '\0';
                    dl.chanIdx = (uint16_t)idx;
                    dl.isFloat = (strcmp(typeStr, "float") == 0);
                }
            }
        }
    }
    f.close();

    Serial.print("[INI] Channels: "); Serial.print(numChannels);
    Serial.print("  ochBlockSize: "); Serial.print(ochBlockSize);
    Serial.print("  Datalog: "); Serial.println(numDLChannels);

    if (ochBlockSize == 0)         { Serial.println("[INI] ERROR: ochBlockSize not found."); return false; }
    if (ochBlockSize > OCH_BUF_SIZE) {
        Serial.print("[INI] ERROR: ochBlockSize="); Serial.print(ochBlockSize);
        Serial.print(" > OCH_BUF_SIZE="); Serial.println(OCH_BUF_SIZE);
        return false;
    }
    if (numChannels == 0)          { Serial.println("[INI] ERROR: No scalar channels parsed."); return false; }
    return true;
}

// ─────────────────────────────────────────────────────────────
//  Log file management
// ─────────────────────────────────────────────────────────────
static bool openNextLogFile() {
    char name[40];

    if (rtcOK) {
        time_t t = now();
        static const char* const mo[] = {
            "", "Jan", "Feb", "Mar", "Apr", "May", "Jun",
            "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
        };
        int h = hour(t);
        const char* ampm = (h < 12) ? "am" : "pm";
        int h12 = h % 12; if (h12 == 0) h12 = 12;

        char folder[16];
        snprintf(folder, sizeof(folder), "%s %d %04d", mo[month(t)], day(t), year(t));
        if (!SD.exists(folder)) {
            if (!SD.mkdir(folder)) {
                Serial.print("[SD] Cannot create folder "); Serial.println(folder);
                return false;
            }
        }

        char base[16];
        snprintf(base, sizeof(base), "%02d%02d%s %s %d", h12, minute(t), ampm, mo[month(t)], day(t));

        snprintf(name, sizeof(name), "%s/%s.msl", folder, base);
        if (SD.exists(name)) {
            bool found = false;
            for (int i = 1; i <= 99; i++) {
                snprintf(name, sizeof(name), "%s/%s_%02d.msl", folder, base, i);
                if (!SD.exists(name)) { found = true; break; }
            }
            if (!found) {
                Serial.println("[SD] Cannot find free timestamped slot!");
                return false;
            }
        }
    } else {
        bool found = false;
        for (int i = 1; i <= 999; i++) {
            snprintf(name, sizeof(name), "LOG%03d.msl", i);
            if (!SD.exists(name)) { found = true; break; }
        }
        if (!found) {
            Serial.println("[SD] No free log slot (LOG001–LOG999 all exist)!");
            return false;
        }
    }

    logFile = SD.open(name, FILE_WRITE);
    if (!logFile) {
        Serial.print("[SD] Cannot create "); Serial.println(name);
        return false;
    }
    Serial.print("[SD] Log: "); Serial.println(name);
    return true;
}

static void writeHeader() {
    logFile.print("Time");
    if (numDLChannels > 0) {
        for (uint16_t i = 0; i < numDLChannels; i++) {
            logFile.print('\t'); logFile.print(dlChannels[i].label);
        }
    } else {
        for (uint16_t i = 0; i < numChannels; i++) {
            logFile.print('\t'); logFile.print(channels[i].name);
        }
    }
    logFile.println();
    logFile.print("s");
    if (numDLChannels > 0) {
        for (uint16_t i = 0; i < numDLChannels; i++) {
            logFile.print('\t'); logFile.print(channels[dlChannels[i].chanIdx].unit);
        }
    } else {
        for (uint16_t i = 0; i < numChannels; i++) {
            logFile.print('\t'); logFile.print(channels[i].unit);
        }
    }
    logFile.println();
    logFile.flush();
}

// ─────────────────────────────────────────────────────────────
//  Blob decoding
// ─────────────────────────────────────────────────────────────
static float decodeChannel(const uint8_t* blob, const Channel& ch) {
    const uint8_t* src = blob + ch.offset;
    float raw = 0;
    switch (ch.tc) {
        case TC_U08: raw = (float)src[0]; break;
        case TC_S08: raw = (float)(int8_t)src[0]; break;
        case TC_U16: { uint16_t v; memcpy(&v, src, 2); raw = (float)v; break; }
        case TC_S16: { int16_t  v; memcpy(&v, src, 2); raw = (float)v; break; }
        case TC_U32: { uint32_t v; memcpy(&v, src, 4); raw = (float)v; break; }
        case TC_S32: { int32_t  v; memcpy(&v, src, 4); raw = (float)v; break; }
        case TC_F32: {             memcpy(&raw,  src, 4);              break; }
        default: break;
    }
    return raw * ch.mul + ch.add;
}

static void writeRow(uint32_t nowMs) {
    float timeSec = (float)(nowMs - logStartMs) / 1000.0f;
    char buf[20];
    dtostrf(timeSec, 1, 3, buf);
    logFile.print(buf);

    uint16_t count = numDLChannels > 0 ? numDLChannels : numChannels;
    for (uint16_t i = 0; i < count; i++) {
        const Channel& ch = numDLChannels > 0
            ? channels[dlChannels[i].chanIdx] : channels[i];
        bool asFloat = numDLChannels > 0 ? dlChannels[i].isFloat : true;
        float val = decodeChannel(ochBuffer, ch);
        logFile.print('\t');
        if (asFloat) { dtostrf(val, 1, 3, buf); logFile.print(buf); }
        else         { logFile.print((int32_t)val); }
    }
    logFile.println();
}

// ─────────────────────────────────────────────────────────────
//  RusEFI communication
// ─────────────────────────────────────────────────────────────
static void flushSerial() { while (userial.available()) userial.read(); }

static bool readResponse(char* dst, size_t maxLen, uint32_t timeoutMs) {
    size_t i = 0;
    uint32_t deadline = millis() + timeoutMs;
    while (millis() < deadline) {
        myusb.Task();
        while (userial.available()) {
            uint8_t c = userial.read();
            if (c == '\0' || c == '\n') { dst[i] = '\0'; return (i > 0); }
            if (c >= 0x20 && i < maxLen - 1) {
                dst[i++] = c;
                deadline = millis() + 500;
            }
        }
    }
    dst[i] = '\0';
    return (i > 0);
}

static uint32_t crc32(const uint8_t* data, size_t len) {
    uint32_t crc = 0xFFFFFFFF;
    while (len--) {
        crc ^= *data++;
        for (int i = 0; i < 8; i++)
            crc = (crc & 1) ? ((crc >> 1) ^ 0xEDB88320) : (crc >> 1);
    }
    return ~crc;
}

static bool requestOCH() {
    const uint8_t offL = 0, offH = 0;
    const uint8_t cntL = (uint8_t)( ochBlockSize       & 0xFF);
    const uint8_t cntH = (uint8_t)((ochBlockSize >> 8) & 0xFF);

    uint8_t pl[5] = {'O', offL, offH, cntL, cntH};
    uint32_t checksum = crc32(pl, 5);
    flushSerial();

    uint8_t frame[11] = {
        0x00, 0x05, 'O', offL, offH, cntL, cntH,
        (uint8_t)(checksum >> 24), (uint8_t)(checksum >> 16),
        (uint8_t)(checksum >> 8 ), (uint8_t)(checksum      )
    };
    userial.write(frame, 11);
    myusb.Task(); myusb.Task();

    static uint8_t rxBuf[OCH_BUF_SIZE + 8];
    uint16_t rx = 0;
    const uint16_t toRead = ochBlockSize + 7;
    uint32_t deadline = millis() + 1500;
    while (rx < toRead && millis() < deadline) {
        myusb.Task();
        while (userial.available() && rx < toRead) {
            rxBuf[rx++] = userial.read();
            deadline = millis() + 200;
        }
    }

    if (rx == 0) { Serial.println("[ECU] No response"); return false; }

    if (rx >= (uint16_t)(ochBlockSize + 3) && rxBuf[2] == 0x00) {
        memcpy(ochBuffer, rxBuf + 3, ochBlockSize);
        return true;
    }

    Serial.print("[ECU] Bad rx="); Serial.print(rx);
    Serial.print(" first16: ");
    for (int i = 0; i < min((int)rx, 16); i++) {
        if (rxBuf[i] < 0x10) Serial.print('0');
        Serial.print(rxBuf[i], HEX); Serial.print(' ');
    }
    Serial.println();
    return false;
}

// ─────────────────────────────────────────────────────────────
//  State helpers
// ─────────────────────────────────────────────────────────────
static void enterState(State s) { state = s; stateEnterMs = millis(); }

static void onDisconnect() {
    Serial.println("[USB] ECU disconnected.");
    if (logOpen) {
        logFile.flush();
        logFile.close();
        logOpen = false;
        Serial.println("[SD]  Log closed.");
    }
    numChannels = 0; numDLChannels = 0; ochBlockSize = 0; signature[0] = '\0';
    setLED(&PAT_WAIT);
    enterState(State::WaitDevice);
}

// ─────────────────────────────────────────────────────────────
//  setup()
// ─────────────────────────────────────────────────────────────
void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    Serial.begin(115200);

    Serial.println("================================");
    Serial.println(" RusEFI Teensy 4.1 Data Logger ");
    Serial.println("================================");

#ifndef DISABLE_MTP
    MTP.begin();
#endif

    Serial.print("[SD]  Init... ");
    if (!SD.begin(BUILTIN_SDCARD)) {
        Serial.println("FAILED — check card is inserted.");
        setLED(&PAT_ERROR);
        enterState(State::ErrorSD);
    } else {
        Serial.println("OK");
#ifndef DISABLE_MTP
        MTP.addFilesystem(SD, "TeensySDLogger");
        Serial.println("[MTP] SD registered as TeensySDLogger.");
#endif
        setLED(&PAT_WAIT);
        enterState(State::WaitDevice);
    }

    // Internal RTC — auto-update to compile time if stored time is stale.
    // On every flash the compiled timestamp advances, so the RTC stays fresh.
    Serial.print("[RTC] ");
    setSyncProvider(rtcGetTime);
    time_t compiled = compileTime();
    if (Teensy3Clock.get() < compiled) {
        Teensy3Clock.set(compiled);
        setTime(compiled);
        Serial.print("updated to compile time: ");
    }
    if (year() >= 2024) {
        rtcOK = true;
        char tbuf[24];
        snprintf(tbuf, sizeof(tbuf), "%04d-%02d-%02d %02d:%02d:%02d",
            year(), month(), day(), hour(), minute(), second());
        Serial.println(tbuf);
    } else {
        Serial.println("time invalid — using sequential filenames.");
    }

    myusb.begin();
    Serial.println("[USB] Host started. Waiting for ECU...");
}

// ─────────────────────────────────────────────────────────────
//  loop()
// ─────────────────────────────────────────────────────────────
void loop() {
#ifndef DISABLE_MTP
    MTP.loop();
#endif
    myusb.Task();
    updateLED();

    if (Serial.available()) {
        char cmd = (char)Serial.read();

        if (cmd == 't' || cmd == 'T') {
            // Set RTC to compile time — use after VBAT replacement
            time_t ct = compileTime();
            Teensy3Clock.set(ct);
            setTime(ct);
            rtcOK = true;
            char tbuf[24];
            snprintf(tbuf, sizeof(tbuf), "%04d-%02d-%02d %02d:%02d:%02d",
                year(), month(), day(), hour(), minute(), second());
            Serial.print("[RTC] Set to compile time: "); Serial.println(tbuf);
        }

        if (cmd == 's' || cmd == 'S') {
            if (state == State::Logging) {
                if (logOpen) { logFile.flush(); logFile.close(); logOpen = false; }
                Serial.println("[CMD] Logging stopped. Power-cycle to resume.");
#ifndef DISABLE_MTP
                MTP.send_DeviceResetEvent();
#endif
            }
            setLED(&PAT_MTP);
            enterState(State::Stopped);
        }
    }

    if (state == State::ErrorSD) return;

    if (!userial && state != State::WaitDevice && state != State::Stopped) {
        onDisconnect();
        return;
    }

    switch (state) {

    case State::WaitDevice:
        if (userial) {
            Serial.println("[USB] ECU detected.");
            setLED(&PAT_CONNECT);
            enterState(State::AssertDTR);
        }
        break;

    case State::AssertDTR:
        if (millis() - stateEnterMs >= 300) {
            Serial.println("[USB] Asserting DTR + RTS...");
            userial.setDTR(true);
            userial.setRTS(true);
            delay(200);
            flushSerial();
            Serial.println("[TS]  Requesting signature...");
            userial.write('S');
            enterState(State::GetSignature);
        }
        break;

    case State::GetSignature:
        if (userial.available()) {
            if (readResponse(signature, sizeof(signature), 2000)) {
                Serial.print("[ECU] Signature: "); Serial.println(signature);
                sigToFilename(signature, iniFilename, sizeof(iniFilename));
                Serial.print("[INI] Looking for: "); Serial.print(iniFilename);
                Serial.println(" (fallback: DEFAULT.INI)");
                enterState(State::LoadINI);
            } else {
                Serial.println("[ECU] Empty response — retrying...");
                delay(1000);
                flushSerial();
                userial.write('S');
                stateEnterMs = millis();
            }
        } else if (millis() - stateEnterMs > 4000) {
            Serial.println("[ECU] Signature timeout — retrying...");
            flushSerial();
            userial.write('S');
            stateEnterMs = millis();
        }
        break;

    case State::LoadINI: {
        bool ok = false;
        if (SD.exists(iniFilename))       { ok = parseINI(iniFilename); }
        else if (SD.exists("DEFAULT.INI")) { Serial.println("[INI] Using DEFAULT.INI"); ok = parseINI("DEFAULT.INI"); }
        else {
            Serial.println("[INI] No INI found on SD card!");
            Serial.print  ("[INI] Expected: "); Serial.println(iniFilename);
            Serial.println("[INI] Or rename your INI to DEFAULT.INI");
        }

        if (ok) {
            Serial.println("[TS]  Sending 'F' (CRC binary mode)...");
            flushSerial();
            userial.write('F');
            char fResp[8] = {};
            readResponse(fResp, sizeof(fResp), 1000);
            Serial.print("[TS]  F response: \""); Serial.print(fResp); Serial.println("\"");
            flushSerial();
            delay(50);

            if (openNextLogFile()) {
                logStartMs = millis();
                lastSyncMs = millis();
                writeHeader();
                logOpen    = true;
                lastPollMs = millis();
                setLED(&PAT_LOG);
                enterState(State::Logging);
                Serial.print("[LOG] Logging ");
                Serial.print(numDLChannels > 0 ? numDLChannels : numChannels);
                Serial.println(" channels. Go!");
            } else {
                setLED(&PAT_ERROR);
                enterState(State::ErrorINI);
            }
        } else {
            setLED(&PAT_ERROR);
            enterState(State::ErrorINI);
        }
        break;
    }

    case State::Logging:
        if (millis() - lastPollMs >= POLL_INTERVAL_MS) {
            lastPollMs = millis();
            if (requestOCH()) writeRow(millis());
        }
        if (logOpen && (uint32_t)(millis() - lastSyncMs) >= SYNC_INTERVAL_MS) {
            lastSyncMs = millis();
            logFile.flush();
        }
        break;

    case State::Stopped:
        break;

    case State::ErrorINI:
        if (millis() - stateEnterMs > 10000) {
            stateEnterMs = millis();
            Serial.println("[ERR] No INI — power-cycle after inserting SD.");
            Serial.print  ("[ERR] Expected: "); Serial.println(iniFilename);
            Serial.println("[ERR] Or: DEFAULT.INI");
        }
        break;

    default: break;
    }
}
