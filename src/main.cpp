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
//  6. Open next free LOGxxx.msl, write MSL header (two tab-separated rows)
//  6. Send 'F' → "001" once to activate CRC binary protocol
//  7. Poll ECU with CRC-framed 'O' at 20 Hz; decode blob; write MSL rows
//  9. On USB disconnect: flush/close log, return to step 2
//
//  SD card layout
//  /<XXXXXXXX>.INI   — INI named by djb2 hash of signature
//  /DEFAULT.INI      — fallback (rename your INI to this for
//                      single-tune setups)
//  /LOG001.msl …     — auto-incremented log files (MegaLogViewer native format)
//
//  Serial commands (USB device port)
//  s  — stop logging; SD immediately accessible via MTP; power-cycle to resume
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
#include <SdFat.h>
#include <RTClib.h>
#include <MTP_Teensy.h>

// ─── Configuration ──────────────────────────────────────────
static constexpr uint8_t  LED_PIN          = 13;
static constexpr uint16_t MAX_CHANNELS     = 300;   // max parsed channels
static constexpr uint16_t OCH_BUF_SIZE     = 2948;  // must be >= ochBlockSize
static constexpr uint32_t POLL_INTERVAL_MS = 50;    // 20 Hz
static constexpr uint32_t SYNC_INTERVAL_MS = 1000;  // SD sync cadence — max data loss on power-off

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

// [Datalog] ordered channel list — subset of channels[], in [Datalog] order.
// Populated from the [Datalog] section; if empty we fall back to all channels.
struct DLChannel {
    char     label[40];   // display label from [Datalog]
    uint16_t chanIdx;     // index into channels[]
    bool     isFloat;     // false → output as int32_t (no decimal places)
};

// ─── State machine ──────────────────────────────────────────
enum class State : uint8_t {
    WaitDevice,
    AssertDTR,
    GetSignature,
    LoadINI,
    Logging,
    Stopped,    // logging explicitly stopped via 's' command; SD accessible via MTP
    ErrorSD,
    ErrorINI,
};

// ─── USB host ───────────────────────────────────────────────
USBHost             myusb;
USBHub              hub1(myusb);
USBSerial_BigBuffer userial(myusb, 1);  // claim all CDC devices

// ─── RTC ────────────────────────────────────────────────────
RTC_DS3231 rtc;
bool       rtcOK = false;   // true once RTC is present and time is set

// ─── SD ─────────────────────────────────────────────────────
SdFs   sd;
FsFile logFile;

// ─── Channel table (runtime, from INI) ──────────────────────
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
uint32_t rowCount     = 0;
uint32_t logStartMs   = 0;   // millis() when current log file was opened
uint32_t lastSyncMs   = 0;   // millis() of last SD sync
bool     logOpen      = false;
char     signature[64]   = {};
char     iniFilename[13] = {};   // 8.3 FAT filename

// ─────────────────────────────────────────────────────────────
//  LED
// ─────────────────────────────────────────────────────────────
struct LedPattern { uint16_t onMs; uint16_t offMs; };
constexpr LedPattern PAT_WAIT    = {500, 500};   // 1 Hz
constexpr LedPattern PAT_CONNECT = {100, 100};   // 5 Hz
constexpr LedPattern PAT_LOG     = { 50, 950};   // short flash
constexpr LedPattern PAT_MTP     = {200, 200};   // 2.5 Hz — stopped, SD on MTP
constexpr LedPattern PAT_ERROR   = {  0,   0};   // solid on

const LedPattern* ledPat  = &PAT_WAIT;
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
//  String helpers
// ─────────────────────────────────────────────────────────────
static void trimRight(char* s) {
    int n = (int)strlen(s);
    while (n > 0 && (s[n-1]==' '||s[n-1]=='\t'||s[n-1]=='\r'||s[n-1]=='\n'))
        s[--n] = '\0';
}

// Read one line from FsFile (strips \r, breaks on \n or EOF).
// Returns true while there is still data (or a non-empty line just read).
static bool readLine(FsFile& f, char* buf, size_t maxLen) {
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

// Consume one comma-delimited field from *p (handles "quoted" fields).
// Advances *p past the field and the following comma.
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

// Parse one [OutputChannels] line into ch.
// Format: name = scalar, TYPE, OFFSET, "unit", MUL, ADD[, ...]
static bool parseChannelLine(const char* line, Channel& ch) {
    const char* eq = strchr(line, '=');
    if (!eq) return false;

    // Name: characters before '=', no whitespace
    size_t ni = 0;
    for (const char* p = line; p < eq && ni < sizeof(ch.name)-1; p++)
        if (*p != ' ' && *p != '\t') ch.name[ni++] = *p;
    ch.name[ni] = '\0';
    if (ni == 0) return false;

    const char* p = eq + 1;
    while (*p == ' ' || *p == '\t') p++;

    if (strncmp(p, "scalar", 6) != 0) return false;    // skip bits/arrays
    p += 6;
    while (*p == ' ' || *p == '\t') p++;
    if (*p == ',') p++;

    char f[32];   // must hold scientific notation like "3.333333333333333E-4" (20 chars)

    consumeField(p, f, sizeof(f));
    TypeCode tc = strToTC(f);
    if (tc == TC_UNKNOWN) return false;

    consumeField(p, f, sizeof(f));
    uint16_t offset = (uint16_t)atoi(f);
    if (offset >= OCH_BUF_SIZE) return false;

    consumeField(p, ch.unit, sizeof(ch.unit));

    consumeField(p, f, sizeof(f));
    float mul = atof(f);

    consumeField(p, f, sizeof(f));
    float add = atof(f);

    ch.offset = offset;
    ch.tc     = tc;
    ch.mul    = mul;
    ch.add    = add;
    return true;
}

static int16_t findChannelByName(const char* name) {
    for (uint16_t i = 0; i < numChannels; i++)
        if (strcmp(channels[i].name, name) == 0) return (int16_t)i;
    return -1;
}

static bool parseINI(const char* filename) {
    Serial.print("[INI] Reading: "); Serial.println(filename);
    FsFile f = sd.open(filename, FILE_READ);
    if (!f) { Serial.println("[INI] File not found!"); return false; }

    numChannels   = 0;
    ochBlockSize  = 0;
    numDLChannels = 0;
    bool inOCH    = false;
    bool inDL     = false;
    char line[256];

    uint16_t lineCount = 0;
    while (readLine(f, line, sizeof(line))) {
        if (++lineCount % 50 == 0) myusb.Task(); // keep USB alive during long SD read
        // strip inline comment
        char* sc = strchr(line, ';');
        if (sc) *sc = '\0';
        trimRight(line);
        // trim leading whitespace (INI lines are often tab-indented)
        char* lp = line;
        while (*lp == ' ' || *lp == '\t') lp++;
        if (lp != line) memmove(line, lp, strlen(lp) + 1);
        if (line[0] == '\0') continue;

        if (line[0] == '[') {
            inOCH = (strncmp(line, "[OutputChannels]", 16) == 0);
            inDL  = (strncmp(line, "[Datalog]",         9) == 0);
            continue;
        }

        // ochBlockSize can appear anywhere in the file
        if (ochBlockSize == 0 && strncmp(line, "ochBlockSize", 12) == 0) {
            const char* eq = strchr(line, '=');
            if (eq) ochBlockSize = (uint16_t)atoi(eq + 1);
        }

        if (inOCH && numChannels < MAX_CHANNELS) {
            Channel ch = {};
            if (parseChannelLine(line, ch))
                channels[numChannels++] = ch;
        }

        // [Datalog] entries: "entry = channelName, "Label", float|int, "fmt""
        // [Datalog] comes AFTER [OutputChannels] in the INI, so channels[] is ready.
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

    if (ochBlockSize == 0) {
        Serial.println("[INI] ERROR: ochBlockSize not found."); return false;
    }
    if (ochBlockSize > OCH_BUF_SIZE) {
        Serial.print("[INI] ERROR: ochBlockSize="); Serial.print(ochBlockSize);
        Serial.print(" > OCH_BUF_SIZE="); Serial.println(OCH_BUF_SIZE);
        Serial.println("[INI] Increase OCH_BUF_SIZE at top of main.cpp.");
        return false;
    }
    if (numChannels == 0) {
        Serial.println("[INI] ERROR: No scalar channels parsed."); return false;
    }
    return true;
}

// ─────────────────────────────────────────────────────────────
//  Log file management
// ─────────────────────────────────────────────────────────────
static bool openNextLogFile() {
    char name[28];   // longest: "YYYYMMDD/HHMMSS_99.msl" = 23 chars

    if (rtcOK) {
        DateTime now = rtc.now();

        // Date folder: YYYYMMDD
        char folder[9];
        snprintf(folder, sizeof(folder), "%04d%02d%02d",
            now.year(), now.month(), now.day());
        if (!sd.exists(folder)) {
            if (!sd.mkdir(folder)) {
                Serial.print("[SD] Cannot create folder "); Serial.println(folder);
                return false;
            }
        }

        // File: HHMMSS.msl — if that exact second already exists, append _N
        snprintf(name, sizeof(name), "%s/%02d%02d%02d.msl",
            folder, now.hour(), now.minute(), now.second());
        if (sd.exists(name)) {
            bool found = false;
            for (int i = 1; i <= 99; i++) {
                snprintf(name, sizeof(name), "%s/%02d%02d%02d_%02d.msl",
                    folder, now.hour(), now.minute(), now.second(), i);
                if (!sd.exists(name)) { found = true; break; }
            }
            if (!found) {
                Serial.println("[SD] Cannot find free timestamped slot!");
                return false;
            }
        }
    } else {
        // Fallback: sequential LOGxxx.msl in root
        bool found = false;
        for (int i = 1; i <= 999; i++) {
            snprintf(name, sizeof(name), "LOG%03d.msl", i);
            if (!sd.exists(name)) { found = true; break; }
        }
        if (!found) {
            Serial.println("[SD] No free log slot (LOG001–LOG999 all exist)!");
            return false;
        }
    }

    logFile = sd.open(name, O_WRONLY | O_CREAT);
    if (!logFile) {
        Serial.print("[SD] Cannot create "); Serial.println(name);
        return false;
    }
    Serial.print("[SD] Log: "); Serial.println(name);
    return true;
}

// MSL format: two tab-separated header rows.
//   Row 1: channel labels  ("Time", "RPM", "CLT", ...)
//   Row 2: channel units   ("s",    "RPM", "deg C", ...)
static void writeHeader() {
    // ── Row 1: labels ──
    logFile.print("Time");
    if (numDLChannels > 0) {
        for (uint16_t i = 0; i < numDLChannels; i++) {
            logFile.print('\t');
            logFile.print(dlChannels[i].label);
        }
    } else {
        for (uint16_t i = 0; i < numChannels; i++) {
            logFile.print('\t');
            logFile.print(channels[i].name);
        }
    }
    logFile.println();

    // ── Row 2: units ──
    logFile.print("s");
    if (numDLChannels > 0) {
        for (uint16_t i = 0; i < numDLChannels; i++) {
            logFile.print('\t');
            logFile.print(channels[dlChannels[i].chanIdx].unit);
        }
    } else {
        for (uint16_t i = 0; i < numChannels; i++) {
            logFile.print('\t');
            logFile.print(channels[i].unit);
        }
    }
    logFile.println();
    logFile.sync();
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
    // Time in seconds relative to log open, 3 decimal places
    float timeSec = (float)(nowMs - logStartMs) / 1000.0f;
    char buf[20];
    dtostrf(timeSec, 1, 3, buf);
    logFile.print(buf);

    uint16_t count = numDLChannels > 0 ? numDLChannels : numChannels;
    for (uint16_t i = 0; i < count; i++) {
        const Channel& ch = numDLChannels > 0
            ? channels[dlChannels[i].chanIdx]
            : channels[i];
        bool asFloat = numDLChannels > 0 ? dlChannels[i].isFloat : true;
        float val = decodeChannel(ochBuffer, ch);
        logFile.print('\t');
        if (asFloat) {
            dtostrf(val, 1, 3, buf);
            logFile.print(buf);
        } else {
            logFile.print((int32_t)val);
        }
    }
    logFile.println();
    ++rowCount;
}

// ─────────────────────────────────────────────────────────────
//  RusEFI communication
// ─────────────────────────────────────────────────────────────
static void flushSerial() {
    while (userial.available()) userial.read();
}

// Read null- or newline-terminated ASCII response.
// Extends deadline on each received byte; returns true if non-empty.
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
                deadline = millis() + 500;  // extend on each printable byte
            }
        }
    }
    dst[i] = '\0';
    return (i > 0);
}

// CRC32/ISO-HDLC (poly=0xEDB88320, init=0xFFFFFFFF) — RusEFI binary protocol checksum
static uint32_t crc32(const uint8_t* data, size_t len) {
    uint32_t crc = 0xFFFFFFFF;
    while (len--) {
        crc ^= *data++;
        for (int i = 0; i < 8; i++)
            crc = (crc & 1) ? ((crc >> 1) ^ 0xEDB88320) : (crc >> 1);
    }
    return ~crc;
}

// Diagnostic: send TS_CRC_CHECK_COMMAND ('k') and dump raw response.
// INI format: 'k%2i%2o%2c' — index(2LE), offset(2LE), count(2LE) = 7-byte payload.
static void probKCommand(uint16_t index, uint16_t offset, uint16_t count) {
    uint8_t iL  = (uint8_t)( index  & 0xFF), iH  = (uint8_t)(( index >> 8) & 0xFF);
    uint8_t oL  = (uint8_t)( offset & 0xFF), oH  = (uint8_t)((offset >> 8) & 0xFF);
    uint8_t cL  = (uint8_t)( count  & 0xFF), cH  = (uint8_t)(( count >> 8) & 0xFF);
    uint8_t pl[7] = {'k', iL, iH, oL, oH, cL, cH};
    uint32_t cs = crc32(pl, 7);
    uint8_t frame[13] = {
        0x00, 0x07,
        'k', iL, iH, oL, oH, cL, cH,
        (uint8_t)(cs >> 24), (uint8_t)(cs >> 16),   // CRC32 BIG-ENDIAN (MSB first)
        (uint8_t)(cs >> 8 ), (uint8_t)(cs      )
    };
    Serial.print("[K]   frame: ");
    for (int i = 0; i < 13; i++) {
        if (frame[i] < 0x10) Serial.print('0');
        Serial.print(frame[i], HEX); Serial.print(' ');
    }
    Serial.println();
    flushSerial();
    userial.write(frame, 13);
    myusb.Task(); myusb.Task();

    // Collect up to count+8 bytes (status + data + crc) within 2 s
    static uint8_t kBuf[OCH_BUF_SIZE + 8];
    uint16_t rx = 0;
    const uint16_t maxRx = (uint16_t)min((uint32_t)(count + 8), (uint32_t)(OCH_BUF_SIZE + 8));
    uint32_t deadline = millis() + 2000;
    while (rx < maxRx && millis() < deadline) {
        myusb.Task();
        while (userial.available() && rx < maxRx) {
            kBuf[rx++] = userial.read();
            deadline = millis() + 300;
        }
    }
    Serial.print("[K]   rx bytes: "); Serial.println(rx);
    if (rx > 0) {
        Serial.print("[K]   hex: ");
        for (uint16_t i = 0; i < rx; i++) {
            if (kBuf[i] < 0x10) Serial.print('0');
            Serial.print(kBuf[i], HEX); Serial.print(' ');
            if ((i & 0x1F) == 0x1F) Serial.print("\n        ");
        }
        Serial.println();
    }
}

// Request OCH blob via RusEFI binary protocol (2024+ firmware).
//
// Protocol framing (every command except legacy 'S'):
//   Request  → [uint16_BE dataLen][command][payload][uint32_BE CRC32(command+payload)]
//   Response → [uint16_BE dataLen][0x00 OK][data...][uint32_BE CRC32]
//              dataLen = 1 (status) + len(data)
//
// CONFIRMED from Java CRCTest.testPackPacket():
//   makeCrc32Packet({'S'}) → {0,1, 0x53, 0x20,0x60,0xEF,0xC3}
//   CRC32('S')=0x2060EFC3 stored as 20 60 EF C3 → MSB first = BIG-ENDIAN
//
// 'S' bypasses framing (legacy compat).  'O', 'A', etc. require it.
// Without the uint16 length prefix the ECU reads command byte as length_H,
// waits for thousands of bytes, times out, and never responds.
//
// RusEFI CRC binary protocol — confirmed from Java unit test:
//   GetOutputsCommand(offset=400, count=300) → fullPacket = [0x4F,0x90,0x01,0x2C,0x01]
//   Parameters are LITTLE-ENDIAN (offset first, then count).
//   makeCrc32Packet(fullPacket) → [uint16_BE len=5][fullPacket][uint32_LE CRC32] = 11 bytes
//
// For offset=0, count=ochBlockSize (e.g. 2948 = 0x0B84):
//   payload = ['O', 0x00, 0x00, 0x84, 0x0B]   ← count in LE: low byte first!
//   frame   = [0x00, 0x05, 'O', 0x00, 0x00, 0x84, 0x0B, crc0, crc1, crc2, crc3]
//
// Response: [0x00 status][ochBlockSize bytes data][uint32_LE CRC32]
static bool requestOCH() {
    // Offset = 0, count = ochBlockSize — parameters in LITTLE-ENDIAN
    const uint8_t offL = 0,    offH = 0;
    const uint8_t cntL = (uint8_t)( ochBlockSize       & 0xFF);   // LSB first
    const uint8_t cntH = (uint8_t)((ochBlockSize >> 8) & 0xFF);

    // payload = command + LE params (5 bytes)
    uint8_t pl[5] = {'O', offL, offH, cntL, cntH};
    uint32_t checksum = crc32(pl, 5);

    flushSerial();

    // Build and send: [uint16_BE len=5]['O'][offL][offH][cntL][cntH][uint32_LE CRC32]
    uint8_t frame[11] = {
        0x00, 0x05,                               // dataLen = 5 (BE)
        'O', offL, offH, cntL, cntH,              // command + LE params
        (uint8_t)(checksum >> 24),                // CRC32 BIG-ENDIAN (MSB first)
        (uint8_t)(checksum >> 16),
        (uint8_t)(checksum >> 8 ),
        (uint8_t)(checksum      )
    };

    static bool firstPoll = true;
    if (firstPoll) {
        firstPoll = false;
        Serial.print("[OCH] frame: ");
        for (int i = 0; i < 11; i++) {
            if (frame[i] < 0x10) Serial.print('0');
            Serial.print(frame[i], HEX); Serial.print(' ');
        }
        Serial.println();
    }

    userial.write(frame, 11);
    myusb.Task(); myusb.Task();

    // Response format: [uint16_BE dataLen][status=0x00][ochBlockSize bytes][uint32_BE CRC]
    //   dataLen = ochBlockSize + 1  (status byte counted in length)
    //   Total bytes = 2 + dataLen + 4 = ochBlockSize + 7
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

    if (rx == 0) {
        Serial.println("[ECU] No response");
        return false;
    }

    // rxBuf[0:1] = length prefix, rxBuf[2] = status byte, rxBuf[3..] = OCH data
    if (rx >= (uint16_t)(ochBlockSize + 3) && rxBuf[2] == 0x00) {
        memcpy(ochBuffer, rxBuf + 3, ochBlockSize);
        return true;
    }

    // Unexpected — dump first 16 bytes
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
        logFile.sync();
        logFile.close();
        logOpen = false;
        Serial.println("[SD]  Log closed.");
    }
    numChannels   = 0;
    numDLChannels = 0;
    ochBlockSize  = 0;
    signature[0]  = '\0';
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
    uint32_t t = millis();
    while (!Serial && millis() - t < 3000);

    Serial.println("================================");
    Serial.println(" RusEFI Teensy 4.1 Data Logger ");
    Serial.println("================================");

    // Verify CRC32/ISO-HDLC: CRC32("123456789") must equal 0xCBF43926
    {
        const uint8_t tv[] = "123456789";
        uint32_t got = crc32(tv, 9);
        Serial.print("[CRC] Self-test: 0x"); Serial.print(got, HEX);
        Serial.println(got == 0xCBF43926 ? " OK" : " WRONG — algorithm is broken!");
    }

    MTP.begin();   // start MTP before SD so storage can be registered below

    Serial.print("[SD]  Init... ");
    if (!sd.begin(SdioConfig(FIFO_SDIO))) {
        Serial.println("FAILED — check card is inserted.");
        setLED(&PAT_ERROR);
        enterState(State::ErrorSD);
    } else {
        Serial.println("OK");
        MTP.addFilesystem(sd, "TeensySDLogger");   // SD visible on PC via MTP any time
        Serial.println("[MTP] SD registered — plug Teensy into PC to browse logs.");
        setLED(&PAT_WAIT);
        enterState(State::WaitDevice);
    }

    // RTC (DS3231 on Wire/I2C — SDA=18, SCL=19 on Teensy 4.1)
    Serial.print("[RTC] Init... ");
    if (!rtc.begin()) {
        Serial.println("not found — filenames will be sequential (LOG001.msl ...)");
    } else if (rtc.lostPower()) {
        Serial.println("lost power — set the time via serial then reflash, or use /set command");
        Serial.println("[RTC] Falling back to sequential filenames.");
    } else {
        rtcOK = true;
        DateTime now = rtc.now();
        char tbuf[24];
        snprintf(tbuf, sizeof(tbuf), "%04d-%02d-%02d %02d:%02d:%02d",
            now.year(), now.month(), now.day(),
            now.hour(), now.minute(), now.second());
        Serial.println(tbuf);
    }

    myusb.begin();
    Serial.println("[USB] Host started. Waiting for ECU...");
}

// ─────────────────────────────────────────────────────────────
//  loop()
// ─────────────────────────────────────────────────────────────
void loop() {
    MTP.loop();    // handle PC ↔ SD transfers via MTP (no-op if no PC connected)
    myusb.Task();
    updateLED();

    // 's' command: stop logging and stay in MTP-only mode until power cycle
    if (Serial.available()) {
        char cmd = (char)Serial.read();
        if (cmd == 's' || cmd == 'S') {
            if (state == State::Logging) {
                if (logOpen) { logFile.sync(); logFile.close(); logOpen = false; }
                Serial.println("[CMD] Logging stopped. SD accessible via MTP. Power-cycle to resume.");
                MTP.send_DeviceResetEvent();   // notify PC that storage changed
            }
            setLED(&PAT_MTP);
            enterState(State::Stopped);
        }
    }

    // SD error: nothing to do but blink solid
    if (state == State::ErrorSD) return;

    // Detect USB disconnect — but not from Stopped (logging already off, MTP still useful)
    if (!userial && state != State::WaitDevice && state != State::Stopped) {
        onDisconnect();
        return;
    }

    switch (state) {

    // ── Wait for USB device ────────────────────────────────
    case State::WaitDevice:
        if (userial) {
            Serial.println("[USB] ECU detected.");
            setLED(&PAT_CONNECT);
            enterState(State::AssertDTR);
        }
        break;

    // ── Let enumeration settle, then assert DTR ────────────
    case State::AssertDTR:
        if (millis() - stateEnterMs >= 300) {
            Serial.println("[USB] Asserting DTR + RTS...");
            userial.setDTR(true);
            userial.setRTS(true);
            delay(200);  // give RusEFI time to open port
            flushSerial();

            // Get signature in text mode — no 'F' yet so there's no CRC-mode
            // timeout risk during the (potentially long) INI parse that follows.
            Serial.println("[TS]  Requesting signature ('S')...");
            userial.write('S');
            enterState(State::GetSignature);
        }
        break;

    // ── Read firmware signature ────────────────────────────
    case State::GetSignature:
        if (userial.available()) {
            if (readResponse(signature, sizeof(signature), 2000)) {
                Serial.print("[ECU] Signature: "); Serial.println(signature);
                sigToFilename(signature, iniFilename, sizeof(iniFilename));
                Serial.print("[INI] Hash filename : "); Serial.println(iniFilename);
                Serial.println("[INI] Fallback name: DEFAULT.INI");

                enterState(State::LoadINI);
            } else {
                Serial.println("[ECU] Empty response — retrying in 1 s...");
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

    // ── Parse INI from SD ──────────────────────────────────
    case State::LoadINI: {
        bool ok = false;
        if (sd.exists(iniFilename)) {
            ok = parseINI(iniFilename);
        } else if (sd.exists("DEFAULT.INI")) {
            Serial.println("[INI] Hash file absent — trying DEFAULT.INI");
            ok = parseINI("DEFAULT.INI");
        } else {
            Serial.println("[INI] No INI file found on SD card!");
            Serial.print  ("[INI] Expected: "); Serial.println(iniFilename);
            Serial.println("[INI] Or rename your INI to DEFAULT.INI");
        }

        if (ok) {
            // Activate CRC binary protocol once per connection, right before
            // we start polling. Do NOT re-send 'F' in requestOCH — TunerStudio
            // sends it exactly once per session and CRC mode stays active.
            Serial.println("[TS]  Sending 'F' (CRC binary mode)...");
            flushSerial();
            userial.write('F');
            char fResp[8] = {};
            readResponse(fResp, sizeof(fResp), 1000);
            Serial.print("[TS]  F response: \""); Serial.print(fResp); Serial.println("\"");
            flushSerial();
            delay(50);  // brief settle before first CRC command

            // ── Probe 'k' (TS_CRC_CHECK_COMMAND): index=0, offset=0, count=ochBlockSize ──
            Serial.println("[K]   Probing 'k' command (index=0, offset=0, count=ochBlockSize)...");
            probKCommand(0, 0, ochBlockSize);

            if (openNextLogFile()) {
                logStartMs = millis();
                lastSyncMs = millis();
                writeHeader();
                logOpen    = true;
                rowCount   = 0;
                lastPollMs = millis();
                setLED(&PAT_LOG);
                enterState(State::Logging);
                uint16_t logCount = numDLChannels > 0 ? numDLChannels : numChannels;
                Serial.print("[LOG] Logging "); Serial.print(logCount);
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

    // ── Main logging loop ──────────────────────────────────
    case State::Logging:
        if (millis() - lastPollMs >= POLL_INTERVAL_MS) {
            lastPollMs = millis();
            if (requestOCH()) {
                writeRow(millis());
            }
        }
        // Sync to SD once per second so a hard power-off loses at most ~1 s of data
        if (logOpen && (uint32_t)(millis() - lastSyncMs) >= SYNC_INTERVAL_MS) {
            lastSyncMs = millis();
            logFile.sync();
        }
        break;

    // ── Logging stopped by 's' command ────────────────────
    case State::Stopped:
        // MTP.loop() at top of loop() handles everything.
        // Power-cycle required to resume logging.
        break;

    // ── INI / log-file error ───────────────────────────────
    case State::ErrorINI:
        // Remind the user every 10 s over serial
        if (millis() - stateEnterMs > 10000) {
            stateEnterMs = millis();
            Serial.println("[ERR] Waiting for INI file — power-cycle after inserting SD.");
            Serial.print  ("[ERR] Expected filename: "); Serial.println(iniFilename);
            Serial.println("[ERR] Or: DEFAULT.INI");
        }
        break;

    default:
        break;
    }
}
