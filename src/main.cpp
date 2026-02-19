// ============================================================
//  RusEFI Teensy 4.1 Data Logger — main.cpp
// ============================================================
//
//  Boot sequence
//  1. Init SD card (halt + solid LED on failure)
//  2. Start USB host, wait for ECU
//  3. Assert DTR → send 'S' → read firmware signature
//  4. Hash signature → look for <XXXXXXXX>.INI on SD card
//     Falls back to DEFAULT.INI if hash file not present
//  5. Parse INI: ochBlockSize + [OutputChannels] channel table
//  6. Open next free LOGxxx.CSV, write CSV header
//  7. Poll ECU with 'O' at 20 Hz; decode blob; write rows
//  8. On USB disconnect: flush/close log, return to step 2
//
//  SD card layout
//  /<XXXXXXXX>.INI   — INI named by djb2 hash of signature
//  /DEFAULT.INI      — fallback (rename your INI to this for
//                      single-tune setups)
//  /LOG001.CSV …     — auto-incremented log files
//
//  LED patterns (pin 13)
//  Slow blink  1 Hz  — waiting for ECU
//  Fast blink  5 Hz  — connecting / handshake
//  Short flash 1 Hz  — logging OK
//  Solid on          — error (SD or INI)
// ============================================================

#include <Arduino.h>
#include <USBHost_t36.h>
#include <SdFat.h>

// ─── Configuration ──────────────────────────────────────────
static constexpr uint8_t  LED_PIN          = 13;
static constexpr uint16_t MAX_CHANNELS     = 300;   // max parsed channels
static constexpr uint16_t OCH_BUF_SIZE     = 2048;  // must be >= ochBlockSize
static constexpr uint32_t POLL_INTERVAL_MS = 50;    // 20 Hz
static constexpr uint16_t FLUSH_EVERY_ROWS = 100;   // SD sync cadence

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

// ─── State machine ──────────────────────────────────────────
enum class State : uint8_t {
    WaitDevice,
    AssertDTR,
    GetSignature,
    LoadINI,
    Logging,
    ErrorSD,
    ErrorINI,
};

// ─── USB host ───────────────────────────────────────────────
USBHost             myusb;
USBHub              hub1(myusb);
USBSerial_BigBuffer userial(myusb, 1);  // claim all CDC devices

// ─── SD ─────────────────────────────────────────────────────
SdFs   sd;
FsFile logFile;

// ─── Channel table (runtime, from INI) ──────────────────────
Channel  channels[MAX_CHANNELS];
uint16_t numChannels  = 0;
uint16_t ochBlockSize = 0;
uint8_t  ochBuffer[OCH_BUF_SIZE];

// ─── State vars ─────────────────────────────────────────────
State    state        = State::WaitDevice;
uint32_t stateEnterMs = 0;
uint32_t lastPollMs   = 0;
uint32_t rowCount     = 0;
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

    char f[20];

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

static bool parseINI(const char* filename) {
    Serial.print("[INI] Reading: "); Serial.println(filename);
    FsFile f = sd.open(filename, FILE_READ);
    if (!f) { Serial.println("[INI] File not found!"); return false; }

    numChannels  = 0;
    ochBlockSize = 0;
    bool inOCH   = false;
    char line[256];

    while (readLine(f, line, sizeof(line))) {
        // strip inline comment
        char* sc = strchr(line, ';');
        if (sc) *sc = '\0';
        trimRight(line);
        if (line[0] == '\0') continue;

        if (line[0] == '[') {
            inOCH = (strncmp(line, "[OutputChannels]", 16) == 0);
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
    }
    f.close();

    Serial.print("[INI] Channels: "); Serial.print(numChannels);
    Serial.print("  ochBlockSize: "); Serial.println(ochBlockSize);

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
    char name[16];
    for (int i = 1; i <= 999; i++) {
        snprintf(name, sizeof(name), "LOG%03d.CSV", i);
        if (!sd.exists(name)) {
            logFile = sd.open(name, O_WRONLY | O_CREAT);
            if (!logFile) {
                Serial.print("[SD] Cannot create "); Serial.println(name);
                return false;
            }
            Serial.print("[SD] Log: "); Serial.println(name);
            return true;
        }
    }
    Serial.println("[SD] No free log slot (LOG001–LOG999 all exist)!");
    return false;
}

static void writeHeader() {
    logFile.print("Time(ms)");
    for (uint16_t i = 0; i < numChannels; i++) {
        logFile.print(',');
        logFile.print(channels[i].name);
        if (channels[i].unit[0]) {
            logFile.print('(');
            logFile.print(channels[i].unit);
            logFile.print(')');
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

static void writeRow(uint32_t ts) {
    logFile.print(ts);
    char buf[20];
    for (uint16_t i = 0; i < numChannels; i++) {
        float val = decodeChannel(ochBuffer, channels[i]);
        dtostrf(val, 1, 3, buf);
        // Trim trailing zeros: "2400.000" → "2400", "14.500" → "14.5"
        char* dot = strchr(buf, '.');
        if (dot) {
            char* end = buf + strlen(buf) - 1;
            while (end > dot && *end == '0') *end-- = '\0';
            if (*end == '.') *end = '\0';
        }
        logFile.print(',');
        logFile.print(buf);
    }
    logFile.println();
    if (++rowCount % FLUSH_EVERY_ROWS == 0) logFile.sync();
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

// Request OCH blob ('O'). Returns true if exactly ochBlockSize bytes received.
static bool requestOCH() {
    flushSerial();
    userial.write('O');
    uint16_t rx = 0;
    uint32_t deadline = millis() + 500;
    while (rx < ochBlockSize && millis() < deadline) {
        myusb.Task();
        while (userial.available() && rx < ochBlockSize) {
            ochBuffer[rx++] = userial.read();
            deadline = millis() + 100;  // extend on each byte
        }
    }
    if (rx != ochBlockSize) {
        Serial.print("[ECU] Short read: "); Serial.print(rx);
        Serial.print('/'); Serial.println(ochBlockSize);
        return false;
    }
    return true;
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
    numChannels  = 0;
    ochBlockSize = 0;
    signature[0] = '\0';
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

    Serial.print("[SD]  Init... ");
    if (!sd.begin(SdioConfig(FIFO_SDIO))) {
        Serial.println("FAILED — check card is inserted.");
        setLED(&PAT_ERROR);
        enterState(State::ErrorSD);
    } else {
        Serial.println("OK");
        setLED(&PAT_WAIT);
        enterState(State::WaitDevice);
    }

    myusb.begin();
    Serial.println("[USB] Host started. Waiting for ECU...");
}

// ─────────────────────────────────────────────────────────────
//  loop()
// ─────────────────────────────────────────────────────────────
void loop() {
    myusb.Task();
    updateLED();

    // SD error: nothing to do but blink solid
    if (state == State::ErrorSD) return;

    // Detect USB disconnect from any active state
    if (!userial && state != State::WaitDevice) {
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
            Serial.println("[TS]  Requesting signature...");
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
            if (openNextLogFile()) {
                writeHeader();
                logOpen    = true;
                rowCount   = 0;
                lastPollMs = millis();
                setLED(&PAT_LOG);
                enterState(State::Logging);
                Serial.print("[LOG] Logging "); Serial.print(numChannels);
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
