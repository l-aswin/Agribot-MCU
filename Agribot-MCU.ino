// Agribot ESP8266 Motor Controller Firmware
// Protocol: ASCII newline-terminated frames over UART
// See: documentation/mcu_system_requirements.md

#include <Ticker.h>

// ── Calibration constants (SR-M31) ────────────────────────────────────────────
#define SERIAL_BAUD_RATE   115200
#define SPEED_CM_PER_S     10.0f    // robot linear speed in cm/s
#define TURN_DEG_PER_S     90.0f    // robot turn rate in degrees/s

// ── LED pin assignments (SR-M12) ──────────────────────────────────────────────
#define LED_STARTUP   D0   // GPIO 16 — green, startup status
#define LED_ERROR     D7   // GPIO 13 — red,   error/connectivity status
#define LED_VEHICLE   D4   // GPIO  2 — green, vehicle status

// ── Motor pin assignments ─────────────────────────────────────────────────────
#define PIN_MTR_FWD_A  D1   // left  motor forward  (GPIO 5)
#define PIN_MTR_FWD_B  D2   // right motor forward  (GPIO 4)
#define PIN_MTR_BWD_A  D5   // left  motor backward (GPIO 14)
#define PIN_MTR_BWD_B  D6   // right motor backward (GPIO 12)

// ── Frame parser (SR-M33) ─────────────────────────────────────────────────────
#define MAX_FRAME_LEN  64
static char    s_buf[MAX_FRAME_LEN + 1];
static uint8_t s_bufIdx = 0;

// ── Movement state ────────────────────────────────────────────────────────────
static Ticker        s_moveTicker;
static volatile bool s_moveActive   = false;
static volatile bool s_moveDoneFlag = false;

// ── LED pattern engine ────────────────────────────────────────────────────────
// Each LED has an independent state machine. Steps alternate HIGH/LOW starting
// at index 0 (HIGH = ON). stepCount == 0 means static state (no pattern).
#define MAX_LED_STEPS  12

struct LedChannel {
    uint8_t  pin;
    uint16_t steps[MAX_LED_STEPS];
    uint8_t  stepCount;
    uint8_t  stepIdx;
    bool     repeat;
    uint32_t nextMs;
};

static LedChannel s_ledStartup = { LED_STARTUP, {}, 0, 0, false, 0 };
static LedChannel s_ledError   = { LED_ERROR,   {}, 0, 0, false, 0 };
static LedChannel s_ledVehicle = { LED_VEHICLE, {}, 0, 0, false, 0 };

// ── Forward declarations ──────────────────────────────────────────────────────
static void stopMotors();
static void onMoveDone();
static void processFrame(const char* frame);
static void handleMovement(const char* prefix, float value);
static void handleLed(const char* status);
static void ledSolid(LedChannel& ch, bool on);
static void ledPattern(LedChannel& ch, const uint16_t* steps, uint8_t count, bool repeat);
static void ledNBlink(LedChannel& ch, uint8_t n, uint16_t pauseMs);
static void tickLed(LedChannel& ch);

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    pinMode(PIN_MTR_FWD_A, OUTPUT);
    pinMode(PIN_MTR_FWD_B, OUTPUT);
    pinMode(PIN_MTR_BWD_A, OUTPUT);
    pinMode(PIN_MTR_BWD_B, OUTPUT);
    pinMode(LED_STARTUP,   OUTPUT);
    pinMode(LED_ERROR,     OUTPUT);
    pinMode(LED_VEHICLE,   OUTPUT);

    stopMotors();
    digitalWrite(LED_STARTUP, LOW);
    digitalWrite(LED_ERROR,   LOW);
    digitalWrite(LED_VEHICLE, LOW);

    // Serial RX is interrupt-buffered by the ESP8266 Arduino core (SR-M06).
    Serial.begin(SERIAL_BAUD_RATE);
}

void loop() {
    // ── Drain RX buffer and assemble newline-terminated frames (SR-M28) ───────
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            s_buf[s_bufIdx] = '\0';
            if (s_bufIdx > 0) processFrame(s_buf);
            s_bufIdx = 0;
        } else if (s_bufIdx < MAX_FRAME_LEN) {
            s_buf[s_bufIdx++] = c;
        } else {
            // Frame too long — flush and signal parse error (SR-M33)
            s_bufIdx = 0;
            Serial.print("ERR:3\n");
        }
    }

    // Publish DONE outside Ticker callback context (SR-M13)
    if (s_moveDoneFlag) {
        s_moveDoneFlag = false;
        Serial.print("DONE\n");
    }

    tickLed(s_ledStartup);
    tickLed(s_ledError);
    tickLed(s_ledVehicle);
}

// ── Motor helpers ─────────────────────────────────────────────────────────────
static void stopMotors() {
    digitalWrite(PIN_MTR_FWD_A, LOW);
    digitalWrite(PIN_MTR_FWD_B, LOW);
    digitalWrite(PIN_MTR_BWD_A, LOW);
    digitalWrite(PIN_MTR_BWD_B, LOW);
}

// Called by Ticker after movement duration elapses (SR-M13, SR-M14)
static void onMoveDone() {
    stopMotors();
    s_moveActive   = false;
    s_moveDoneFlag = true;
}

// ── Frame dispatcher ──────────────────────────────────────────────────────────
static void processFrame(const char* frame) {
    // PING handshake (SR-M07–SR-M09)
    if (strcmp(frame, "PING") == 0) {
        Serial.print("OK\n");
        return;
    }

    // STP abort (SR-M21–SR-M23)
    if (strcmp(frame, "STP") == 0) {
        if (s_moveActive) {
            s_moveTicker.detach();
            stopMotors();
            s_moveActive = false;
        }
        Serial.print("DONE\n");
        return;
    }

    // LED command — no reply (SR-M10, SR-M11)
    if (strncmp(frame, "LED:", 4) == 0) {
        handleLed(frame + 4);
        return;
    }

    // Movement frames: PREFIX:VALUE
    const char* colon = strchr(frame, ':');
    if (colon == nullptr) {
        Serial.print("ERR:3\n");
        return;
    }

    float value = atof(colon + 1);
    if (value <= 0.0f) {
        Serial.print("ERR:4\n");
        return;
    }

    uint8_t prefixLen = (uint8_t)(colon - frame);
    if (prefixLen == 0 || prefixLen > 3) {
        Serial.print("ERR:3\n");
        return;
    }
    char prefix[4] = {0};
    memcpy(prefix, frame, prefixLen);

    handleMovement(prefix, value);
}

// ── Movement handler (SR-M13–SR-M20) ─────────────────────────────────────────
static void handleMovement(const char* prefix, float value) {
    if (s_moveActive) {
        Serial.print("ERR:2\n");
        return;
    }

    uint32_t durationMs = 0;

    if (strcmp(prefix, "FWD") == 0) {
        durationMs = (uint32_t)((value / SPEED_CM_PER_S) * 1000.0f);
        Serial.print("ACK\n");
        digitalWrite(PIN_MTR_FWD_A, HIGH);
        digitalWrite(PIN_MTR_FWD_B, HIGH);
        digitalWrite(PIN_MTR_BWD_A, HIGH);
        digitalWrite(PIN_MTR_BWD_B, HIGH);

    } else if (strcmp(prefix, "BWD") == 0) {
        durationMs = (uint32_t)((value / SPEED_CM_PER_S) * 1000.0f);
        Serial.print("ACK\n");
        digitalWrite(PIN_MTR_BWD_A, HIGH);
        digitalWrite(PIN_MTR_BWD_B, HIGH);

    } else if (strcmp(prefix, "TRT") == 0) {
        durationMs = (uint32_t)((value / TURN_DEG_PER_S) * 1000.0f);
        Serial.print("ACK\n");
        digitalWrite(PIN_MTR_FWD_A, HIGH);

    } else if (strcmp(prefix, "TLT") == 0) {
        durationMs = (uint32_t)((value / TURN_DEG_PER_S) * 1000.0f);
        Serial.print("ACK\n");
        digitalWrite(PIN_MTR_BWD_B, HIGH);

    } else {
        Serial.print("ERR:3\n");
        return;
    }

    s_moveActive = true;
    s_moveTicker.once_ms(durationMs, onMoveDone);
}

// ── LED channel helpers ───────────────────────────────────────────────────────

static void ledSolid(LedChannel& ch, bool on) {
    ch.stepCount = 0;
    digitalWrite(ch.pin, on ? HIGH : LOW);
}

static void ledPattern(LedChannel& ch, const uint16_t* steps, uint8_t count, bool repeat) {
    memcpy(ch.steps, steps, count * sizeof(uint16_t));
    ch.stepCount = count;
    ch.stepIdx   = 0;
    ch.repeat    = repeat;
    digitalWrite(ch.pin, HIGH);
    ch.nextMs    = millis() + steps[0];
}

// N blinks of 1 s ON / 1 s OFF, then pauseMs solid OFF, repeat.
// Implements the "blink N times continuously" patterns from SR-M12.
static void ledNBlink(LedChannel& ch, uint8_t n, uint16_t pauseMs) {
    uint8_t total = n * 2;
    if (total == 0 || total > MAX_LED_STEPS) return;
    for (uint8_t i = 0; i < total - 1; i++)
        ch.steps[i] = 1000;
    ch.steps[total - 1] = pauseMs;
    ch.stepCount = total;
    ch.stepIdx   = 0;
    ch.repeat    = true;
    digitalWrite(ch.pin, HIGH);
    ch.nextMs    = millis() + 1000;
}

// Advance one LED channel's state machine — called every loop() iteration.
static void tickLed(LedChannel& ch) {
    if (ch.stepCount == 0) return;
    if ((int32_t)(millis() - ch.nextMs) < 0) return;

    ch.stepIdx++;
    if (ch.stepIdx >= ch.stepCount) {
        if (ch.repeat) {
            ch.stepIdx = 0;
        } else {
            ch.stepCount = 0;
            return;
        }
    }

    digitalWrite(ch.pin, (ch.stepIdx % 2 == 0) ? HIGH : LOW);
    ch.nextMs += ch.steps[ch.stepIdx];
}

// ── LED status handler (SR-M10, SR-M11, SR-M12) ───────────────────────────────
static void handleLed(const char* status) {
    // ── Startup status (D0 green) ─────────────────────────────────────────────
    if (strcmp(status, "STARTUP_INPROGRESS") == 0) {
        // Fast blink: 200 ms ON / 200 ms OFF, repeat
        static const uint16_t p[] = {200, 200};
        ledPattern(s_ledStartup, p, 2, true);
        return;
    }
    if (strcmp(status, "STARTUP_OK") == 0) {
        ledSolid(s_ledStartup, true);
        return;
    }

    // ── Error status (D7 red) ─────────────────────────────────────────────────
    if (strcmp(status, "CAMERA_FAIL") == 0) {
        // Blink once continuously: 1 s ON, 1 s OFF, repeat
        ledNBlink(s_ledError, 1, 1000);
        return;
    }
    if (strcmp(status, "SERVER_UNREACHABLE") == 0) {
        // Blink 2 times continuously: 2× (1 s ON, 1 s OFF), 5 s OFF, repeat
        ledNBlink(s_ledError, 2, 5000);
        return;
    }
    if (strcmp(status, "AUTH_FAIL") == 0) {
        // Blink 3 times continuously: 3× (1 s ON, 1 s OFF), 5 s OFF, repeat
        ledNBlink(s_ledError, 3, 5000);
        return;
    }
    if (strcmp(status, "CLEAR_ERROR_LED") == 0) {
        ledSolid(s_ledError, false);
        return;
    }

    // ── Vehicle status (D4 green) ─────────────────────────────────────────────
    if (strcmp(status, "VEHICLE_IDLE") == 0) {
        // Blink once continuously: 1 s ON, 1 s OFF, repeat
        ledNBlink(s_ledVehicle, 1, 1000);
        return;
    }
    if (strcmp(status, "VEHICLE_WORKING") == 0) {
        ledSolid(s_ledVehicle, true);
        return;
    }
    if (strcmp(status, "SHUTDOWN") == 0) {
        ledSolid(s_ledVehicle, false);
        return;
    }

    // Unrecognised status — silently ignore (SR-M11)
}
