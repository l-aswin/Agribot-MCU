# System Requirements Specification — Agribot ESP8266 MCU

**Source:** `edge_device_system_requirements.md` (SR-07, SR-08, SR-39, SR-41–SR-45)
**Platform:** ESP8266 (NodeMCU) | **Language:** Arduino C++ (ESP8266 Arduino Core)

---

## 1. System Overview

| Component | Detail |
|-----------|--------|
| MCU | ESP8266 (NodeMCU) |
| Host | NVIDIA Jetson Nano |
| Interface | USB Serial (UART) |
| Motor control | GPIO-driven motor driver (pin driven HIGH/LOW) |
| Timing | Non-blocking `Ticker` library |
| UART RX | Interrupt-based (`Serial` RX interrupt) |

The ESP8266 acts as a subordinate motor controller. It receives text frames from the Jetson Nano over UART, drives the motors accordingly, and replies with status frames. It must remain responsive to abort commands at all times during active movement.

---

## 2. UART Communication

### 2.1 Frame Format

- **SR-M01:** All frames SHALL be newline-terminated ASCII strings (`\n`). No binary framing is used.
- **SR-M02:** The ESP8266 SHALL support the following received (Jetson → ESP) frame types:

| Frame | Purpose |
|-------|---------|
| `PING\n` | Startup handshake — check ESP is alive |
| `LED:{status}\n` | Set LED indicator state (see Section 4) |
| `FWD:{dist:.1f}\n` | Move forward by `dist` centimetres |
| `BWD:{dist:.1f}\n` | Move backward by `dist` centimetres |
| `TLT:{angle:.1f}\n` | Turn left by `angle` degrees |
| `TRT:{angle:.1f}\n` | Turn right by `angle` degrees |
| `STP\n` | Abort current movement immediately |

- **SR-M03:** The ESP8266 SHALL transmit the following (ESP → Jetson) frame types:

| Frame | Trigger |
|-------|---------|
| `OK\n` | Response to `PING\n` |
| `ACK\n` | Movement command received and motion started |
| `DONE\n` | Movement completed (or aborted via `STP\n`) |
| `ERR:{code}\n` | Movement failed (see Section 5) |

- **SR-M04:** Distance values in all frames are in centimetres; angle values are in degrees. Both are encoded as a single decimal value with one fractional digit (e.g. `FWD:45.0\n`, `TLT:90.0\n`).

### 2.2 Serial Configuration

- **SR-M05:** The ESP8266 SHALL initialise the UART at the baud rate configured in the Jetson's `config.json` (`serial_baud_rate`). The firmware constant SHALL default to **115200 baud** and SHALL match the value used by the Jetson.
- **SR-M06:** The ESP8266 SHALL use interrupt-based UART reception (`Serial` RX interrupt) so that incoming bytes are buffered and the MCU remains responsive to `STP\n` frames during active movement. Blocking `Serial.readString()` or `Serial.readStringUntil()` with long timeouts SHALL NOT be used inside the main loop or any movement routine.

---

## 3. Startup Handshake

- **SR-M07:** On power-on or reset, the ESP8266 SHALL initialise the serial port and enter a listening state immediately. No delay or splash output SHALL be emitted before `PING\n` handling is ready.
- **SR-M08:** On receiving `PING\n`, the ESP8266 SHALL reply with `OK\n` within 100 ms.
- **SR-M09:** The ESP8266 SHALL accept and process `PING\n` at any time (not only at startup), so the Jetson can re-verify liveness without requiring a hard reset.

---

## 4. LED Indicator Control

- **SR-M10:** On receiving `LED:{status}\n`, the ESP8266 SHALL update the onboard or external LED to reflect the status. The mapping of status strings to LED patterns is implementation-defined and SHALL be documented in the firmware source. The ESP8266 SHALL NOT reply to `LED` frames.
- **SR-M11:** The ESP8266 SHALL silently ignore any `LED:{status}\n` frame with an unrecognised status value (no `ERR` reply).
- **SR-M12:** The following status values SHALL be handled:

LED indicator shows Startup status, Error Status and Device status  

**Startup status**
|status value | LED color | GPIO pin | LED behaviour |
| :---- | :---- | :---- | :---- |
| STARTUP_INPROGRESS | Green  | D0 (GPIO 16\) | Fast blink |
| STARTUP_OK | Green | D0 (GPIO 16\) | Solid ON |

**Error status**
|status value | LED color | GPIO pin | LED behaviour |
| :---- | :---- | :---- | :---- |
| CAMERA_FAIL | Red | D7 (GPIO 13\) | Blink once continuously |
| SERVER_UNREACHABLE | Red | D7 (GPIO 13\) | Blink 2 times continuously |
| AUTH_FAIL | Red | D7 (GPIO 13\) | Blink 3 times continuously |
| CLEAR_ERROR_LED | Red | D7 (GPIO 13\) | Solid OFF |

   
**Vehicle status**

|status value | LED color | GPIO pin | LED behaviour |
| :---- | :---- | :---- | :---- |
| VEHICLE_IDLE | Green | D4 (GPIO 2\) | Blink once continuously |
| VEHICLE_WORKING | Green | D4 (GPIO 2\) | Solid ON |
| SHUTDOWN | Green | D4 (GPIO 2\) | Solid OFF |

Blink once continuously \- on for 1second off for  1 second repeatedly  
Blink 2 times continuously \- on for 1second off for  1 second repeat 2 times, then off for 5 second  
Blink 3 times continuously \- on for 1second off for  1 second repeat 3 times, then off for 5 second  

> Note: `LED:ERR:4` is never sent by the Jetson (serial unavailable at that stage); the ESP8266 need not handle it specially.

---

## 5. Movement Commands

### 5.1 General Movement Behaviour

- **SR-M13:** On receiving a valid movement frame (`FWD`, `BWD`, `TLT`, `TRT`), the ESP8266 SHALL:
  1. Reply with `ACK\n` immediately to confirm receipt.
  2. Begin motor actuation.
  3. Use a non-blocking `Ticker`-based timer to track movement duration.
  4. When motion completes normally, stop the motor (set GPIO LOW) and reply with `DONE\n`.
- **SR-M14:** The ESP8266 SHALL NOT use blocking `delay()` calls for movement duration. All timing SHALL use the `Ticker` library or equivalent non-blocking timer so the main loop remains free to receive `STP\n` during movement.
- **SR-M15:** Only one movement command SHALL be active at a time. If a second movement frame is received while a movement is already in progress, the ESP8266 SHALL reply with `ERR:2\n` and ignore the new command.
- **SR-M16:** Distance-to-duration conversion (cm → milliseconds) SHALL be performed using a calibrated constant stored in firmware. This constant SHALL be configurable at compile time. The conversion method (e.g. fixed speed assumption) SHALL be documented in the firmware source.

### 5.2 Forward and Backward Movement

- **SR-M17:** On `FWD:{dist:.1f}\n`, the ESP8266 SHALL drive the motors forward for the duration corresponding to `dist` cm, then stop and send `DONE\n`.
- **SR-M18:** On `BWD:{dist:.1f}\n`, the ESP8266 SHALL drive the motors backward for the duration corresponding to `dist` cm, then stop and send `DONE\n`.

### 5.3 Turn Commands

- **SR-M19:** On `TLT:{angle:.1f}\n`, the ESP8266 SHALL execute a left turn of `angle` degrees (using a calibrated angle-to-duration constant), then stop and send `DONE\n`.
- **SR-M20:** On `TRT:{angle:.1f}\n`, the ESP8266 SHALL execute a right turn of `angle` degrees, then stop and send `DONE\n`.

### 5.4 Abort — `STP` Command

- **SR-M21:** The ESP8266 SHALL monitor the UART RX buffer for `STP\n` continuously during active movement using a non-blocking read in the main loop.
- **SR-M22:** On receiving `STP\n` during active movement, the ESP8266 SHALL:
  1. Immediately set the motor GPIO pin(s) LOW to stop the motor.
  2. Cancel the active `Ticker` timer.
  3. Reply with `DONE\n` so the Jetson's blocking DONE-wait exits immediately.
- **SR-M23:** On receiving `STP\n` while no movement is active (idle state), the ESP8266 SHALL reply with `DONE\n` and remain in idle state. This ensures the Jetson's flush-and-discard pattern works correctly even if `STP\n` arrives slightly early.

---

## 6. Error Codes

- **SR-M24:** When a movement command cannot be executed or fails mid-motion, the ESP8266 SHALL send `ERR:{code}\n` instead of (or before) `DONE\n`.
- **SR-M25:** The following error codes SHALL be defined:

| Code | Cause |
|------|-------|
| `1` | Hardware fault detected (motor driver fault pin asserted or similar) |
| `2` | Command rejected — another movement already in progress |
| `3` | Unrecognised frame received (unknown command prefix) |
| `4` | Frame parse error — value field is malformed or out of range |

> Additional implementation-specific error codes MAY be added; their meaning SHALL be documented in the firmware source.

- **SR-M26:** After sending `ERR:{code}\n`, the ESP8266 SHALL stop any in-progress motor actuation, cancel any active timer, and return to idle state ready for the next command.

---

## 7. Concurrency and Timing

- **SR-M27:** The firmware SHALL implement a non-blocking architecture: the `loop()` function SHALL execute in short, bounded iterations with no blocking calls. All time-consuming operations (movement duration, LED pattern timing) SHALL be driven by `Ticker` callbacks or `millis()`-based state machines.
- **SR-M28:** The UART RX interrupt handler SHALL buffer incoming bytes into a ring buffer or `String` accumulator. The main loop SHALL drain this buffer each iteration, assembling complete newline-terminated frames and dispatching them to the command handler.
- **SR-M29:** The worst-case latency from the ESP8266 receiving the final byte of `STP\n` to the motor GPIO going LOW SHALL not exceed **10 ms**. This bound applies regardless of any other processing in `loop()`.

---

## 8. Non-Functional Requirements

- **SR-M30:** The firmware SHALL compile cleanly under the ESP8266 Arduino Core (latest stable release) with no warnings at the default warning level.
- **SR-M31:** All calibration constants (speed cm/s, turn rate deg/s, baud rate) SHALL be defined as named `#define` or `const` values at the top of the source file, not as magic numbers inline.
- **SR-M32:** The firmware SHALL perform a self-test on boot: verify the motor driver GPIO pins can be driven HIGH and LOW without fault, and verify UART is ready. If any self-test step fails, the ESP8266 SHALL enter a safe idle state and await `PING\n` without driving motors.
- **SR-M33:** The firmware SHALL be tolerant of partial or corrupted UART frames caused by line noise. If a frame does not match any known prefix after accumulating up to 64 bytes without a newline, the buffer SHALL be flushed and the ESP8266 SHALL reply with `ERR:3\n`.

---

## 9. Frame Interaction Diagram

```
Jetson Nano                         ESP8266
    |                                   |
    |--- PING\n ----------------------->|
    |<-- OK\n --------------------------|
    |                                   |
    |--- LED:STARTUP_OK\n ------------->|  (no reply)
    |                                   |
    |--- FWD:45.0\n ------------------->|
    |<-- ACK\n --------------------------|
    |          [motor runs for 45 cm]   |
    |<-- DONE\n ------------------------|
    |                                   |
    |--- FWD:30.0\n ------------------->|
    |<-- ACK\n --------------------------|
    |     [Jetson sends STP mid-move]   |
    |--- STP\n ------------------------->|
    |          [motor stops immediately]|
    |<-- DONE\n ------------------------|
    |                                   |
```
