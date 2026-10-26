# AI Learner Rover — Firmware

Firmware developed for the **AI Learner Rover**, a final-year Electrical and Computer Engineering project at the University of Cape Town (UCT). The rover receives movement plans from a computer-based host application and executes them on physical hardware.

This repository preserves **two hardware implementations** of the rover:

1. **Original ESP32 rover** — an Arduino/PlatformIO implementation in which the ESP32 handles networking, sensors, and motor control.
2. **Micromouse-based rover** — a later implementation that separates communication from motion control: an **ESP32 acts as a Wi-Fi/WebSocket gateway**, while an **STM32L4-based UCT Micromouse** executes feedback-controlled movement.

Both implementations use the same high-level JSON path-command format, but their motion-control and heading-feedback mechanisms differ.

> **Repository scope:** This is the *robot firmware* repository. The desktop host application, camera-based perception, and LLM planning are separate parts of the overall AI Learner Rover system.

## System architecture

```text
                         AI Learner Host (computer)
                    ┌────────────────────────────────┐
                    │ Camera-based perception        │
                    │ LLM-assisted path planning     │
                    │ Command execution & monitoring │
                    └───────────────┬────────────────┘
                                    │ Wi-Fi / WebSocket
                                    │ JSON messages, port 81
                  ┌─────────────────┴──────────────────┐
                  │                                    │
        Original rover                          Micromouse-based rover
    ┌─────────────────────┐                  ┌─────────────────────────┐
    │ ESP32               │                  │ ESP32                   │
    │ Network + commands  │                  │ Network + JSON gateway  │
    │ Ultrasonic + battery│                  └────────────┬────────────┘
    │ Motors + encoders   │                               │ UART, 115200 baud
    └─────────────────────┘                  ┌────────────▼────────────┐
                                             │ STM32L4 / UCT Micromouse│
                                             │ MicroPython: remote.py  │
                                             │ Motion PID: motion.py   │
                                             │ Encoders + motors       │
                                             └─────────────────────────┘
```

In the newer architecture, the ESP32 interprets host commands and forwards motion requests to the STM32. The STM32 runs the low-level control loop; motion execution does not depend on waiting for a cloud-model response.

## Repository structure

```text
AI-Rover-Firmware-Test-main/
├── platformio.ini                 # PlatformIO configuration for ESP32
├── src/
│   └── main.cpp                   # ORIGINAL rover: ESP32 application
├── lib/                           # Original rover's local C++ libraries
│   ├── Battery/                   # Battery voltage / SOC support
│   ├── encoder/                   # Wheel-encoder handling
│   ├── Motor/                     # Motor control
│   ├── StraightDrive/             # Straight-line movement
│   ├── TurnDrive/                 # Turning movement
│   ├── ultrasonic/                # Ultrasonic distance sensor
│   ├── vehicle/                   # Vehicle control abstraction
│   ├── ESP32PWM/                  # PWM support
│   └── ESP32Servo/                # Servo support
├── Micromouse_Code/
│   ├── ESP32/
│   │   └── main.cpp               # NEW rover: ESP32 WebSocket-to-UART gateway
│   └── STM32L4/
│       ├── remote.py              # STM32 UART protocol and command dispatcher
│       └── motion.py              # STM32 encoder/gyro motion primitives and PID
├── include/
└── test/
```

**Important:** The root `platformio.ini` and `src/main.cpp` build the **original rover** by default. `Micromouse_Code/ESP32/main.cpp` is the separate ESP32 application for the Micromouse-based rover; it is **not automatically selected** by that default build. The STM32 `.py` files are MicroPython programs, not PlatformIO C++ sources.

## 1. Original ESP32 rover

**Code:** [`src/main.cpp`](src/main.cpp) and [`lib/`](lib/)

The original platform uses a single ESP32 for both communication and physical actuation. The firmware:

- Connects to a configured Wi-Fi network and runs a WebSocket server on **port 81**.
- Receives JSON movement paths containing forward, backward, and turn commands.
- Uses local motor, encoder, straight-drive, and turn-drive libraries to execute movements.
- Reads an ultrasonic sensor before a forward movement to check whether the requested distance is clear.
- Requests heading observations from the host computer to verify and correct turns.
- Reports battery voltage, estimated state of charge, and ultrasonic distance when telemetry is enabled.

**Original-rover behaviour:** A `BACK` command is implemented as a camera-verified **180° turn followed by forward movement**, rather than driving the wheels directly in reverse. Turning relies on a host-supplied heading measurement and can attempt corrective turns. Consequently, this version depends more heavily on the external camera during motion.

### Build and upload: original rover

Requirements:

- ESP32 development board compatible with PlatformIO's `esp32dev` target
- [PlatformIO](https://platformio.org/) (CLI or VS Code extension)
- USB connection and the original rover's motor/sensor hardware

From the repository root:

```bash
pio run
pio run -t upload
pio device monitor -b 115200
```

The root [`platformio.ini`](platformio.ini) selects the **Arduino** framework and declares the **WebSockets** (`links2004/WebSockets`) and **ArduinoJson** (`bblanchon/ArduinoJson`) dependencies. Ensure the serial upload port and physical pin assignments are appropriate for your board.

## 2. Micromouse-based rover

**Code:** [`Micromouse_Code/ESP32/`](Micromouse_Code/ESP32/) and [`Micromouse_Code/STM32L4/`](Micromouse_Code/STM32L4/)

This version splits responsibilities between two controllers.

### ESP32 — communication gateway

**File:** [`Micromouse_Code/ESP32/main.cpp`](Micromouse_Code/ESP32/main.cpp)

The ESP32:

- Connects to Wi-Fi and hosts the **same port-81 JSON WebSocket interface** used by the original rover.
- Validates and queues incoming movement paths.
- Converts each path command to a short, newline-terminated UART command.
- Waits for the STM32's completion/error response before acknowledging that command to the host.
- Checks that the STM32 responds to a status request before starting a path.
- Sends a stop request if a movement times out or the path's WebSocket client disconnects during execution.

Unlike the original implementation, this gateway does **not** drive the motors or ask the host for a camera heading after turns.

### STM32L4 — motion controller

**Files:** [`remote.py`](Micromouse_Code/STM32L4/remote.py) and [`motion.py`](Micromouse_Code/STM32L4/motion.py)

The STM32L4 runs the project's **MicroPython environment with the `uct_mouse` hardware module**:

| File | Responsibility |
| --- | --- |
| `remote.py` | Reads newline-delimited UART commands; parses distances and angles; dispatches movement; returns `ACK`, `DONE`, `ERR`, `OK`, or `STOPPED`; and checks for stop requests during motion. |
| `motion.py` | Implements the low-level motor-control routines, wheel-encoder feedback, PID distance/straightness control, encoder-based or gyro-based turning, and calibration/test helpers. |

Straight-line driving uses two feedback loops: one for **distance travelled** and another for **left/right wheel alignment**. Motion is paced with a **10 ms** control-loop period. The remote interface currently selects **encoder-based turns** (`USE_ENCODER_TURNS = True`), although a gyro-based turning routine is also available in `motion.py`.

The STM32 MicroPython runtime and its `uct_mouse` module are **prerequisites** and are not included in this repository.

### ESP32 ↔ STM32 wiring

The supplied code selects **USART1** on the STM32 (`UART_ID = 1`) and **Serial2** on the ESP32, both at **115200 baud, 8N1**.

| ESP32 | STM32L4 / UCT Micromouse | Connection |
| --- | --- | --- |
| GPIO17 / TX2 | PB7 / USART1 RX (J2 pin 8) | ESP32 → STM32 |
| GPIO16 / RX2 | PB6 / USART1 TX (J2 pin 7) | STM32 → ESP32 |
| GND | GND | Common ground |

Use **3.3 V logic**, cross TX/RX as shown, and connect the grounds. The comments in `remote.py` indicate that this USART1 connection shares the programming-header interface with ST-Link; disconnect conflicting programming connections before using the UART link. Check the pinout against the exact board revision before wiring.

### Run the Micromouse ESP32 gateway

Use the root [`platformio.ini`](platformio.ini) settings in a **separate ESP32 PlatformIO project**, but use **`Micromouse_Code/ESP32/main.cpp` as that project's `src/main.cpp`**. This keeps the original implementation intact.

For example, from the repository root in a Bash-compatible terminal:

```bash
mkdir -p micromouse-esp32-build/src
cp platformio.ini micromouse-esp32-build/platformio.ini
cp Micromouse_Code/ESP32/main.cpp micromouse-esp32-build/src/main.cpp
pio run -d micromouse-esp32-build
pio run -d micromouse-esp32-build -t upload
pio device monitor -b 115200
```

Alternatively, create an `esp32dev` project in the PlatformIO VS Code extension and copy the gateway source into its `src/main.cpp`.

### Run the STM32 MicroPython code

Install/use the compatible UCT Micromouse MicroPython firmware that provides `uct_mouse`, then copy both files in `Micromouse_Code/STM32L4/` to the board's MicroPython filesystem. For example, with [`mpremote`](https://docs.micropython.org/en/latest/reference/mpremote.html), replacing `COM9` with the correct serial port:

```bash
mpremote connect COM9 fs cp Micromouse_Code/STM32L4/motion.py :motion.py
mpremote connect COM9 fs cp Micromouse_Code/STM32L4/remote.py :remote.py
mpremote connect COM9 repl
```

To launch the UART listener manually from the MicroPython REPL:

```python
import remote
remote.run()
```

For automatic startup, create a **`main.py` on the STM32** containing the two lines above. A startup `main.py` is not included in this archive. Only enable automatic motion control after verifying wiring and calibration; ensure the robot is safely supported during initial tests.

### Motion calibration

Review the constants near the beginning of `motion.py` for your physical chassis, especially:

- `TICKS_PER_M` — conversion from encoder counts to travelled distance; the file identifies its initial value as a simulator-derived placeholder.
- `DEADBAND` — motor command required to overcome static friction.
- `TRACK_M` — wheel separation used for encoder-based turns.
- PID gains, speed limits, tolerances, and gyro calibration values.

`motion.py` includes calibration/test helpers such as `calibrate_ticks()`, `find_deadband()`, and `test_gyro()`. Parameters need validation on the actual hardware; the presence of feedback control alone does not guarantee a specific positioning accuracy.

## Communication protocols

### Host ↔ ESP32: WebSocket/JSON

Connect the host to:

```text
ws://<esp32-ip-address>:81
```

Both ESP32 implementations accept the following **path structure**. Distances are in **millimetres** and turn angles are in **degrees**:

```json
{
  "t": "path",
  "id": 1,
  "cmds": [
    ["FWD", 200.0],
    ["TURN", 90.0],
    ["BACK", 50.0],
    ["STOP"]
  ]
}
```

| Host command | Meaning |
| --- | --- |
| `["FWD", distance_mm]` | Move forward by a positive distance. |
| `["BACK", distance_mm]` | Move backward by a positive distance; implementation differs between versions. |
| `["TURN", angle_deg]` | Rotate: **positive = right**, **negative = left**. |
| `["STOP"]` | Required final command/terminator in the path. **It is not a separate emergency-stop message.** |

Every supplied path is validated before execution. Successful commands receive an acknowledgement; failures are reported as a negative acknowledgement (`nack`). Typical responses:

```json
{"t":"ack","of":"cmd","id":1,"i":0}
{"t":"ack","of":"path","id":1}
{"t":"nack","id":1,"i":0,"reason":"stm32_offline"}
```

The firmware also supports:

```json
{"t":"ping","ts":12345}
{"t":"telem","on":true}
{"t":"telem","on":false}
```

A `ping` produces a `pong` with the supplied timestamp. The intended telemetry JSON structure is:

```json
{"t":"telem","vbat":7.82,"soc":74,"dist":14.0}
```

Here, `vbat` is battery voltage (V), `soc` is estimated charge (%), and `dist` is forward distance (**cm**). On the original rover this distance comes from the ultrasonic sensor. In the newer ESP32 gateway, the parsing code expects it from an STM32-supplied centre time-of-flight reading (mm converted to cm), but see **Current integration notes** below.

### ESP32 ↔ STM32: UART/text protocol (new rover only)

The gateway translates host commands as follows:

| Host path command | STM32 serial request | STM32 behaviour |
| --- | --- | --- |
| `["FWD", 200.0]` | `F:200.0mm` | Encoder-controlled forward movement. |
| `["BACK", 50.0]` | `B:50.0mm` | Encoder-controlled reverse movement. |
| `["TURN", 90.0]` | `R:90.0` | Right turn. |
| `["TURN", -90.0]` | `L:90.0` | Left turn. |
| Internal stop request | `S` | Request to abort/stop motion. |
| Connectivity/status check | `?` | Request status, including battery voltage. |

Commands are sent as **one text line per request**. The STM32's `remote.py` accepts `mm`, `cm`, or `m` for linear distances and degrees for turns. It returns text responses such as:

```text
ACK F:200.0mm
DONE F 199.2mm
OK vbatt=3.92V
STOPPED
ERR <reason>
```

The STM32 motion convention is **positive = left** internally, whereas the host JSON convention is **positive = right**; the ESP32 gateway and `remote.py` handle that translation.

## Current integration notes

The following details matter when using the files in this repository as supplied:

1. **Telemetry is not fully joined up in the Micromouse version.** The ESP32 gateway sends `TEL:ON`/`TEL:OFF` and expects `TEL vbat=... dist=...` messages, but the included `STM32L4/remote.py` does **not** handle these commands or transmit periodic telemetry. The `?` status query does work, but that is distinct from the live telemetry stream. Completing telemetry requires corresponding STM32-side support.
2. **Obstacle checking differs by generation.** The original rover checks its ultrasonic sensor before a forward move. The supplied STM32 movement routines do not implement an equivalent automatic centre-ToF stop check, even though the ESP32 understands an `obstacle` error response if the STM32 were to send one.
3. **Path `STOP` is a terminator.** The final `["STOP"]` acknowledges the end of a planned path; it does not send the UART `S` command. In the Micromouse gateway, `S` is sent on detected command timeout or loss of the executing WebSocket client.
4. **Wi-Fi settings are stored in source code.** Update `WIFI_SSID` and `WIFI_PASS` in the appropriate ESP32 `main.cpp` before use. **Remove/replace real credentials before publishing a public repository** and rotate any credentials that have already been exposed.
5. **Battery SOC is an estimate.** The original rover uses its battery library and a two-cell configuration. The newer gateway uses a voltage-based estimate with `BATTERY_CELLS` initially set to `1`; verify this against the battery actually installed.

## Project context

The firmware forms the execution layer of the AI Learner Rover project. The host application turns camera-derived scene information and a language-described goal into structured movement commands. The firmware then carries out those commands and returns execution status. Moving from the original rover to the Micromouse-based platform was intended to provide more deterministic wheel motion through dedicated encoder-feedback control, while keeping the external command interface broadly consistent.

This repository is retained to document **both the original design and the later two-controller implementation**.
