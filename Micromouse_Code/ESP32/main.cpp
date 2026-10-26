// AI Learner Rover - WebSocket command interpreter (STM32 micromouse edition).
//
// Same host protocol as the old all-ESP32 rover (see command_protocol.docx):
//   laptop GUI  <--WebSocket/JSON-->  ESP32  <--UART (text lines)-->  STM32 micromouse
//
// The ESP32 no longer drives motors itself. It decodes the JSON path and
// translates every command into a one-line text command for the STM32
// (remote.py):
//
//      FWD  200   ->  F:200.0mm
//      BACK 100   ->  B:100.0mm        (real reverse, no 180 turn any more)
//      TURN +30   ->  R:30.0           (+ = right)
//      TURN -30   ->  L:30.0           (- = left)
//
// Turns are done on the STM32 with its encoders, so the camera heading
// request/reply loop of the old rover is gone ("req_heading" is never sent).
// Telemetry "dist" is now the CENTRE time-of-flight sensor, in cm.
//
// Wiring (3.3 V, TX/RX crossed, common GND):
//   ESP32 GPIO17 (TX2) -> STM32 RX (PB7, J2 pin 8)
//   ESP32 GPIO16 (RX2) <- STM32 TX (PB6, J2 pin 7)
//   ESP32 GND          -> STM32 GND
//
// Libraries: WebSockets (links2004), ArduinoJson v7.
#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <math.h>
#include <string>

const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";

// ---- UART to the STM32 --------------------------------------------------
#define STM_RX_PIN 16                        // ESP32 RX2 <- STM32 TX (PB6)
#define STM_TX_PIN 17                        // ESP32 TX2 -> STM32 RX (PB7)
static const uint32_t STM_BAUD = 115200;

// ---- battery ------------------------------------------------------------
// Number of Li-ion cells in series in the mouse's battery, used ONLY for the
// state-of-charge estimate. Compare the "vbat" in telemetry with a multimeter
// reading: ~3.7-4.2 V means 1 cell; ~7.4-8.4 V means 2 cells.
static const int BATTERY_CELLS = 1;

// ---- timeouts -----------------------------------------------------------
static const unsigned long STM_PING_TIMEOUT_MS = 700;   // per ping attempt
static const int STM_PING_ATTEMPTS = 3;
static const unsigned long STM_STOP_ACK_MS = 1500;      // wait for STOPPED after "S"

WebSocketsServer webSocket(81);

bool executingPath = false;
bool telemetryEnabled = false;
int telemetryClient = -1;

int pathClient = -1;              // client whose path is running
bool pathClientGone = false;      // it disconnected mid-path -> stop the mouse

// IMPORTANT: never execute a complete path from inside the WebSocket callback
// (same reason as the old rover). Queue the raw path and run it from loop().
bool pathPending = false;
uint8_t pendingPathClient = 0;
std::string pendingPathJson;

enum class CommandResult
{
    Success,
    InvalidCommand,
    Obstacle,
    Stm32Timeout,
    Stm32Offline,
    Stopped,
    TurnFailed,
    DriveFailed
};

// =========================================================================
// JSON replies to the laptop (identical to the old rover)
// =========================================================================
void sendPong(uint8_t clientNum, JsonDocument& in)
{
    JsonDocument reply;
    reply["t"] = "pong";
    if (!in["ts"].isNull()) reply["ts"] = in["ts"];
    char out[96];
    size_t n = serializeJson(reply, out);
    webSocket.sendTXT(clientNum, out, n);
}

void sendAckCmd(uint8_t clientNum, long pathId, int cmdIndex)
{
    JsonDocument reply;
    reply["t"] = "ack";
    reply["of"] = "cmd";
    reply["id"] = pathId;
    reply["i"] = cmdIndex;
    char out[96];
    size_t n = serializeJson(reply, out);
    webSocket.sendTXT(clientNum, out, n);
}

void sendAckPath(uint8_t clientNum, long pathId)
{
    JsonDocument reply;
    reply["t"] = "ack";
    reply["of"] = "path";
    reply["id"] = pathId;
    char out[96];
    size_t n = serializeJson(reply, out);
    webSocket.sendTXT(clientNum, out, n);
}

void sendNack(uint8_t clientNum, long pathId, int cmdIndex, const char* reason)
{
    JsonDocument reply;
    reply["t"] = "nack";
    reply["id"] = pathId;
    reply["i"] = cmdIndex;
    reply["reason"] = reason;
    char out[144];
    size_t n = serializeJson(reply, out);
    webSocket.sendTXT(clientNum, out, n);
}

void sendInvalidCommand(uint8_t clientNum)
{
    JsonDocument reply;
    reply["t"] = "invalid";
    reply["message"] = "invalid command";
    char out[96];
    size_t n = serializeJson(reply, out);
    webSocket.sendTXT(clientNum, out, n);
}

const char* commandResultReason(CommandResult result)
{
    switch (result)
    {
        case CommandResult::InvalidCommand: return "invalid_command";
        case CommandResult::Obstacle:       return "obstacle";
        case CommandResult::Stm32Timeout:   return "stm32_timeout";
        case CommandResult::Stm32Offline:   return "stm32_offline";
        case CommandResult::Stopped:        return "stopped";
        case CommandResult::TurnFailed:     return "turn_failed";
        case CommandResult::DriveFailed:    return "drive_failed";
        case CommandResult::Success:
        default:                            return "command_failed";
    }
}

// =========================================================================
// UART link to the STM32
// =========================================================================
static char stmBuf[96];
static size_t stmLen = 0;

// Non-blocking: returns true when a whole line has arrived.
static bool readStmLine(char* out, size_t outSize)
{
    while (Serial2.available())
    {
        char c = (char)Serial2.read();
        if (c == '\n' || c == '\r')
        {
            if (stmLen > 0)
            {
                stmBuf[stmLen] = '\0';
                strncpy(out, stmBuf, outSize - 1);
                out[outSize - 1] = '\0';
                stmLen = 0;
                return true;
            }
        }
        else if (stmLen < sizeof(stmBuf) - 1)
        {
            stmBuf[stmLen++] = c;
        }
        else
        {
            stmLen = 0;                        // overlong junk: start over
        }
    }
    return false;
}

static bool startsWith(const char* s, const char* prefix)
{
    return strncmp(s, prefix, strlen(prefix)) == 0;
}

static void sendStm(const char* cmd)
{
    Serial2.print(cmd);
    Serial2.print('\n');
    Serial.printf("  ESP32 -> STM32: %s\n", cmd);
}

// ---- telemetry ----------------------------------------------------------
// Rough Li-ion state of charge from the per-cell voltage (linear between
// these points). Only an estimate - it moves with load.
static float estimateSoc(float vbat)
{
    static const float volts[] = {4.20f, 4.10f, 4.00f, 3.90f, 3.80f,
                                  3.70f, 3.60f, 3.50f, 3.30f, 3.00f};
    static const float pct[]   = {100, 90, 80, 65, 50, 35, 20, 10, 3, 0};
    const int N = sizeof(volts) / sizeof(volts[0]);

    const float v = vbat / (float)BATTERY_CELLS;
    if (v >= volts[0]) return 100.0f;
    if (v <= volts[N - 1]) return 0.0f;
    for (int i = 0; i < N - 1; i++)
    {
        if (v <= volts[i] && v >= volts[i + 1])
        {
            const float f = (v - volts[i + 1]) / (volts[i] - volts[i + 1]);
            return pct[i + 1] + f * (pct[i] - pct[i + 1]);
        }
    }
    return 0.0f;
}

// "TEL vbat=3.92 dist=250"  ->  {"t":"telem","vbat":3.92,"soc":81,"dist":25.0}
// dist is the CENTRE time-of-flight reading converted mm -> cm (-1 = nothing
// in range / sensor error), matching the old ultrasonic field.
static void forwardTelemetry(const char* line)
{
    if (!telemetryEnabled || telemetryClient < 0) return;

    float vbat = NAN;
    long distMM = -1;
    const char* p = strstr(line, "vbat=");
    if (p) vbat = (float)atof(p + 5);
    p = strstr(line, "dist=");
    if (p) distMM = atol(p + 5);
    if (!isfinite(vbat)) return;

    const float soc = estimateSoc(vbat);

    JsonDocument sample;
    sample["t"] = "telem";
    sample["vbat"] = roundf(vbat * 100.0f) / 100.0f;
    sample["soc"] = static_cast<int>(lroundf(soc));
    sample["dist"] = (distMM >= 0) ? roundf((float)distMM) / 10.0f : -1.0f;

    char out[128];
    size_t n = serializeJson(sample, out);
    webSocket.sendTXT(static_cast<uint8_t>(telemetryClient), out, n);
}

// Idle pump: forward telemetry, log anything else (boot text, OK ..., etc.)
static void pumpStmIdle()
{
    char line[96];
    while (readStmLine(line, sizeof(line)))
    {
        if (startsWith(line, "TEL")) forwardTelemetry(line);
        else Serial.printf("  STM32: %s\n", line);
    }
}

// Is the mouse alive? Ping with "?" until it answers "OK ...".
static bool pingStm32()
{
    char line[96];
    pumpStmIdle();                                  // drop stale lines first
    for (int attempt = 0; attempt < STM_PING_ATTEMPTS; attempt++)
    {
        sendStm("?");
        const unsigned long t0 = millis();
        while (millis() - t0 < STM_PING_TIMEOUT_MS)
        {
            webSocket.loop();
            while (readStmLine(line, sizeof(line)))
            {
                if (startsWith(line, "TEL")) { forwardTelemetry(line); continue; }
                Serial.printf("  STM32: %s\n", line);
                if (startsWith(line, "OK")) return true;
            }
            delay(1);
        }
    }
    return false;
}

// After asking the mouse to stop, wait for its STOPPED so a late reply can't
// be mistaken for the answer to the NEXT command.
static void waitStopAck()
{
    char line[96];
    const unsigned long t0 = millis();
    while (millis() - t0 < STM_STOP_ACK_MS)
    {
        webSocket.loop();
        while (readStmLine(line, sizeof(line)))
        {
            if (startsWith(line, "TEL")) { forwardTelemetry(line); continue; }
            Serial.printf("  STM32: %s\n", line);
            if (startsWith(line, "STOPPED")) return;
        }
        delay(1);
    }
}

// Send one command to the mouse and wait until it reports the outcome.
static CommandResult runStm(const char* cmd, unsigned long timeoutMS,
                            CommandResult failResult)
{
    pumpStmIdle();                                  // drop stale lines first
    sendStm(cmd);

    char line[96];
    const unsigned long t0 = millis();
    while (millis() - t0 < timeoutMS)
    {
        webSocket.loop();                           // keep the network alive

        if (pathClientGone)                         // laptop vanished: halt
        {
            sendStm("S");
            return CommandResult::Stopped;
        }

        while (readStmLine(line, sizeof(line)))
        {
            if (startsWith(line, "TEL")) { forwardTelemetry(line); continue; }
            Serial.printf("  STM32: %s\n", line);
            if (startsWith(line, "DONE"))    return CommandResult::Success;
            if (startsWith(line, "STOPPED")) return CommandResult::Stopped;
            if (startsWith(line, "ERR"))
            {
                if (strstr(line, "obstacle")) return CommandResult::Obstacle;
                return failResult;
            }
            // ACK / READY / boot text: ignore
        }
        delay(1);
    }

    Serial.println("  STM32 did not finish in time - sending stop");
    sendStm("S");
    waitStopAck();
    return CommandResult::Stm32Timeout;
}

// =========================================================================
// Telemetry switch from the laptop
// =========================================================================
void handleTelemetrySwitch(uint8_t clientNum, JsonDocument& doc)
{
    JsonVariant on = doc["on"];
    if (!on.is<bool>())
    {
        sendInvalidCommand(clientNum);
        return;
    }
    telemetryEnabled = on.as<bool>();
    telemetryClient = telemetryEnabled ? static_cast<int>(clientNum) : -1;
    // The STM32 decides the rate (1 Hz) and streams samples while this is on.
    sendStm(telemetryEnabled ? "TEL:ON" : "TEL:OFF");
}

// =========================================================================
// Commands
// =========================================================================
bool validateCommand(JsonArrayConst cmd)
{
    if (cmd.isNull() || cmd.size() == 0 || !cmd[0].is<const char*>())
        return false;
    const char* op = cmd[0].as<const char*>();
    if (strcmp(op, "STOP") == 0) return cmd.size() == 1;
    if (strcmp(op, "FWD") != 0 &&
        strcmp(op, "BACK") != 0 &&
        strcmp(op, "TURN") != 0)
        return false;
    if (cmd.size() != 2 || !cmd[1].is<float>()) return false;
    const float value = cmd[1].as<float>();
    if (!isfinite(value)) return false;
    if ((strcmp(op, "FWD") == 0 || strcmp(op, "BACK") == 0) && value <= 0.0f)
        return false;
    return strcmp(op, "TURN") != 0 || fabsf(value) >= 1.0f;
}

CommandResult executeCommand(uint8_t /*clientNum*/, JsonArrayConst cmd)
{
    if (!validateCommand(cmd)) return CommandResult::InvalidCommand;
    const char* op = cmd[0].as<const char*>();
    if (strcmp(op, "STOP") == 0) return CommandResult::Success;

    const float value = cmd[1].as<float>();
    char line[32];

    if (strcmp(op, "FWD") == 0 || strcmp(op, "BACK") == 0)
    {
        const bool fwd = strcmp(op, "FWD") == 0;
        snprintf(line, sizeof(line), "%c:%.1fmm", fwd ? 'F' : 'B', value);
        // The mouse gives up after 3 s + 10 s/m; allow a margin on top.
        const unsigned long timeoutMS = 7000UL + (unsigned long)(10.0f * value);
        return runStm(line, timeoutMS, CommandResult::DriveFailed);
    }

    // TURN: protocol says +right / -left, done on the mouse with its encoders.
    snprintf(line, sizeof(line), "%c:%.1f", value > 0.0f ? 'R' : 'L', fabsf(value));
    const unsigned long timeoutMS = 7000UL + (unsigned long)(34.0f * fabsf(value));
    return runStm(line, timeoutMS, CommandResult::TurnFailed);
}

void handlePath(uint8_t clientNum, JsonDocument& doc)
{
    const long pathId = doc["id"] | 0;
    JsonArrayConst cmds = doc["cmds"];

    if (cmds.isNull() || cmds.size() == 0)
    {
        sendInvalidCommand(clientNum);
        return;
    }

    // Reject a bad path before anything moves. STOP must be last.
    int validationIndex = 0;
    for (JsonVariantConst cmdVar : cmds)
    {
        JsonArrayConst cmd = cmdVar.as<JsonArrayConst>();
        if (!validateCommand(cmd))
        {
            sendNack(clientNum, pathId, validationIndex, "invalid_command");
            return;
        }
        const char* op = cmd[0].as<const char*>();
        const bool isLast = validationIndex == static_cast<int>(cmds.size()) - 1;
        if ((strcmp(op, "STOP") == 0) != isLast)
        {
            sendNack(clientNum, pathId, validationIndex, "invalid_command");
            return;
        }
        ++validationIndex;
    }

    // Make sure the mouse is actually there before we promise anything.
    if (!pingStm32())
    {
        Serial.println("  STM32 not answering - path refused");
        sendNack(clientNum, pathId, 0, commandResultReason(CommandResult::Stm32Offline));
        return;
    }

    Serial.printf("  Executing path id=%ld with %u commands\n", pathId, cmds.size());
    executingPath = true;
    pathClient = clientNum;
    pathClientGone = false;

    int index = 0;
    for (JsonVariantConst cmdVar : cmds)
    {
        JsonArrayConst cmd = cmdVar.as<JsonArrayConst>();
        const char* op = cmd[0].as<const char*>();
        Serial.printf("  [cmd %d] %s\n", index, op);

        const CommandResult result = executeCommand(clientNum, cmd);
        webSocket.loop();

        if (pathClientGone)                 // nobody left to tell
        {
            Serial.println("  Client left mid-path - stopped");
            break;
        }
        if (result != CommandResult::Success)
        {
            sendNack(clientNum, pathId, index, commandResultReason(result));
            Serial.printf("  NACK at cmd %d: %s\n", index, commandResultReason(result));
            executingPath = false;
            pathClient = -1;
            return;
        }
        sendAckCmd(clientNum, pathId, index);
        webSocket.loop();
        ++index;
    }

    if (!pathClientGone)
    {
        sendAckPath(clientNum, pathId);
        Serial.printf("  Path id=%ld complete\n", pathId);
    }
    executingPath = false;
    pathClient = -1;
}

// =========================================================================
// WebSocket events
// =========================================================================
void onWebSocketEvent(uint8_t clientNum, WStype_t type, uint8_t* payload, size_t length)
{
    switch (type)
    {
        case WStype_CONNECTED:
        {
            IPAddress ip = webSocket.remoteIP(clientNum);
            Serial.printf("[%u] Connected from %s\n", clientNum, ip.toString().c_str());
            break;
        }

        case WStype_DISCONNECTED:
            Serial.printf("[%u] Disconnected\n", clientNum);
            if (telemetryClient == clientNum)
            {
                telemetryEnabled = false;
                telemetryClient = -1;
                sendStm("TEL:OFF");
            }
            if (executingPath && pathClient == clientNum)
                pathClientGone = true;      // runStm() will send the stop
            if (pathPending && pendingPathClient == clientNum)
            {
                pathPending = false;
                pendingPathJson.clear();
            }
            break;

        case WStype_TEXT:
        {
            Serial.printf("[%u] RX: %.*s\n", clientNum, static_cast<int>(length), payload);

            JsonDocument doc;
            DeserializationError error = deserializeJson(doc, payload, length);
            if (error || !doc.is<JsonObject>())
            {
                Serial.println("  Invalid JSON/syntax");
                sendInvalidCommand(clientNum);
                return;
            }

            const char* messageType = doc["t"];
            if (!messageType)
            {
                sendInvalidCommand(clientNum);
                return;
            }

            if (strcmp(messageType, "telem") == 0)
            {
                handleTelemetrySwitch(clientNum, doc);
                return;
            }

            if (strcmp(messageType, "ping") == 0)
            {
                sendPong(clientNum, doc);
                return;
            }

            if (strcmp(messageType, "path") == 0)
            {
                const long pathId = doc["id"] | 0;
                if (executingPath || pathPending)
                {
                    sendNack(clientNum, pathId, -1, "busy");
                    return;
                }
                pendingPathJson.assign(reinterpret_cast<const char*>(payload), length);
                pendingPathClient = clientNum;
                pathPending = true;
                Serial.printf("  Queued path id=%ld for execution\n", pathId);
                return;
            }

            // "heading", "speed", "pid" and anything unknown: rejected, exactly
            // like the old rover. (Headings are no longer used - turns are
            // encoder-based now.)
            sendInvalidCommand(clientNum);
            break;
        }

        default:
            break;
    }
}

void processPendingPath()
{
    if (!pathPending || executingPath) return;

    const uint8_t clientNum = pendingPathClient;
    const std::string pathJson = pendingPathJson;
    pathPending = false;
    pendingPathJson.clear();

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, pathJson.data(), pathJson.size());
    if (error || !doc.is<JsonObject>())
    {
        Serial.printf("  Queued path JSON invalid: %s\n", error.c_str());
        sendInvalidCommand(clientNum);
        return;
    }
    handlePath(clientNum, doc);
}

// =========================================================================
void setup()
{
    Serial.begin(115200);
    Serial2.begin(STM_BAUD, SERIAL_8N1, STM_RX_PIN, STM_TX_PIN);
    setCpuFrequencyMhz(240);
    delay(500);
    Serial.println("Rover (STM32 micromouse) starting...");

    Serial.printf("Connecting to %s ", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(300);
        Serial.print(".");
    }
    Serial.println();
    Serial.print("Rover IP: ");
    Serial.println(WiFi.localIP());

    webSocket.begin();
    webSocket.onEvent(onWebSocketEvent);
    Serial.println("WebSocket server on port 81");
    Serial.println("Connect: websocat ws://<rover-ip>:81");

    Serial.println(pingStm32() ? "STM32 micromouse answered."
                               : "WARNING: STM32 not answering (check wiring / remote.run()).");
}

void loop()
{
    webSocket.loop();
    pumpStmIdle();            // forwards telemetry while no path is running
    processPendingPath();
}
