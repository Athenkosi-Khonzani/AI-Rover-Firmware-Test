// AI Learner Rover - WebSocket command interpreter.
// Motion primitives are intentionally kept in their existing libraries.

#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <math.h>

#include <Battery.h>
#include <StraightDrive.h>
#include <TurnDrive.h>
#include <ultrasonic.h>

const char* WIFI_SSID = "my_wind_comp";
const char* WIFI_PASS = "12345678";

static const unsigned long TELEMETRY_INTERVAL_MS = 1000;
static const unsigned long HEADING_TIMEOUT_MS = 5000;
static const float TURN_SUCCESS_TOLERANCE_DEG = 1.0f;
static const uint8_t MAX_TURN_CORRECTIONS = 3;

WebSocketsServer webSocket(81);
battery BatteryMonitor;
ultrasonic Ultrasonic;

bool executingPath = false;
bool telemetryEnabled = false;
int telemetryClient = -1;
unsigned long lastTelemetryMS = 0;

bool waitingForHeading = false;
bool headingReady = false;
int headingClient = -1;
float receivedHeadingDeg = 0.0f;

enum class CommandResult
{
    Success,
    InvalidCommand,
    Obstacle,
    UltrasonicError,
    HeadingTimeout,
    TurnFailed,
    TurnCorrectionFailed,
    DriveFailed
};

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

void sendNack(uint8_t clientNum, long pathId, int cmdIndex,
              const char* reason)
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

void sendHeadingRequest(uint8_t clientNum)
{
    JsonDocument request;
    request["t"] = "req_heading";
    char out[64];
    size_t n = serializeJson(request, out);
    webSocket.sendTXT(clientNum, out, n);
}

const char* commandResultReason(CommandResult result)
{
    switch (result)
    {
        case CommandResult::InvalidCommand:       return "invalid_command";
        case CommandResult::Obstacle:             return "obstacle";
        case CommandResult::UltrasonicError:      return "ultrasonic_error";
        case CommandResult::HeadingTimeout:       return "heading_timeout";
        case CommandResult::TurnFailed:           return "turn_failed";
        case CommandResult::TurnCorrectionFailed: return "turn_correction_failed";
        case CommandResult::DriveFailed:          return "drive_failed";
        case CommandResult::Success:
        default:                                  return "command_failed";
    }
}

void sendTelemetryIfDue()
{
    // No telemetry is sent before an explicit telem-on message.
    if (!telemetryEnabled || telemetryClient < 0) return;

    const unsigned long now = millis();
    if (now - lastTelemetryMS < TELEMETRY_INTERVAL_MS) return;
    lastTelemetryMS = now;

    const float voltage = BatteryMonitor.ReadVoltage(8);
    const float soc = BatteryMonitor.EstimateSOC(voltage);
    const float distanceCM = Ultrasonic.Ranging();

    JsonDocument sample;
    sample["t"] = "telem";
    sample["vbat"] = roundf(voltage * 100.0f) / 100.0f;
    sample["soc"] = isfinite(soc) ? static_cast<int>(lroundf(soc)) : -1;
    sample["dist"] = (isfinite(distanceCM) && distanceCM >= 0.0f)
                         ? roundf(distanceCM * 10.0f) / 10.0f
                         : -1.0f;

    char out[128];
    size_t n = serializeJson(sample, out);
    webSocket.sendTXT(static_cast<uint8_t>(telemetryClient), out, n);
}

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
    if (telemetryEnabled)
    {
        lastTelemetryMS = millis() - TELEMETRY_INTERVAL_MS;
        sendTelemetryIfDue();
    }
}

// Camera headings are 0 <= heading < 360 degrees.
float normalizeHeading(float angleDeg)
{
    float result = fmodf(angleDeg, 360.0f);
    if (result < 0.0f) result += 360.0f;
    return result;
}

// Shortest rotation from current to target. Positive=right, negative=left.
float signedHeadingError(float targetDeg, float currentDeg)
{
    float error = normalizeHeading(targetDeg) - normalizeHeading(currentDeg);
    if (error > 180.0f) error -= 360.0f;
    if (error < -180.0f) error += 360.0f;
    return error;
}

bool requestHeading(uint8_t clientNum, float& headingDeg)
{
    headingReady = false;
    waitingForHeading = true;
    headingClient = clientNum;
    sendHeadingRequest(clientNum);

    const unsigned long startMS = millis();
    while (!headingReady && millis() - startMS < HEADING_TIMEOUT_MS)
    {
        // This lets the camera's heading reply enter the callback while the
        // current path handler is waiting.
        webSocket.loop();
        sendTelemetryIfDue();
        delay(1);
    }

    waitingForHeading = false;
    headingClient = -1;
    if (!headingReady) return false;

    headingDeg = normalizeHeading(receivedHeadingDeg);
    return true;
}

bool runTurnPrimitive(float signedAngleDeg)
{
    if (signedAngleDeg < 0.0f) return turnLeft(-signedAngleDeg);
    return turnRight(signedAngleDeg);
}

CommandResult executeVerifiedTurn(uint8_t clientNum, float requestedAngleDeg)
{
    if (!isfinite(requestedAngleDeg) || fabsf(requestedAngleDeg) < 1.0f)
        return CommandResult::InvalidCommand;

    float initialHeading = 0.0f;
    if (!requestHeading(clientNum, initialHeading))
        return CommandResult::HeadingTimeout;

    // Protocol convention: +TURN is right, -TURN is left.
    const float targetHeading =
        normalizeHeading(initialHeading + requestedAngleDeg);

    Serial.printf("  Heading %.2f deg -> target %.2f deg\n",
                  initialHeading, targetHeading);

    if (!runTurnPrimitive(requestedAngleDeg))
        return CommandResult::TurnFailed;

    uint8_t corrections = 0;
    while (true)
    {
        float measuredHeading = 0.0f;
        if (!requestHeading(clientNum, measuredHeading))
            return CommandResult::HeadingTimeout;

        const float errorDeg =
            signedHeadingError(targetHeading, measuredHeading);

        Serial.printf("  Camera heading %.2f deg, error %.2f deg\n",
                      measuredHeading, errorDeg);

        if (fabsf(errorDeg) < TURN_SUCCESS_TOLERANCE_DEG)
            return CommandResult::Success;

        if (corrections >= MAX_TURN_CORRECTIONS)
            return CommandResult::TurnCorrectionFailed;

        ++corrections;
        Serial.printf("  Turn correction %u/%u: %.2f deg\n",
                      corrections, MAX_TURN_CORRECTIONS, errorDeg);

        if (!runTurnPrimitive(errorDeg))
            return CommandResult::TurnFailed;
    }
}

CommandResult executeSafeStraight(float distanceMM)
{
    if (!isfinite(distanceMM) || distanceMM <= 0.0f)
        return CommandResult::InvalidCommand;

    const float clearDistanceCM = Ultrasonic.Ranging();
    if (!isfinite(clearDistanceCM) || clearDistanceCM < 0.0f)
        return CommandResult::UltrasonicError;

    const float requestedDistanceCM = distanceMM / 10.0f;
    Serial.printf("  Straight request %.1f cm; ultrasonic %.1f cm\n",
                  requestedDistanceCM, clearDistanceCM);

    // Units are converted before comparison: command mm -> ultrasonic cm.
    if (requestedDistanceCM >= clearDistanceCM)
        return CommandResult::Obstacle;

    return goStraight(distanceMM)
               ? CommandResult::Success
               : CommandResult::DriveFailed;
}

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

    if ((strcmp(op, "FWD") == 0 || strcmp(op, "BACK") == 0) &&
        value <= 0.0f)
        return false;

    return strcmp(op, "TURN") != 0 || fabsf(value) >= 1.0f;
}

CommandResult executeCommand(uint8_t clientNum, JsonArrayConst cmd)
{
    if (!validateCommand(cmd)) return CommandResult::InvalidCommand;

    const char* op = cmd[0].as<const char*>();
    if (strcmp(op, "STOP") == 0) return CommandResult::Success;

    const float value = cmd[1].as<float>();
    if (strcmp(op, "FWD") == 0) return executeSafeStraight(value);

    if (strcmp(op, "BACK") == 0)
    {
        // goStraight(-value) is unsupported. Face the opposite direction using
        // the camera-verified turn, then drive a positive distance.
        CommandResult result = executeVerifiedTurn(clientNum, 180.0f);
        if (result != CommandResult::Success) return result;
        return executeSafeStraight(value);
    }

    return executeVerifiedTurn(clientNum, value);
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

    // Reject a bad path before starting any motors. STOP must be last.
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
        const bool isLast =
            validationIndex == static_cast<int>(cmds.size()) - 1;
        if ((strcmp(op, "STOP") == 0) != isLast)
        {
            sendNack(clientNum, pathId, validationIndex, "invalid_command");
            return;
        }
        ++validationIndex;
    }

    Serial.printf("  Executing path id=%ld with %u commands\n",
                  pathId, cmds.size());
    executingPath = true;

    int index = 0;
    for (JsonVariantConst cmdVar : cmds)
    {
        JsonArrayConst cmd = cmdVar.as<JsonArrayConst>();
        const char* op = cmd[0].as<const char*>();
        Serial.printf("  [cmd %d] %s\n", index, op);

        const CommandResult result = executeCommand(clientNum, cmd);

        // Catch telemetry and network work up between blocking primitives.
        webSocket.loop();
        sendTelemetryIfDue();

        if (result != CommandResult::Success)
        {
            sendNack(clientNum, pathId, index, commandResultReason(result));
            Serial.printf("  NACK at cmd %d: %s\n",
                          index, commandResultReason(result));
            executingPath = false;
            return;
        }

        sendAckCmd(clientNum, pathId, index);
        webSocket.loop();
        ++index;
    }

    sendAckPath(clientNum, pathId);
    Serial.printf("  Path id=%ld complete\n", pathId);
    executingPath = false;
}

void onWebSocketEvent(uint8_t clientNum, WStype_t type,
                      uint8_t* payload, size_t length)
{
    switch (type)
    {
        case WStype_CONNECTED:
        {
            IPAddress ip = webSocket.remoteIP(clientNum);
            Serial.printf("[%u] Connected from %s\n",
                          clientNum, ip.toString().c_str());
            break;
        }

        case WStype_DISCONNECTED:
            Serial.printf("[%u] Disconnected\n", clientNum);
            if (telemetryClient == clientNum)
            {
                telemetryEnabled = false;
                telemetryClient = -1;
            }
            if (headingClient == clientNum)
            {
                waitingForHeading = false;
                headingClient = -1;
            }
            break;

        case WStype_TEXT:
        {
            Serial.printf("[%u] RX: %.*s\n",
                          clientNum, static_cast<int>(length), payload);

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

            if (strcmp(messageType, "heading") == 0)
            {
                JsonVariant deg = doc["deg"];
                const float heading = deg.as<float>();
                if (!waitingForHeading || headingClient != clientNum ||
                    !deg.is<float>() || !isfinite(heading) ||
                    heading < 0.0f || heading > 360.0f)
                {
                    sendInvalidCommand(clientNum);
                    return;
                }
                receivedHeadingDeg = normalizeHeading(heading);
                headingReady = true;
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
                if (executingPath)
                {
                    const long pathId = doc["id"] | 0;
                    sendNack(clientNum, pathId, -1, "busy");
                    return;
                }
                handlePath(clientNum, doc);
                return;
            }

            sendInvalidCommand(clientNum);
            break;
        }

        default:
            break;
    }
}

void setup()
{
    Serial.begin(115200);
    setCpuFrequencyMhz(240);
    delay(500);

    Serial.println("Rover starting...");
    BatteryMonitor.Init();
    BatteryMonitor.SetCellCount(2);
    Ultrasonic.Init();

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
}

void loop()
{
    webSocket.loop();
    sendTelemetryIfDue();
}
