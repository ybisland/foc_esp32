#include "serial_command.h"
#include <Arduino.h>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>

static FOC_Controller *foc_controller = nullptr;

static bool is_serial_space(char c)
{
    return c == ' ' || c == '\t';
}

static char *next_serial_token(char *&cursor)
{
    while (is_serial_space(*cursor))
        ++cursor;
    if (*cursor == '\0')
        return nullptr;

    char *token = cursor;
    while (*cursor != '\0' && !is_serial_space(*cursor))
        ++cursor;
    if (*cursor != '\0')
        *cursor++ = '\0';
    return token;
}

static bool parse_float_token(const char *token, float &value)
{
    errno = 0;
    char *end = nullptr;
    value = std::strtof(token, &end);
    return end != token && *end == '\0' && errno != ERANGE && std::isfinite(value);
}

static void print_serial_help()
{
    Serial.println("Commands:");
    Serial.println("  help");
    Serial.println("  mode stop|velocity|current");
    Serial.println("  speed <rad/s>     iq <A>     cutoff <Hz>");
    Serial.println("  velpid <kp> <ki> <kd> <ramp>");
    Serial.println("  curpid <kp> <ki> <kd> <ramp>");
    Serial.println("Velocity PID outputs Iq (A).");
    Serial.println("Set motion limits through the API before selecting a closed-loop mode.");
    Serial.println("Velocity mode requires velocity and current limits; current mode requires current limits.");
    Serial.println("Current sensing is configured by configure(); calibrateSensor() calibrates current zero first.");
    Serial.println("PID commands are optional; current PID settings are used until changed.");
}

void serial_command_init(FOC_Controller *foc)
{
    foc_controller = foc;
    Serial.println("FOC ready; send help for commands.");
}

static void handle_serial_command(char *line)
{
    char *cursor = line;
    char *command = next_serial_token(cursor);
    if (command == nullptr)
        return;

    if (std::strcmp(command, "help") == 0)
    {
        if (next_serial_token(cursor) != nullptr)
            Serial.println("ERR: help takes no arguments");
        else
            print_serial_help();
        return;
    }

    if (std::strcmp(command, "mode") == 0)
    {
        char *mode_text = next_serial_token(cursor);
        if (mode_text == nullptr || next_serial_token(cursor) != nullptr)
        {
            Serial.println("ERR: usage: mode stop|velocity|current");
            return;
        }

        if (std::strcmp(mode_text, "stop") == 0)
        {
            foc_controller->setMode(FOC_MODE_STOPPED);
            Serial.println("OK mode stop");
            return;
        }

        if (std::strcmp(mode_text, "velocity") == 0)
        {
            if (!foc_controller->setMode(FOC_MODE_VELOCITY_CLOSED_LOOP))
            {
                Serial.println("ERR: configure velocity and current limits through the API first");
                return;
            }
            Serial.println("OK mode velocity");
        }
        else if (std::strcmp(mode_text, "current") == 0)
        {
            if (!foc_controller->setMode(FOC_MODE_CURRENT_LOOP))
            {
                Serial.println("ERR: configure current limits through the API first");
                return;
            }
            Serial.println("OK mode current");
        }
        else
        {
            Serial.println("ERR: mode must be stop, velocity, or current");
        }
        return;
    }

    if (std::strcmp(command, "velpid") == 0 ||
        std::strcmp(command, "curpid") == 0)
    {
        float gains[4];
        bool valid = true;
        for (float &gain : gains)
        {
            char *token = next_serial_token(cursor);
            if (token == nullptr || !parse_float_token(token, gain))
            {
                valid = false;
                break;
            }
        }
        if (!valid || next_serial_token(cursor) != nullptr || gains[3] < 0.0f)
        {
            Serial.println("ERR: usage: <velpid|curpid> <kp> <ki> <kd> <ramp>=0");
            return;
        }

        if (std::strcmp(command, "velpid") == 0)
        {
            foc_controller->setVelPID(gains[0], gains[1], gains[2], gains[3]);
            Serial.println("OK velpid");
        }
        else
        {
            foc_controller->setCurPID(gains[0], gains[1], gains[2], gains[3]);
            Serial.println("OK curpid");
        }
        return;
    }

    if (std::strcmp(command, "speed") == 0 ||
        std::strcmp(command, "cutoff") == 0 ||
        std::strcmp(command, "iq") == 0)
    {
        char *token = next_serial_token(cursor);
        float value;
        if (token == nullptr || next_serial_token(cursor) != nullptr ||
            !parse_float_token(token, value))
        {
            Serial.println("ERR: expected one finite numeric value");
            return;
        }

        if (std::strcmp(command, "cutoff") == 0)
        {
            if (value <= 0.0f)
            {
                Serial.println("ERR: cutoff must be positive");
                return;
            }
            foc_controller->setVelocityFilterCutoff(value);
            Serial.println("OK cutoff");
            return;
        }

        if (std::strcmp(command, "speed") == 0)
        {
            foc_controller->setTargetVelocity(value);
            Serial.println("OK speed target");
        }
        else
        {
            foc_controller->SetTargetCurrent(value);
            Serial.println("OK Iq target");
        }
    }
    else
    {
        Serial.println("ERR: unknown command; send help");
    }
}

void receive_serial_commands()
{
    static char line[64];
    static size_t length = 0;
    static bool discard_line = false;
    const size_t max_bytes_per_loop = 32;

    for (size_t count = 0; count < max_bytes_per_loop && Serial.available() > 0; ++count)
    {
        const int incoming = Serial.read();
        if (incoming < 0)
            break;

        if (incoming == '\r' || incoming == '\n')
        {
            if (discard_line)
                Serial.println("ERR: command too long or contains NUL");
            else if (length > 0)
            {
                line[length] = '\0';
                handle_serial_command(line);
            }
            length = 0;
            discard_line = false;
        }
        else if (!discard_line)
        {
            if (incoming == '\0' || length == sizeof(line) - 1)
                discard_line = true;
            else
                line[length++] = static_cast<char>(incoming);
        }
    }
}