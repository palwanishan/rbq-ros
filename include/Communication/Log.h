// Copyright 2023 Rainbow Robotics Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <iomanip>
#include <iostream>

class Log
{
public:
    Log() {

    }
    ~Log() {
        
    }

    enum LOG_LVL : char {
        LEVEL_ERROR,
        LEVEL_WARNING,
        LEVEL_INFO,
        LEVEL_DEBUG,
    };

    static void LogMessage(const char *_message)
    {
        //printf("[INFO] %s\n", _message);
        Log::LogMessage(_message, LEVEL_DEBUG);
        return;
    }

    static LOG_LVL responseLogLevel;

    static void LogMessage(const char *_message, const LOG_LVL _level)
    {
        std::string lvl_;
        switch (_level) {
        case LEVEL_ERROR: {
            lvl_ = "[ERROR]";
        }
        case LEVEL_WARNING: {
            lvl_ = "[WARNING]";
        }
        case LEVEL_INFO: {
            lvl_ = "[INFO]";
        }
        case LEVEL_DEBUG: {
            lvl_ = "[DEGUB]";
        }
        }

        printf("%s %s\n", lvl_.data(), _message);
        return;
    }

    static void SetLogLevel(const int &_lvl)
    {
        switch (_lvl) {
        case (LEVEL_ERROR): {
            responseLogLevel = LEVEL_ERROR;
            break;
        }
        case (LEVEL_WARNING): {
            responseLogLevel = LEVEL_WARNING;
            break;
        }
        case (LEVEL_INFO): {
            responseLogLevel = LEVEL_INFO;
            break;
        }
        case (LEVEL_DEBUG): {
            responseLogLevel = LEVEL_DEBUG;
            break;
        }
        default: {
            LogMessage("Log level is not identified, setting to default", Log::LEVEL_WARNING);
            responseLogLevel = LEVEL_ERROR;
            break;
        }
        }
    }

    static void PrintBytes(
        std::ostream &out, const char *title, const char *data, size_t dataLen, bool format = true)
    {
        out << title << std::endl;
        out << std::setfill('0');
        for (size_t i = 0; i < dataLen; ++i) {
            out << std::hex << std::setw(2) << (int) data[i];
            if (format) {
                out << (((i + 1) % 16 == 0) ? "\n" : " ");
            }
        }
        out << std::endl;
    }
};