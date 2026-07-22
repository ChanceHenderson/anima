#pragma once
#include <Arduino.h>
#include <LittleFS.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "motortune.h"

// Per-blaster tuning data, persisted to LittleFS: one MotorTune (feed-forward
// curve + gain grid) per wheel. This is one-per-blaster - it describes the
// motors/wheels/battery combo - so it lives in its own file, separate from
// the firing profiles.
// load()/save() touch flash, so only call them from core 0 - same rule as the
// profile saves handled in loop().

#define TUNING_FILE_PATH "/tuning.cfg"

class TuningConstants {
public:
    MotorTune left;
    MotorTune right;

    void set_defaults()
    {
        left.set_defaults();
        right.set_defaults();
    }

    bool is_sane() const
    {
        return left.is_sane() && right.is_sane();
    }

    // Returns true if valid constants were loaded from flash.
    // Falls back to the defaults (and returns false) on any failure,
    // deleting the file if it exists but is corrupt or an old version.
    bool load()
    {
        set_defaults();
        if (!LittleFS.begin()) return false;
        bool loaded = false;
        if (LittleFS.exists(TUNING_FILE_PATH))
        {
            File f = LittleFS.open(TUNING_FILE_PATH, "r");
            if (f)
            {
                Record rec;
                if (f.size() == sizeof(Record) &&
                    f.read((uint8_t*)&rec, sizeof(Record)) == sizeof(Record) &&
                    rec.magic == RECORD_MAGIC &&
                    rec.version == RECORD_VERSION &&
                    rec.checksum == checksum(rec))
                {
                    left = rec.left;
                    right = rec.right;
                    loaded = is_sane();
                }
                f.close();
            }
            if (!loaded)
            {
                LittleFS.remove(TUNING_FILE_PATH); // corrupt or outdated - delete it
                set_defaults();
            }
        }
        LittleFS.end();
        return loaded;
    }

    bool save() const
    {
        if (!is_sane()) return false;
        Record rec;
        memset(&rec, 0, sizeof(rec)); // zero padding bytes so the checksum is deterministic
        rec.magic = RECORD_MAGIC;
        rec.version = RECORD_VERSION;
        rec.left = left;
        rec.right = right;
        rec.checksum = checksum(rec);
        if (!LittleFS.begin()) return false;
        bool ok = false;
        File f = LittleFS.open(TUNING_FILE_PATH, "w");
        if (f)
        {
            ok = (f.write((const uint8_t*)&rec, sizeof(Record)) == sizeof(Record));
            f.close();
        }
        LittleFS.end();
        return ok;
    }

private:
    static constexpr uint32_t RECORD_MAGIC = 0x414E4954; // "ANIT"
    static constexpr uint16_t RECORD_VERSION = 2;        // v2: per-wheel gain grids

    struct Record {
        uint32_t magic;
        uint16_t version;
        uint16_t reserved;
        MotorTune left;
        MotorTune right;
        uint32_t checksum;
    };

    static uint32_t checksum(const Record& rec)
    {
        const uint8_t* p = (const uint8_t*)&rec;
        uint32_t sum = 0xA5A5A5A5;
        for (size_t i = 0; i < offsetof(Record, checksum); i++) sum = (sum * 31) + p[i];
        return sum;
    }
};
