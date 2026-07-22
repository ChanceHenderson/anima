#pragma once
#include <Arduino.h>
#include <LittleFS.h>
#include <stddef.h>
#include <stdint.h>

// Solenoid push-duration settings, persisted to LittleFS in their own file
// so the profile files never change shape. The solenoid gets a longer push
// as the battery sags; these two values are the endpoints of that mapping
// (full charge = 17.0 V, mostly dead = 14.6 V).
// load()/save() touch flash, so only call them from core 0 - same rule as
// the profile and tuning saves handled in loop().

#define SOLENOID_FILE_PATH "/solenoid.cfg"
#define DEFAULT_NOID_MS_FULL 16   // push duration at full charge
#define DEFAULT_NOID_MS_DEAD 26   // push duration at low battery
#define NOID_MS_MIN 8
#define NOID_MS_MAX 40

class SolenoidSettings {
public:
    uint8_t ms_full = DEFAULT_NOID_MS_FULL;
    uint8_t ms_dead = DEFAULT_NOID_MS_DEAD;

    void set_defaults()
    {
        ms_full = DEFAULT_NOID_MS_FULL;
        ms_dead = DEFAULT_NOID_MS_DEAD;
    }

    bool is_sane() const
    {
        return ms_full >= NOID_MS_MIN && ms_full <= NOID_MS_MAX &&
               ms_dead >= NOID_MS_MIN && ms_dead <= NOID_MS_MAX;
    }

    // Returns true if valid settings were loaded from flash.
    // Falls back to the defaults (and returns false) on any failure,
    // deleting the file if it exists but is corrupt.
    bool load()
    {
        set_defaults();
        if (!LittleFS.begin()) return false;
        bool loaded = false;
        if (LittleFS.exists(SOLENOID_FILE_PATH))
        {
            File f = LittleFS.open(SOLENOID_FILE_PATH, "r");
            if (f)
            {
                Record rec;
                if (f.size() == sizeof(Record) &&
                    f.read((uint8_t*)&rec, sizeof(Record)) == sizeof(Record) &&
                    rec.magic == RECORD_MAGIC &&
                    rec.version == RECORD_VERSION &&
                    rec.checksum == checksum(rec))
                {
                    ms_full = rec.ms_full;
                    ms_dead = rec.ms_dead;
                    loaded = is_sane();
                }
                f.close();
            }
            if (!loaded)
            {
                LittleFS.remove(SOLENOID_FILE_PATH); // corrupt - delete it
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
        rec.ms_full = ms_full;
        rec.ms_dead = ms_dead;
        rec.checksum = checksum(rec);
        if (!LittleFS.begin()) return false;
        bool ok = false;
        File f = LittleFS.open(SOLENOID_FILE_PATH, "w");
        if (f)
        {
            ok = (f.write((const uint8_t*)&rec, sizeof(Record)) == sizeof(Record));
            f.close();
        }
        LittleFS.end();
        return ok;
    }

private:
    static constexpr uint32_t RECORD_MAGIC = 0x414E4953; // "ANIS"
    static constexpr uint16_t RECORD_VERSION = 1;

    struct Record {
        uint32_t magic;
        uint16_t version;
        uint8_t ms_full;
        uint8_t ms_dead;
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
