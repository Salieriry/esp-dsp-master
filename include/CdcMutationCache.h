#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

class CdcMutationCache {
public:
    bool matches(uint16_t sequence, uint8_t type,
                 const uint8_t* payload, size_t payload_length) const {
        return valid_ && sequence == sequence_ && type == type_ &&
               payload_length == payload_length_ &&
               (payload_length == 0 ||
                memcmp(payload, payload_, payload_length) == 0);
    }

    void remember(uint16_t sequence, uint8_t type, const uint8_t* payload,
                  size_t payload_length, uint8_t status) {
        if (payload_length > sizeof(payload_)) {
            reset();
            return;
        }
        valid_ = true;
        sequence_ = sequence;
        type_ = type;
        payload_length_ = (uint8_t)payload_length;
        if (payload_length) memcpy(payload_, payload, payload_length);
        status_ = status;
    }

    uint8_t status() const { return status_; }
    void reset() { valid_ = false; }

private:
    bool valid_ = false;
    uint16_t sequence_ = 0;
    uint8_t type_ = 0;
    uint8_t payload_length_ = 0;
    uint8_t payload_[255]{};
    uint8_t status_ = 0;
};
