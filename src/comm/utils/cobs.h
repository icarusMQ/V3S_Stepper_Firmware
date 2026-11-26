/**
 * @file    cobs.h
 * @brief   Consistent Overhead Byte Stuffing (COBS) encode/decode.
 *
 * Used to frame protocol packets on byte streams (USB serial). An encoded
 * frame never contains 0x00, so 0x00 is used as the frame delimiter and
 * plain debug text (which never contains 0x00) can share the same stream.
 */
#pragma once
#include <stddef.h>
#include <stdint.h>

/** Worst-case encoded size for `len` input bytes (without delimiters). */
static inline size_t cobs_max_encoded(size_t len) { return len + (len / 254) + 1; }

/** Encode `len` bytes into `out` (size >= cobs_max_encoded(len)). Returns encoded length. */
static inline size_t cobs_encode(const uint8_t *in, size_t len, uint8_t *out) {
    size_t read = 0, write = 1, code_idx = 0;
    uint8_t code = 1;
    while (read < len) {
        if (in[read] == 0) {
            out[code_idx] = code;
            code = 1;
            code_idx = write++;
            read++;
        } else {
            out[write++] = in[read++];
            if (++code == 0xFF) {
                out[code_idx] = code;
                code = 1;
                code_idx = write++;
            }
        }
    }
    out[code_idx] = code;
    return write;
}

/** Decode `len` bytes (no delimiters) into `out` (size >= len). Returns decoded length, 0 on error. */
static inline size_t cobs_decode(const uint8_t *in, size_t len, uint8_t *out) {
    size_t read = 0, write = 0;
    while (read < len) {
        uint8_t code = in[read];
        if (code == 0 || read + code > len) return 0;
        read++;
        for (uint8_t i = 1; i < code; i++) {
            if (read >= len) return 0;
            out[write++] = in[read++];
        }
        if (code != 0xFF && read != len) out[write++] = 0;
    }
    return write;
}
