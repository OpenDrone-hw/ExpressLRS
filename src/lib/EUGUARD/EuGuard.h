#pragma once
/*
 * EU compliance guard for the receiver's telemetry transmissions.
 *
 * The receiver only transmits in its telemetry slots. This guard decides per
 * slot whether the packet may go on air; a refused slot is skipped exactly
 * like a listen-before-talk refusal (the radio stays in receive), so the
 * uplink and the hopping are untouched and a stock transmitter still links.
 *
 * 2.4 GHz, EN 300 328 V2.2.2 clause 4.3.1.4.3.1, non-adaptive FHSS:
 *   accumulated transmit time on one hopping frequency at most 15 ms within
 *   any 225 ms (N = 15). Budget used here: 12 ms.
 *
 * 868 MHz, EN 300 220-2 V3.3.1 (Annex B bands):
 *   - no transmission on a channel whose occupied band (500 kHz) leaves
 *     863.0 to 868.6 MHz, which keeps clear of 868.6-868.7, 869.2-869.4 and
 *     869.65-869.7 MHz and of every band edge above 865 MHz;
 *   - duty cycle 1 % per hour in 865.0 to 868.6 MHz, 0.1 % per hour in 863.0
 *     to 865.0 MHz (a channel straddling 865.0 MHz counts as 0.1 %).
 *   Duty cycle is enforced by one token bucket per band. Over any window of T
 *   seconds the airtime is at most capacity + rate * T, so with the values
 *   below one hour holds at most 0.36 s + 0.9 % * 3600 s = 32.76 s (limit
 *   36 s) and 3.276 s (limit 3.6 s).
 *
 * Every limit is one named constant below. The guard is compiled into every
 * EU build (Regulatory_Domain_EU_CE_2400); EU_TLM_GUARD_DISABLE removes it.
 */

#include <stdint.h>
#include <stddef.h>

#if defined(Regulatory_Domain_EU_CE_2400) && !defined(EU_TLM_GUARD_DISABLE)
#define EU_TLM_GUARD 1
#endif

// ---- EN 300 328, 2.4 GHz dwell time ----
constexpr uint32_t EU2G4_DWELL_WINDOW_US = 225000;  // observation period
constexpr uint32_t EU2G4_DWELL_LIMIT_US = 15000;    // the standard's limit
constexpr uint32_t EU2G4_DWELL_BUDGET_US = 12000;   // what the guard allows
constexpr size_t EU2G4_HISTORY_LEN = 128;           // > max telemetry packets in 225 ms (500/s)

// ---- EN 300 220-2, 868 MHz ----
constexpr uint32_t EU868_OCCUPIED_BW_HZ = 500000;   // LoRa BW500; GFSK 300 kbit/s is narrower
constexpr uint32_t EU868_ALLOWED_LOW_HZ = 863000000;
constexpr uint32_t EU868_ALLOWED_HIGH_HZ = 868600000;
constexpr uint32_t EU868_DUTY_0P1_BELOW_HZ = 865000000; // any part below: 0.1 % band
constexpr uint32_t EU868_RULES_LOW_HZ = 862000000;      // the 868 rules apply to
constexpr uint32_t EU868_RULES_HIGH_HZ = 876000000;     // frequencies in this range
constexpr uint32_t EU868_DUTY_1PCT_LIMIT_PPM = 10000;   // the standard's limits
constexpr uint32_t EU868_DUTY_0P1PCT_LIMIT_PPM = 1000;
constexpr uint32_t EU868_DUTY_1PCT_RATE_PPM = 9000;     // token refill: 0.9 %
constexpr uint32_t EU868_DUTY_0P1PCT_RATE_PPM = 900;    // 0.09 %
constexpr uint32_t EU868_DUTY_1PCT_CAPACITY_US = 360000; // burst: 1 % of the hour limit
constexpr uint32_t EU868_DUTY_0P1PCT_CAPACITY_US = 36000;
constexpr uint64_t EU868_DUTY_PERIOD_US = 3600ULL * 1000000ULL;

enum EuBand868 : uint8_t
{
    EU868_EXCLUDED = 0,
    EU868_DUTY_0P1PCT = 1,
    EU868_DUTY_1PCT = 2,
};

enum EuGuardResult : uint8_t
{
    EU_GUARD_OK = 0,
    EU_GUARD_DWELL,        // 2.4 GHz: 225 ms budget on this channel used up
    EU_GUARD_HISTORY_FULL, // 2.4 GHz: could not keep the history, refuse
    EU_GUARD_EXCLUDED,     // 868: channel not allowed for transmit
    EU_GUARD_DUTY,         // 868: duty cycle budget used up
};

struct EuGuardTxRecord
{
    uint64_t startUs;
    uint32_t freqKey;
    uint32_t airtimeUs;
};

struct EuGuardBucket
{
    uint64_t tokens; // airtime credit in us * 1e6 (ppm units)
};

struct EuGuard
{
    bool initialised;
    uint64_t lastRefillUs;
    EuGuardBucket duty[3]; // indexed by EuBand868
    EuGuardTxRecord history[EU2G4_HISTORY_LEN];
    size_t head;
    // statistics
    uint32_t allowed;
    uint32_t refused[5];
};

EuBand868 euGuardBand868(uint32_t freqHz);

void euGuardReset(EuGuard *g);

/**
 * Decide on one telemetry packet and record it when allowed.
 * @param nowUs     monotonic time, 64-bit, at the start of the transmission
 * @param subGHz    true for the 868 band (freqHz must then be the frequency in Hz)
 * @param freqKey   unique per hopping channel (2.4 GHz: the radio frequency register)
 * @param freqHz    centre frequency in Hz (only used when subGHz)
 * @param airtimeUs time on air of the packet
 */
EuGuardResult euGuardTx(EuGuard *g, uint64_t nowUs, bool subGHz, uint32_t freqKey, uint32_t freqHz, uint32_t airtimeUs);

// Airtime on channel freqKey within the 225 ms ending at nowUs (for tests)
uint32_t euGuardDwellUs(const EuGuard *g, uint64_t nowUs, uint32_t freqKey);
