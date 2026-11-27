#include "EuGuard.h"
#include <string.h>

#if !defined(ICACHE_RAM_ATTR)
#if defined(UNIT_TEST)
#define ICACHE_RAM_ATTR
#else
#include "targets.h"
#endif
#endif

EuBand868 euGuardBand868(uint32_t freqHz)
{
    const uint32_t lo = freqHz - EU868_OCCUPIED_BW_HZ / 2;
    const uint32_t hi = freqHz + EU868_OCCUPIED_BW_HZ / 2;
    if (lo < EU868_ALLOWED_LOW_HZ || hi > EU868_ALLOWED_HIGH_HZ)
        return EU868_EXCLUDED;
    if (lo < EU868_DUTY_0P1_BELOW_HZ)
        return EU868_DUTY_0P1PCT;
    return EU868_DUTY_1PCT;
}

static uint64_t capacityOf(EuBand868 band)
{
    return (uint64_t)(band == EU868_DUTY_1PCT ? EU868_DUTY_1PCT_CAPACITY_US : EU868_DUTY_0P1PCT_CAPACITY_US) * 1000000ULL;
}

static uint64_t rateOf(EuBand868 band)
{
    return band == EU868_DUTY_1PCT ? EU868_DUTY_1PCT_RATE_PPM : EU868_DUTY_0P1PCT_RATE_PPM;
}

void euGuardReset(EuGuard *g)
{
    memset(g, 0, sizeof(*g));
}

static void ICACHE_RAM_ATTR refill(EuGuard *g, uint64_t nowUs)
{
    if (!g->initialised)
    {
        g->initialised = true;
        g->lastRefillUs = nowUs;
        // Empty at boot: a reset never adds budget
        g->duty[EU868_DUTY_0P1PCT].tokens = 0;
        g->duty[EU868_DUTY_1PCT].tokens = 0;
        return;
    }
    if (nowUs <= g->lastRefillUs)
        return;
    const uint64_t dt = nowUs - g->lastRefillUs;
    g->lastRefillUs = nowUs;
    for (int b = EU868_DUTY_0P1PCT; b <= EU868_DUTY_1PCT; b++)
    {
        const uint64_t cap = capacityOf((EuBand868)b);
        // dt is at most a few hours in practice; clamp to keep the product in range
        const uint64_t add = (dt > EU868_DUTY_PERIOD_US ? EU868_DUTY_PERIOD_US : dt) * rateOf((EuBand868)b);
        g->duty[b].tokens = (g->duty[b].tokens + add > cap) ? cap : g->duty[b].tokens + add;
    }
}

uint32_t ICACHE_RAM_ATTR euGuardDwellUs(const EuGuard *g, uint64_t nowUs, uint32_t freqKey)
{
    uint32_t sum = 0;
    for (size_t i = 0; i < EU2G4_HISTORY_LEN; i++)
    {
        const EuGuardTxRecord &r = g->history[i];
        if (r.airtimeUs == 0 || r.freqKey != freqKey)
            continue;
        // count every transmission that overlaps the window, in full
        if (r.startUs + r.airtimeUs + EU2G4_DWELL_WINDOW_US > nowUs)
            sum += r.airtimeUs;
    }
    return sum;
}

EuGuardResult ICACHE_RAM_ATTR euGuardTx(EuGuard *g, uint64_t nowUs, bool subGHz, uint32_t freqKey, uint32_t freqHz, uint32_t airtimeUs)
{
    EuGuardResult res = EU_GUARD_OK;
    refill(g, nowUs);

    if (subGHz && freqHz >= EU868_RULES_LOW_HZ && freqHz <= EU868_RULES_HIGH_HZ)
    {
        const EuBand868 band = euGuardBand868(freqHz);
        const uint64_t need = (uint64_t)airtimeUs * 1000000ULL;
        if (band == EU868_EXCLUDED)
            res = EU_GUARD_EXCLUDED;
        else if (g->duty[band].tokens < need)
            res = EU_GUARD_DUTY;
        else
            g->duty[band].tokens -= need;
    }
    else if (!subGHz)
    {
        // The window that matters ends when this packet ends
        const uint64_t endUs = nowUs + airtimeUs;
        if (euGuardDwellUs(g, endUs, freqKey) + airtimeUs > EU2G4_DWELL_BUDGET_US)
        {
            res = EU_GUARD_DWELL;
        }
        else
        {
            // Never overwrite a record that is still inside a window
            EuGuardTxRecord &slot = g->history[g->head];
            if (slot.airtimeUs != 0 && slot.startUs + slot.airtimeUs + EU2G4_DWELL_WINDOW_US > nowUs)
            {
                res = EU_GUARD_HISTORY_FULL;
            }
            else
            {
                slot.startUs = nowUs;
                slot.freqKey = freqKey;
                slot.airtimeUs = airtimeUs;
                g->head = (g->head + 1) % EU2G4_HISTORY_LEN;
            }
        }
    }

    if (res == EU_GUARD_OK)
        g->allowed++;
    else
        g->refused[res]++;
    return res;
}
