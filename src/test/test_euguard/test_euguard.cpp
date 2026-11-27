// Native tests for the EU telemetry guard (lib/EUGUARD)
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <unity.h>
#include "common.h"
#include "FHSS.h"
#include "EuGuard.h"

static EuGuard g;

void setUp() { euGuardReset(&g); }
void tearDown() {}

void test_868_channel_classes(void)
{
    // Stock EU868 plan: 863.275 + n * 0.525 MHz
    const EuBand868 expect[13] = {
        EU868_EXCLUDED,                                          // 863.275: 25 kHz from 863.0
        EU868_DUTY_0P1PCT, EU868_DUTY_0P1PCT,                    // 863.800 864.325
        EU868_DUTY_0P1PCT,                                       // 864.850 straddles 865.0
        EU868_DUTY_1PCT, EU868_DUTY_1PCT, EU868_DUTY_1PCT,       // 865.375 865.900 866.425
        EU868_DUTY_1PCT, EU868_DUTY_1PCT,                        // 866.950 867.475
        EU868_EXCLUDED,                                          // 868.000 straddles 868.0
        EU868_EXCLUDED,                                          // 868.525 overlaps 868.6-868.7
        EU868_EXCLUDED,                                          // 869.050 overlaps 869.2-869.4
        EU868_EXCLUDED,                                          // 869.575 overlaps 869.65-869.7
    };
    for (int n = 0; n < 13; n++)
    {
        char msg[32];
        snprintf(msg, sizeof(msg), "channel %d", n);
        TEST_ASSERT_EQUAL_MESSAGE(expect[n], euGuardBand868(863275000UL + n * 525000UL), msg);
    }
    TEST_ASSERT_EQUAL(EU868_EXCLUDED, euGuardBand868(862900000UL)); // below 863.0
    TEST_ASSERT_EQUAL(EU868_EXCLUDED, euGuardBand868(863340000UL));  // 863.09: inside the edge guard
    TEST_ASSERT_EQUAL(EU868_DUTY_0P1PCT, euGuardBand868(863350000UL)); // 863.10 to 863.60
    TEST_ASSERT_EQUAL(EU868_EXCLUDED, euGuardBand868(867760000UL));  // 867.51 to 868.01
    TEST_ASSERT_EQUAL(EU868_DUTY_1PCT, euGuardBand868(867750000UL)); // 867.50 to 868.00
}

void test_868_excluded_never_sends(void)
{
    for (int i = 0; i < 1000; i++)
        TEST_ASSERT_EQUAL(EU_GUARD_EXCLUDED, euGuardTx(&g, 1000ULL * i, true, 0, 868525000UL, 100));
}

void test_868_outside_rules_not_limited(void)
{
    // A non-EU sub-GHz domain (e.g. 915 MHz) is not judged by the 868 rules
    for (int i = 0; i < 1000; i++)
        TEST_ASSERT_EQUAL(EU_GUARD_OK, euGuardTx(&g, 1000ULL * i, true, 0, 915000000UL, 900));
}

void test_868_duty_bucket(void)
{
    const uint32_t f1 = 866425000UL;  // 1 % band
    const uint32_t f01 = 863800000UL; // 0.1 % band
    // Empty at boot: nothing may be sent at once, so power cycling adds nothing
    uint64_t t = 5000000ULL;
    uint32_t sent = 0;
    while (euGuardTx(&g, t, true, 0, f1, 1000) == EU_GUARD_OK)
        sent++;
    TEST_ASSERT_EQUAL(0, sent);
    TEST_ASSERT_EQUAL(EU_GUARD_DUTY, euGuardTx(&g, t, true, 0, f01, 1000));
    // Refill: 0.9 % of 10 s = 90 ms -> 90 packets of 1 ms
    t += 10000000ULL;
    sent = 0;
    while (euGuardTx(&g, t, true, 0, f01, 1000) == EU_GUARD_OK)
        sent++;
    TEST_ASSERT_EQUAL(9, sent); // 0.09 % of 10 s
    t += 10000000ULL;
    sent = 0;
    while (euGuardTx(&g, t, true, 0, f1, 1000) == EU_GUARD_OK)
        sent++;
    TEST_ASSERT_EQUAL(180, sent); // the 1 % band was not used: 0.9 % of 20 s
    // Refill is capped at the capacity
    t += 10ULL * 3600ULL * 1000000ULL;
    sent = 0;
    while (euGuardTx(&g, t, true, 0, f1, 1000) == EU_GUARD_OK)
        sent++;
    TEST_ASSERT_EQUAL(EU868_DUTY_1PCT_CAPACITY_US / 1000, sent);
    sent = 0;
    while (euGuardTx(&g, t, true, 0, f01, 1000) == EU_GUARD_OK)
        sent++;
    TEST_ASSERT_EQUAL(EU868_DUTY_0P1PCT_CAPACITY_US / 1000, sent);
}

void test_2g4_dwell_budget(void)
{
    // 1.5 ms packets on one channel every 10 ms: 8 fit into 12 ms
    uint32_t sent = 0;
    for (int i = 0; i < 20; i++)
        if (euGuardTx(&g, 10000ULL * i, false, 42, 0, 1500) == EU_GUARD_OK)
            sent++;
    TEST_ASSERT_EQUAL(8, sent);
    TEST_ASSERT_LESS_OR_EQUAL(EU2G4_DWELL_BUDGET_US, euGuardDwellUs(&g, 200000, 42));
    // Other channels are independent
    TEST_ASSERT_EQUAL(EU_GUARD_OK, euGuardTx(&g, 200000, false, 43, 0, 1500));
    // Once the window has passed, the channel is free again
    TEST_ASSERT_EQUAL(EU_GUARD_OK, euGuardTx(&g, 70000 + 225000 + 1500, false, 42, 0, 1500));
}

void test_2g4_history_full_refuses(void)
{
    // More than EU2G4_HISTORY_LEN packets inside 225 ms on different channels
    uint32_t refused = 0;
    for (uint32_t i = 0; i < EU2G4_HISTORY_LEN + 10; i++)
        if (euGuardTx(&g, 100ULL * i, false, i, 0, 50) == EU_GUARD_HISTORY_FULL)
            refused++;
    TEST_ASSERT_EQUAL(10, refused);
}

// Brute force check: no channel exceeds the limit in any 225 ms window
struct Tx { uint64_t t; uint32_t ch; uint32_t air; };
static void assertDwellCompliant(const std::vector<Tx> &txs)
{
    for (size_t i = 0; i < txs.size(); i++)
    {
        const uint64_t end = txs[i].t + txs[i].air;
        uint32_t sum = 0;
        for (size_t j = i + 1; j-- > 0;)
        {
            if (txs[j].t + txs[j].air + EU2G4_DWELL_WINDOW_US <= end)
                break;
            if (txs[j].ch == txs[i].ch)
                sum += txs[j].air;
        }
        TEST_ASSERT_LESS_OR_EQUAL(EU2G4_DWELL_LIMIT_US, sum);
    }
}

// Run the receiver's telemetry schedule (rx_main.cpp: OtaNonce % tlmDenom == 0
// after the hop) over the real FHSS sequence and report what survives.
// interval, time on air and hop interval from the air rate tables in common.cpp
struct Rate2g4 { const char *name; uint32_t interval; uint32_t toa; uint8_t hop; };
static void simulate2g4(const Rate2g4 &r, uint8_t denom, double seconds)
{
    euGuardReset(&g);
    FHSSrandomiseFHSSsequence(0x5EED1234);
    std::vector<Tx> sent;
    uint32_t slots = 0;
    uint8_t ch = FHSSsequence[0];
    const uint32_t packets = (uint32_t)(seconds * 1e6 / r.interval);
    for (uint32_t nonce = 1; nonce < packets; nonce++)
    {
        if (nonce % r.hop == 0)
        {
            FHSSptr = (FHSSptr + 1) % FHSSgetSequenceCount();
            ch = FHSSsequence[FHSSptr];
        }
        if (nonce % denom != 0)
            continue;
        slots++;
        const uint64_t t = (uint64_t)nonce * r.interval;
        if (euGuardTx(&g, t, false, ch, 0, r.toa) == EU_GUARD_OK)
            sent.push_back({t, ch, r.toa});
    }
    assertDwellCompliant(sent);
    char msg[160];
    snprintf(msg, sizeof(msg), "2.4 GHz %s tlm 1:%u: %.1f of %.1f telemetry/s sent (%.2f %%)",
        r.name, denom, sent.size() / seconds, slots / seconds,
        slots ? 100.0 * sent.size() / slots : 100.0);
    TEST_MESSAGE(msg);
}

void test_2g4_simulation(void)
{
    const Rate2g4 rates[] = {
        {"flrc1000", 1000, 389, 2},
        {"flrc500", 2000, 389, 2},
        {"gfsk1000 (LR1121)", 1000, 690, 2},
        {"lora500", 2000, 1507, 4},
        {"lora333 8ch", 3003, 2374, 4},
        {"lora250", 4000, 3300, 4},
        {"lora150", 6666, 5871, 4},
        {"lora100 8ch", 10000, 7605, 4},
        {"lora50", 20000, 10798, 2},
    };
    const uint8_t denoms[] = {2, 4, 16, 128};
    for (const Rate2g4 &r : rates)
        for (uint8_t d : denoms)
            simulate2g4(r, d, 60.0);
}

// 868 MHz: LR1121 sub-GHz rates (common.cpp), 13 channels, random block order
struct Rate868 { const char *name; uint32_t interval; uint32_t toa; uint8_t hop; };
static void simulate868(const Rate868 &r, uint8_t denom, double seconds)
{
    euGuardReset(&g);
    srand(1234);
    uint8_t seq[13];
    for (int i = 0; i < 13; i++) seq[i] = i;
    int pos = 0;
    uint32_t slots = 0, sent = 0;
    uint64_t air[3] = {0, 0, 0};
    // rolling one-hour check with 1 s resolution
    const uint32_t secs = (uint32_t)seconds;
    std::vector<uint64_t> perSec[3];
    for (int b = 0; b < 3; b++) perSec[b].assign(secs + 1, 0);
    const uint32_t packets = (uint32_t)(seconds * 1e6 / r.interval);
    uint8_t ch = seq[0];
    for (uint32_t nonce = 1; nonce < packets; nonce++)
    {
        if (nonce % r.hop == 0)
        {
            if (++pos == 13)
            {
                pos = 0;
                for (int i = 12; i > 0; i--) { int j = rand() % (i + 1); uint8_t x = seq[i]; seq[i] = seq[j]; seq[j] = x; }
            }
            ch = seq[pos];
        }
        if (nonce % denom != 0)
            continue;
        slots++;
        const uint64_t t = (uint64_t)nonce * r.interval;
        const uint32_t f = 863275000UL + ch * 525000UL;
        if (euGuardTx(&g, t, true, 0, f, r.toa) == EU_GUARD_OK)
        {
            const EuBand868 b = euGuardBand868(f);
            TEST_ASSERT_NOT_EQUAL(EU868_EXCLUDED, b);
            sent++;
            air[b] += r.toa;
            perSec[b][t / 1000000ULL] += r.toa;
        }
    }
    // any 3600 s window (the run is one hour plus): at most the limit
    for (int b = EU868_DUTY_0P1PCT; b <= EU868_DUTY_1PCT; b++)
    {
        const uint64_t limit = 3600ULL * (b == EU868_DUTY_1PCT ? EU868_DUTY_1PCT_LIMIT_PPM : EU868_DUTY_0P1PCT_LIMIT_PPM);
        uint64_t win = 0;
        for (uint32_t s = 0; s <= secs; s++)
        {
            win += perSec[b][s];
            if (s >= 3600) win -= perSec[b][s - 3600];
            TEST_ASSERT_LESS_OR_EQUAL(limit, win);
        }
    }
    char msg[200];
    snprintf(msg, sizeof(msg), "868 MHz %s tlm 1:%u: %.2f of %.1f telemetry/s sent; duty 1%% band %.3f %%, 0.1%% band %.4f %%",
        r.name, denom, sent / seconds, slots / seconds, 100.0 * air[EU868_DUTY_1PCT] / (seconds * 1e6),
        100.0 * air[EU868_DUTY_0P1PCT] / (seconds * 1e6));
    TEST_MESSAGE(msg);
}

void test_868_simulation(void)
{
    const Rate868 rates[] = {
        {"868gfsk1000", 1000, 658, 2},
        {"868lora250", 4000, 3216, 4},
        {"868lora200", 5000, 4380, 4},
        {"868lora100", 10000, 8770, 4},
        {"868lora50", 20000, 18560, 4},
        {"868lora25", 40000, 29950, 2},
    };
    const uint8_t denoms[] = {2, 16, 128};
    for (const Rate868 &r : rates)
        for (uint8_t d : denoms)
            simulate868(r, d, 3700.0);
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_868_channel_classes);
    RUN_TEST(test_868_excluded_never_sends);
    RUN_TEST(test_868_outside_rules_not_limited);
    RUN_TEST(test_868_duty_bucket);
    RUN_TEST(test_2g4_dwell_budget);
    RUN_TEST(test_2g4_history_full_refuses);
    RUN_TEST(test_2g4_simulation);
    RUN_TEST(test_868_simulation);
    UNITY_END();
    return 0;
}
