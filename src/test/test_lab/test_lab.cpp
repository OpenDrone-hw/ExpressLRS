// Native tests for the OpenRX lab console (lib/LAB/lab_core.cpp)
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <unity.h>
#include "common.h"
#include "lab_core.h"

static LabCmd parse(const char *line, LabModel model = LAB_MODEL_SX1281)
{
    LabCmd c;
    labParse(line, model, &c);
    return c;
}

void test_channels(void)
{
    TEST_ASSERT_EQUAL_UINT32(2400400000UL, labChannelHz(LAB_BAND_2G4, 0));
    TEST_ASSERT_EQUAL_UINT32(2440400000UL, labChannelHz(LAB_BAND_2G4, 40));
    TEST_ASSERT_EQUAL_UINT32(2479400000UL, labChannelHz(LAB_BAND_2G4, 79));
    TEST_ASSERT_EQUAL_UINT32(863275000UL, labChannelHz(LAB_BAND_868, 0));
    TEST_ASSERT_EQUAL_UINT32(866425000UL, labChannelHz(LAB_BAND_868, 6));
    TEST_ASSERT_EQUAL_UINT32(869575000UL, labChannelHz(LAB_BAND_868, 12));
}

void test_parse_simple(void)
{
    TEST_ASSERT_EQUAL(LAB_CMD_NONE, parse("").type);
    TEST_ASSERT_EQUAL(LAB_CMD_NONE, parse("   ").type);
    TEST_ASSERT_EQUAL(LAB_CMD_INFO, parse("info").type);
    TEST_ASSERT_EQUAL(LAB_CMD_INFO, parse("  INFO ").type);
    TEST_ASSERT_EQUAL(LAB_CMD_STOP, parse("stop").type);
    TEST_ASSERT_EQUAL(LAB_CMD_ERR, parse("stop now").type);
    TEST_ASSERT_EQUAL(LAB_CMD_ERR, parse("hello").type);
    TEST_ASSERT_EQUAL(LAB_CMD_LINK_RX, parse("link rx").type);
}

void test_parse_cw(void)
{
    LabCmd c = parse("cw 2g4 mid");
    TEST_ASSERT_EQUAL(LAB_CMD_CW, c.type);
    TEST_ASSERT_EQUAL(LAB_BAND_2G4, c.band);
    TEST_ASSERT_EQUAL(40, c.channel);

    c = parse("cw 2g4 low");
    TEST_ASSERT_EQUAL(0, c.channel);
    c = parse("cw 2g4 high");
    TEST_ASSERT_EQUAL(79, c.channel);
    c = parse("cw 2g4 17");
    TEST_ASSERT_EQUAL(17, c.channel);
    TEST_ASSERT_EQUAL(LAB_CMD_ERR, parse("cw 2g4 80").type);
    TEST_ASSERT_EQUAL(LAB_CMD_ERR, parse("cw 2g4 -1").type);
    TEST_ASSERT_EQUAL(LAB_CMD_ERR, parse("cw 2g4 4a").type);
    TEST_ASSERT_EQUAL(LAB_CMD_ERR, parse("cw 2g4").type);

    // 868 only on the LR1121 model
    c = parse("cw 868 mid");
    TEST_ASSERT_EQUAL(LAB_CMD_ERR, c.type);
    TEST_ASSERT_NOT_NULL(strstr(c.err, "not available"));
    c = parse("cw 868 mid", LAB_MODEL_LR1121);
    TEST_ASSERT_EQUAL(LAB_CMD_CW, c.type);
    TEST_ASSERT_EQUAL(LAB_BAND_868, c.band);
    TEST_ASSERT_EQUAL(6, c.channel);
    TEST_ASSERT_EQUAL(LAB_CMD_ERR, parse("cw 868 13", LAB_MODEL_LR1121).type);
    // 868 high is the highest channel the EU build transmits on: 868.000 MHz
    c = parse("cw 868 high", LAB_MODEL_LR1121);
    TEST_ASSERT_EQUAL(LAB_CMD_CW, c.type);
    TEST_ASSERT_EQUAL(9, c.channel);
    TEST_ASSERT_EQUAL_UINT32(868000000UL, labChannelHz(LAB_BAND_868, c.channel));
    // 868.525, 869.050, 869.575 MHz: no transmit test modes, receive is fine
    for (int ch = 10; ch <= 12; ch++)
    {
        char line[32];
        snprintf(line, sizeof(line), "tx 868 %d lora", ch);
        TEST_ASSERT_EQUAL(LAB_CMD_ERR, parse(line, LAB_MODEL_LR1121).type);
        snprintf(line, sizeof(line), "cw 868 %d", ch);
        TEST_ASSERT_EQUAL(LAB_CMD_ERR, parse(line, LAB_MODEL_LR1121).type);
        snprintf(line, sizeof(line), "ref 868 %d gfsk", ch);
        TEST_ASSERT_EQUAL(LAB_CMD_ERR, parse(line, LAB_MODEL_LR1121).type);
        snprintf(line, sizeof(line), "rx 868 %d lora", ch);
        TEST_ASSERT_EQUAL(LAB_CMD_RX, parse(line, LAB_MODEL_LR1121).type);
    }
}

void test_parse_modulated(void)
{
    LabCmd c = parse("tx 2g4 low flrc");
    TEST_ASSERT_EQUAL(LAB_CMD_TX, c.type);
    TEST_ASSERT_EQUAL(LAB_MOD_FLRC, c.mod);
    c = parse("rx 2g4 mid lora");
    TEST_ASSERT_EQUAL(LAB_CMD_RX, c.type);
    TEST_ASSERT_EQUAL(LAB_MOD_LORA, c.mod);
    c = parse("ref 2g4 high lora");
    TEST_ASSERT_EQUAL(LAB_CMD_REF, c.type);
    TEST_ASSERT_EQUAL(79, c.channel);

    TEST_ASSERT_EQUAL(LAB_CMD_ERR, parse("tx 2g4 low gfsk").type);
    TEST_ASSERT_EQUAL(LAB_CMD_ERR, parse("tx 2g4 low flrc", LAB_MODEL_LR1121).type);
    c = parse("tx 868 low gfsk", LAB_MODEL_LR1121);
    TEST_ASSERT_EQUAL(LAB_CMD_TX, c.type);
    TEST_ASSERT_EQUAL(LAB_MOD_GFSK, c.mod);
    TEST_ASSERT_EQUAL(LAB_CMD_ERR, parse("tx 2g4 low").type);
    TEST_ASSERT_EQUAL(LAB_CMD_ERR, parse("tx 2g4 low lora extra").type);
}

void test_parse_link(void)
{
    LabCmd c = parse("link tx lora500 2");
    TEST_ASSERT_EQUAL(LAB_CMD_LINK_TX, c.type);
    TEST_ASSERT_EQUAL_STRING("lora500", c.rate->name);
    TEST_ASSERT_EQUAL(RATE_LORA_2G4_500HZ, c.rate->enumRate);
    TEST_ASSERT_EQUAL(2, c.tlm);

    TEST_ASSERT_EQUAL(LAB_CMD_LINK_TX, parse("link tx flrc1000 128").type);
    TEST_ASSERT_EQUAL(LAB_CMD_ERR, parse("link tx flrc1000 4").type);
    TEST_ASSERT_EQUAL(LAB_CMD_ERR, parse("link tx gfsk1000 2").type);
    TEST_ASSERT_EQUAL(LAB_CMD_ERR, parse("link tx lora500").type);
    TEST_ASSERT_EQUAL(LAB_CMD_ERR, parse("link").type);

    c = parse("link tx 868lora25 16", LAB_MODEL_LR1121);
    TEST_ASSERT_EQUAL(LAB_CMD_LINK_TX, c.type);
    TEST_ASSERT_EQUAL(RATE_LORA_900_25HZ, c.rate->enumRate);
    TEST_ASSERT_EQUAL(16, c.tlm);
    TEST_ASSERT_EQUAL(LAB_CMD_LINK_TX, parse("link tx gfsk1000 2", LAB_MODEL_LR1121).type);
    TEST_ASSERT_EQUAL(LAB_CMD_LINK_TX, parse("link tx 868gfsk1000 128", LAB_MODEL_LR1121).type);
    TEST_ASSERT_EQUAL(LAB_CMD_ERR, parse("link tx flrc500 2", LAB_MODEL_LR1121).type);

    // Dual band needs two LR1121; Mono has one
    c = parse("link tx dual150 2", LAB_MODEL_LR1121);
    TEST_ASSERT_EQUAL(LAB_CMD_ERR, c.type);
    TEST_ASSERT_NOT_NULL(strstr(c.err, "two radios"));
}

void test_rate_lookup(void)
{
    size_t n;
    const LabRate *t = labRateTable(&n);
    TEST_ASSERT_EQUAL(9, n);
    for (size_t i = 0; i < n; i++)
    {
        TEST_ASSERT_EQUAL_PTR(&t[i], labRateByName(t[i].name));
        TEST_ASSERT_EQUAL_PTR(&t[i], labRateByEnum(t[i].enumRate));
    }
    TEST_ASSERT_NULL(labRateByName("lora1000"));
    // modem settings used by the test modes
    TEST_ASSERT_EQUAL(RATE_FLRC_2G4_1000HZ, labModEnumRate(LAB_MODEL_SX1281, LAB_BAND_2G4, LAB_MOD_FLRC));
    TEST_ASSERT_EQUAL(RATE_LORA_2G4_500HZ, labModEnumRate(LAB_MODEL_LR1121, LAB_BAND_2G4, LAB_MOD_LORA));
    TEST_ASSERT_EQUAL(RATE_FSK_2G4_1000HZ, labModEnumRate(LAB_MODEL_LR1121, LAB_BAND_2G4, LAB_MOD_GFSK));
    TEST_ASSERT_EQUAL(RATE_LORA_900_250HZ, labModEnumRate(LAB_MODEL_LR1121, LAB_BAND_868, LAB_MOD_LORA));
    TEST_ASSERT_EQUAL(RATE_FSK_900_1000HZ_8CH, labModEnumRate(LAB_MODEL_LR1121, LAB_BAND_868, LAB_MOD_GFSK));
}

void test_packet_roundtrip(void)
{
    uint8_t buf[13];
    uint32_t seq = 0;
    const uint8_t id = labModeId(LAB_BAND_2G4, LAB_MOD_LORA);
    for (size_t len = 8; len <= sizeof(buf); len += 5)
    {
        labPacketBuild(buf, len, id, 0x12345678);
        TEST_ASSERT_TRUE(labPacketParse(buf, len, id, &seq));
        TEST_ASSERT_EQUAL_HEX32(0x12345678, seq);
        // wrong mode
        TEST_ASSERT_FALSE(labPacketParse(buf, len, labModeId(LAB_BAND_2G4, LAB_MOD_FLRC), &seq));
        // any bit flip is caught
        for (size_t byte = 0; byte < len; byte++)
        {
            buf[byte] ^= 0x10;
            TEST_ASSERT_FALSE(labPacketParse(buf, len, id, &seq));
            buf[byte] ^= 0x10;
        }
    }
    TEST_ASSERT_EQUAL_HEX16(0x29B1, labCrc16((const uint8_t *)"123456789", 9)); // CRC-16/CCITT-FALSE check value
}

void test_per_counter(void)
{
    LabPerCounter c;
    labPerReset(&c);

    // nothing received: nominal expected, 100 %
    LabPerWindow w = labPerWindow(&c, 1000);
    TEST_ASSERT_EQUAL(0, w.received);
    TEST_ASSERT_EQUAL(1000, w.expected);
    TEST_ASSERT_EQUAL(1000, w.perX10);
    TEST_ASSERT_FALSE(w.hasRssi);

    // 1000 packets, every tenth lost
    for (uint32_t s = 100; s < 1100; s++)
        if (s % 10)
            labPerPacket(&c, s, -50);
    w = labPerWindow(&c, 1000);
    TEST_ASSERT_EQUAL(900, w.received);
    TEST_ASSERT_EQUAL(999, w.expected); // first packet starts the count
    TEST_ASSERT_EQUAL(99, w.perX10);    // 9.9 %
    TEST_ASSERT_TRUE(w.hasRssi);
    TEST_ASSERT_EQUAL(-50, w.rssi);

    // continuous next window, duplicates ignored
    for (uint32_t s = 1100; s < 2100; s++)
    {
        labPerPacket(&c, s, -60);
        if (s == 1500)
            labPerPacket(&c, s, -60);
    }
    w = labPerWindow(&c, 1000);
    TEST_ASSERT_EQUAL(1000, w.received);
    TEST_ASSERT_EQUAL(1000, w.expected);
    TEST_ASSERT_EQUAL(0, w.perX10);

    // ref restarted at 0: counted as a fresh start, not as a huge loss
    for (uint32_t s = 0; s < 500; s++)
        labPerPacket(&c, s, -70);
    w = labPerWindow(&c, 1000);
    TEST_ASSERT_EQUAL(500, w.received);
    TEST_ASSERT_EQUAL(500, w.expected);

    // an empty window is not counted twice when packets resume
    w = labPerWindow(&c, 1000);
    TEST_ASSERT_EQUAL(1000, w.perX10);
    for (uint32_t s = 1500; s < 2500; s++)
        labPerPacket(&c, s, -70);
    w = labPerWindow(&c, 1000);
    TEST_ASSERT_EQUAL(1000, w.received);
    TEST_ASSERT_EQUAL(1000, w.expected);
}

void test_tlm(void)
{
    TEST_ASSERT_EQUAL(2, labTlmEnumToDenom(TLM_RATIO_1_2));
    TEST_ASSERT_EQUAL(128, labTlmEnumToDenom(TLM_RATIO_1_128));
    TEST_ASSERT_EQUAL(TLM_RATIO_1_16, labTlmDenomToEnum(16));
    TEST_ASSERT_EQUAL(TLM_RATIO_STD, labTlmDenomToEnum(3));
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_channels);
    RUN_TEST(test_parse_simple);
    RUN_TEST(test_parse_cw);
    RUN_TEST(test_parse_modulated);
    RUN_TEST(test_parse_link);
    RUN_TEST(test_rate_lookup);
    RUN_TEST(test_packet_roundtrip);
    RUN_TEST(test_per_counter);
    RUN_TEST(test_tlm);
    UNITY_END();
    return 0;
}

// Unity setUp/tearDown
void setUp() {}
void tearDown() {}
