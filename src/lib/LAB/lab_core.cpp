#include "lab_core.h"
#include "common.h"
#include "EuGuard.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define MASK_SX1281 (1U << LAB_MODEL_SX1281)
#define MASK_LR1121 (1U << LAB_MODEL_LR1121)

static const LabRate rates[] = {
    {"lora50",      RATE_LORA_2G4_50HZ,      MASK_SX1281 | MASK_LR1121, true},
    {"lora500",     RATE_LORA_2G4_500HZ,     MASK_SX1281 | MASK_LR1121, true},
    {"flrc500",     RATE_FLRC_2G4_500HZ,     MASK_SX1281,               true},
    {"flrc1000",    RATE_FLRC_2G4_1000HZ,    MASK_SX1281,               true},
    {"gfsk1000",    RATE_FSK_2G4_1000HZ,     MASK_LR1121,               true},
    {"868lora25",   RATE_LORA_900_25HZ,      MASK_LR1121,               true},
    {"868lora250",  RATE_LORA_900_250HZ,     MASK_LR1121,               true},
    {"868gfsk1000", RATE_FSK_900_1000HZ_8CH, MASK_LR1121,               true},
    // ExpressLRS runs the dual band modes only on receivers with two LR1121
    // (isSupportedRFRate()). OpenRX Mono has one.
    {"dual150",     RATE_LORA_DUAL_150HZ,    MASK_LR1121,               false},
};

#define ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))

const LabRate *labRateTable(size_t *count)
{
    *count = ARRAY_LEN(rates);
    return rates;
}

const LabRate *labRateByName(const char *name)
{
    for (size_t i = 0; i < ARRAY_LEN(rates); i++)
        if (strcmp(rates[i].name, name) == 0)
            return &rates[i];
    return nullptr;
}

const LabRate *labRateByEnum(uint8_t enumRate)
{
    for (size_t i = 0; i < ARRAY_LEN(rates); i++)
        if (rates[i].enumRate == enumRate)
            return &rates[i];
    return nullptr;
}

uint8_t labChannelCount(LabBand band)
{
    return band == LAB_BAND_2G4 ? 80 : 13;
}

uint32_t labChannelHz(LabBand band, uint8_t channel)
{
    // Same plan as FHSS.cpp: EU_CE_2400 2400.4 to 2479.4 MHz in 80 steps,
    // EU868 863.275 to 869.575 MHz in 13 steps
    if (band == LAB_BAND_2G4)
        return 2400400000UL + (uint32_t)channel * 1000000UL;
    return 863275000UL + (uint32_t)channel * 525000UL;
}

const char *labBandName(LabBand band)
{
    return band == LAB_BAND_2G4 ? "2g4" : "868";
}

const char *labModName(LabMod mod)
{
    switch (mod)
    {
    case LAB_MOD_LORA: return "lora";
    case LAB_MOD_FLRC: return "flrc";
    default: return "gfsk";
    }
}

bool labModelHasBand(LabModel model, LabBand band)
{
    return band == LAB_BAND_2G4 || model == LAB_MODEL_LR1121;
}

bool labModelHasMod(LabModel model, LabMod mod)
{
    if (mod == LAB_MOD_LORA)
        return true;
    if (mod == LAB_MOD_FLRC)
        return model == LAB_MODEL_SX1281;
    return model == LAB_MODEL_LR1121;
}

uint8_t labModEnumRate(LabModel model, LabBand band, LabMod mod)
{
    if (band == LAB_BAND_868)
        return mod == LAB_MOD_LORA ? RATE_LORA_900_250HZ : RATE_FSK_900_1000HZ_8CH;
    if (mod == LAB_MOD_LORA)
        return RATE_LORA_2G4_500HZ;
    if (mod == LAB_MOD_FLRC)
        return RATE_FLRC_2G4_1000HZ;
    (void)model;
    return RATE_FSK_2G4_1000HZ;
}

const char *labModDescription(LabBand band, LabMod mod)
{
    if (band == LAB_BAND_868)
        return mod == LAB_MOD_LORA ? "LoRa SF5 BW500kHz CR4/8 (ExpressLRS 250Hz)"
                                   : "GFSK 300kbit/s dev 100kHz (ExpressLRS 1000Hz)";
    if (mod == LAB_MOD_LORA)
        return "LoRa SF5 BW812.5kHz CR4/6 (ExpressLRS 500Hz)";
    if (mod == LAB_MOD_FLRC)
        return "FLRC 650kbit/s CR1/2 (ExpressLRS 1000Hz)";
    return "GFSK 300kbit/s dev 100kHz (ExpressLRS 1000Hz)";
}

bool labChannelTxAllowed(LabBand band, uint8_t channel)
{
    if (band == LAB_BAND_2G4)
        return channel < labChannelCount(band);
    return euGuardBand868(labChannelHz(band, channel)) != EU868_EXCLUDED;
}

/***** parser *****/

#define MAX_TOKENS 6

static size_t tokenize(char *s, char *tok[])
{
    size_t n = 0;
    while (*s)
    {
        while (*s && isspace((unsigned char)*s))
            *s++ = '\0';
        if (!*s)
            break;
        if (n == MAX_TOKENS)
            return n + 1; // too many
        tok[n++] = s;
        while (*s && !isspace((unsigned char)*s))
        {
            *s = (char)tolower((unsigned char)*s);
            s++;
        }
    }
    return n;
}

static bool setErr(LabCmd *out, const char *msg)
{
    out->type = LAB_CMD_ERR;
    snprintf(out->err, sizeof(out->err), "%s", msg);
    return false;
}

static bool parseBand(const char *t, LabModel model, LabCmd *out)
{
    if (strcmp(t, "2g4") == 0)
        out->band = LAB_BAND_2G4;
    else if (strcmp(t, "868") == 0)
        out->band = LAB_BAND_868;
    else
        return setErr(out, model == LAB_MODEL_LR1121 ? "band must be 2g4 or 868" : "band must be 2g4");
    if (!labModelHasBand(model, out->band))
        return setErr(out, "band 868 not available on this model");
    return true;
}

static bool parseChannel(const char *t, LabCmd *out)
{
    const uint8_t count = labChannelCount(out->band);
    if (strcmp(t, "low") == 0)
        out->channel = 0;
    else if (strcmp(t, "mid") == 0)
        out->channel = count / 2;
    else if (strcmp(t, "high") == 0)
    {
        // highest channel the EU build transmits on (868: 868.000 MHz, see EuGuard)
        out->channel = count - 1;
        while (out->channel > 0 && !labChannelTxAllowed(out->band, out->channel))
            out->channel--;
    }
    else
    {
        size_t len = strlen(t);
        if (len == 0 || len > 3)
            goto bad;
        for (size_t i = 0; i < len; i++)
            if (!isdigit((unsigned char)t[i]))
                goto bad;
        unsigned v = (unsigned)atoi(t);
        if (v >= count)
            goto bad;
        out->channel = (uint8_t)v;
    }
    return true;
bad:
    snprintf(out->err, sizeof(out->err), "channel must be low, mid, high or 0 to %u", count - 1);
    out->type = LAB_CMD_ERR;
    return false;
}

static bool checkTxChannel(LabCmd *out)
{
    if (labChannelTxAllowed(out->band, out->channel))
        return true;
    const uint32_t hz = labChannelHz(out->band, out->channel);
    snprintf(out->err, sizeof(out->err), "channel %u (%lu.%03lu MHz) is not used for transmit in the EU build",
        out->channel, (unsigned long)(hz / 1000000UL), (unsigned long)((hz / 1000UL) % 1000UL));
    out->type = LAB_CMD_ERR;
    return false;
}

static bool parseMod(const char *t, LabModel model, LabCmd *out)
{
    if (strcmp(t, "lora") == 0)
        out->mod = LAB_MOD_LORA;
    else if (strcmp(t, "flrc") == 0)
        out->mod = LAB_MOD_FLRC;
    else if (strcmp(t, "gfsk") == 0)
        out->mod = LAB_MOD_GFSK;
    else
        goto bad;
    if (labModelHasMod(model, out->mod))
        return true;
bad:
    return setErr(out, model == LAB_MODEL_LR1121 ? "modulation must be lora or gfsk" : "modulation must be lora or flrc");
}

void labParse(const char *line, LabModel model, LabCmd *out)
{
    memset(out, 0, sizeof(*out));
    char buf[LAB_LINE_MAX + 1];
    if (strlen(line) > LAB_LINE_MAX)
    {
        setErr(out, "line too long");
        return;
    }
    strcpy(buf, line);
    char *tok[MAX_TOKENS];
    size_t n = tokenize(buf, tok);
    if (n == 0)
    {
        out->type = LAB_CMD_NONE;
        return;
    }
    if (n > MAX_TOKENS)
    {
        setErr(out, "too many arguments");
        return;
    }

    const char *c = tok[0];
    if (strcmp(c, "info") == 0 || strcmp(c, "stop") == 0)
    {
        if (n != 1)
        {
            snprintf(out->err, sizeof(out->err), "usage: %s", c);
            out->type = LAB_CMD_ERR;
            return;
        }
        out->type = c[0] == 'i' ? LAB_CMD_INFO : LAB_CMD_STOP;
        return;
    }
    if (strcmp(c, "cw") == 0)
    {
        if (n != 3)
        {
            setErr(out, "usage: cw <band> <ch>");
            return;
        }
        if (parseBand(tok[1], model, out) && parseChannel(tok[2], out) && checkTxChannel(out))
            out->type = LAB_CMD_CW;
        return;
    }
    if (strcmp(c, "tx") == 0 || strcmp(c, "rx") == 0 || strcmp(c, "ref") == 0)
    {
        if (n != 4)
        {
            snprintf(out->err, sizeof(out->err), "usage: %s <band> <ch> <mod>", c);
            out->type = LAB_CMD_ERR;
            return;
        }
        if (parseBand(tok[1], model, out) && parseChannel(tok[2], out) && parseMod(tok[3], model, out))
        {
            if (out->band == LAB_BAND_868 && out->mod == LAB_MOD_FLRC)
            {
                setErr(out, "modulation must be lora or gfsk");
                return;
            }
            out->type = c[0] == 't' ? LAB_CMD_TX : (c[1] == 'x' ? LAB_CMD_RX : LAB_CMD_REF);
            if (out->type != LAB_CMD_RX && !checkTxChannel(out))
                return;
        }
        return;
    }
    if (strcmp(c, "link") == 0)
    {
        if (n == 2 && strcmp(tok[1], "rx") == 0)
        {
            out->type = LAB_CMD_LINK_RX;
            return;
        }
        if (n != 4 || strcmp(tok[1], "tx") != 0)
        {
            setErr(out, "usage: link tx <rate> <tlm> or link rx");
            return;
        }
        const LabRate *r = labRateByName(tok[2]);
        if (r == nullptr || !(r->modelMask & (1U << model)))
        {
            char list[80] = "";
            for (size_t i = 0; i < ARRAY_LEN(rates); i++)
            {
                if (!(rates[i].modelMask & (1U << model)) || !rates[i].singleRadioOk)
                    continue;
                if (list[0])
                    strcat(list, ", ");
                strcat(list, rates[i].name);
            }
            snprintf(out->err, sizeof(out->err), "rate must be one of %s", list);
            out->type = LAB_CMD_ERR;
            return;
        }
        if (!r->singleRadioOk)
        {
            snprintf(out->err, sizeof(out->err), "rate %s needs two radios, this model has one", r->name);
            out->type = LAB_CMD_ERR;
            return;
        }
        if (strcmp(tok[3], "2") != 0 && strcmp(tok[3], "16") != 0 && strcmp(tok[3], "128") != 0)
        {
            setErr(out, "tlm must be 2, 16 or 128");
            return;
        }
        out->rate = r;
        out->tlm = (uint8_t)atoi(tok[3]);
        out->type = LAB_CMD_LINK_TX;
        return;
    }
    snprintf(out->err, sizeof(out->err), "unknown command %s (info, cw, tx, rx, ref, link, stop)", c);
    out->type = LAB_CMD_ERR;
}

/***** packets *****/

#define LAB_PKT_MAGIC 0xA7

uint16_t labCrc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

uint8_t labModeId(LabBand band, LabMod mod)
{
    return (uint8_t)(0x50 | (band << 2) | mod);
}

void labPacketBuild(uint8_t *buf, size_t len, uint8_t modeId, uint32_t seq)
{
    buf[0] = LAB_PKT_MAGIC;
    buf[1] = modeId;
    buf[2] = (uint8_t)seq;
    buf[3] = (uint8_t)(seq >> 8);
    buf[4] = (uint8_t)(seq >> 16);
    buf[5] = (uint8_t)(seq >> 24);
    for (size_t i = 6; i + 2 < len; i++)
        buf[i] = (uint8_t)(0x5A ^ i);
    const uint16_t crc = labCrc16(buf, len - 2);
    buf[len - 2] = (uint8_t)(crc >> 8);
    buf[len - 1] = (uint8_t)crc;
}

bool labPacketParse(const uint8_t *buf, size_t len, uint8_t modeId, uint32_t *seq)
{
    if (len < 8 || buf[0] != LAB_PKT_MAGIC || buf[1] != modeId)
        return false;
    const uint16_t crc = labCrc16(buf, len - 2);
    if (buf[len - 2] != (uint8_t)(crc >> 8) || buf[len - 1] != (uint8_t)crc)
        return false;
    *seq = (uint32_t)buf[2] | ((uint32_t)buf[3] << 8) | ((uint32_t)buf[4] << 16) | ((uint32_t)buf[5] << 24);
    return true;
}

/***** packet error rate *****/

void labPerReset(LabPerCounter *c)
{
    memset(c, 0, sizeof(*c));
}

void labPerPacket(LabPerCounter *c, uint32_t seq, int8_t rssi)
{
    // A gap larger than this is a restarted ref, not lost packets
    constexpr uint32_t MAX_GAP = 5000;
    if (c->started && seq == c->lastSeq)
        return; // duplicate
    if (c->started && seq > c->lastSeq && seq - c->lastSeq <= MAX_GAP)
        c->expected += seq - c->lastSeq;
    else
        c->expected += 1; // first packet, or the ref restarted
    c->started = true;
    c->lastSeq = seq;
    c->received += 1;
    c->rssiSum += rssi;
}

LabPerWindow labPerWindow(LabPerCounter *c, uint32_t nominal)
{
    LabPerWindow w = {};
    w.received = c->received;
    w.expected = c->expected;
    if (w.received == 0)
        w.expected = nominal;
    if (w.expected < w.received)
        w.expected = w.received;
    w.perX10 = w.expected == 0 ? 1000 : (uint16_t)((w.expected - w.received) * 1000ULL / w.expected);
    w.hasRssi = w.received != 0;
    w.rssi = w.hasRssi ? (int8_t)(c->rssiSum / (int32_t)w.received) : 0;
    // After an empty window the outage was already counted from the nominal
    // rate: count the next packet as a fresh start
    if (w.received == 0)
        c->started = false;
    c->received = 0;
    c->expected = 0;
    c->rssiSum = 0;
    return w;
}

uint8_t labTlmEnumToDenom(uint8_t tlmEnum)
{
    if (tlmEnum <= TLM_RATIO_NO_TLM || tlmEnum >= TLM_RATIO_DISARMED)
        return 0;
    return (uint8_t)(1U << (8 + TLM_RATIO_NO_TLM - tlmEnum));
}

uint8_t labTlmDenomToEnum(uint8_t denom)
{
    for (uint8_t e = TLM_RATIO_1_128; e <= TLM_RATIO_1_2; e++)
        if (labTlmEnumToDenom(e) == denom)
            return e;
    return TLM_RATIO_STD;
}
