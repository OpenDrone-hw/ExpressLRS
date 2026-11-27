#pragma once
/*
 * OpenRX lab test firmware: hardware independent part.
 *
 * Command parser, channel plan, test packet format and packet error rate
 * counter used by the serial test console (lab_rx.cpp, lab_tx.cpp). Nothing
 * in here touches the radio, so it is covered by the native unit tests in
 * test/test_lab.
 *
 * The command set is the interface of the NextPCB lab procedure OPENRX-LAB,
 * section 8.
 */

#include <stdint.h>
#include <stddef.h>

#define LAB_RELEASE "openrx-lab-v1"
#define LAB_BANNER "OPENRX LAB READY"
#define LAB_LINE_MAX 64

enum LabModel : uint8_t
{
    LAB_MODEL_SX1281 = 0, // OpenRX Lite U.FL: 2.4 GHz LoRa and FLRC
    LAB_MODEL_LR1121 = 1, // OpenRX Mono: 2.4 GHz and 868 MHz, LoRa and GFSK
};

enum LabBand : uint8_t
{
    LAB_BAND_2G4 = 0,
    LAB_BAND_868 = 1,
};

enum LabMod : uint8_t
{
    LAB_MOD_LORA = 0,
    LAB_MOD_FLRC = 1,
    LAB_MOD_GFSK = 2,
};

enum LabCmdType : uint8_t
{
    LAB_CMD_NONE = 0, // empty line
    LAB_CMD_ERR,      // parse error, see LabCmd::err
    LAB_CMD_INFO,
    LAB_CMD_STOP,
    LAB_CMD_CW,
    LAB_CMD_TX,
    LAB_CMD_RX,
    LAB_CMD_REF,
    LAB_CMD_LINK_TX,
    LAB_CMD_LINK_RX,
    LAB_CMD_WIFI,
};

struct LabRate
{
    const char *name;
    uint8_t enumRate;   // expresslrs_RFrates_e
    uint8_t modelMask;  // bit per LabModel
    bool singleRadioOk; // false: needs two LR1121 (dual band modes)
};

struct LabCmd
{
    LabCmdType type;
    LabBand band;
    uint8_t channel;
    LabMod mod;
    const LabRate *rate;
    uint8_t tlm;        // telemetry ratio denominator, 1:tlm
    char err[LAB_LINE_MAX + 32];
};

// Channel plan, identical to the ExpressLRS FHSS tables (EU_CE_2400, EU868)
uint8_t labChannelCount(LabBand band);
uint32_t labChannelHz(LabBand band, uint8_t channel);
const char *labBandName(LabBand band);
const char *labModName(LabMod mod);
bool labModelHasBand(LabModel model, LabBand band);
bool labModelHasMod(LabModel model, LabMod mod);

// Human readable modem settings of a test mode, for the OK line
const char *labModDescription(LabBand band, LabMod mod);
// Whether the EU build transmits on this channel at all (868: EuGuard)
bool labChannelTxAllowed(LabBand band, uint8_t channel);

// ExpressLRS air rate whose modem settings a test mode uses for <mod> on <band>
uint8_t labModEnumRate(LabModel model, LabBand band, LabMod mod);

// Link rates of section 8
const LabRate *labRateByName(const char *name);
const LabRate *labRateByEnum(uint8_t enumRate);
const LabRate *labRateTable(size_t *count);

// Parse one console line (no line terminator). Always fills out->type.
void labParse(const char *line, LabModel model, LabCmd *out);

// Test packets: [0] magic, [1] mode id, [2..5] sequence (LE), filler,
// last 2 bytes CRC16-CCITT over everything before. len 8 to 16.
uint16_t labCrc16(const uint8_t *data, size_t len);
uint8_t labModeId(LabBand band, LabMod mod);
void labPacketBuild(uint8_t *buf, size_t len, uint8_t modeId, uint32_t seq);
bool labPacketParse(const uint8_t *buf, size_t len, uint8_t modeId, uint32_t *seq);

// Packet error rate over one-second windows, from the sequence numbers
struct LabPerCounter
{
    bool started;
    uint32_t lastSeq;
    uint32_t received;
    uint32_t expected;
    int32_t rssiSum;
};

struct LabPerWindow
{
    uint32_t received;
    uint32_t expected;
    uint16_t perX10;    // PER in 0.1 % steps
    bool hasRssi;
    int8_t rssi;        // mean RSSI of the window, dBm
};

void labPerReset(LabPerCounter *c);
void labPerPacket(LabPerCounter *c, uint32_t seq, int8_t rssi);
// Close the window. nominal: packets the ref sends per window, used when
// nothing was received.
LabPerWindow labPerWindow(LabPerCounter *c, uint32_t nominal);

// "1:N" etc.
uint8_t labTlmEnumToDenom(uint8_t tlmEnum);
uint8_t labTlmDenomToEnum(uint8_t denom);
