// OpenRX lab test firmware: receiver application (app0)
#include "targets.h"

#if defined(OPENRX_LAB) && !defined(UNIT_TEST) && defined(TARGET_RX) && defined(PLATFORM_ESP32)

#include "lab_runtime.h"
#include "common.h"
#include "FHSS.h"
#include "POWERMGNT.h"
#include "CRSFRouter.h"
#include "EuGuard.h"

#include <esp_timer.h>

extern uint8_t getLq();
#if defined(EU_TLM_GUARD)
extern EuGuard euTlmGuard;
#endif

// Fixed radio identities for the test packets, so any lab board matches any
// other board of the same model (the link modes use the binding phrase)
static constexpr uint32_t LAB_FLRC_SYNC = 0x4C414231; // "LAB1"
static constexpr uint16_t LAB_FLRC_CRC = 0x4C42;
static constexpr uint8_t LAB_FSK_SYNC1 = 0x4C;
static constexpr uint8_t LAB_FSK_SYNC2 = 0x42;

static char pendingLine[LAB_LINE_MAX + 1];
static bool pendingValid;
static bool mailboxRead;

static LabCmd cur;                 // the running test mode
static const expresslrs_mod_settings_s *curMod;
static const expresslrs_rf_pref_params_s *curPerf;
static uint8_t curModeId;
static volatile uint32_t txSeq;
static volatile bool txBusy;
static volatile uint32_t lastTxDoneMs;
static esp_timer_handle_t refTimer;
static LabPerCounter per;
static portMUX_TYPE perMux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t lastReportMs;
#if defined(EU_TLM_GUARD)
static uint32_t lastGuardAllowed, lastGuardRefused;
#endif

static void readMailboxOnce()
{
    if (mailboxRead)
        return;
    mailboxRead = true;
    pendingValid = labMailboxTake(pendingLine, sizeof(pendingLine));
}

void labRxSetupSerial()
{
    labConsoleBegin();
}

bool labRxBootLink()
{
    readMailboxOnce();
    if (!pendingValid || connectionState == radioFailed)
        return false;
    LabCmd c;
    labParse(pendingLine, labThisModel(), &c);
    if (c.type != LAB_CMD_LINK_RX)
        return false;
    pendingValid = false;
    labState = LAB_STATE_LINK_RX;
    return true;
}

static bool radioOk()
{
    if (connectionState == radioFailed || connectionState == hardwareUndefined)
    {
        labPrintErr("radio not found");
        return false;
    }
    return true;
}

static void printFreq(LabBand band, uint8_t ch)
{
    const uint32_t hz = labChannelHz(band, ch);
    Serial.printf("%s ch %u %lu.%03lu MHz", labBandName(band), ch,
        (unsigned long)(hz / 1000000UL), (unsigned long)((hz / 1000UL) % 1000UL));
}

static int powerTargetDbm(LabBand band)
{
#if defined(RADIO_LR1121)
    return band == LAB_BAND_868 ? OPENRX_LAB_TARGET_868_DBM : OPENRX_LAB_TARGET_2G4_DBM;
#else
    (void)band;
    return OPENRX_LAB_TARGET_2G4_DBM;
#endif
}

/***** radio *****/

static void configRadio(const LabCmd &c)
{
    const uint8_t idx = enumRatetoIndex((expresslrs_RFrates_e)labModEnumRate(labThisModel(), c.band, c.mod));
    curMod = get_elrs_airRateConfig(idx);
    curPerf = get_elrs_RFperfParams(idx);
    curModeId = labModeId(c.band, c.mod);
    const uint32_t freq = FREQ_HZ_TO_REG_VAL(labChannelHz(c.band, c.channel));
#if defined(RADIO_SX128X)
    Radio.Config(curMod->bw, curMod->sf, curMod->cr, freq, curMod->PreambleLen, false, curMod->PayloadLength,
        LAB_FLRC_SYNC, LAB_FLRC_CRC, curMod->radio_type == RADIO_TYPE_SX128x_FLRC);
#elif defined(RADIO_LR1121)
    const bool fsk = curMod->radio_type == RADIO_TYPE_LR1121_GFSK_900 || curMod->radio_type == RADIO_TYPE_LR1121_GFSK_2G4;
    Radio.Config(curMod->bw, curMod->sf, curMod->cr, freq, curMod->PreambleLen, false, curMod->PayloadLength,
        fsk, LAB_FSK_SYNC1, LAB_FSK_SYNC2);
#endif
}

static void radioStandby()
{
    // Config() leaves the radio in standby: no transmission, no reception
    LabCmd c = {};
    c.band = LAB_BAND_2G4;
    c.channel = labChannelCount(LAB_BAND_2G4) / 2;
    c.mod = LAB_MOD_LORA;
    configRadio(c);
}

static void ICACHE_RAM_ATTR sendNext()
{
    WORD_ALIGNED_ATTR uint8_t pkt[16];
    labPacketBuild(pkt, curMod->PayloadLength, curModeId, txSeq++);
    txBusy = true;
    Radio.TXnb(pkt, false, nullptr, SX12XX_Radio_1);
}

static void ICACHE_RAM_ATTR labTxDone()
{
    txBusy = false;
    lastTxDoneMs = millis();
    if (labState == LAB_STATE_TX)
        sendNext(); // back to back
}

static bool ICACHE_RAM_ATTR labRxDone(SX12xxDriverCommon::rx_status status)
{
    if (status != SX12xxDriverCommon::SX12XX_RX_OK)
        return false;
    uint32_t seq;
    if (!labPacketParse((const uint8_t *)Radio.RXdataBuffer, curMod->PayloadLength, curModeId, &seq))
        return false;
    Radio.GetLastPacketStats();
    portENTER_CRITICAL_ISR(&perMux);
    labPerPacket(&per, seq, Radio.LastPacketRSSI);
    portEXIT_CRITICAL_ISR(&perMux);
    return true;
}

static void refTick(void *)
{
    if (labState == LAB_STATE_REF && !txBusy)
        sendNext();
}

static void stopTestMode()
{
    if (refTimer)
        esp_timer_stop(refTimer);
    const LabState was = labState;
    labState = LAB_STATE_IDLE;
    Radio.TXdoneCallback = SX12xxDriverCommon::nullCallbackTx;
    Radio.RXdoneCallback = SX12xxDriverCommon::nullCallbackRx;
    if (was != LAB_STATE_IDLE)
        radioStandby();
    txBusy = false;
}

static void startTestMode(const LabCmd &c)
{
    cur = c;
    txSeq = 0;
    if (c.type == LAB_CMD_CW)
    {
        labState = LAB_STATE_CW;
        Radio.startCWTest(labChannelHz(c.band, c.channel), SX12XX_Radio_1);
        Serial.printf("OK cw ");
        printFreq(c.band, c.channel);
        Serial.printf(" power %ddBm\r\n", powerTargetDbm(c.band));
        return;
    }

    configRadio(c);
    const uint32_t rate = 1000000UL / curMod->interval;
    Serial.printf("OK %s ", c.type == LAB_CMD_TX ? "tx" : c.type == LAB_CMD_RX ? "rx" : "ref");
    printFreq(c.band, c.channel);
    Serial.printf(" %s %s", labModName(c.mod), labModDescription(c.band, c.mod));

    if (c.type == LAB_CMD_RX)
    {
        portENTER_CRITICAL(&perMux);
        labPerReset(&per);
        portEXIT_CRITICAL(&perMux);
        Radio.RXdoneCallback = labRxDone;
        labState = LAB_STATE_RX;
        Radio.RXnb();
        lastReportMs = millis();
        Serial.printf(" expecting %lu packets/s\r\n", (unsigned long)rate);
        return;
    }

    Radio.TXdoneCallback = labTxDone;
    lastTxDoneMs = millis();
    if (c.type == LAB_CMD_TX)
    {
        labState = LAB_STATE_TX;
        Serial.printf(" back to back, power %ddBm\r\n", powerTargetDbm(c.band));
        sendNext();
        return;
    }
    // ref: numbered packets at the packet rate of the modulation
    if (!refTimer)
    {
        esp_timer_create_args_t args = {};
        args.callback = refTick;
        args.name = "labref";
        esp_timer_create(&args, &refTimer);
    }
    labState = LAB_STATE_REF;
    Serial.printf(" %lu packets/s, power %ddBm\r\n", (unsigned long)rate, powerTargetDbm(c.band));
    esp_timer_start_periodic(refTimer, curMod->interval);
}

/***** console *****/

static void execute(const char *line)
{
    LabCmd c;
    labParse(line, labThisModel(), &c);
    switch (c.type)
    {
    case LAB_CMD_NONE:
        return;
    case LAB_CMD_ERR:
        labPrintErr(c.err);
        return;
    case LAB_CMD_INFO:
        labPrintInfo();
        return;
    default:
        break;
    }

    // Leaving the receiver link: restart, the stored command runs after boot
    if (labState == LAB_STATE_LINK_RX)
    {
        labRestartWith(c.type == LAB_CMD_STOP ? "stop" : line, c.type == LAB_CMD_LINK_TX);
        return;
    }

    stopTestMode();
    switch (c.type)
    {
    case LAB_CMD_STOP:
        Serial.printf("OK stop\r\n");
        return;
    case LAB_CMD_LINK_TX:
        labRestartWith(line, true);
        return;
    case LAB_CMD_LINK_RX:
        labRestartWith(line, false);
        return;
    default:
        if (radioOk())
            startTestMode(c);
        return;
    }
}

void labRxBegin()
{
    readMailboxOnce();
    if (labState == LAB_STATE_LINK_RX)
    {
        Serial.printf("OK link rx\r\n");
        return;
    }
    const bool radioPresent = connectionState != radioFailed && connectionState != hardwareUndefined;
    if (radioPresent)
        radioStandby();
    if (pendingValid)
    {
        pendingValid = false;
        execute(pendingLine);
        return;
    }
    Serial.printf("\r\n%s\r\n", LAB_BANNER);
    if (connectionState == radioFailed)
        labPrintErr("radio not found");
    else if (connectionState == hardwareUndefined)
        labPrintErr("hardware layout missing");
}

static void reportLink(uint32_t now)
{
    if (now - lastReportMs < 1000)
        return;
    lastReportMs = now;
    if (connectionState != connected)
    {
        Serial.printf("LINK searching\r\n");
        return;
    }
    const LabRate *r = labRateByEnum(ExpressLRS_currAirRate_Modparams->enum_rate);
    Serial.printf("LINK connected LQ %u%% RSSI %ddBm rate ", getLq(), -(int)linkStats.uplink_RSSI_1);
    if (r)
        Serial.printf("%s", r->name);
    else
        Serial.printf("%luHz", (unsigned long)(1000000UL / ExpressLRS_currAirRate_Modparams->interval));
    Serial.printf(" tlm 1:%u", ExpressLRS_currTlmDenom);
#if defined(EU_TLM_GUARD)
    uint32_t refused = 0;
    for (uint32_t v : euTlmGuard.refused)
        refused += v;
    Serial.printf(" tlm sent %lu skipped %lu", (unsigned long)(euTlmGuard.allowed - lastGuardAllowed),
        (unsigned long)(refused - lastGuardRefused));
    lastGuardAllowed = euTlmGuard.allowed;
    lastGuardRefused = refused;
#endif
    Serial.printf("\r\n");
}

bool labRxLoop(uint32_t now)
{
    char line[LAB_LINE_MAX + 1];
    if (labReadLine(line, sizeof(line)))
        execute(line);

    if (labState == LAB_STATE_LINK_RX)
    {
        reportLink(now);
        return false; // run the normal receiver
    }

    if (labState == LAB_STATE_RX && now - lastReportMs >= 1000)
    {
        lastReportMs += 1000;
        if (now - lastReportMs > 1000)
            lastReportMs = now;
        portENTER_CRITICAL(&perMux);
        const LabPerWindow w = labPerWindow(&per, 1000000UL / curMod->interval);
        portEXIT_CRITICAL(&perMux);
        Serial.printf("RX received %lu expected %lu PER %u.%u%%", (unsigned long)w.received,
            (unsigned long)w.expected, w.perX10 / 10, w.perX10 % 10);
        if (w.hasRssi)
            Serial.printf(" RSSI %ddBm\r\n", w.rssi);
        else
            Serial.printf(" RSSI -\r\n");
    }

    // A lost TX-done interrupt must not stop a transmit mode
    if ((labState == LAB_STATE_TX || labState == LAB_STATE_REF) && millis() - lastTxDoneMs > 200)
    {
        lastTxDoneMs = millis();
        txBusy = false;
        if (labState == LAB_STATE_TX)
            sendNext();
    }
    return true; // idle or test mode: the receiver state machine stays off
}

#endif
