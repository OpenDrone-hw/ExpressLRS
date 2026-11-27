// OpenRX lab test firmware: transmitter application (app1), "link tx"
//
// The unmodified ExpressLRS transmitter (tx_main.cpp) runs on the receiver
// board as RX-as-TX. Instead of a CRSF handset, LabHandset supplies fixed
// channel values (sticks centred, AUX low) and the serial test console.
#include "targets.h"

#if defined(OPENRX_LAB) && !defined(UNIT_TEST) && defined(TARGET_TX) && defined(PLATFORM_ESP32)

#include "lab_runtime.h"
#include "common.h"
#include "elrs_eeprom.h"
#include "config.h"
#include "handset.h"
#include "POWERMGNT.h"
#include "crsf_protocol.h"

extern void ModelUpdateReq();

static LabCmd linkCmd;
static char linkLine[LAB_LINE_MAX + 1];
static bool linkValid;

void labTxBootCheck()
{
    char line[LAB_LINE_MAX + 1];
    const bool have = labBootCommand(line, sizeof(line));
    if (have)
        labParse(line, labThisModel(), &linkCmd);
    if (have && linkCmd.type == LAB_CMD_LINK_TX)
    {
        strlcpy(linkLine, line, sizeof(linkLine));
        linkValid = true;
        labState = LAB_STATE_LINK_TX;
        labSetResumeMode(line); // resume after a brownout or watchdog reset
        return;
    }
    // Power-up, or anything but link tx: back to the receiver application
    if (labSelectApp(false))
    {
        if (have)
            labMailboxPut(line);
        else if (labResumeStopped)
            labMailboxPut(LAB_RESUME_STOPPED_LINE); // the receiver prints the error
        esp_restart();
    }
    // No receiver application: stay here, idle (LabHandset never connects)
}

void labTxApplyConfig()
{
    if (!linkValid)
        return;
    config.SetRate(enumRatetoIndex((expresslrs_RFrates_e)linkCmd.rate->enumRate));
    config.SetTlm(labTlmDenomToEnum(linkCmd.tlm));
    config.SetPower(PWR_10mW); // the one EU level of the layout
    config.SetDynamicPower(false);
    config.SetModelMatch(false);
}

class LabHandset final : public Handset
{
public:
    void Begin() override
    {
        labConsoleBegin();
        for (unsigned ch = 0; ch < CRSF_NUM_CHANNELS; ch++)
            ChannelData[ch] = ch < 4 ? CRSF_CHANNEL_VALUE_MID : CRSF_CHANNEL_VALUE_MIN;
        SetArmed(false);
        if (!linkValid)
        {
            labPrintErr("receiver application missing in this image");
            return;
        }
        if (connectionState == radioFailed || connectionState == hardwareUndefined)
        {
            linkValid = false; // never transmit, never resume this mode
            labState = LAB_STATE_IDLE;
            labSetResumeMode("");
            labPrintErr(connectionState == radioFailed ? "radio not found" : "hardware layout missing");
            return;
        }
        Serial.printf("OK link tx %s tlm 1:%u power ", linkCmd.rate->name, linkCmd.tlm);
#if defined(RADIO_LR1121)
        Serial.printf("2g4 %ddBm 868 %ddBm\r\n", OPENRX_LAB_TARGET_2G4_DBM, OPENRX_LAB_TARGET_868_DBM);
#else
        Serial.printf("2g4 %ddBm\r\n", OPENRX_LAB_TARGET_2G4_DBM);
#endif
        // Never call RCDataReceived(): with no handset data ever received the
        // transmitter sends the fixed ChannelData (bench mode in SendRCdataToRF)
        if (connected)
            connected();
        ModelUpdateReq(); // no handset will send a model ID: start now
    }

    void End() override {}

    void handleInput() override
    {
        char line[LAB_LINE_MAX + 1];
        if (labReadLine(line, sizeof(line)))
            execute(line);
        labResumeStableCheck(millis());
        report();
    }

private:
    uint32_t lastReport = 0;

    void execute(const char *line)
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
        case LAB_CMD_LINK_TX:
            if (linkValid && c.rate == linkCmd.rate && c.tlm == linkCmd.tlm)
            {
                Serial.printf("OK link tx %s tlm 1:%u\r\n", c.rate->name, c.tlm);
                return;
            }
            labRestartWith(line, true);
            return;
        default:
            labRestartWith(c.type == LAB_CMD_STOP ? "stop" : line, false);
            return;
        }
    }

    void report()
    {
        const uint32_t now = millis();
        if (!linkValid || now - lastReport < 1000)
            return;
        lastReport = now;
        Serial.printf("LINK TX %s rate %s tlm 1:%u\r\n", connectionState == ::connected ? "connected" : "searching",
            linkCmd.rate->name, ExpressLRS_currTlmDenom);
    }
};

Handset *labTxCreateHandset()
{
    return new LabHandset();
}

#endif
