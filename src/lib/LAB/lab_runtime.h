#pragma once
/*
 * OpenRX lab test firmware, on-target part (built with -DOPENRX_LAB).
 *
 * One flash image holds two applications:
 *   app0 (ota_0): the ExpressLRS receiver with the test console. Boots at
 *                 power-up, idle. Runs cw, tx, rx, ref and link rx.
 *   app1 (ota_1): the ExpressLRS transmitter (RX-as-TX on the same board)
 *                 with an internal channel source. Runs link tx only.
 * A command that needs the other application, or that leaves a link mode, is
 * stored in NVS and the board restarts into the right application, which
 * executes it. The stored command is honoured only after a software reset,
 * so a power-up always starts idle in app0. After a brownout, watchdog or
 * panic reset the last link mode (link rx / link tx) resumes by itself.
 */

#if defined(OPENRX_LAB) && !defined(UNIT_TEST)

#include <Arduino.h>
#include "lab_core.h"

#if !defined(OPENRX_LAB_TARGET_2G4_DBM)
#error "OPENRX_LAB_TARGET_2G4_DBM (declared 2.4 GHz conducted power) must be set by the build recipe"
#endif
#if defined(RADIO_LR1121) && !defined(OPENRX_LAB_TARGET_868_DBM)
#error "OPENRX_LAB_TARGET_868_DBM (declared 868 MHz conducted power) must be set by the build recipe"
#endif

enum LabState : uint8_t
{
    LAB_STATE_IDLE = 0,
    LAB_STATE_CW,
    LAB_STATE_TX,
    LAB_STATE_RX,
    LAB_STATE_REF,
    LAB_STATE_LINK_RX,
    LAB_STATE_LINK_TX,
    LAB_STATE_WIFI,
};

extern volatile LabState labState;

// shared (lab_common.cpp)
LabModel labThisModel();
void labConsoleBegin();
bool labReadLine(char *buf, size_t len); // non-blocking, true when a line is complete
void labPrintInfo();
void labPrintErr(const char *msg);
void labMailboxPut(const char *line);
// The command to run at boot: the stored command after a software reset, the
// last link mode after a brownout/watchdog/panic reset, nothing at power-up
bool labBootCommand(char *line, size_t len);
void labSetResumeMode(const char *line); // "" clears
extern bool labResumeStopped;            // set by labBootCommand() after a crash loop
void labResumeStableCheck(uint32_t nowMs); // call from the loop: resets the resume count
#define LAB_RESUME_STOPPED_LINE "!resume-stopped"
#define LAB_RESUME_STOPPED_ERR "resume stopped after 3 resets"
bool labWifiRequested();
bool labSelectApp(bool txApp);
// Store the command and restart into the application; returns only on failure
void labRestartWith(const char *line, bool txApp);
uint32_t labLedColour(uint32_t nowMs); // for devRGB

// receiver application (lab_rx.cpp), called from rx_main.cpp
void labRxSetupSerial();
bool labRxBootLink();    // true: start the normal receiver now (link rx)
void labRxBegin();       // end of setup(): banner or stored command
bool labRxLoop(uint32_t now); // true: skip the rest of the receiver loop

// transmitter application (lab_tx.cpp), called from tx_main.cpp / devHandset.cpp
void labTxBootCheck();   // first thing in setup(): back to app0 unless link tx
void labTxApplyConfig(); // after config.Load(): rate, telemetry, power
class Handset;
Handset *labTxCreateHandset();

#endif
