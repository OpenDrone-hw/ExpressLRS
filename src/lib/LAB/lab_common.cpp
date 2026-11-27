#include "targets.h"

#if defined(OPENRX_LAB) && !defined(UNIT_TEST) && defined(PLATFORM_ESP32)

#include "lab_runtime.h"
#include "common.h"
#include "options.h"
#include "FHSS.h"

#include <Preferences.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <esp_system.h>

volatile LabState labState = LAB_STATE_IDLE;

#define STR2(x) #x
#define STR(x) STR2(x)

LabModel labThisModel()
{
#if defined(RADIO_LR1121)
    return LAB_MODEL_LR1121;
#else
    return LAB_MODEL_SX1281;
#endif
}

void labConsoleBegin()
{
    // UART0 on the RX/TX pads (GPIO20/21), the receiver's serial port
    int8_t rx = GPIO_PIN_RCSIGNAL_RX != UNDEF_PIN ? GPIO_PIN_RCSIGNAL_RX : -1;
    int8_t tx = GPIO_PIN_RCSIGNAL_TX != UNDEF_PIN ? GPIO_PIN_RCSIGNAL_TX : -1;
    Serial.setRxBufferSize(256);
    Serial.begin(115200, SERIAL_8N1, rx, tx);
}

bool labReadLine(char *buf, size_t len)
{
    static char line[LAB_LINE_MAX + 1];
    static size_t pos = 0;
    static bool overflow = false;
    while (Serial.available())
    {
        const int c = Serial.read();
        if (c == '\r' || c == '\n')
        {
            if (overflow)
            {
                overflow = false;
                pos = 0;
                labPrintErr("line too long");
                continue;
            }
            if (pos == 0)
                continue; // empty line, or LF after CR
            line[pos] = '\0';
            pos = 0;
            strlcpy(buf, line, len);
            return true;
        }
        if (c == '\b' || c == 0x7f)
        {
            if (pos)
                pos--;
            continue;
        }
        if (c < 0x20 || c > 0x7e)
            continue;
        if (pos < LAB_LINE_MAX)
            line[pos++] = (char)c;
        else
            overflow = true;
    }
    return false;
}

void labPrintErr(const char *msg)
{
    Serial.printf("ERR %s\r\n", msg);
}

#if defined(RADIO_LR1121)
static void domainName(char *out, size_t len, const char *elrs)
{
    // "EU868" -> "EU_868"
    size_t o = 0;
    for (size_t i = 0; elrs[i] && o + 2 < len; i++)
    {
        if (i > 0 && isdigit((unsigned char)elrs[i]) && !isdigit((unsigned char)elrs[i - 1]))
            out[o++] = '_';
        out[o++] = elrs[i];
    }
    out[o] = '\0';
}
#endif

void labPrintInfo()
{
    uint8_t mac[6] = {0};
    esp_efuse_mac_get_default(mac);
    const char *sha = commit;

    Serial.printf("OK info\r\n");
    Serial.printf("model %s\r\n", product_name);
    Serial.printf("firmware %s %s\r\n", LAB_RELEASE, sha);
#if defined(Regulatory_Domain_EU_CE_2400)
    const char *d2g4 = "EU_CE_2400";
#else
    const char *d2g4 = "ISM_2400";
#endif
#if defined(RADIO_LR1121)
    char dsub[16];
    domainName(dsub, sizeof(dsub), FHSSconfig ? FHSSconfig->domain : "none");
    Serial.printf("domain %s %s\r\n", d2g4, dsub);
    Serial.printf("power 2g4 %ddBm 868 %ddBm\r\n", OPENRX_LAB_TARGET_2G4_DBM, OPENRX_LAB_TARGET_868_DBM);
    Serial.printf("chip 2g4 %ddBm 868 %ddBm\r\n",
        POWER_OUTPUT_VALUES_DUAL_COUNT ? POWER_OUTPUT_VALUES_DUAL[0] : 0,
        POWER_OUTPUT_VALUES_COUNT ? POWER_OUTPUT_VALUES[0] : 0);
    if (POWER_OUTPUT_VALUES_COUNT != 1 || POWER_OUTPUT_VALUES_DUAL_COUNT != 1)
        labPrintErr("layout has more than one power level per band");
#else
    Serial.printf("domain %s\r\n", d2g4);
    Serial.printf("power 2g4 %ddBm\r\n", OPENRX_LAB_TARGET_2G4_DBM);
    Serial.printf("chip 2g4 %ddBm\r\n", POWER_OUTPUT_VALUES_COUNT ? POWER_OUTPUT_VALUES[0] : 0);
    if (POWER_OUTPUT_VALUES_COUNT != 1)
        labPrintErr("layout has more than one power level");
#endif
    Serial.printf("mac %02x:%02x:%02x:%02x:%02x:%02x\r\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

/***** mailbox and application switch *****/

static const char *NVS_NS = "orxlab";
static const char *NVS_KEY = "cmd";
static const char *NVS_MODE = "mode";

void labMailboxPut(const char *line)
{
    Preferences p;
    if (p.begin(NVS_NS, false))
    {
        p.putString(NVS_KEY, line);
        p.end();
    }
}

bool labBootCommand(char *line, size_t len)
{
    line[0] = '\0';
    Preferences p;
    if (!p.begin(NVS_NS, false))
        return false;
    String cmd = p.getString(NVS_KEY, "");
    if (cmd.length())
        p.remove(NVS_KEY);
    String mode = p.getString(NVS_MODE, "");
    String use;
    switch (esp_reset_reason())
    {
    case ESP_RST_SW: // our own restart: the stored command
        use = cmd;
        break;
    case ESP_RST_BROWNOUT: // supply dip, ESD, watchdog, crash: resume the link
    case ESP_RST_PANIC:
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:
        use = mode;
        break;
    default: // power-up, reset pin, unknown: idle
        if (mode.length())
            p.remove(NVS_MODE);
        break;
    }
    p.end();
    if (use.length() == 0)
        return false;
    strlcpy(line, use.c_str(), len);
    return true;
}

void labSetResumeMode(const char *line)
{
    Preferences p;
    if (!p.begin(NVS_NS, false))
        return;
    if (line[0])
    {
        if (p.getString(NVS_MODE, "") != line)
            p.putString(NVS_MODE, line);
    }
    else if (p.isKey(NVS_MODE))
        p.remove(NVS_MODE);
    p.end();
}

bool labWifiRequested()
{
    return labState == LAB_STATE_WIFI;
}

bool labSelectApp(bool txApp)
{
    const esp_partition_t *p = esp_partition_find_first(ESP_PARTITION_TYPE_APP,
        txApp ? ESP_PARTITION_SUBTYPE_APP_OTA_1 : ESP_PARTITION_SUBTYPE_APP_OTA_0, nullptr);
    if (p == nullptr)
        return false;
    if (esp_ota_get_boot_partition() == p)
        return true;
    return esp_ota_set_boot_partition(p) == ESP_OK; // validates the image
}

void labRestartWith(const char *line, bool txApp)
{
    if (!labSelectApp(txApp))
    {
        labPrintErr(txApp ? "transmitter application missing in this image" : "receiver application missing in this image");
        return;
    }
    labSetResumeMode(""); // the next application sets it again for a link mode
    labMailboxPut(line);
    Serial.flush();
    delay(10);
    esp_restart();
}

/***** LED *****/

uint32_t labLedColour(uint32_t nowMs)
{
    constexpr uint32_t WHITE = 0x404040;
    constexpr uint32_t RED = 0x800000;
    constexpr uint32_t BLUE = 0x000080;
    constexpr uint32_t ORANGE = 0x802000;
    constexpr uint32_t GREEN = 0x008000;
    constexpr uint32_t MAGENTA = 0x600060;

    if (connectionState == radioFailed || connectionState == hardwareUndefined)
        return (nowMs / 100) % 2 ? RED : 0; // fast flash
    switch (labState)
    {
    case LAB_STATE_CW:
    case LAB_STATE_TX:
    case LAB_STATE_REF:
        return RED;
    case LAB_STATE_RX:
        return BLUE;
    case LAB_STATE_WIFI:
        return MAGENTA;
    case LAB_STATE_LINK_RX:
    case LAB_STATE_LINK_TX:
        if (connectionState == connected)
            return GREEN;
        return (nowMs / 500) % 2 ? ORANGE : 0; // slow flash
    default:
        return WHITE;
    }
}

#endif
