/** @file
  Copyright (c) 2020 zxystd. All rights reserved.
  SPDX-License-Identifier: GPL-3.0-only
**/

//
//  BtIntel.cpp
//  IntelBluetoothFirmware
//
//  Created by zxystd on 2019/11/17.
//  Copyright © 2019 zxystd. All rights reserved.
//

#include "BtIntel.h"
#include "Log.h"

#define super OSObject
OSDefineMetaClassAndAbstractStructors(BtIntel, OSObject)

bool BtIntel::
initWithDevice(IOService *client, IOUSBHostDevice *dev)
{
    XYLog("%s\n", __PRETTY_FUNCTION__);
    if (!super::init()) {
        return false;
    }
    memset(loadedFirmwareName, 0, sizeof(loadedFirmwareName));
    
    m_pUSBDeviceController = new USBDeviceController();
    if (!m_pUSBDeviceController || !m_pUSBDeviceController->init(client, dev)) {
        return false;
    }
    if (!m_pUSBDeviceController->initConfiguration()) {
        return false;
    }
    if (!m_pUSBDeviceController->findInterface()) {
        return false;
    }
    if (!m_pUSBDeviceController->findPipes()) {
        return false;
    }
    return true;
}

void BtIntel::
free()
{
    XYLog("%s\n", __PRETTY_FUNCTION__);
    OSSafeReleaseNULL(m_pUSBDeviceController);
    super::free();
}

bool BtIntel::
getFirmwareName(char *fwname, size_t len)
{
    if (!fwname || len == 0) {
        return false;
    }
    strncpy(fwname, this->loadedFirmwareName, len - 1);
    fwname[len - 1] = '\0';
    return true;
}

bool BtIntel::
intelSendHCISync(HciCommandHdr *cmd, void *event, uint32_t eventBufSize, uint32_t *size, int timeout)
{
//    XYLog("%s cmd: 0x%02x len: %d\n", __PRETTY_FUNCTION__, cmd->opcode, cmd->len);
    IOReturn ret;
    uint8_t localBuf[CMD_BUF_MAX_SIZE];
    uint32_t localSize = 0;
    void *targetBuf = event ? event : localBuf;
    uint32_t targetBufSize = event ? eventBufSize : sizeof(localBuf);
    uint32_t *targetSize = size ? size : &localSize;

    if ((ret = m_pUSBDeviceController->sendHCIRequest(cmd, timeout)) != kIOReturnSuccess) {
        XYLog("%s sendHCIRequest failed: %s %d\n", __FUNCTION__, m_pUSBDeviceController->stringFromReturn(ret), ret);
        return false;
    }

    /* The interrupt pipe does not only carry the answer to this command: a vendor event or a
     * Command Complete for another opcode (e.g. the 0x0000 "ready" event) can arrive first.
     * Taking the first packet as the answer made setup fail at random (wrong size, wrong data),
     * so wait for the Command Complete / Command Status that belongs to this opcode. */
    uint64_t deadline = 0;
    uint64_t now = 0;
    clock_interval_to_deadline(timeout > 0 ? (uint32_t)timeout : 1, kMillisecondScale, &deadline);
    for (;;) {
        clock_get_uptime(&now);
        if (now >= deadline) {
            XYLog("%s timeout waiting for the response to 0x%04x\n", __FUNCTION__, OSSwapLittleToHostInt16(cmd->opcode));
            return false;
        }
        uint64_t remainingNs = 0;
        absolutetime_to_nanoseconds(deadline - now, &remainingNs);
        uint32_t remainingMs = (uint32_t)max((uint64_t)1, remainingNs / NSEC_PER_MSEC);

        *targetSize = 0;
        if ((ret = m_pUSBDeviceController->interruptPipeRead(targetBuf, targetBufSize, targetSize, remainingMs)) != kIOReturnSuccess) {
            XYLog("%s interruptPipeRead failed: %s %d\n", __FUNCTION__, m_pUSBDeviceController->stringFromReturn(ret), ret);
            return false;
        }
        if (*targetSize < HCI_EVENT_HDR_SIZE)
            continue;

        HciResponse *resp = (HciResponse *)targetBuf;
        if (resp->evt.evt == HCI_EV_CMD_COMPLETE && *targetSize >= 5 && resp->opcode == cmd->opcode) {
            if (*targetSize >= 6 && resp->data[0] != 0x00) {
                XYLog("%s cmd 0x%04x failed with HCI status 0x%02x\n", __FUNCTION__, OSSwapLittleToHostInt16(cmd->opcode), resp->data[0]);
                return false;
            }
            return true;
        }
        if (resp->evt.evt == HCI_EV_CMD_STATUS && *targetSize >= HCI_EVENT_HDR_SIZE + sizeof(HciCmdStatus)) {
            HciCmdStatus *status = (HciCmdStatus *)((uint8_t *)targetBuf + HCI_EVENT_HDR_SIZE);
            if (status->opcode == cmd->opcode) {
                if (status->status != 0x00) {
                    XYLog("%s cmd 0x%04x failed with HCI status 0x%02x\n", __FUNCTION__, OSSwapLittleToHostInt16(cmd->opcode), status->status);
                    return false;
                }
                return true;
            }
        }
        XYLog("%s skipping event 0x%02x while waiting for 0x%04x\n", __FUNCTION__, resp->evt.evt, OSSwapLittleToHostInt16(cmd->opcode));
    }
}

bool BtIntel::
waitForVendorEvent(uint8_t subEvent, void *event, uint32_t eventBufSize, uint32_t *size, uint32_t timeoutMs)
{
    if (!event || eventBufSize < 3)
        return false;

    uint64_t deadline = 0;
    uint64_t now = 0;
    clock_interval_to_deadline(timeoutMs > 0 ? timeoutMs : 1, kMillisecondScale, &deadline);
    for (;;) {
        clock_get_uptime(&now);
        if (now >= deadline) {
            XYLog("%s timeout waiting for vendor event 0x%02x\n", __FUNCTION__, subEvent);
            return false;
        }
        uint64_t remainingNs = 0;
        absolutetime_to_nanoseconds(deadline - now, &remainingNs);
        uint32_t remainingMs = (uint32_t)max((uint64_t)1, remainingNs / NSEC_PER_MSEC);

        uint32_t localSize = 0;
        IOReturn ret = m_pUSBDeviceController->interruptPipeRead(event, eventBufSize, &localSize, remainingMs);
        if (ret != kIOReturnSuccess) {
            XYLog("%s interruptPipeRead failed: %s %d\n", __FUNCTION__, m_pUSBDeviceController->stringFromReturn(ret), ret);
            return false;
        }
        if (size)
            *size = localSize;

        const uint8_t *bytes = (const uint8_t *)event;
        if (localSize >= 3 && bytes[0] == 0xff && bytes[2] == subEvent)
            return true;

        XYLog("%s skipping event 0x%02x while waiting for vendor event 0x%02x\n", __FUNCTION__, localSize ? bytes[0] : 0, subEvent);
    }
}

bool BtIntel::
intelSendHCISyncEvent(HciCommandHdr *cmd, void *event, uint32_t eventBufSize, uint32_t *size, uint8_t syncEvent, int timeout)
{
    IOReturn ret;
    if (!event || eventBufSize == 0) {
        return false;
    }
    if ((ret = m_pUSBDeviceController->sendHCIRequest(cmd, timeout)) != kIOReturnSuccess) {
        XYLog("%s sendHCIRequest failed: %s %d\n", __FUNCTION__, m_pUSBDeviceController->stringFromReturn(ret), ret);
        return false;
    }
    uint64_t deadline = 0;
    uint64_t now = 0;
    clock_interval_to_deadline(timeout > 0 ? (uint32_t)timeout : 1, kMillisecondScale, &deadline);
    do {
        uint32_t localSize = 0;
        clock_get_uptime(&now);
        if (now >= deadline) {
            XYLog("%s timeout waiting for event 0x%02x\n", __FUNCTION__, syncEvent);
            break;
        }
        uint64_t remainingNs = 0;
        absolutetime_to_nanoseconds(deadline - now, &remainingNs);
        uint32_t remainingMs = (uint32_t)max((uint64_t)1, remainingNs / NSEC_PER_MSEC);
        ret = m_pUSBDeviceController->interruptPipeRead(event, eventBufSize, &localSize, remainingMs);
        if (ret != kIOReturnSuccess) {
            XYLog("%s interruptPipeRead failed: %s %d\n", __FUNCTION__, m_pUSBDeviceController->stringFromReturn(ret), ret);
            break;
        }
        if (size) {
            *size = localSize;
        }
        if (localSize >= 1 && *(uint8_t *)event == syncEvent) {
            return true;
        }
    } while (true);
    return false;
}

bool BtIntel::
intelBulkHCISync(HciCommandHdr *cmd, void *event, uint32_t eventBufSize, uint32_t *size, int timeout)
{
//    XYLog("%s cmd: 0x%02x len: %d\n", __FUNCTION__, cmd->opcode, cmd->len);
    IOReturn ret;
    uint8_t localBuf[CMD_BUF_MAX_SIZE];
    uint32_t localSize = 0;
    void *targetBuf = event ? event : localBuf;
    uint32_t targetBufSize = event ? eventBufSize : sizeof(localBuf);
    uint32_t *targetSize = size ? size : &localSize;

    if ((ret = m_pUSBDeviceController->bulkWrite(cmd, HCI_COMMAND_HDR_SIZE + cmd->len, timeout)) != kIOReturnSuccess) {
        XYLog("%s bulkWrite failed: %s %d\n", __FUNCTION__, m_pUSBDeviceController->stringFromReturn(ret), ret);
        return false;
    }
    if ((ret = m_pUSBDeviceController->bulkPipeRead(targetBuf, targetBufSize, targetSize, timeout)) != kIOReturnSuccess) {
        XYLog("%s bulkPipeRead failed: %s %d\n", __FUNCTION__, m_pUSBDeviceController->stringFromReturn(ret), ret);
        return false;
    }
    HciResponse *resp = (HciResponse *)targetBuf;
    if (*targetSize >= 6 && targetBufSize >= 6 && resp->evt.evt == HCI_EV_CMD_COMPLETE &&
        resp->opcode == cmd->opcode && resp->data[0] != 0x00) {
        XYLog("%s cmd 0x%04x failed with HCI status 0x%02x\n", __FUNCTION__, cmd->opcode, resp->data[0]);
        return false;
    }
    return true;
}

bool BtIntel::
securedSend(uint8_t fragmentType, uint32_t len, const uint8_t *fragment)
{
    bool ret = true;
    uint8_t buf[CMD_BUF_MAX_SIZE];
    HciCommandHdr *hciCommand = (HciCommandHdr *)buf;
    
    while (len > 0) {
        uint8_t fragment_len = (len > 252) ? 252 : len;
        
        memset(buf, 0, sizeof(buf));
        hciCommand->opcode = OSSwapHostToLittleInt16(0xfc09);
        hciCommand->len = fragment_len + 1;
        hciCommand->data[0] = fragmentType;
        memcpy(hciCommand->data + 1, fragment, fragment_len);
        
        if (!(ret = intelBulkHCISync(hciCommand, NULL, 0, NULL, HCI_INIT_TIMEOUT))) {
            XYLog("secure send failed\n");
            return ret;
        }
        
        len -= fragment_len;
        fragment += fragment_len;
    }
    
    return ret;
}

bool BtIntel::
intelVersionInfo(IntelVersion *ver)
{
    const char *variant;
    
    /* The hardware platform number has a fixed value of 0x37 and
     * for now only accept this single value.
     */
    if (ver->hw_platform != 0x37) {
        XYLog("Unsupported Intel hardware platform (%u)\n",
              ver->hw_platform);
        return false;
    }
    
    /* Check for supported iBT hardware variants of this firmware
     * loading method.
     *
     * This check has been put in place to ensure correct forward
     * compatibility options when newer hardware variants come along.
     */
    switch (ver->hw_variant) {
        case 0x0b:      /* SfP */
        case 0x0c:      /* WsP */
        case 0x11:      /* JfP */
        case 0x12:      /* ThP */
        case 0x13:      /* HrP */
        case 0x14:      /* CcP */
            break;
        default:
            XYLog("Unsupported Intel hardware variant (%u)\n",
                  ver->hw_variant);
            return false;
    }
    
    switch (ver->fw_variant) {
        case 0x06:
            variant = "Bootloader";
            break;
        case 0x23:
            variant = "Firmware";
            break;
        default:
            XYLog("Unsupported firmware variant(%02x)\n", ver->fw_variant);
            return false;
    }
    
    XYLog("%s revision %u.%u build %u week %u %u\n",
          variant, ver->fw_revision >> 4, ver->fw_revision & 0x0f,
          ver->fw_build_num, ver->fw_build_ww,
          2000 + ver->fw_build_yy);
    
    return true;
}

bool BtIntel::
intelBoot(uint32_t bootAddr)
{
    uint8_t buf[CMD_BUF_MAX_SIZE];
    uint32_t actLen = 0;

    memset(buf, 0, sizeof(buf));
    
    if (!sendIntelReset(bootAddr)) {
        XYLog("Intel Soft Reset failed\n");
        resetToBootloader();
        return false;
    }
    
    /* The bootloader will not indicate when the device is ready. This
     * is done by the operational firmware sending bootup notification.
     *
     * Booting into operational firmware should not take longer than
     * 1 second. However if that happens, then just fail the setup
     * since something went wrong.
     */
    // Other events may come first; wait specifically for the bootup notification (vendor event 0x02)
    if (!waitForVendorEvent(0x02, buf, sizeof(buf), &actLen, 1000)) {
        XYLog("Intel boot failed, reset to bootloader\n");
        resetToBootloader();
        return false;
    }
    XYLog("Notify: Device reboot done\n");
    return true;
}

bool BtIntel::
loadDDCConfig(const char *ddcFileName)
{
    const uint8_t *fw_ptr;
    uint8_t buf[CMD_BUF_MAX_SIZE];
    HciCommandHdr *cmd = (HciCommandHdr *)buf;
    
    OSData *fwData = requestFirmwareData(ddcFileName);
    
    if (fwData == NULL) {
        XYLog("DDC file not found: %s\n", ddcFileName);
        return false;
    }
    
    XYLog("Load DDC config: %s %d\n", ddcFileName, fwData->getLength());
    
    const uint8_t *base = (const uint8_t *)fwData->getBytesNoCopy();
    size_t totalLen = fwData->getLength();
    bool ret = true;
    fw_ptr = base;
    
    /* DDC file contains one or more DDC structure which has
     * Length (1 byte), DDC ID (2 bytes), and DDC value (Length - 2).
     */
    while ((size_t)(fw_ptr - base) < totalLen) {
        size_t remain = totalLen - (size_t)(fw_ptr - base);
        size_t cmd_plen = (size_t)fw_ptr[0] + sizeof(uint8_t);
        if (cmd_plen > remain || cmd_plen > (CMD_BUF_MAX_SIZE - HCI_COMMAND_HDR_SIZE)) {
            XYLog("DDC config record length error (%zu, remain: %zu)\n", cmd_plen, remain);
            ret = false;
            break;
        }

        cmd->opcode = OSSwapHostToLittleInt16(0xfc8b);
        cmd->len = (uint8_t)cmd_plen;
        memcpy(cmd->data, fw_ptr, cmd->len);
        if (!intelSendHCISync(cmd, NULL, 0, NULL, HCI_INIT_TIMEOUT)) {
            XYLog("Failed to send Intel_Write_DDC\n");
            ret = false;
            break;
        }

        fw_ptr += cmd_plen;
    }
    OSSafeReleaseNULL(fwData);
    
    if (ret)
        XYLog("Load DDC config done\n");
    return ret;
}
