//
//  IntelBTPatcher.cpp
//  IntelBTPatcher
//
//  Created by zxystd <zxystd@foxmail.com> on 2021/2/8.
//
#define __ACIDANTHERA_MAC_SDK 2600
#include <Headers/kern_api.hpp>
#include <Headers/kern_util.hpp>
#include <Headers/plugin_start.hpp>
#include <IOKit/IOLocks.h>
#include <sys/random.h>
#include <libkern/OSAtomic.h>

#include "IntelBTPatcher.hpp"

static CIntelBTPatcher ibtPatcher;
static CIntelBTPatcher *callbackIBTPatcher = nullptr;

static const char *bootargOff[] {
    "-ibtcompatoff"
};

static const char *bootargDebug[] {
    "-ibtcompatdbg"
};

static const char *bootargBeta[] {
    "-ibtcompatbeta"
};

PluginConfiguration ADDPR(config) {
    xStringify(PRODUCT_NAME),
    parseModuleVersion(xStringify(MODULE_VERSION)),
    LiluAPI::AllowNormal | LiluAPI::AllowInstallerRecovery | LiluAPI::AllowSafeMode,
    bootargOff,
    arrsize(bootargOff),
    bootargDebug,
    arrsize(bootargDebug),
    bootargBeta,
    arrsize(bootargBeta),
    KernelVersion::MountainLion,
    KernelVersion::Tahoe,
    []() {
        ibtPatcher.init();
    }
};

static const char *IntelBTPatcher_IOBluetoothFamily[] { "/System/Library/Extensions/IOBluetoothFamily.kext/Contents/MacOS/IOBluetoothFamily" };

static KernelPatcher::KextInfo IntelBTPatcher_IOBluetoothInfo {
    "com.apple.iokit.IOBluetoothFamily",
    IntelBTPatcher_IOBluetoothFamily,
    1,
    {true, true},
    {},
    KernelPatcher::KextInfo::Unloaded
};

static const char *IntelBTPatcher_IOUSBHostFamily[] {
    "/System/Library/Extensions/IOUSBHostFamily.kext/Contents/MacOS/IOUSBHostFamily" };

static KernelPatcher::KextInfo IntelBTPatcher_IOUsbHostInfo {
    "com.apple.iokit.IOUSBHostFamily",
    IntelBTPatcher_IOUSBHostFamily,
    1,
    {true, true},
    {},
    KernelPatcher::KextInfo::Unloaded
};

void *CIntelBTPatcher::_hookPipeInstance = nullptr;
IOUSBHostDevice *CIntelBTPatcher::_hookDeviceInstance = nullptr;
bool CIntelBTPatcher::_randomAddressInit = false;
bool CIntelBTPatcher::_supportsLe2MPhy = false;
static IOSimpleLock *leMetaLock = nullptr;

static uint64_t generation;
static unsigned commandCredits;
static uint64_t firmwareEpoch;
static uint8_t randomAddressHci[9] = {0x05, 0x20, 6};
static IONotifier *detachNotifier;
static AsyncOwnerData completionPool[16];
static bool poolBusy[16];
static bool randomAddressError;
// Injected commands wait for a free command slot (Num_HCI_Command_Packets of the last Command
// Complete/Status). Log once when that skips them, so a controller that only reports one slot,
// which the host command itself uses, shows up in the log instead of silently losing both fixes.
static bool loggedNoAddressCredit, loggedNoDuplicateCredit;

bool CIntelBTPatcher::init()
{
    DBGLOG(DRV_NAME, "%s", __PRETTY_FUNCTION__);
    callbackIBTPatcher = this;
    leMetaLock = IOSimpleLockAlloc();
    if (!leMetaLock) return false;
    do {
        read_random(randomAddressHci + 3, 6);
        randomAddressHci[8] |= 0xc0;
    } while ((!memcmp(randomAddressHci + 3, "\0\0\0\0\0\xc0", 6)) ||
             (!memcmp(randomAddressHci + 3, "\xff\xff\xff\xff\xff\xff", 6)));
    if (getKernelVersion() >= KernelVersion::Monterey) {
        auto match = IOService::serviceMatching("IOUSBHostDevice");
        if (!match) return false;
        detachNotifier = IOService::addMatchingNotification(gIOTerminatedNotification, match, detached, nullptr);
        match->release();
        if (!detachNotifier) return false;
    }
    if (getKernelVersion() < KernelVersion::Monterey) {
        lilu.onKextLoadForce(&IntelBTPatcher_IOBluetoothInfo, 1,
        [](void *user, KernelPatcher &patcher, size_t index, mach_vm_address_t address, size_t size) {
            callbackIBTPatcher->processKext(patcher, index, address, size);
        }, this);
    } else {
        lilu.onKextLoadForce(&IntelBTPatcher_IOUsbHostInfo, 1,
        [](void *user, KernelPatcher &patcher, size_t index, mach_vm_address_t address, size_t size) {
            callbackIBTPatcher->processKext(patcher, index, address, size);
        }, this);
    }
    return true;
}

void CIntelBTPatcher::free()
{
    DBGLOG(DRV_NAME, "%s", __PRETTY_FUNCTION__);
    if (detachNotifier) detachNotifier->remove();
    if (leMetaLock) {
        IOSimpleLockFree(leMetaLock);
        leMetaLock = nullptr;
    }
}

void CIntelBTPatcher::processKext(KernelPatcher &patcher, size_t index, mach_vm_address_t address, size_t size)
{
    DBGLOG(DRV_NAME, "%s", __PRETTY_FUNCTION__);
    if (getKernelVersion() < KernelVersion::Monterey) {
        if (IntelBTPatcher_IOBluetoothInfo.loadIndex == index) {
            DBGLOG(DRV_NAME, "%s", IntelBTPatcher_IOBluetoothInfo.id);
            
            KernelPatcher::RouteRequest findQueueRequestRequest {
                "__ZN25IOBluetoothHostController17FindQueuedRequestEtP22BluetoothDeviceAddresstbPP21IOBluetoothHCIRequest",
                newFindQueueRequest,
                oldFindQueueRequest
            };
            patcher.routeMultiple(index, &findQueueRequestRequest, 1, address, size);
            if (patcher.getError() == KernelPatcher::Error::NoError) {
                DBGLOG(DRV_NAME, "routed %s", findQueueRequestRequest.symbol);
            } else {
                SYSLOG(DRV_NAME, "failed to resolve %s, error = %d", findQueueRequestRequest.symbol, static_cast<int>(patcher.getError()));
                patcher.clearError();
            }
            
        }
    } else {
        if (IntelBTPatcher_IOUsbHostInfo.loadIndex == index) {
            SYSLOG(DRV_NAME, "%s", IntelBTPatcher_IOUsbHostInfo.id);
            
            KernelPatcher::RouteRequest hostDeviceRequest {
            "__ZN15IOUSBHostDevice13deviceRequestEP9IOServiceRN11StandardUSB13DeviceRequestEPvP18IOMemoryDescriptorRjP19IOUSBHostCompletionj",
                newHostDeviceRequest,
                oldHostDeviceRequest
            };
            patcher.routeMultiple(index, &hostDeviceRequest, 1, address, size);
            if (patcher.getError() == KernelPatcher::Error::NoError) {
                SYSLOG(DRV_NAME, "routed %s", hostDeviceRequest.symbol);
            } else {
                SYSLOG(DRV_NAME, "failed to resolve %s, error = %d", hostDeviceRequest.symbol, static_cast<int>(patcher.getError()));
                patcher.clearError();
            }

            KernelPatcher::RouteRequest asyncIORequest {
            "__ZN13IOUSBHostPipe2ioEP18IOMemoryDescriptorjP19IOUSBHostCompletionj",
                newAsyncIO,
                oldAsyncIO
            };
            patcher.routeMultiple(index, &asyncIORequest, 1, address, size);
            if (patcher.getError() == KernelPatcher::Error::NoError) {
                SYSLOG(DRV_NAME, "routed %s", asyncIORequest.symbol);
            } else {
                SYSLOG(DRV_NAME, "failed to resolve %s, error = %d", asyncIORequest.symbol, static_cast<int>(patcher.getError()));
                patcher.clearError();
            }

            // macOS 26.5+: third parameter became ConfigurationDescriptor (was SuperSpeedEndpointCompanionDescriptor)
            KernelPatcher::RouteRequest initPipeConfigRequest {
            "__ZN13IOUSBHostPipe28initWithDescriptorsAndOwnersEPKN11StandardUSB18EndpointDescriptorEPKNS0_23ConfigurationDescriptorEP22AppleUSBHostControllerP15IOUSBHostDeviceP18IOUSBHostInterfaceht",
                newInitPipeConfig,
                oldInitPipeConfig
            };
            patcher.routeMultiple(index, &initPipeConfigRequest, 1, address, size);
            if (patcher.getError() == KernelPatcher::Error::NoError) {
                SYSLOG(DRV_NAME, "routed %s", initPipeConfigRequest.symbol);
            } else {
                DBGLOG(DRV_NAME, "failed to resolve %s, error = %d", initPipeConfigRequest.symbol, static_cast<int>(patcher.getError()));
                patcher.clearError();
            }

            KernelPatcher::RouteRequest initPipeRequest {
            "__ZN13IOUSBHostPipe28initWithDescriptorsAndOwnersEPKN11StandardUSB18EndpointDescriptorEPKNS0_37SuperSpeedEndpointCompanionDescriptorEP22AppleUSBHostControllerP15IOUSBHostDeviceP18IOUSBHostInterfaceht",
                newInitPipe,
                oldInitPipe
            };
            patcher.routeMultiple(index, &initPipeRequest, 1, address, size);
            if (patcher.getError() == KernelPatcher::Error::NoError) {
                SYSLOG(DRV_NAME, "routed %s", initPipeRequest.symbol);
            } else {
                DBGLOG(DRV_NAME, "failed to resolve %s, error = %d", initPipeRequest.symbol, static_cast<int>(patcher.getError()));
                patcher.clearError();
            }
        }
    }
}

#pragma mark - For Bigsur-, patch unhandled 0x2019 opcode

#define HCI_OP_LE_START_ENCRYPTION 0x2019

IOReturn CIntelBTPatcher::newFindQueueRequest(void *that, unsigned short arg1, void *addr, unsigned short arg2, bool arg3, void **hciRequestPtr)
{
    IOReturn ret = FunctionCast(newFindQueueRequest, callbackIBTPatcher->oldFindQueueRequest)(that, arg1, addr, arg2, arg3, hciRequestPtr);
    if (ret != 0 && arg1 == HCI_OP_LE_START_ENCRYPTION) {
        ret = FunctionCast(newFindQueueRequest, callbackIBTPatcher->oldFindQueueRequest)(that, arg1, addr, 0xffff, arg3, hciRequestPtr);
        DBGLOG(DRV_NAME, "%s ret: %d arg1: 0x%04x arg2: 0x%04x arg3: %d ptr: %p", __FUNCTION__, ret, arg1, arg2, arg3, hciRequestPtr ? *hciRequestPtr : nullptr);
    }
    return ret;
}

#pragma mark - For Monterey+ patch for intercept HCI REQ and RESP

#define VENDOR_USB_INTEL 0x8087

#define MAX_HCI_BUF_LEN                 255
#define HCI_OP_RESET                    0x0c03
#define HCI_OP_LE_SET_SCAN_PARAM        0x200B
#define HCI_OP_LE_SET_SCAN_ENABLE       0x200C
#define HCI_OP_LE_READ_REMOTE_FEATURES  0x2016

struct LeMetaSlot {
    uint16_t handle;
    uint8_t pending;
    uint8_t stage;
    bool inUse;
};

static constexpr size_t kMaxLeMetaSlots = 16;
static LeMetaSlot leMetaSlots[kMaxLeMetaSlots] = {};
static void resetLeMetaState()
{
    randomAddressError = false;
    ++generation;
    commandCredits = 0;
    memset(leMetaSlots, 0, sizeof(leMetaSlots));
}

static LeMetaSlot *findOrAllocLeMetaSlotLocked(uint16_t handle, bool allocate)
{
    LeMetaSlot *freeSlot = nullptr;
    for (size_t i = 0; i < kMaxLeMetaSlots; i++) {
        if (leMetaSlots[i].inUse) {
            if (leMetaSlots[i].handle == handle)
                return &leMetaSlots[i];
        } else if (!freeSlot) {
            freeSlot = &leMetaSlots[i];
        }
    }
    if (allocate) {
        if (!freeSlot) return nullptr;
        freeSlot->handle = handle;
        freeSlot->pending = 0;
        freeSlot->stage = 0;
        freeSlot->inUse = true;
        return freeSlot;
    }
    return nullptr;
}

// Drops whatever is still recorded for this connection handle. Duplicate reads can go unanswered
// (the controller may reject them, or the link drops first) and handles are reused by later
// connections: stale state would then rewrite the *real* features event of the next connection into a
// fake PHY / Data Length event, and bluetoothd never finishes setting that connection up.
static void clearLeMetaSlot(uint16_t handle, uint64_t epoch)
{
    if (leMetaLock)
        IOSimpleLockLock(leMetaLock);
    LeMetaSlot *slot = findOrAllocLeMetaSlotLocked(handle, false);
    if (slot && epoch == generation) {
        slot->pending = 0;
        slot->stage = 0;
        slot->inUse = false;
    }
    if (leMetaLock)
        IOSimpleLockUnlock(leMetaLock);
}

static bool addLeMetaPending(uint16_t handle, uint8_t count, uint64_t epoch)
{
    bool added = false;
    if (leMetaLock)
        IOSimpleLockLock(leMetaLock);
    LeMetaSlot *slot = epoch == generation && commandCredits ? findOrAllocLeMetaSlotLocked(handle, true) : nullptr;
    if (slot && slot->pending < 2) {
        --commandCredits;
        slot->pending += count;
        added = true;
    }
    if (leMetaLock)
        IOSimpleLockUnlock(leMetaLock);
    return added;
}

static void rollbackLeMetaPending(uint16_t handle, uint8_t count, uint64_t epoch)
{
    if (leMetaLock)
        IOSimpleLockLock(leMetaLock);
    LeMetaSlot *slot = findOrAllocLeMetaSlotLocked(handle, false);
    if (slot && epoch == generation) {
        uint8_t dec = min(slot->pending, count);
        slot->pending -= dec;
        if (slot->pending == 0) {
            slot->inUse = false;
            slot->stage = 0;
        }
    }
    if (leMetaLock)
        IOSimpleLockUnlock(leMetaLock);
}

IOReturn CIntelBTPatcher::newHostDeviceRequest(void *that, IOService *provider, StandardUSB::DeviceRequest &request, void *data, IOMemoryDescriptor *descriptor, unsigned int &length, IOUSBHostCompletion *completion, unsigned int timeout)
{
    HciCommandHdr *hdr = nullptr;
    uint32_t hdrLen = 0;
    char hciBuf[MAX_HCI_BUF_LEN] = {0};

    // Until the Intel pipe has been captured, only look at Intel devices: this hook sees the class
    // requests of every USB device, and acting on look-alike opcodes of e.g. a HID device would
    // inject Bluetooth commands into it.
    auto dev = static_cast<IOUSBHostDevice *>(that);
    bool classCommand = dev && request.bRequest == 0 && request.wLength >= 3 &&
        (request.bmRequestType & (kDeviceRequestDirectionMask | kDeviceRequestTypeMask)) ==
            (kRequestTypeClass << kDeviceRequestTypePhase);
    if (!classCommand || (provider && provider->metaCast("IntelBluetoothFirmware")))
        return FunctionCast(newHostDeviceRequest, callbackIBTPatcher->oldHostDeviceRequest)(that, provider, request, data, descriptor, length, completion, timeout);
    if (dev->getProperty("FirmwareLoaded") == kOSBooleanFalse &&
        dev->getProperty("IntelBTFirmwareGeneration")) {
        length = 0;
        return dev->isInactive() ? kIOReturnNoDevice : kIOReturnNotReady;
    }
    auto property = dev->copyProperty("IntelBTFirmwareGeneration");
    auto number = OSDynamicCast(OSNumber, property);
    uint64_t deviceEpoch = number ? number->unsigned64BitValue() : 0;
    OSSafeReleaseNULL(property);
    IOSimpleLockLock(leMetaLock);
    if (that == _hookDeviceInstance && firmwareEpoch != deviceEpoch) {
        firmwareEpoch = deviceEpoch;
        _randomAddressInit = false;
        resetLeMetaState();
    }
    bool isIntelBT = that == _hookDeviceInstance;
    uint64_t epoch = generation;
    IOSimpleLockUnlock(leMetaLock);
    isIntelBT = isIntelBT && !dev->isInactive() && dev->getProperty("FirmwareLoaded") == kOSBooleanTrue;

    if (isIntelBT) {
        if (data == nullptr) {
            if (descriptor != nullptr) {
                bool prepared = (getKernelVersion() < KernelVersion::Sequoia) ||
                                (descriptor->prepare(kIODirectionOut) == kIOReturnSuccess);
                if (prepared) {
                    if (descriptor->getLength() > 0) {
                        hdrLen = (uint32_t)min(descriptor->getLength(), (IOByteCount)MAX_HCI_BUF_LEN);
                        hdrLen = descriptor->readBytes(0, hciBuf, min(hdrLen, (uint32_t)request.wLength));
                    }
                    if (getKernelVersion() >= KernelVersion::Sequoia)
                        descriptor->complete(kIODirectionOut);
                }
            }
            if (hdrLen >= sizeof(HciCommandHdr))
                hdr = (HciCommandHdr *)hciBuf;
        } else if (request.wLength >= sizeof(HciCommandHdr)) {
            hdr = (HciCommandHdr *)data;
            hdrLen = request.wLength;
        }
    }

    if (hdr && hdrLen == request.wLength && hdrLen == sizeof(*hdr) + hdr->len) {
        IOSimpleLockLock(leMetaLock);
        if (epoch == generation && commandCredits) --commandCredits; // host command first
        IOSimpleLockUnlock(leMetaLock);
        // HCI reset, we need to send Random address again and reset states
        if (hdr->opcode == HCI_OP_RESET) {
            IOSimpleLockLock(leMetaLock);
            if (epoch == generation) { _randomAddressInit = false; resetLeMetaState(); }
            IOSimpleLockUnlock(leMetaLock);
        } else if (hdrLen >= 9 && hdr->opcode == HCI_OP_LE_SET_SCAN_PARAM && hdr->data[5] == 0x1) {
            IOSimpleLockLock(leMetaLock);
            if (randomAddressError) { _randomAddressInit = false; randomAddressError = false; }
            bool send = epoch == generation && !_randomAddressInit && commandCredits;
            bool noCredit = epoch == generation && !_randomAddressInit && !commandCredits;
            if (send) {
                _randomAddressInit = true;
                --commandCredits;
            }
            IOSimpleLockUnlock(leMetaLock);
            if (noCredit && !__atomic_exchange_n(&loggedNoAddressCredit, true, __ATOMIC_RELAXED))
                SYSLOG(DRV_NAME, "random address not sent: no free HCI command slot (further skips are not logged)");
            if (send) {
                auto addressRequest = request;
                addressRequest.wLength = sizeof(randomAddressHci);
                unsigned sent = 0;
                auto ret = FunctionCast(newHostDeviceRequest, callbackIBTPatcher->oldHostDeviceRequest)(
                    that, provider, addressRequest, randomAddressHci, nullptr, sent, nullptr, timeout);
                if (ret || sent != sizeof(randomAddressHci)) {
                    IOSimpleLockLock(leMetaLock);
                    if (epoch == generation) _randomAddressInit = false;
                    IOSimpleLockUnlock(leMetaLock);
                }
            }
        } else if (hdr->opcode == HCI_OP_LE_READ_REMOTE_FEATURES && hdrLen == 5) {
            uint16_t handle = ((uint16_t)hdr->data[0] | ((uint16_t)hdr->data[1] << 8)) & 0x0FFF;
            clearLeMetaSlot(handle, epoch);
            uint8_t duplicate[5];
            memcpy(duplicate, hdr, sizeof(duplicate));
            auto dupRequest = request;
            unsigned reserved = 0;
            while (reserved < 2 && addLeMetaPending(handle, 1, epoch)) ++reserved;
            if (reserved < 2 && !__atomic_exchange_n(&loggedNoDuplicateCredit, true, __ATOMIC_RELAXED))
                SYSLOG(DRV_NAME, "LE Read Remote Features: %u of 2 duplicates reserved, no free HCI command slot or tracking slot (further skips are not logged)", reserved);
            if (reserved) {
                dev->retain();
                if (provider) provider->retain();
            }
            IOReturn host = FunctionCast(newHostDeviceRequest, callbackIBTPatcher->oldHostDeviceRequest)(
                that, provider, request, data, descriptor, length, completion, timeout);
            if (host != kIOReturnSuccess) {
                rollbackLeMetaPending(handle, reserved, epoch);
                if (reserved) {
                    if (provider) provider->release();
                    dev->release();
                }
                return host;
            }
            // #514: send TWO extra reads -> two duplicate 0x04 events to rewrite
            // (1st dup -> fake PHY Update Complete, 2nd dup -> fake LE Data Length Change).
            for (unsigned i = 0; i < reserved; ++i) {
                IOSimpleLockLock(leMetaLock);
                bool current = epoch == generation;
                IOSimpleLockUnlock(leMetaLock);
                if (!current || dev->isInactive()) break;
                unsigned int dupLen = 0;
                IOReturn dupRet = FunctionCast(newHostDeviceRequest, callbackIBTPatcher->oldHostDeviceRequest)(that, provider, dupRequest, duplicate, nullptr, dupLen, nullptr, timeout);
                if (dupRet != kIOReturnSuccess)
                    rollbackLeMetaPending(handle, 1, epoch);
            }
            if (reserved) {
                if (provider) provider->release();
                dev->release();
            }
            return host;
        }
#if DEBUG
        uint8_t dirIdx = (request.bmRequestType & kDeviceRequestDirectionMask) >> kDeviceRequestDirectionPhase;
        uint8_t typeIdx = (request.bmRequestType & kDeviceRequestTypeMask) >> kDeviceRequestTypePhase;
        uint8_t recpIdx = (request.bmRequestType & kDeviceRequestRecipientMask) >> kDeviceRequestRecipientPhase;
        const char *dirName = dirIdx < arrsize(requestDirectionNames) ? requestDirectionNames[dirIdx] : "Reserved";
        const char *typeName = typeIdx < arrsize(requestTypeNames) ? requestTypeNames[typeIdx] : "Reserved";
        const char *recpName = recpIdx < arrsize(requestRecipientNames) ? requestRecipientNames[recpIdx] : "Reserved";
        DBGLOG(DRV_NAME, "[%s] bRequest: 0x%x direction: %s type: %s recipient: %s wValue: 0x%02x wIndex: 0x%02x opcode: 0x%04x len: %d length: %d async: %d", provider ? provider->getName() : "unknown", request.bRequest, dirName, typeName, recpName, request.wValue, request.wIndex, hdr->opcode, hdr->len, request.wLength, completion != nullptr);
        if (hdrLen) {
            const char *dump = _hexDumpHCIData((uint8_t *)hdr, hdrLen);
            if (dump) {
                DBGLOG(DRV_NAME, "[Request]: %s", dump);
                IOFree((void *)dump, hdrLen * 3 + 1);
            }
        }
#endif
    }
    return FunctionCast(newHostDeviceRequest, callbackIBTPatcher->oldHostDeviceRequest)(that, provider, request, data, descriptor, length, completion, timeout);
}

#define HCI_EVT_DISCONN_COMPLETE                      0x05
#define HCI_EVT_LE_META                               0x3E
#define HCI_EVT_LE_META_READ_REMOTE_FEATURES_COMPLETE 0x04

static void releaseCompletion(AsyncOwnerData *context)
{
    context->dataBuffer->release();
    int index = context->poolSlot;
    if (index < 0) {
        delete context;
        return;
    }
    IOSimpleLockLock(leMetaLock);
    poolBusy[index] = false;
    IOSimpleLockUnlock(leMetaLock);
}

static void asyncIOCompletion(void* owner, void* parameter, IOReturn status, uint32_t bytesTransferred)
{
    AsyncOwnerData *asyncOwner = (AsyncOwnerData *)owner;
    IOMemoryDescriptor* dataBuffer = asyncOwner->dataBuffer;
    // #514 3-stage rewrite of the repeated LE Read Remote Features Complete (0x04) events:
    //   stage 0 = the real one (pass through), 1 = rewrite to fake PHY Update Complete,
    //   2 = rewrite to fake LE Data Length Change (clears bluetoothd's LE_READY_WAIT_FOR_LENGTH).

    if (status == kIOReturnSuccess && dataBuffer && bytesTransferred >= 5 && bytesTransferred <= MAX_HCI_BUF_LEN) {
        uint8_t buffer[MAX_HCI_BUF_LEN] = {0};
        IODirection bufDir = dataBuffer->getDirection();
        bool prepared = (getKernelVersion() < KernelVersion::Sequoia) ||
                        (dataBuffer->prepare(bufDir) == kIOReturnSuccess);
        if (prepared) {
            bool valid = dataBuffer->readBytes(0, buffer, bytesTransferred) == bytesTransferred &&
                         bytesTransferred == 2U + buffer[1];
            HciEventHdr *hdr = (HciEventHdr *)buffer;
            if (valid && ((buffer[0] == 0x0e && bytesTransferred >= 5) || (buffer[0] == 0x0f && bytesTransferred == 6))) {
                IOSimpleLockLock(leMetaLock);
                if (asyncOwner->generation == generation) {
                    commandCredits = buffer[buffer[0] == 0x0e ? 2 : 3];
                    if (buffer[0] == 0x0e && bytesTransferred == 6 && buffer[3] == 5 && buffer[4] == 0x20)
                        randomAddressError = buffer[5] != 0;
                }
                IOSimpleLockUnlock(leMetaLock);
            } else if (valid && hdr->evt == HCI_EVT_DISCONN_COMPLETE && bytesTransferred == 6 && !hdr->data[0]) {
                // The link is gone: forget any duplicate events still expected for it
                clearLeMetaSlot(((uint16_t)hdr->data[1] | ((uint16_t)hdr->data[2] << 8)) & 0x0FFF, asyncOwner->generation);
            } else if (valid && bytesTransferred == 14 && hdr->evt == HCI_EVT_LE_META &&
                       hdr->data[0] == HCI_EVT_LE_META_READ_REMOTE_FEATURES_COMPLETE && hdr->data[1]) {
                clearLeMetaSlot((hdr->data[2] | (hdr->data[3] << 8)) & 0xfff, asyncOwner->generation);
            } else if (valid && bytesTransferred == 14 && !hdr->data[1] && hdr->evt == HCI_EVT_LE_META && hdr->data[0] == HCI_EVT_LE_META_READ_REMOTE_FEATURES_COMPLETE) {
                uint16_t handle = ((uint16_t)hdr->data[2] | ((uint16_t)hdr->data[3] << 8)) & 0x0FFF;
                uint8_t actionStage = 0;
                if (leMetaLock)
                    IOSimpleLockLock(leMetaLock);
                LeMetaSlot *slot = findOrAllocLeMetaSlotLocked(handle, false);
                bool twoM = CIntelBTPatcher::supportsLe2MPhy();
                if (asyncOwner->generation == generation && slot && slot->pending > 0) {
                    actionStage = slot->stage;
                    if (slot->stage == 0) {
                        slot->stage = 1;            // real features complete -> pass through
                    } else if (slot->stage == 1) {
                        slot->stage = 2;
                        slot->pending--;
                        if (!slot->pending) slot->inUse = false;
                    } else {
                        slot->pending--;
                        slot->stage = 0;
                        if (slot->pending == 0)
                            slot->inUse = false;
                    }
                }
                if (leMetaLock)
                    IOSimpleLockUnlock(leMetaLock);

                if (actionStage == 1) {             // 1st duplicate -> fake PHY Update Complete
                    uint8_t phy = twoM ? 0x02 : 0x01;
                    uint8_t localPhyEvent[8] = {0x3E, 0x06, 0x0C, hdr->data[1], hdr->data[2], hdr->data[3], phy, phy};
                    if (dataBuffer->getLength() >= sizeof(localPhyEvent)) {
                        dataBuffer->writeBytes(0, localPhyEvent, sizeof(localPhyEvent));
                        bytesTransferred = sizeof(localPhyEvent);
                    }
                } else if (actionStage == 2) {      // 2nd duplicate -> fake LE Data Length Change
                    uint8_t localDLCEvent[13] = {0x3E, 0x0B, 0x07, hdr->data[2], hdr->data[3], 0x1B, 0x00, 0x48, 0x01, 0x1B, 0x00, 0x48, 0x01};
                    if (dataBuffer->getLength() >= sizeof(localDLCEvent)) {
                        dataBuffer->writeBytes(0, localDLCEvent, sizeof(localDLCEvent));
                        bytesTransferred = sizeof(localDLCEvent);
                    }
                }
            }
            if (getKernelVersion() >= KernelVersion::Sequoia)
                dataBuffer->complete(bufDir);
        }
    }
    if (asyncOwner->action)
        asyncOwner->action(asyncOwner->owner, parameter, status, bytesTransferred);
    releaseCompletion(asyncOwner);
}

IOReturn CIntelBTPatcher::
newAsyncIO(void *that, IOMemoryDescriptor* dataBuffer, uint32_t bytesTransferred, IOUSBHostCompletion* completion, uint32_t completionTimeoutMs)
{
    AsyncOwnerData *ownerData = nullptr;
    IOUSBHostCompletion localCompletion;
    IOUSBHostCompletion *targetCompletion = completion;

    IOSimpleLockLock(leMetaLock);
    bool observe = that == _hookPipeInstance && completion && dataBuffer;
    uint64_t epoch = generation;
    int slot = -1;
    if (observe) {
        for (unsigned i = 0; i < arrsize(poolBusy); ++i)
            if (!poolBusy[i]) { poolBusy[i] = true; slot = i; break; }
    }
    IOSimpleLockUnlock(leMetaLock);
    if (observe) {
        ownerData = slot >= 0 ? &completionPool[slot] : new AsyncOwnerData;
        if (ownerData) {
            dataBuffer->retain();
            *ownerData = {completion->owner, dataBuffer, completion->action, epoch, slot};
            localCompletion.owner = ownerData;
            localCompletion.action = asyncIOCompletion;
            localCompletion.parameter = completion->parameter;
            targetCompletion = &localCompletion;
        }
    }
    IOReturn ret = FunctionCast(newAsyncIO, callbackIBTPatcher->oldAsyncIO)(that, dataBuffer, bytesTransferred, targetCompletion, completionTimeoutMs);
    if (ret != kIOReturnSuccess && ownerData)
        releaseCompletion(ownerData);
    return ret;
}

void CIntelBTPatcher::captureIntelInterruptPipe(void *that, StandardUSB::EndpointDescriptor const *descriptor, IOUSBHostDevice *device)
{
    if (!device || !descriptor)
        return;
    // Only capture pipe after IntelBluetoothFirmware has finished loading firmware and published "FirmwareLoaded" property.
    // During early boot, IntelBluetoothFirmware uses temporary pipes to upload firmware; we must never intercept those.
    if (!device->getProperty("IntelBTFirmwareGeneration") && device->getProperty("FirmwareLoaded") != kOSBooleanTrue)
        return;
    const StandardUSB::DeviceDescriptor *deviceDescriptor = device->getDeviceDescriptor();
    if (deviceDescriptor && deviceDescriptor->idVendor == VENDOR_USB_INTEL) {
        uint8_t epType = StandardUSB::getEndpointType(descriptor);
        uint8_t epDir = StandardUSB::getEndpointDirection(descriptor);
        if (epType == kIOUSBEndpointTypeInterrupt && epDir == StandardUSB::kEndpointDescriptorDirectionIn) {
            uint16_t pid = USBToHost16(deviceDescriptor->idProduct);
            IOSimpleLockLock(leMetaLock);
            CIntelBTPatcher::_hookPipeInstance = that;
            CIntelBTPatcher::_hookDeviceInstance = device;
            _randomAddressInit = false;
            _supportsLe2MPhy = (pid != 0x07dc && pid != 0x0a2a && pid != 0x0aa7 && pid != 0x0a2b);
            resetLeMetaState();
            IOSimpleLockUnlock(leMetaLock);
        }
    }
}

bool CIntelBTPatcher::detached(void *, void *, IOService *service, IONotifier *)
{
    IOSimpleLockLock(leMetaLock);
    if (_hookDeviceInstance == service) {
        _hookDeviceInstance = nullptr;
        _hookPipeInstance = nullptr;
        _randomAddressInit = false;
        resetLeMetaState();
    }
    IOSimpleLockUnlock(leMetaLock);
    return true;
}

bool CIntelBTPatcher::
newInitPipe(void *that, StandardUSB::EndpointDescriptor const *descriptor, StandardUSB::SuperSpeedEndpointCompanionDescriptor const *superDescriptor, AppleUSBHostController *controller, IOUSBHostDevice *device, IOUSBHostInterface *interface, unsigned char a7, unsigned short a8)
{
    bool ret = FunctionCast(newInitPipe, callbackIBTPatcher->oldInitPipe)(that, descriptor, superDescriptor, controller, device, interface, a7, a8);
    if (ret)
        captureIntelInterruptPipe(that, descriptor, device);
    return ret;
}

bool CIntelBTPatcher::
newInitPipeConfig(void *that, StandardUSB::EndpointDescriptor const *descriptor, StandardUSB::ConfigurationDescriptor const *configDescriptor, AppleUSBHostController *controller, IOUSBHostDevice *device, IOUSBHostInterface *interface, unsigned char a7, unsigned short a8)
{
    bool ret = FunctionCast(newInitPipeConfig, callbackIBTPatcher->oldInitPipeConfig)(that, descriptor, configDescriptor, controller, device, interface, a7, a8);
    if (ret)
        captureIntelInterruptPipe(that, descriptor, device);
    return ret;
}
