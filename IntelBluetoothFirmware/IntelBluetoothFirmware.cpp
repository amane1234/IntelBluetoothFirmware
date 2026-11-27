//
//  IntelBluetoothFirmware.cpp
//  IntelBluetoothFirmware
//
//  Created by zxystd on 2019/11/17.
//  Copyright © 2019 zxystd. All rights reserved.
//

#include "BtIntel.h"

#include "IntelBluetoothFirmware.hpp"
#include <libkern/libkern.h>
#include <libkern/OSKextLib.h>
#include <libkern/version.h>
#include <libkern/OSTypes.h>
#include <IOKit/usb/StandardUSB.h>
#include "Hci.h"
#include "linux.h"
#include "Log.h"
#include <kern/thread.h>

#include <IOKit/IOMessage.h>
#include <IOKit/pwr_mgt/RootDomain.h>

#include "IntelBluetoothOpsGen1.hpp"
#include "IntelBluetoothOpsGen2.hpp"
#include "IntelBluetoothOpsGen3.hpp"

#define super IOService
OSDefineMetaClassAndStructors(IntelBluetoothFirmware, IOService)

//com.apple.iokit.IOBluetoothHostControllerUSBTransport

enum { kMyOffPowerState = 0, kMyOnPowerState = 1 };

#define kIOPMPowerOff 0

static IOPMPowerState myTwoStates[2] =
{
    {1, kIOPMPowerOff, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    // inputPowerRequirement kIOPMLowPower (antes kIOPMPowerOn): un hijo de energia que exige kIOPMPowerOn
    // desactiva la suspension por inactividad del IOUSBHostDevice (IOUSBHostFamily.h), y el controlador
    // nunca se suspendia: el paquete de la CPU no bajaba de PC3.
    {1, kIOPMPowerOn, kIOPMPowerOn, kIOPMLowPower, 0, 0, 0, 0, 0, 0, 0, 0}
};

static IOReturn capabilityChanged(void *target, void *, UInt32 message, IOService *, void *arg, vm_size_t)
{
    if (message == kIOMessageSystemCapabilityChange && arg) {
        const auto *change = static_cast<IOPMSystemCapabilityChangeParameters *>(arg);
        if ((change->changeFlags & kIOPMSystemCapabilityWillChange) &&
            !(change->toCapabilities & kIOPMSystemCapabilityGraphics))
            static_cast<IntelBluetoothFirmware *>(target)->updateCapabilities(change->toCapabilities);
        if (change->changeFlags & kIOPMSystemCapabilityDidChange) {
            auto service = static_cast<IntelBluetoothFirmware *>(target);
            service->updateCapabilities(change->toCapabilities);
            const char *phase = "Sleep";
            if (change->toCapabilities & kIOPMSystemCapabilityCPU)
                phase = change->toCapabilities & kIOPMSystemCapabilityGraphics ? "FullWake" : "DarkWake";
            service->setProperty("IntelBTSystemWakeState", phase);
        }
    }
    return kIOReturnSuccess;
}

bool IntelBluetoothFirmware::init(OSDictionary *dictionary)
{
    XYLog("Driver init()\n");
    if (!super::init(dictionary)) return false;
    stateLock = IOLockAlloc();
    return stateLock != nullptr;
}

void IntelBluetoothFirmware::free() {
    XYLog("Driver free()\n");
    if (stateLock) IOLockFree(stateLock);
    super::free();
}

// Camino de error comun de start(): IOKit solo llama a stop() si start() tuvo exito,
// asi que aqui hay que deshacer PMinit y cerrar el dispositivo nosotros.
bool IntelBluetoothFirmware::fail()
{
    if (wakeNotifier) { wakeNotifier->remove(); wakeNotifier = nullptr; }
    cleanUp();
    if (m_pmStarted) {
        PMstop();
        m_pmStarted = false;
    }
    return false;
}

bool IntelBluetoothFirmware::start(IOService *provider)
{
    XYLog("Driver Start()\n");
    char fwName[64];
    m_pDevice = OSDynamicCast(IOUSBHostDevice, provider);
    if (m_pDevice == NULL) {
        XYLog("Driver Start fail, not usb device\n");
        return false;
    }
    m_pDevice->setProperty("IntelBTFirmwareGeneration", generation, 64);
    m_pDevice->setProperty("FirmwareLoaded", false);
    wakeNotifier = registerPrioritySleepWakeInterest(capabilityChanged, this);
    if (!wakeNotifier) return fail();
    PMinit();
    m_pmStarted = true;
    registerPowerDriver(this, myTwoStates, 2);
    provider->joinPMtree(this);
    makeUsable();
    
    if (!super::start(provider))
        return fail();
    
    if (!m_pDevice->open(this)) {
        XYLog("start fail, can not open device\n");
        return fail();
    }
    if (currentType == kTypeGen1) {
        m_pBTIntel = new IntelBluetoothOpsGen1();
    } else {
        m_pBTIntel = new IntelBluetoothOpsGen3();
    }
    if (!m_pBTIntel || !m_pBTIntel->initWithDevice(this, m_pDevice)) {
        XYLog("start fail, can not init device\n");
        return fail();
    }
    XYLog("BT init succeed\n");
    if (!m_pBTIntel->setup()) {
        m_pDevice->setProperty("FirmwareLoaded", false);
        return fail();
    }
    if (currentType == kTypeGen1) {
        IntelVersion baseline {};
        if (!m_pBTIntel->readVersion(&baseline) || baseline.hw_platform != 0x37)
            return fail();
        gen1Baseline = baseline;
    }
    m_pBTIntel->getFirmwareName(fwName, sizeof(fwName));
    // Close the firmware pipes and the device before allowing the host to submit its
    // first command: FirmwareLoaded is what lets the HCI hook pass host commands.
    IOUSBHostDevice *dev = m_pDevice;
    cleanUp();
    IOLockLock(stateLock);
    started = true;
    publishReg(dev, !needsVerify && !terminating, fwName);
    scheduleVerification(dev);
    IOLockUnlock(stateLock);
    return true;
}

void IntelBluetoothFirmware::publishReg(IOUSBHostDevice *dev, bool isSucceed, const char *fwName)
{
    // Tras un reinicio sin cortar la alimentacion el chip conserva el firmware y no se descarga
    // nada: el nombre queda vacio.
    if (fwName)
        setProperty("fw_name", fwName[0] ? fwName : "(already running)");
    // Monterey+
    if (version_major >= 21)
        dev->setName("Bluetooth USB Host Controller");
    dev->setProperty("FirmwareLoaded", isSucceed);
}

void IntelBluetoothFirmware::cleanUp()
{
    XYLog("Clean up...\n");
    OSSafeReleaseNULL(m_pBTIntel);
    if (m_pDevice) {
        if (m_pDevice->isOpen(this)) {
            m_pDevice->close(this);
        }
        m_pDevice = NULL;
    }
}

struct IntelBluetoothFirmware::WakeJob {
    IntelBluetoothFirmware *service;
    IOUSBHostDevice *dev;
    uint64_t generation;
};

// Called with stateLock held. USB transactions must run outside PM and port workloops.
void IntelBluetoothFirmware::scheduleVerification(IOUSBHostDevice *dev)
{
    if (!dev || !started || terminating || !needsVerify || !fullWake || pending ||
        lastPowerState == kMyOffPowerState)
        return;
    auto job = static_cast<WakeJob *>(IOMalloc(sizeof(WakeJob)));
    if (!job) return;
    *job = {this, dev, generation};
    retain();
    dev->retain();
    pending = true;
    thread_t thread;
    if (kernel_thread_start(verifyWake, job, &thread) == KERN_SUCCESS) {
        thread_deallocate(thread);
        return;
    }
    pending = false;
    dev->release();
    release();
    IOFree(job, sizeof(*job));
}

void IntelBluetoothFirmware::verifyWake(void *context, wait_result_t)
{
    auto job = static_cast<WakeJob *>(context);
    auto self = job->service;
    auto dev = job->dev;
    bool verified = false;
    bool current = false;
    for (unsigned attempt = 0; attempt < 3; ++attempt) {
        IOLockLock(self->stateLock);
        current = !self->terminating && self->fullWake && self->generation == job->generation;
        IOLockUnlock(self->stateLock);
        if (!current || dev->isInactive()) break;
        if (dev->open(self)) {
            auto ops = new IntelBluetoothOpsGen3();
            if (ops && ops->initWithDevice(self, dev, false)) {
                if (self->currentType == kTypeGen1) {
                    IntelVersion ver {};
                    verified = ops->readVersion(&ver) &&
                               !memcmp(&ver, &self->gen1Baseline, sizeof(ver));
                } else {
                    verified = ops->isOperational();
                }
            }
            OSSafeReleaseNULL(ops);
            dev->close(self);
        }
        if (verified) break;
        if (attempt < 2) IOSleep(100U << attempt);
    }
    IOLockLock(self->stateLock);
    current = !self->terminating && self->generation == job->generation && self->fullWake;
    if (current && verified) {
        self->needsVerify = false;
        dev->setProperty("FirmwareLoaded", true);
    }
    IOLockUnlock(self->stateLock);
    // A busy host interface cannot be borrowed. A new USB attachment runs the
    // normal loader, including firmware download if power was lost.
    if (current && !verified && !dev->isInactive()) {
        auto ret = dev->reset();
        if (ret) XYLog("Wake re-enumeration failed: %d\n", ret);
    }
    IOLockLock(self->stateLock);
    self->pending = false;
    if (self->generation != job->generation)
        self->scheduleVerification(dev);
    IOLockUnlock(self->stateLock);
    dev->release();
    self->release();
    IOFree(job, sizeof(*job));
}

void IntelBluetoothFirmware::updateCapabilities(uint32_t caps)
{
    IOLockLock(stateLock);
    auto dev = OSDynamicCast(IOUSBHostDevice, getProvider());
    bool full = (caps & kIOPMSystemCapabilityGraphics) && (caps & kIOPMSystemCapabilityCPU);
    if (fullWake != full) ++generation;
    fullWake = full;
    if (dev && !terminating && !(caps & kIOPMSystemCapabilityCPU) && !needsVerify) {
        ++generation;
        dev->setProperty("IntelBTFirmwareGeneration", generation, 64);
        needsVerify = true;
        dev->setProperty("FirmwareLoaded", false);
    }
    scheduleVerification(dev);
    IOLockUnlock(stateLock);
}

IOReturn IntelBluetoothFirmware::setPowerState(unsigned long ordinal, IOService *what)
{
    if (what != this || ordinal > kMyOnPowerState) return IOPMAckImplied;
    IOLockLock(stateLock);
    auto dev = OSDynamicCast(IOUSBHostDevice, getProvider());
    if (dev && ordinal != lastPowerState && !terminating) {
        bool asleep = ordinal == kMyOffPowerState;
        lastPowerState = ordinal;
        dev->setProperty("IntelBTTransportAsleep", asleep);
        scheduleVerification(dev);
    }
    IOLockUnlock(stateLock);
    return IOPMAckImplied;
}

bool IntelBluetoothFirmware::willTerminate(IOService *provider, IOOptionBits options)
{
    IOLockLock(stateLock);
    terminating = true;
    ++generation;
    provider->setProperty("FirmwareLoaded", false);
    IOLockUnlock(stateLock);
    return super::willTerminate(provider, options);
}

void IntelBluetoothFirmware::stop(IOService *provider)
{
    XYLog("Driver Stop()\n");
    IOLockLock(stateLock);
    terminating = true;
    ++generation;
    IOLockUnlock(stateLock);
    if (m_pmStarted) {
        PMstop();
        m_pmStarted = false;
    }
    if (wakeNotifier) {
        wakeNotifier->remove();
        wakeNotifier = nullptr;
    }
    provider->setProperty("FirmwareLoaded", false);
    super::stop(provider);
}

IOService * IntelBluetoothFirmware::probe(IOService *provider, SInt32 *score)
{
    XYLog("Driver Probe()\n");
    if (!super::probe(provider, score)) {
        XYLog("super probe failed\n");
        return NULL;
    }
    m_pDevice = OSDynamicCast(IOUSBHostDevice, provider);
    if (!m_pDevice) {
        XYLog("is not usb device\n");
        return NULL;
    }
    UInt16 vendorID = USBToHost16(m_pDevice->getDeviceDescriptor()->idVendor);
    UInt16 productID = USBToHost16(m_pDevice->getDeviceDescriptor()->idProduct);
    XYLog("name=%s, class=%s, vendorID=0x%04X, productID=0x%04X\n", m_pDevice->getName(), provider->metaClass->getClassName(), vendorID, productID);
    if (productID == 0x07dc || productID == 0x0a2a || productID == 0x0aa7) {
        currentType = kTypeGen1;
    } else if (productID == 0x0032 || productID == 0x0033 || productID == 0x0035
               || productID == 0x0036 || productID == 0x0038) {
        currentType = kTypeGen3;
    } else {
        currentType = kTypeGen2;
    }
    m_pDevice = NULL;
    return this;
}
