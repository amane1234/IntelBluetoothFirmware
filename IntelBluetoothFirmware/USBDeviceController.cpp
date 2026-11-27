//
//  USBHCIController.cpp
//  IntelBluetoothFirmware
//
//  Created by qcwap on 2021/5/21.
//  Copyright © 2021 zxystd. All rights reserved.
//

#include "USBDeviceController.hpp"
#include "Log.h"
#include "Hci.h"

#define super OSObject
OSDefineMetaClassAndStructors(USBDeviceController, OSObject)

#define kReadBufferSize 4096

bool USBDeviceController::
init(IOService *client, IOUSBHostDevice *dev)
{
    XYLog("%s\n", __PRETTY_FUNCTION__);
    if (!super::init()) {
        return false;
    }
    mReadBuffer = IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task
                                                              , kIODirectionIn, kReadBufferSize);
    if (!mReadBuffer) {
        XYLog("Fail to alloc read buffer\n");
        return false;
    }
    
    if (mReadBuffer->prepare(kIODirectionIn) != kIOReturnSuccess) {
        OSSafeReleaseNULL(mReadBuffer);
        return false;
    }
    m_pDevice = dev;
    m_pClient = client;
    return true;
}

bool USBDeviceController::
findInterface()
{
    XYLog("%s\n", __PRETTY_FUNCTION__);
    OSIterator* iterator = m_pDevice->getChildIterator(gIOServicePlane);
    OSObject* candidate = NULL;
    if (iterator == NULL) {
        XYLog("can not create child iterator\n");
        return false;
    }
    while((candidate = iterator->getNextObject()) != NULL) {
        IOUSBHostInterface* interfaceCandidate = OSDynamicCast(IOUSBHostInterface, candidate);
        if(interfaceCandidate != NULL) {
            XYLog("Found interface!!!\n");
            m_pInterface = interfaceCandidate;
            m_pInterface->retain();
            break;
        }
    }
    OSSafeReleaseNULL(iterator);
    if (m_pInterface == NULL) {
        return false;
    }
    if (!m_pInterface->open(m_pClient)) {
        XYLog("can not open interface\n");
        return false;
    }
    return true;
}

static void releaseRead(InterruptRead *);

void USBDeviceController::
free()
{
    XYLog("%s\n", __PRETTY_FUNCTION__);
    if (m_pBulkWritePipe) {
        m_pBulkWritePipe->abort();
        OSSafeReleaseNULL(m_pBulkWritePipe);
    }
    if (m_pBulkReadPipe) {
        m_pBulkReadPipe->abort();
        OSSafeReleaseNULL(m_pBulkReadPipe);
    }
    if (m_pInterruptReadPipe) {
        m_pInterruptReadPipe->abort();
        OSSafeReleaseNULL(m_pInterruptReadPipe);
    }
    if (cachedRead) releaseRead(cachedRead);
    OSSafeReleaseNULL(bulkBuffer);
    if (mReadBuffer) {
        mReadBuffer->complete(kIODirectionIn);
        OSSafeReleaseNULL(mReadBuffer);
    }
    if (m_pInterface) {
        if (m_pClient && m_pInterface->isOpen(m_pClient)) {
            m_pInterface->close(m_pClient);
            m_pClient = NULL;
        }
        OSSafeReleaseNULL(m_pInterface);
    }
    m_pDevice = NULL;
    super::free();
}

bool USBDeviceController::
initConfiguration()
{
    XYLog("%s\n", __PRETTY_FUNCTION__);
    uint8_t configIndex = 0;
    uint8_t configNum = m_pDevice->getDeviceDescriptor()->bNumConfigurations;
    if (configNum == 0 || configIndex >= configNum) {
        XYLog("config num error (num: %d)\n", configNum);
        return false;
    }
    const StandardUSB::ConfigurationDescriptor *configDescriptor = m_pDevice->getConfigurationDescriptor(configIndex);
    if (!configDescriptor) {
        XYLog("getConfigurationDescriptor(%d) failed\n", configIndex);
        return false;
    }
    XYLog("set configuration to %d\n", configDescriptor->bConfigurationValue);
    IOReturn ret = m_pDevice->setConfiguration(configDescriptor->bConfigurationValue);
    if (ret != kIOReturnSuccess) {
        XYLog("set device configuration to %d failed\n", configDescriptor->bConfigurationValue);
        return false;
    }
    return true;
}

bool USBDeviceController::
findPipes()
{
    XYLog("%s\n", __PRETTY_FUNCTION__);
    const StandardUSB::ConfigurationDescriptor *configDescriptor;
    const StandardUSB::InterfaceDescriptor *interfaceDescriptor;
    const EndpointDescriptor *endpointDescriptor;
    uint8_t epDirection, epType;
    
    configDescriptor = m_pInterface->getConfigurationDescriptor();
    interfaceDescriptor = m_pInterface->getInterfaceDescriptor();
    if (configDescriptor == NULL || interfaceDescriptor == NULL) {
        XYLog("Find descriptor NULL\n");
        return false;
    }
    endpointDescriptor = NULL;
    while ((endpointDescriptor = StandardUSB::getNextEndpointDescriptor(configDescriptor, interfaceDescriptor, endpointDescriptor))) {
        epDirection = StandardUSB::getEndpointDirection(endpointDescriptor);
        epType = StandardUSB::getEndpointType(endpointDescriptor);
        if (epDirection == kUSBIn && epType == kUSBInterrupt) {
            XYLog("Found Interrupt endpoint!\n");
            m_pInterruptReadPipe = m_pInterface->copyPipe(StandardUSB::getEndpointAddress(endpointDescriptor));
            if (m_pInterruptReadPipe == NULL) {
                XYLog("Copy InterruptReadPipe pipe fail\n");
                return false;
            }
        } else {
            if (epDirection == kUSBOut && epType == kUSBBulk) {
                XYLog("Found Bulk out endpoint!\n");
                m_pBulkWritePipe = m_pInterface->copyPipe(StandardUSB::getEndpointAddress(endpointDescriptor));
                if (m_pBulkWritePipe == NULL) {
                    XYLog("Copy Bulk pipe fail\n");
                    return false;
                }
            } else {
                if (epDirection == kUSBIn && epType == kUSBBulk) {
                    XYLog("Found Bulk in endpoint!\n");
                    m_pBulkReadPipe = m_pInterface->copyPipe(StandardUSB::getEndpointAddress(endpointDescriptor));
                    if (m_pBulkReadPipe == NULL) {
                        XYLog("Copy Bulk pipe fail\n");
                        return false;
                    }
                }
            }
        }
    }
    return (m_pInterruptReadPipe != NULL && m_pBulkWritePipe != NULL && m_pBulkReadPipe != NULL);
}

IOReturn USBDeviceController::
bulkPipeRead(void *buf, uint32_t buf_size, uint32_t *size, uint32_t timeout)
{
    uint32_t actualLength = 0;
    IOReturn ret = m_pBulkReadPipe->io(mReadBuffer, (uint32_t)mReadBuffer->getLength(), actualLength, timeout);
    if (ret == kIOUSBPipeStalled) {
        m_pBulkReadPipe->clearStall(true);
        ret = m_pBulkReadPipe->io(mReadBuffer, (uint32_t)mReadBuffer->getLength(), actualLength, timeout);
    }
    if (ret == kIOReturnSuccess) {
        if (buf && actualLength > buf_size) {
            return kIOReturnOverrun;
        }
        if (buf) {
            memcpy(buf, mReadBuffer->getBytesNoCopy(), min(actualLength, buf_size));
        }
        if (size) {
            *size = min(actualLength, buf_size);
        }
    } else {
        XYLog("%s failed: %s %d\n", __FUNCTION__, stringFromReturn(ret), ret);
    }
    return ret;
}

// The waiter and the USB completion own separate references. A timeout can
// return before the cancellation callback without exposing stack memory.
struct InterruptRead {
    int refs;
    IOLock *lock;
    IOBufferMemoryDescriptor *buffer;
    IOUSBHostCompletion completion;
    bool done;
    IOReturn status;
    uint32_t transferred;
};

static void releaseRead(InterruptRead *read)
{
    if (__atomic_sub_fetch(&read->refs, 1, __ATOMIC_ACQ_REL) == 0) {
        OSSafeReleaseNULL(read->buffer);
        if (read->lock)
            IOLockFree(read->lock);
        IOFree(read, sizeof(*read));
    }
}

void USBDeviceController::
interruptHandler(void *owner, void *, IOReturn status, uint32_t bytesTransferred)
{
    auto read = static_cast<InterruptRead *>(owner);
    IOLockLock(read->lock);
    read->status = status;
    read->transferred = bytesTransferred;
    read->done = true;
    IOLockWakeup(read->lock, read, true);
    IOLockUnlock(read->lock);
    releaseRead(read);
}

IOReturn USBDeviceController::
interruptPipeRead(void *buf, uint32_t capacity, uint32_t *size, uint32_t timeout)
{
    if (size)
        *size = 0;
    auto read = __atomic_exchange_n(&cachedRead, nullptr, __ATOMIC_ACQ_REL);
    if (!read) {
        read = static_cast<InterruptRead *>(IOMallocZero(sizeof(InterruptRead)));
        if (!read) return kIOReturnNoMemory;
        read->refs = 1;
        read->lock = IOLockAlloc();
        read->buffer = IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task, kIODirectionIn, kReadBufferSize);
        if (!read->lock || !read->buffer) {
            releaseRead(read);
            return kIOReturnNoMemory;
        }
        read->completion.owner = read;
        read->completion.action = interruptHandler;
    }
    read->done = false;
    __atomic_add_fetch(&read->refs, 1, __ATOMIC_RELAXED);
    uint64_t deadline;
    clock_interval_to_deadline(timeout, kMillisecondScale, &deadline);
    auto ret = m_pInterruptReadPipe->io(read->buffer, kReadBufferSize, &read->completion, 0);
    if (ret == kIOUSBPipeStalled && m_pInterruptReadPipe->clearStall(true) == kIOReturnSuccess)
        ret = m_pInterruptReadPipe->io(read->buffer, kReadBufferSize, &read->completion, 0);
    if (ret != kIOReturnSuccess) {
        releaseRead(read); // USB did not accept the transfer.
    } else {
        IOLockLock(read->lock);
        while (!read->done) {
            if (IOLockSleepDeadline(read->lock, read, deadline, THREAD_UNINT) != THREAD_AWAKENED)
                break;
        }
        bool expired = !read->done;
        ret = expired ? kIOReturnTimeout : read->status;
        if (!ret && (!read->transferred || read->transferred > kReadBufferSize ||
                     (buf && read->transferred > capacity)))
            ret = kIOReturnBadMedia;
        if (!ret) {
            if (buf)
                memcpy(buf, read->buffer->getBytesNoCopy(), read->transferred);
            if (size)
                *size = read->transferred;
        }
        IOLockUnlock(read->lock);
        if (expired)
            m_pInterruptReadPipe->abort(IOUSBHostIOSource::kAbortAsynchronous);
    }
    InterruptRead *empty = nullptr;
    if (ret || !__atomic_compare_exchange_n(&cachedRead, &empty, read, false, __ATOMIC_RELEASE, __ATOMIC_RELAXED))
        releaseRead(read);
    return ret;
}

IOReturn USBDeviceController::
sendHCIRequest(HciCommandHdr *cmd, uint32_t timeout)
{
    uint32_t actualLength;
    StandardUSB::DeviceRequest request =
    {
        .bmRequestType = makeDeviceRequestbmRequestType(kRequestDirectionOut, kRequestTypeClass, kRequestRecipientDevice),
        .bRequest = 0,
        .wValue = 0,
        .wIndex = 0,
        .wLength = (uint16_t)(HCI_COMMAND_HDR_SIZE + cmd->len)
    };
    
    auto ret = m_pDevice->deviceRequest(m_pClient, request, cmd, actualLength, timeout);
    return !ret && actualLength != request.wLength ? kIOReturnUnderrun : ret;
}

IOReturn USBDeviceController::
bulkWrite(const void *data, uint32_t length, uint32_t timeout)
{
    if (!data || !length || length > 258 || !m_pBulkWritePipe)
        return kIOReturnBadArgument;
    // The loader submits one firmware fragment and waits for its acknowledgement
    // before sending the next. Keep one writable descriptor for that sequence.
    if (!bulkBuffer)
        bulkBuffer = IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task, kIODirectionOut, 258);
    if (!bulkBuffer) return kIOReturnNoMemory;
    memcpy(bulkBuffer->getBytesNoCopy(), data, length);
    uint32_t transferred = 0;
    auto result = m_pBulkWritePipe->io(bulkBuffer, length, transferred, timeout);
    return !result && transferred != length ? kIOReturnUnderrun : result;
}

const char* USBDeviceController::
stringFromReturn(IOReturn code)
{
    return m_pDevice->stringFromReturn(code);
}
