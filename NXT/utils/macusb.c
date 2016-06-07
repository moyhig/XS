#include "macusb.h"

static IOUSBDeviceInterface**		fDevice;
static IOUSBInterfaceInterface182**	fInterface;

IOReturn OpenDevice(short vendorID, short productID)
{
  mach_port_t 		 masterPort;
  CFMutableDictionaryRef matchingDict;
  kern_return_t		 err;
  io_object_t            usbDevice;

  // first create a master_port for my task
  err = IOMasterPort(MACH_PORT_NULL, &masterPort);
  PREQUIRENOT(err, Fail_IOMasterPort);
  //PREQUIRE(masterPort, Fail_IOMasterPort);
  
  // Set up the matching criteria for the devices we're interested in
  // Interested in instances of class IOUSBDevice and its subclasses
  matchingDict = IOServiceMatching(kIOUSBDeviceClassName);
  //PREQUIRE(matchingDict, Fail_IOServiceMatching);

  CFDictionarySetValue(matchingDict, CFSTR(kUSBVendorID),
		       CFNumberCreate(kCFAllocatorDefault,
				      kCFNumberShortType, &vendorID)); 
  CFDictionarySetValue(matchingDict, CFSTR(kUSBProductID), 
		       CFNumberCreate(kCFAllocatorDefault,
				      kCFNumberShortType, &productID)); 

  io_iterator_t iter;
  err = IOServiceGetMatchingServices(masterPort, matchingDict, &iter);
  mach_port_deallocate(mach_task_self(), masterPort);
  PREQUIRENOT(err, Fail_IOServiceGetMatchingServices);
	
  err = -1;
  usbDevice = IOIteratorNext(iter);
  IOObjectRelease(iter);
  PREQUIRE(usbDevice, Fail_NoMatch);

  err = GetInterfaceForService(usbDevice,
			       kIOUSBDeviceUserClientTypeID,
			       kIOUSBDeviceInterfaceID, &fDevice);
IOObjectRelease(usbDevice);
  PREQUIRENOT(err, Fail_CreateInterface);
		
  // need to open the device in order to change its state
  err = (*fDevice)->USBDeviceOpen(fDevice);
  PREQUIRENOT(err, Fail_USBDeviceOpen);

  return 0;
	
 Fail_USBDeviceOpen:
  (*fDevice)->Release(fDevice);
  fDevice = 0;
 Fail_CreateInterface:
 Fail_NoMatch:
 Fail_IOServiceGetMatchingServices:
  //Fail_IOServiceMatching:
 Fail_IOMasterPort:
  return err;
}

IOReturn Configure(int index)
{
  IOReturn err;
  IOUSBConfigurationDescriptorPtr	confDesc;
  
  // get the configuration descriptor
  err = (*fDevice)->GetConfigurationDescriptorPtr(fDevice, index, &confDesc);
  PREQUIRENOT(err, Fail_GetConfiguration);
  
  err = (*fDevice)->SetConfiguration(fDevice, confDesc->bConfigurationValue);
  PREQUIRENOT(err, Fail_SetConfiguration);
  
  return 0;
  
 Fail_SetConfiguration:
 Fail_GetConfiguration:
  return err;
}

IOReturn GetInterfaceForService(io_service_t service, CFUUIDRef clientID, CFUUIDRef interfaceID, void** interface)
{
  IOCFPlugInInterface **plugInInterface;
  IOReturn err;
  SInt32 score;
	
  err = IOCreatePlugInInterfaceForService(service,
					  clientID, kIOCFPlugInInterfaceID,
					  &plugInInterface, &score);
  PREQUIRENOT(err, Fail_CreatePlugin);

  // I have the device plugin, I need the device interface
  err = (*plugInInterface)->QueryInterface(plugInInterface,
					   CFUUIDGetUUIDBytes(interfaceID), interface);
  (*plugInInterface)->Release(plugInInterface);			// done with this
  PREQUIRENOT(err, Fail_QueryInterface);
  
  return 0;

Fail_QueryInterface:
Fail_CreatePlugin:
  *interface = 0;
 return err;
}

IOReturn OpenInterface() 
{
  IOReturn err;
  IOUSBFindInterfaceRequest request;
  io_iterator_t		    iterator;
  io_service_t		    usbInterface;
  
  request.bInterfaceClass =    kIOUSBFindInterfaceDontCare;
  request.bInterfaceSubClass = kIOUSBFindInterfaceDontCare;
  request.bInterfaceProtocol = kIOUSBFindInterfaceDontCare;
  request.bAlternateSetting =  kIOUSBFindInterfaceDontCare;
   
  err = (*fDevice)->CreateInterfaceIterator(fDevice, &request, &iterator);
  PREQUIRENOT(err, Fail_IterateInterface);
	
  usbInterface = IOIteratorNext(iterator);
  IOObjectRelease(iterator);
  //PREQUIRE(usbInterface, Fail_NoInterface);
	
  err = GetInterfaceForService(usbInterface,
			       kIOUSBInterfaceUserClientTypeID, kIOUSBInterfaceInterfaceID, &fInterface);
  IOObjectRelease(usbInterface);
  PREQUIRENOT(err, Fail_GetInterface);
		
  err = (*fInterface)->USBInterfaceOpen(fInterface);
  PREQUIRENOT(err, Fail_OpenInterface);

  CFRunLoopSourceRef source;
  err = (*fInterface)->CreateInterfaceAsyncEventSource(fInterface, &source);
  CFRunLoopAddSource(CFRunLoopGetCurrent(), source, kCFRunLoopDefaultMode);

  return 0;
	
 Fail_OpenInterface:
  (void) (*fInterface)->Release(fInterface);
  fInterface  = 0;
 Fail_GetInterface:
 Fail_IterateInterface:
  return err;
}

void Close()
{
  if (fInterface)
    {
      (*fInterface)->USBInterfaceClose(fInterface);
      (*fInterface)->Release(fInterface);
      fInterface = 0;
    }
  
  if (fDevice)
    {
      (*fDevice)->USBDeviceClose(fDevice);
      (*fDevice)->Release(fDevice);
      fDevice = 0;
    }
}

#define kReadPipe 2
#define kWritePipe 1
#define MAX_PACKET 200

long Write(const void *ptr, long length)
{
  const unsigned char *data = (const unsigned char *)ptr;
	
  int total = 0;
	
  while(length > 0)
    {
      IOReturn err;
      int count = length;
      if (count > MAX_PACKET) count = MAX_PACKET;
      err = (*fInterface)->WritePipe(fInterface, kWritePipe, (void*)data, count);
      PREQUIRENOT(err, Fail_WritePipe);
      
      length -= count;
      data += count;
      total += count;
    }
  
 Fail_WritePipe:
  return total;
}

#define kReadPacketSize 64

static unsigned char	fInBuffer[kReadPacketSize];
static unsigned char*	fInBufferStart;
static unsigned char*	fInBufferEnd;

static unsigned char*	fReadPtr;
static int		fReadRemain;
static bool		fReadDone;

#if 0
long Read(const void *ptr, long length, int x)
{
  int size;
  err = (*fInterface)->ReadPipe(fInterface, kReadPipe, (void*)data, &size);
  return size;
}


#else

void ConsumeInBuffer()
{
  while(fReadRemain && (fInBufferStart < fInBufferEnd))
    {
      *fReadPtr++ = *fInBufferStart++;
      fReadRemain--;	
    }
  
  if (fReadRemain==0) fReadDone = true;
}

void StartRead();

void ReadCompletion(IOReturn result, UInt32 n)
{
  if (result == kIOReturnAborted) return;
	
  PREQUIRENOT(result, Fail_ReadCompletion);

  // consume input buffer	
  fInBufferEnd = fInBuffer + n;
  ConsumeInBuffer();
  
  if (!fReadDone)
    {
      StartRead();
    }
  
  return;
  
 Fail_ReadCompletion:
  fReadDone = true;
}

void ReadCompletionGlue(void *refCon, IOReturn result, void *arg0)
{
  ReadCompletion(result, (UInt32)arg0);
}

void StartRead()
{
  // clear the input buffer
  fInBufferStart = fInBufferEnd = fInBuffer;
  
  IOReturn err = (*fInterface)->ReadPipeAsync(fInterface, kReadPipe, fInBuffer, kReadPacketSize, ReadCompletionGlue, NULL);
  PREQUIRENOT(err, Fail_ReadPipe);
  return;		
  
 Fail_ReadPipe:
  fReadDone = true;
  return;
}


long Read(void *data, long length, long timeout_ms)
{
  fReadPtr = (unsigned char *)data;
  fReadRemain = length;
  fReadDone = false;
  
  // consume any previously buffered data
  ConsumeInBuffer();
  
  if (!fReadDone)
    {
      StartRead();
      do
	{
	  SInt32 reason = CFRunLoopRunInMode(kCFRunLoopDefaultMode, timeout_ms / 1000.0, true);
	  if (reason == kCFRunLoopRunTimedOut)
	    {
	      (*fInterface)->AbortPipe(fInterface, kReadPipe);
	      fReadDone = true;
	    }
	} while(!fReadDone);
    }
  
  return fReadPtr - (unsigned char *)data;
}
#endif
