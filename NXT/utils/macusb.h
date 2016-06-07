#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/IOKitLib.h>
#include <IOKit/IOCFPlugIn.h>
#include <IOKit/usb/IOUSBLib.h>
#include <IOKit/IOReturn.h>

#define PREQUIRE(expr, label)		do { if (!(expr)) goto label; } while (0)
#define PREQUIRENOT(expr, label)	do { if (expr) goto label; } while (0)
