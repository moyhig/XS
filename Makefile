BUILDING_HOST_UTILS = true
include ../Makefile.common

# Taiichi (
#XSROOT=/home/jkf/brick/brickos-0.2.6.10/
#XSROOT=/home/lab4/yuasa/legOS/brick/
XSROOT=$(BRICKOS_ROOT)/
# )

# TOOLPREFIX=/usr/local/bin/h8300-hms-

UTIL=$(XSROOT)util/dll-src/
UTILOBJS=ircom.o $(UTIL)rcxtty.o $(UTIL)keepalive.o $(UTIL)lnp.o

STACKSIZE=1024
#STACKSIZE=512

OSVER = brickOS

#OPTIONS=-DDEBUG
#OPTIONS=-DLISTLIB
OPTIONS=


ifneq (,$(findstring $(OSTYPE),Darwin))
	CFLAGS += -m32 -DNQC_RCXLIB -I../util
	CXXFLAGS += $(CFLAGS) -I$(NQC_ROOT)/rcxlib -I$(NQC_ROOT)/platform
	LIBS   += -Wl,-framework -Wl,IOKit -Wl,-framework -Wl,CoreFoundation -lstdc++
	UTILOBJS += nqc_rcxlib.o \
		$(NQC_ROOT)/rcxlib/RCX_USBTowerPipe_osx.o \
		$(NQC_ROOT)/rcxlib/RCX_Pipe.o

nqc_rcxlib.o: ../util/nqc_rcxlib.cpp
	$(CXX) -o $@ -c $< $(CXXFLAGS)
endif

all:: ircom.o xs eval.lx
#all: ircom.o xs xs0 xs1 xs2 eval.lx eval1 eval2.lx

ircom.o: ircom.c
	$(CC) $(CFLAGS) -o $@ -c $< -I$(UTIL) -I$(XSROOT)include/lnp

xs0: front.c eval.c object.h include.h test.c wtobj.c errmsg.c
	$(CC) $(CFLAGS) -o xs0 -DJOINT -DONLINE -DSTACKSIZE=$(STACKSIZE) $(OPTIONS) front.c eval.c

xs1: front.c object.h include.h test.c wtobj.c errmsg.c
	$(CC) $(CFLAGS) -o xs1 $(OPTIONS) front.c

eval1: eval.c object.h test.c wtobj.c xsout1
	$(CC) $(CFLAGS) -o eval1 -DSTACKSIZE=$(STACKSIZE) $(OPTIONS) eval.c

xs2: front.c object.h include.h test.c wtobj.c
	$(CC) $(CFLAGS) -o xs2 -DRCX $(OPTIONS) front.c

front.o: front.c object.h include.h test.c wtobj.c errmsg.c
	$(CC) $(CFLAGS) -o $@ -c $< -DRCX -DIRCOM -DONLINE -I$(UTIL) -I$(XSROOT)include/lnp

xs: front.o $(UTILOBJS)
	$(CC) -m32 -o xs front.o $(UTILOBJS) $(LIBS)

eval.lx: eval.c object.h rcxtest.c
	$(TOOLPREFIX)gcc -DRCX -DIRCOM -DONLINE -DSTACKSIZE=$(STACKSIZE) $(OPTIONS) -O2 -fno-builtin -fomit-frame-pointer -Wall -I$(XSROOT)include -I$(XSROOT)include/lnp -I. -I$(XSROOT)boot/ -c eval.c -o eval.o
	$(TOOLPREFIX)ld -T $(XSROOT)boot/$(OSVER).lds -relax -L$(XSROOT)lib eval.o  -lc -lmint -lfloat -lc++ -o eval.dc1 --oformat coff-h8300 -Ttext 0xb000
	$(TOOLPREFIX)ld -T $(XSROOT)boot/$(OSVER).lds -relax -L$(XSROOT)lib eval.o  -lc -lmint -lfloat -lc++ -o eval.dc2 --oformat coff-h8300 -Ttext 0xb210
	$(TOOLPREFIX)objcopy -I coff-h8300 -O symbolsrec eval.dc1 eval.ds1
	$(TOOLPREFIX)objcopy -I coff-h8300 -O symbolsrec eval.dc2 eval.ds2
	$(XSROOT)util/makelx --display  eval.ds1 eval.ds2 eval.lx
	rm eval.o eval.ds1 eval.ds2 eval.dc1 eval.dc2

eval2.lx: eval.c object.h rcxtest.c xsout2
	$(TOOLPREFIX)gcc -DRCX -DSTACKSIZE=$(STACKSIZE) $(OPTIONS) -O2 -fno-builtin -fomit-frame-pointer -Wall -I$(XSROOT)include -I$(XSROOT)include/lnp -I. -I$(XSROOT)boot/ -c eval.c -o eval2.o
	$(TOOLPREFIX)ld -T $(XSROOT)boot/$(OSVER).lds -relax -L$(XSROOT)lib eval2.o  -lc -lmint -lfloat -lc++ -o eval2.dc1 --oformat coff-h8300 -Ttext 0xb000
	$(TOOLPREFIX)ld -T $(XSROOT)boot/$(OSVER).lds -relax -L$(XSROOT)lib eval2.o  -lc -lmint -lfloat -lc++ -o eval2.dc2 --oformat coff-h8300 -Ttext 0xb210
	$(TOOLPREFIX)objcopy -I coff-h8300 -O symbolsrec eval2.dc1 eval2.ds1
	$(TOOLPREFIX)objcopy -I coff-h8300 -O symbolsrec eval2.dc2 eval2.ds2
	$(XSROOT)util/makelx -s$(STACKSIZE) eval2.ds1 eval2.ds2 eval2.lx
	rm eval2.o eval2.ds1 eval2.ds2 eval2.dc1 eval2.dc2
