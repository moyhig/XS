# Taiichi (
#XSROOT=/home/jkf/brick/brickos-0.2.6.10/
XSROOT=/home/lab4/yuasa/legOS/brick/
# )

TOOLPREFIX=/usr/local/bin/h8300-hms-

UTIL=$(XSROOT)util/dll-src/
UTILOBJS=ircom.o $(UTIL)rcxtty.o $(UTIL)keepalive.o $(UTIL)lnp.o

STACKSIZE=1024
#STACKSIZE=512

OSVER = brickOS

#OPTIONS=-DDEBUG
#OPTIONS=-DLISTLIB
OPTIONS=

all: ircom.o xs xs0 xs1 xs2 eval.lx eval1 eval2.lx

ircom.o: ircom.c
	gcc -o ircom.o -c ircom.c -O2 -Wall -O2 -Wall -I$(UTIL) -I$(XSROOT)include/lnp

xs0: front.c eval.c object.h include.h test.c wtobj.c errmsg.c
	gcc -o xs0 -DJOINT -DONLINE -DSTACKSIZE=$(STACKSIZE) $(OPTIONS) front.c eval.c

xs1: front.c object.h include.h test.c wtobj.c errmsg.c
	gcc -o xs1 $(OPTIONS) front.c

eval1: eval.c object.h test.c wtobj.c xsout1
	gcc -o eval1 -DSTACKSIZE=$(STACKSIZE) $(OPTIONS) eval.c

xs2: front.c object.h include.h test.c wtobj.c
	gcc -o xs2 -DRCX $(OPTIONS) front.c

xs: front.c object.h include.h test.c wtobj.c errmsg.c $(UTILOBJS)
	gcc -o xs -DRCX -DIRCOM -DONLINE $(OPTIONS) front.c $(UTILOBJS) -O2 -Wall -I. -I$(XSROOT)include/lnp -I$(UTIL)

eval.lx: eval.c object.h rcxtest.c
	$(TOOLPREFIX)gcc -DRCX -DIRCOM -DONLINE -DSTACKSIZE=$(STACKSIZE) $(OPTIONS) -O2 -fno-builtin -fomit-frame-pointer -Wall -I$(XSROOT)include -I$(XSROOT)include/lnp -I. -I$(XSROOT)boot/ -c eval.c -o eval.o
	$(TOOLPREFIX)ld -T $(XSROOT)boot/$(OSVER).lds -relax -L$(XSROOT)lib eval.o  -lc -lmint -lfloat -lc++ -o eval.ds1 -Ttext 0xb000
	$(TOOLPREFIX)ld -T $(XSROOT)boot/$(OSVER).lds -relax -L$(XSROOT)lib eval.o  -lc -lmint -lfloat -lc++ -o eval.ds2 -Ttext 0xb210
	$(XSROOT)util/makelx --display  eval.ds1 eval.ds2 eval.lx
	rm eval.o eval.ds1 eval.ds2

eval2.lx: eval.c object.h rcxtest.c xsout2
	$(TOOLPREFIX)gcc -DRCX -DSTACKSIZE=$(STACKSIZE) $(OPTIONS) -O2 -fno-builtin -fomit-frame-pointer -Wall -I$(XSROOT)include -I$(XSROOT)include/lnp -I. -I$(XSROOT)boot/ -c eval.c -o eval2.o
	$(TOOLPREFIX)ld -T $(XSROOT)boot/$(OSVER).lds -relax -L$(XSROOT)lib eval2.o  -lc -lmint -lfloat -lc++ -o eval2.ds1 -Ttext 0xb000
	$(TOOLPREFIX)ld -T $(XSROOT)boot/$(OSVER).lds -relax -L$(XSROOT)lib eval2.o  -lc -lmint -lfloat -lc++ -o eval2.ds2 -Ttext 0xb210
	$(XSROOT)util/makelx -s$(STACKSIZE) eval2.ds1 eval2.ds2 eval2.lx
	rm eval2.o eval2.ds1 eval2.ds2
