#include "eval.h"
#include "kernel_id.h"

static int nxt_output_flag = 0;
unsigned int base_time;
#ifdef OSEK
#define display_refresh display_update
#define display_status_bar() display_status_bar(0)
#define get_system_up_time() systick_get_ms()
#define msleep(X) systick_wait_ms(X)
#else
extern U32 ecrobot_systick_get_ms(void);
#define get_system_up_time() ecrobot_systick_get_ms()
#define msleep(X) dly_tsk(X)
#endif
#define beep() {}

//*****************************************************************************
// TASK			: com
// DESCRIPTION 	: Bluetooth serial communication Task handler
//*****************************************************************************

#ifdef NXT
static unsigned char trdbuf[1024];
static unsigned int trdcnt = 0;
#if 0
static unsigned char twtbuf[1024];
static unsigned int twtcnt = 0;
#endif

#ifdef TOPPERS
#ifdef IRCOM
#include "../../systask/serial.c"
#else
#include "../../systask/serial.c.orig"
#endif
#endif

#ifdef IRCOM
void
com_task(VP_INT exinf)
#ifdef OSEK
{
  int len = 0;
  unsigned char *trdbufp = &trdbuf[trdcnt];

  {
	wai_sem(COM_SEM);
	{
#if 0
	  bt_receive(trdbufp);
	  len = *trdbufp;
	  trdbufp += 2;
#else
   // len = bt_read(trdbufp, 0, 128);
	  len = ecrobot_read_bt(trdbufp, 0, 256);
#endif
	}

	if (len > 0) {
	  if (*trdbufp == CMDACK) {
		ack_flag = 1;
		trdcnt = 0;
	  } else if (*trdbufp == CMDINTERRUPT) {
		interrupt_flag = 1;
		trdcnt = 0;
	  } else
		trdcnt += len;
	}
	sig_sem(COM_SEM);
  }
#if 0 //def NXT_DEBUG
  if ((len > 0) || ack_flag || interrupt_flag) {
	sprintf(smbuf, "%d:read:%d:%02x:%d:%d      ",
			smc++, trdcnt, trdbuf[0], ack_flag, interrupt_flag);
	nxt_debug(smbuf);
  }
#endif
  ext_tsk();
}
#endif /* OSEK */
#ifdef TOPPERS
{
  serial_opn_por(CONSOLE_PORTID);
  bluetooth_connect();
  serial_ctl_por(CONSOLE_PORTID, 0); // set to RAW

  while (1) {
#if 0
	T_SERIAL_RPOR rpor;
	wai_sem(COM_SEM);
	ER ercd = serial_ref_por(CONSOLE_PORTID, &rpor); // non-blocking
	if (ercd == E_OK && rpor.reacnt) {
	  unsigned char *trdbufp = &trdbuf[trdcnt];
	  serial_rea_dat(CONSOLE_PORTID, trdbufp, 1);
	  trdcnt++;
	  if (*trdbufp == CMDACK) {
		ack_flag = 1;
		trdcnt--;
	  } else if (*trdbufp == CMDINTERRUPT) {
		interrupt_flag = 1;
		trdcnt--;
	  } else {
		trdbufp++;
	  }
	  if (rpor.reacnt > 1) {
		serial_rea_dat(CONSOLE_PORTID, trdbufp, rpor.reacnt - 1);
		trdcnt += (rpor.reacnt - 1);
	  }
	} else {
	  if (twtcnt > 0) {
		serial_wri_dat(CONSOLE_PORTID, twtbuf, twtcnt);
		twtcnt = 0;
	  }
	}
	sig_sem(COM_SEM);
#endif
	dly_tsk(500U); /* 5msec wait */
  }
}
#endif /* TOPPERS */
#endif

#ifdef IRCOM
void
nxt_input_handler()
{
  if (trdcnt > 0) {
#if 1
	int r;
  wait_rd:
	r = trdcnt;
	// wait more than two ticks to flush read buffer (at 2.5ms polling itvl.)
#ifdef OSEK
	msleep(96U);
#else
	msleep(6U);
#endif
	if (trdcnt > r) goto wait_rd;
#endif
    wai_sem(COM_SEM);
    {
      int i;
	  unsigned char *trdbufp = trdbuf;
      for (i = 0; i < trdcnt; i++) {
		rdbuf[i] = *trdbufp;
		trdbufp++;
      }
      rdbufp = rdbuf;
      rdbuf_ready = 1;
    }
	trdcnt = 0;
    sig_sem(COM_SEM);
#if 0 // def NXT_DEBUG
	  sprintf(smbuf, "%d:read:%d:%02x        ",
			  smc++, trdcnt, trdbuf[0]);
	  nxt_debug(smbuf);
#endif
  }
}

void
nxt_output_handler(unsigned char *buf, int len)
{
  if (nxt_output_flag) {
    wai_sem(COM_SEM);
    ack_flag = 0;
#ifdef OSEK
	ecrobot_send_bt(buf, 0, len);
#if 0 // def NXT_DEBUG
	sprintf(smbuf, "%d:write:%d:%d          ", smc++, len, trdcnt);
	nxt_debug(smbuf);
#endif
#else /* TOPPERS */
#if 1
	serial_wri_dat(CONSOLE_PORTID, buf, len);
#else // serial I/O should be handled on same priority?
	{
	  int i;
	  unsigned char *twtbufp = twtbuf;
	  for (i = 0; i < len; i++) {
		*twtbufp++ = buf[i];
	  }
	  *twtbufp = 0;
	  twtcnt = len;
	}
  wait_wt:
	msleep(10);		   	// wait for com_task to handle semaphore
	if (twtcnt > 0) goto wait_wt;
#endif
#endif /* TOPPERS */
    sig_sem(COM_SEM);

	{
	  int k = 20;
	  while (k--) {			// wait receive ACK from front
		if (ack_flag) {
#if 0 // def NXT_DEBUG
		  sprintf(smbuf, "%d:ack:%d,%d:%d          ",
				  smc++, k,ack_flag,interrupt_flag);
		  nxt_debug(smbuf);
#endif
		  wai_sem(COM_SEM);  	// sync till flush write buf
		  ack_flag = 0;
		  sig_sem(COM_SEM);
		  break;
		}
		msleep(12U);
	  }
	}
 // msleep(100);
  }
}
#endif /* IRCOM */

void main_task(VP_INT exinf)
{
#ifdef TOPPERS
	show_splash_screen();
	ecrobot_init_nxtstate();
	ecrobot_init_sensors();
#endif
#ifdef OSEK
	ecrobot_init_rs485(9600);
#endif
#if 0
	ecrobot_init_compass_sensor(NXT_PORT_S3);
	ecrobot_cal_compass_sensor(NXT_PORT_S3);
#endif
	show_main_screen();
	display_status_bar();

#ifdef IRCOM
#ifdef OSEK
	do {
	  char device_name[64];
	  display_goto_xy(0, 0);
	  display_string("WAIT BLUETOOTH");
	  display_goto_xy(0, 1);
	  display_string("  CONNECTION");
	  if (ecrobot_get_bt_status() > BT_NO_INIT) {
		display_goto_xy(0, 4);
		display_string(device_name);
		display_refresh();
	  }
	  ecrobot_init_bt_slave("4321");
	  systick_wait_ms(100);
	} while(ecrobot_get_bt_status() < BT_STREAM);

	display_goto_xy(0,2);
	display_string("...CONNECTED");
	display_refresh();
#endif /* OSEK */
#endif /* IRCOM */

#ifdef BALANCE
#if 0
	act_tsk(BALANCE_TASK);
	act_tsk(IR_TASK);
#else
	balance_init();
#endif
#endif

	act_tsk(CYC0_TASK);
	act_tsk(CYC1_TASK);
	act_tsk(CYC2_TASK);
	sta_cyc(CYC0);

	act_tsk(EVAL_TASK);
#ifndef NXT_DEBUG
	sprintf(smbuf, "NXT/XS/eval");
#endif
	while (1) {
#ifndef NXT_DEBUG
	  ecrobot_poll_nxtstate();
	  ecrobot_status_monitor(smbuf); /* LCD display */
#endif
	  dly_tsk(250U); /* 40msec wait */
	}
}

//*****************************************************************************
// TASK			: cyc0
// DESCRIPTION 	: 4msec periodical Task handler
//*****************************************************************************

static unsigned int c0 = 0;
static unsigned int c1 = 0;
static unsigned int c2 = 0;

#ifdef OSEK
static unsigned int cc  =    0;
static unsigned int ccc =   24/2;
#endif
static unsigned int ic0 =    8/2;
static unsigned int ic1 =  100/2;
static unsigned int ic2 = 1000/2;

static int fc0 = 0;
static int fc1 = 0;
static int fc2 = 0;

void cyc0(VP_INT exinf)
{
#if defined(IRCOM) && defined(OSEK)
  if (!(cc++ % ccc)) {
	iact_tsk(COM_TASK);
	cc = 1;
  }
#endif
  //serial_sus_por(CONSOLE_PORTID);
  if (!(c2++ % ic2)) {
	isig_sem(CYC2_SEM);
	c2 = 1;
  }
  if (!(c1++ % ic1)) {
	isig_sem(CYC1_SEM);
	c1 = 1;
  }
  if (!(c0++ % ic0)) {
	isig_sem(CYC0_SEM);
	c0 = 1;
  }
  //serial_res_por(CONSOLE_PORTID);
}

void cyc0_task(VP_INT exinf) // this task is needed by TOPPERS/JSP for pol_{sem,flag}
{
  FLGPTN ptn;
  while (1) {
    wai_sem(CYC0_SEM);
	if (pol_flg(CYC0_0_FLG, 0x01, TWF_ANDW, &ptn) == E_OK) {
	  sig_sem(CYC0_1_SEM);
	}
  }
}

int nxt_wait_cyc0(int itvl, int block)
{
  if (!fc0) {
	ic0 = itvl/4;
	set_flg(CYC0_0_FLG, 0x01);
	fc0 = 1;
  }
  if (block) {
	wai_sem(CYC0_1_SEM); // blocking
  } else {
	ER ercd = pol_sem(CYC0_1_SEM); //non-blocking
	if (ercd != E_OK)
	  return 0;
  }
  fc0 = 0;
  return 1;
}

//*****************************************************************************
// TASK			: cyc1
// DESCRIPTION 	: 100msec periodical Task handler
//*****************************************************************************

void cyc1_task(VP_INT exinf) // this task is needed by TOPPERS/JSP for pol_{sem,flag}
{
  FLGPTN ptn;
  while (1) {
    wai_sem(CYC1_SEM);
	if (pol_flg(CYC1_0_FLG, 0x01, TWF_ANDW, &ptn) == E_OK) {
	  sig_sem(CYC1_1_SEM);
	}
  }
}

int nxt_wait_cyc1(int itvl, int block)
{
  if (!fc1) {
	ic1 = itvl/4;
	set_flg(CYC1_0_FLG, 0x01);
  }
  if (block) {
	wai_sem(CYC1_1_SEM); // blocking
  } else {
	ER ercd = pol_sem(CYC1_1_SEM); //non-blocking
	if (ercd != E_OK)
	  return 0;
  }
  fc1 = 0;
  return 1;
}

//*****************************************************************************
// TASK			: cyc2
// DESCRIPTION 	: 1sec periodical Task handler
//*****************************************************************************

void cyc2_task(VP_INT exinf) // this task is needed by TOPPERS/JSP for pol_{sem,flag}
{
  FLGPTN ptn;
  while (1) {
    wai_sem(CYC2_SEM);
	if (pol_flg(CYC2_0_FLG, 0x01, TWF_ANDW, &ptn) == E_OK) {
	  sig_sem(CYC2_1_SEM);
	}
  }
}

int nxt_wait_cyc2(int itvl, int block)
{
  if (!fc2) {
	ic2 = itvl/4;
	set_flg(CYC2_0_FLG, 0x01);
	fc2 = 1;
  }
  if (block) {
	wai_sem(CYC2_1_SEM); // blocking
  } else {
	ER ercd = pol_sem(CYC2_1_SEM); //non-blocking
	if (ercd != E_OK)
	  return 0;
  }
  fc2 = 0;
  return 1;
}

#ifdef OSEK
void jsp_systick_low_priority(void)
{
  /* check whether JSP already started or not */
  if (get_OS_flag() && (ecrobot_get_bt_status() == BT_STREAM))
	{
		isig_tim();          /* cyclic task dispatcher */
		check_NXT_buttons(); /* this must be called here */
	}
}
#endif

#ifdef IRCOM
void wai_com_sem() {
  wai_sem(COM_SEM);
}

void sig_com_sem() {
  sig_sem(COM_SEM);
}
#endif

#ifdef BALANCE
#if 0
#include "balance.c"
#endif
#endif

#endif /* NXT */

// Local Variables:
// tab-width: 4
// End:
