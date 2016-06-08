#ifdef NXT
char smbuf[64];
#undef NXT_DEBUG
#ifdef NXT_DEBUG
static int smc = 0;
static int smpos = 0;
#define nxt_debug(X)							\
  { /*if ((smpos % 8) == 0) msleep(100); */		\
	display_goto_xy(0, smpos++ % 8);			\
	display_string(X);							\
	display_refresh(); }
#endif /* NXT_DEBUG */
#endif /* NXT */

// Local Variables:
// tab-width: 4
// End:
