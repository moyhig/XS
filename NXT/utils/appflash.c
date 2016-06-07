// gcc -g -m32 appflash.c -Wl,-framework -Wl,IOKit -Wl,-framework -Wl,CoreFoundation

#define FW103

#define FLASH_START_PAGE 128 /* 0x100000 + 32Kbyte(128*256) = 0x108000 */
#define MAX_NUM_FLASH_PAGES       (1023 - FLASH_START_PAGE)
#define DATA_LENGTH 	          64

#include "macusb.c"

#ifdef FW103
#include <time.h>
/*
 * calculate check sum of a data block
 *
 */
static unsigned int calc_check_sum(unsigned char* data, unsigned int len)
{
       int i;
       unsigned int sum;

       sum = 0;
       for(i = 0; i < len; i++)
       {
               sum += (unsigned int)data[i];
       }
       return sum;
}
#endif

int main(int argc, char *argv[])
{
  char *buf;
  long lsize;
  char *fw_file;
  {
    FILE *f;

    printf("#=========================================================#\n");
    printf("#    Application Flash Utility with LEGO fantom driver    #\n");
    printf("#=========================================================#\n");
    
    /* check command format */
    if (argc != 2)
      {
	printf("SYNTAX: %s <Application program(*_rom.bin)>\n"
	       "I.e. %s foo_app_rom.bin\n", argv[0], argv[0]);
	return 0;
      }
    
    /* read application file */
    fw_file = argv[1];
    f = fopen(fw_file, "rb");
    if (f == NULL)
      {
	printf("ERROR: opening file");
	return 0;
      }
    fseek(f, 0, SEEK_END);
    lsize=ftell(f);
    rewind(f);
    
    buf = (char *) malloc(lsize);
    if (buf == NULL)
      {
	printf("ERROR: allocating memory");
	fclose(f);
	return 0;
      }
    
    if (fread(buf, 1, lsize, f) != (int) lsize)
      {
	printf("ERROR: reading file");
	fclose(f);
	return 0;
      }
    fclose(f);

    printf("%s: %d\n", fw_file, lsize);
  }

  {
    if (OpenDevice(0x0694, 0x0002)) {
      fprintf(stderr, "OpenDevice()\n");
      exit(1);
    }
    if (Configure(0)) {
      fprintf(stderr, "Configure()\n");
      exit(1);
    }
    if (OpenInterface()) {
      fprintf(stderr, "OpenInterface()\n");
      exit(1);
    }
    // clear the input buffer
    fInBufferStart = fInBufferEnd = fInBuffer;
  }

#if 0
  {
    UInt8 n_endpoints = 0;
    int i;
    int in_ep = -1, out_ep = -1, ep_type;

    (*fInterface)->GetNumEndpoints(fInterface, &n_endpoints);
    for (i = 1; i <= n_endpoints; ++i) {
      UInt16 maxPacketSize;
      UInt8  direction, number, transferType,interval;
      (*fInterface)->GetPipeProperties(fInterface,
				       (UInt8) i, &direction, &number, &transferType, &maxPacketSize, &interval);
      
      switch (direction) {
      case kUSBIn:
	if (in_ep < 0) {
	  in_ep = i;
	  ep_type = transferType;
	}
	break;
      case kUSBOut:
	if (out_ep < 0) {
	  out_ep = i;
	  ep_type = transferType;
	}
	break;
      }
    }
  }
#endif

  {
    char data[DATA_LENGTH];
#ifndef FW103
    char echo_data[DATA_LENGTH];
#endif
    char wbuf[64+1];
    char rbuf[64+1];
    int i, j, len;

    wbuf[0] = 0x01;
    wbuf[1] = 0xff;
    len = Write(wbuf, 2);
 // printf("%d\n", len);

    {
      static const char ecrobot_sig[] = {'E', 'C', 'R', 'O', 'B', 'O', 'T'};
      int timeout = 100000;
      
      while (timeout-- > 0) {
	len = Read(rbuf, 64, 1000);
	if (len > 0) {
	  break;
	} else {
	  fprintf(stderr, ".");
	  fflush(stderr);
	}
      }
   // printf ("%d\n", len);
      if ((len >= sizeof(ecrobot_sig)+1) && (rbuf[0] == (char)0x02)) {
     //	printf("0x%02x", rbuf[0]);
	for (i = 0; i < sizeof(ecrobot_sig); i++ ) {
	  if (rbuf[i+1] != (char)ecrobot_sig[i]) {
	    fprintf(stderr, "NXT BIOS returns bud signature\n");
	    exit(1);
          }
       // printf("[%c]", rbuf[i+1]);
	}
     //	printf("\n");
      } else {
	fprintf(stderr, "NXT BIOS is not ready\n");
	exit(1);
      }
    }
      
    {
      unsigned short data_num;

      /* check the required number of flash pages */
      data_num = (unsigned short)(lsize/DATA_LENGTH);
      if ((lsize % DATA_LENGTH) != 0) data_num++;
      if (data_num > MAX_NUM_FLASH_PAGES * 4)
	{
	  printf("ERROR: file size is too large to upload.\n");
	  return 0;
	}
   // printf("%d\n", data_num);

      len = Write((char *)&data_num, 2);
   // printf("%d\n", len);
      
      for (i = 0; i < lsize; i += DATA_LENGTH) {
	if (lsize - i >= DATA_LENGTH) {
	  memcpy(data, &buf[i], DATA_LENGTH);
	} else {
	  memset(data, 0, DATA_LENGTH);
	  memcpy(data, &buf[i], lsize - i);
	}

#ifdef FW103
	{
	  unsigned int sum = calc_check_sum((unsigned char *)data, DATA_LENGTH);
	  if (Write((char *)data, DATA_LENGTH) == DATA_LENGTH) {
	    int rx_len = 0;
	    clock_t start = clock();
	    while (rx_len == 0) {
	      rx_len = Read((char *)data, 4, 1000);
	      if ((double)(clock() - start)/CLOCKS_PER_SEC > 5) {
		printf("ERROR: time out occrred.\n");
		goto bad;
	      }
	    }
	    if (rx_len != 4 || sum != *(unsigned int *)data) {
	      // check sum unmatched
	      printf("ERROR: failed to upload program.\n");
	      goto bad;
	    }
	  } else {
	    printf("ERROR: faild to send data.\n");
	    goto bad;	    
	  }
	}
#else
	if ((len = Write((char *)data, DATA_LENGTH)) < 0) {
	  printf("ERROR: failed to send data.\n");
	  goto bad;
	}
     //	printf("[%d", len);
	if ((len = Read((char *)echo_data, DATA_LENGTH, 1000)) < 0) {
	  printf("ERROR: failed to receive data.\n");
	  goto bad;
	}
     //	printf(":%d]", len);
     //	fflush(stdout);
	/* check sent data and echo-back data are same */
	if (memcmp(data, echo_data, DATA_LENGTH) != 0) {
	  printf("ERROR: data is corrupted.\n");
	  goto bad;
	} else {
       // fprintf(stderr, ".");
       // fflush(stderr);
	}
#endif
      }
    }
    
    printf("Successful completion of uploading %s to the NXT.\n", fw_file);
  bad:
    Close();
  }
}
