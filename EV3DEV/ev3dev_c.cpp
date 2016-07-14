// g++ -std=c++0x -DEV3DEV_PLATFORM_BRICKPI ev3dev_c.cpp -lev3dev -Lbuild
// g++ -std=c++0x -DEV3DEV_PLATFORM_BRICKPI -c ev3dev_c.cpp -lev3dev -L../../ev3dev-lang-cpp/build/ -I../../ev3dev-lang-cpp

#include "ev3dev.h"
#include "Kalman.h"

#include <unistd.h>

using namespace ev3dev;

#include "ev3dev_c.h"

static nxt_motor *m[4];
static lego_port *s[4];
static ht_gyro_sensor *g;
static ht_accel_sensor *a;

void ev3dev_init()
{
  int i;

  m[0] = new nxt_motor(OUTPUT_A);
  m[1] = new nxt_motor(OUTPUT_B);
  m[2] = new nxt_motor(OUTPUT_C);
  m[3] = new nxt_motor(OUTPUT_D);

  for (i = 0; i < 4; i++) {
    m[i]->reset();
  }

  s[0] = new lego_port(INPUT_1);
  s[1] = new lego_port(INPUT_2);
  s[2] = new lego_port(INPUT_3);
  s[3] = new lego_port(INPUT_4);

  s[0]->set_mode("nxt-analog");
  usleep(50*1000);
  s[0]->set_set_device("ht-nxt-gyro");
  g = new ht_gyro_sensor();
  //printf("%s\n", g->_path);

  s[1]->set_mode("nxt-i2c");
  usleep(50*1000);
  s[1]->set_set_device("ht-nxt-accel 0x01");
  a = new ht_accel_sensor();
  usleep(100*1000);
  a->set_mode("ALL");
  //printf("mode: %s\n", a->mode());
}

void
ev3dev_motor_set_speed(int port, int speed)
{
  if (speed > 100) speed = 100;
  if (speed < -100) speed = -100;
  m[port-1]->set_speed_sp((int)((speed/100.0)*1020));
  if (speed == 0) {
    m[port-1]->stop();
  } else {
    m[port-1]->run_forever();
  }
}

int
ev3dev_motor_get_count(int port)
{
  return m[port-1]->position();
}

static int gyro_offset = 0;
static int gyro_angle = 0;
static float gyro_angle_pos = 0;
static float gyro_angle_neg = 0;

int
ev3dev_gyro(int port)
{
  return g->value(0) - gyro_offset;
}

#if 1
#define accel_v(i) ((int)r[i])

#include <math.h>

main ()
{
  ev3dev_init();

  //m[0]->set_speed_sp(100);
  //m[0]->run_forever();

  //s[1]->set_mode("nxt-analog");
  //s[1]->set_set_device("ht-nxt-gyro");

#if 0
  // E([gv]) = 0.00006
  {
    int c = 0;
    float gvm[1000];
    float gsum = 0;
    gyro_offset = g->value(0);
    while (1) {
      gvm[c % 300] = (float)(g->value(0)-gyro_offset)/20.0f;
      gsum += gvm[c++];
      if (c % 300 == 0) {
	float gave = gsum / 300.0f;
	gsum = 0;
	for (c = 0; c < 300 ; c++) {
	  gsum += sqrt((gvm[c]-gave)*(gvm[c]-gave));
	}
	printf("%f, %f\n", gave, gsum / 300.0f);
	gsum = 0;
	c = 0;
      }
      usleep(32*1000);
    }
  }
#endif

#if 0
  // E([theta]) = 0.07
  {
    int c = 0;
    float gvm[1000];
    float gsum = 0;
    while (1) {
      int8_t *r = (int8_t *)((a->bin_data()).data());
      int az = ((int)r[0] << 2) + ((int)r[3] & 0x03);
      int ax = ((int)r[1] << 2) + ((int)r[4] & 0x03);
      int ay = ((int)r[2] << 2) + ((int)r[5] & 0x03);
      
      float theta = atan2(ax, az) * 360.0f/(2.0f*M_PI);
      gvm[c % 300] = theta;
      gsum += gvm[c++];
      if (c % 300 == 0) {
	float gave = gsum / 300.0f;
	gsum = 0;
	for (c = 0; c < 300 ; c++) {
	  gsum += sqrt((gvm[c]-gave)*(gvm[c]-gave));
	}
	printf("%f, %f\n", gave, gsum / 300.0f);
	gsum = 0;
	c = 0;
      }
      usleep(32*1000);
    }
  }
#endif


#if 1
  Kalman Kf;
  float kTheta = 0;
#if 0
  bool use_Kf = false;
#else
  bool use_Kf = true;
#endif
  {
    int c = 0;
    float theta = 0.0f;
    while (1) {
      int8_t *r = (int8_t *)((a->bin_data()).data());
      int az = ((int)r[0] << 2) + ((int)r[3] & 0x03);
      int ax = ((int)r[1] << 2) + ((int)r[4] & 0x03);
      int ay = ((int)r[2] << 2) + ((int)r[5] & 0x03);
      
      //float radius = sqrt(ax*ax+az*az);
      //float theta = atan2(ax/radius, az/radius) * (360/(2*M_PI));
      float theta = atan2(ax, az) * 360.0f/(2.0f*M_PI);
      printf("%f: %d, %d\n", theta, ax, az);
      if ((c++ > 10) && (theta < 1.0f) && (theta > -1.0f))
	break;
      usleep(32*1000);
    }
    Kf.setAngle(theta);
    Kf.setQangle(0.0001f);
    Kf.setQbias(0.0001f);
    Kf.setRmeasure(0.1f);
  }
#endif

  gyro_offset = g->value(0);

  m[0]->reset();
  m[0]->set_speed_sp(400);
  m[0]->set_position_sp(400);
  m[0]->run_to_abs_pos();

#if 0
  while (1) {
    int a1 = ev3dev_motor_get_count(1);
    if (a1 > 350) {
      m[0]->set_position_sp(-400);
      m[0]->run_to_abs_pos();
      break;
    }
  }
#endif

  {
    int i = 60;
    int a0 = 0;
    int a1 = 0;
    int pc = 0;
    int nc = 0;

    while (1) {
      a1 = ev3dev_motor_get_count(1);
      if (a1 > 350) {
	m[0]->set_position_sp(-400);
	m[0]->run_to_abs_pos();
      } else if (a1 < -350) {
	m[0]->set_position_sp(400);
	m[0]->run_to_abs_pos();
      }

      {
	int gv = g->value(0) - gyro_offset;
	float gv0 = ((gv > 0) ? (gv /20.0f) : (gv /17.0f));

	if (gv > 0) {
	  gyro_angle_pos += gv /20.0f; // * 0.7081;
	  pc++;
	} else {
	  gyro_angle_neg -= gv /17.0f; // / 0.7081;
	  nc++;
	}

	{
	  int8_t *r = (int8_t *)((a->bin_data()).data());
	  int az = ((int)r[0] << 2) + ((int)r[3] & 0x03);
	  int ax = ((int)r[1] << 2) + ((int)r[4] & 0x03);
	  int ay = ((int)r[2] << 2) + ((int)r[5] & 0x03);
	  
	  //float radius = sqrt(ax*ax+az*az);
	  //float theta = atan2(ax/radius, az/radius) * (360/(2*M_PI));
	  float theta = atan2(ax, az) * 360.0f/(2.0f*M_PI);

	  if (use_Kf)
	    kTheta = Kf.getAngle(theta, gv0 / 0.032f, 0.032f);
#if 0
	  else {
	    if ((pc > 30) && (nc > 30) && (theta < 1.0f) && (theta > -1.0f)) {
	      Kf.setAngle(theta);
	      use_Kf = true;
	      printf("start Kf\n");
	    }
	  }
#endif

	  printf("%d, %f, %f, %f, %f (%d), %f (%d), %f, %f: %f\n", 
		 a1, (a1-a0) / 0.032f,
		 gv0 / 0.032f,
		 (gyro_angle_pos - gyro_angle_neg),
		 gyro_angle_pos, pc,
		 gyro_angle_neg, nc,
		 theta, kTheta,
	         Kf.getBias());
	}
      }
      a0 = a1;

      usleep(32*1000);
    }
  }
}
#endif
