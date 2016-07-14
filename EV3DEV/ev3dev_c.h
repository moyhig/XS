#if defined (__cplusplus)
extern "C" {
#endif

void ev3dev_init();
void ev3dev_motor_set_speed(int, int);
int ev3dev_motor_get_count(int);
int ev3dev_gyro(int port);

#if defined (__cplusplus)
}
#endif
