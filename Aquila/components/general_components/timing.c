#include <sys/time.h>


//функция, возвращающая время в микросенундах со старта приложения
int64_t get_time(void)
{
  struct timeval tv_now;
  gettimeofday(&tv_now, NULL);
  return (int64_t)((int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec);
}