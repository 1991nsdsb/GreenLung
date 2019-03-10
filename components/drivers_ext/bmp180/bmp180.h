#ifndef BMP180_H_
#define BMP180_H_
#include <stdbool.h>



bool bmp180_init(void);
long Get_BMP180UT(void);
long Get_BMP180UP(void);
void Convert_UncompensatedToTrue(long UT,long UP, long *true_ut, long *true_up);

#endif
