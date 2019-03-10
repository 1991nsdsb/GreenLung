#ifndef BUZZER_H
#define BUZZER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
	
	
#define SDP3X_SLAVEADDR 				0x21
#define SDP31_PRODUCT_ID				0x03010188UL
#define SDP32_PRODUCT_ID				0x03010283UL

#define SDP31_SCALE_FACTORS_DIFFPRES 	60
#define SDP32_SCALE_FACTORS_DIFFPRES 	240
#define SDP31_SCALE_FACTORS_TEMP	 	200
#define SDP32_SCALE_FACTORS_TEMP	 	200


bool buzzer_init(void);
long buzzer_on(void);
long buzzer_off(void);


#ifdef __cplusplus
}
#endif

#endif /* BUZZER_H */
