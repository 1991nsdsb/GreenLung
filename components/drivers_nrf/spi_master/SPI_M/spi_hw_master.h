#ifndef SPI_HW_MASTER_H
#define SPI_HW_MASTER_H
#include <stdbool.h>
#include <stdint.h>
#define SPI_SCK_PIN                          12 
#define SPI_MISO_PIN                         15
#define SPI_MOSI_PIN                         13
#define SPI_SS_PIN                           16
#define SPI_WP                               14
#define SPI_HOLD                             11
#define SPI_SS_LOW()       nrf_gpio_pin_clear(SPI_SS_PIN)
#define SPI_SS_HIGH()      nrf_gpio_pin_set(SPI_SS_PIN)
#define SPI_WP_LOCK()      nrf_gpio_pin_clear(SPI_WP)
#define SPI_WP_UNLOCK()    nrf_gpio_pin_set(SPI_WP)
#define SPI_HOLD_LOW()     nrf_gpio_pin_clear(SPI_HOLD)
#define SPI_HOLD_HIGH()    nrf_gpio_pin_set(SPI_HOLD)
#define Dummy_Byte         0xff
void spi_master_init(void);
uint8_t spi_master_transfer(uint8_t *byte);
uint8_t spi_master_read(void);


#endif //SPI_HW_MASTER_H
