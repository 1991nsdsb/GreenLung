#ifndef W25X40_H
#define W25X40_H
#include "nrf_drv_spi.h"
#include "nrf_gpio.h"

#define WRITE_ENABLE	                       0x06
#define WRITE_DISABLE                        0x04
#define READ_DATE	                           0x03
#define READ_STATUS_REG                      0x05
#define WRITE_STATUS_REG                     0x01
#define FAST_READ                            0x0b
#define FAST_READ_DUAL                       0x3b
#define PAGE_PROGRAM                         0x02
#define SECTOR_ERASE                         0x20
#define BLOCK_ERASE                          0x52
#define CHIP_ERASE                           0xc7
#define POWER_DOWN                           0xb9
#define RELEASE_POWER_DOWN                   0xab
#define JEDEC_ID                             0x9f
#define MANU_DEV_ID                          0xef4017
#define BUSY_FLAG                            0x01
#define SPI_FLAG_TIMEOUT                     500

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
#define PER_WRITE_PAGE_SIZE                  256
#define PER_SECTOR_SIZE                      4096  //4KB
#define TOTAL_PAGE													 32768
#define TOTAL_SECTOR                         2048
#define FLASH_LAST_ADDR                 0x7FFFF8   //last 8 byte .  4Mbit    0x7FFFF
extern nrf_drv_spi_t m_spi_master_0;
extern unsigned int flash_storage_addr;

typedef struct
{
		uint32_t first_measure_count_addr;
		uint16_t total_count;
}offline_data;

uint32_t w25x40_ReadId(void);
bool w25x40_Init(void);
void w25x40_WriteEnable(void);
void w25x40_WaitForWriteEnd(void);
void w25x40_PageWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void w25x40_BufferWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void w25x40_BufferRead(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void w25x40_SectorErase(uint32_t SectorAddr);
void w25x40_Simulate_PageErase(uint32_t EraseAddr);
void w25x40_ChipErase(void);
bool flash_is_used(uint8_t *buf, uint8_t len);
uint32_t w25x40_Storage_Init(void);	
offline_data w25x40_read_offline_data(uint32_t addr);
bool offline_is_tranfered(uint32_t addr);
uint32_t w25x40_sector_read_offline_data(uint32_t sector_first_addr, uint16_t NumByteToRead);

#endif //W25X40_H
