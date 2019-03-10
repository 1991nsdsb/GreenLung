#include "spi_hw_master.h"

void spi_master_init(void)
{
		nrf_gpio_cfg_output(SPI_WP);
		nrf_gpio_pin_set(SPI_WP);
    nrf_gpio_cfg_output(SPI_HOLD);
		nrf_gpio_pin_set(SPI_HOLD);
		nrf_gpio_cfg_output(SPI_SS_PIN);
		nrf_gpio_pin_set(SPI_SS_PIN);
		nrf_gpio_cfg_output(SPI_SCK_PIN);
		nrf_gpio_pin_set(SPI_SCK_PIN);
		nrf_gpio_cfg_output(SPI_MOSI_PIN);
		nrf_gpio_pin_set(SPI_MOSI_PIN);
		nrf_gpio_cfg_input(SPI_MISO_PIN);
		nrf_gpio_pin_set(SPI_MISO_PIN);
		NRF_SPI0->EVENTS_READY = 0;
		NRF_SPI0->PSELSCK = SPI_SCK_PIN;
		NRF_SPI0->PSELMOSI = SPI_MOSI_PIN;
		NRF_SPI0->PSELMISO = SPI_MISO_PIN;
		NRF_SPI0->FREQUENCY = NRF_DRV_SPI_FREQ_4M;
		NRF_SPI0->CONFIG = SPI_CONFIG_ORDER_MsbFirst << SPI_CONFIG_ORDER_Pos | SPI_CONFIG_CPHA_Leading << SPI_CONFIG_CPHA_Pos\
			| SPI_CONFIG_CPOL_ActiveHigh << SPI_CONFIG_CPOL_Pos ;
		NRF_SPI0->ENABLE = SPI_ENABLE_ENABLE_Enabled << SPI_ENABLE_ENABLE_Pos;
}

uint8_t spi_master_transfer(uint8_t *byte)
{
		NRF_SPI0->EVENTS_READY = 0;
		NRF_SPI0->TXD = *byte;
		while(NRF_SPI0->EVENTS_READY == 0);
		return uint8_t(NRF_SPI0->RXD);
}

uint8_t spi_master_read(void)
{
		return spi_master_transfer(Dummy_Byte);
}

