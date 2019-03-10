#include "w25x40.h"
#include "boards.h"
#include "bsp_config.h"
//extern void PWM_on(void);
//uint8_t *m_tx_buf = NULL;
//uint8_t tx_size = 0;
//uint8_t *m_rx_buf = NULL;
//uint8_t rx_size = 0;
uint32_t w25x40_ReadId(void)
{
		uint8_t jedec_id = JEDEC_ID;
		uint8_t miso_data[4] = {0};

	  nrf_drv_spi_transfer(&m_spi_master_0, &jedec_id, sizeof(jedec_id), miso_data, 4); //�⺯����ÿ����һ������һ��������JEDEC_IDʱ��Ҳ��ȡһ����������ȷID������1��ʼ
		SPI_SS_HIGH();

		return (miso_data[1] << 16 | miso_data[2] << 8 | miso_data[3]);
}
bool w25x40_Init(void)
{
		uint32_t man_dev_ID;
		
		man_dev_ID = w25x40_ReadId();
		
	  if(man_dev_ID == MANU_DEV_ID)
			  return true;
		else
			  return false;
}

void w25x40_WriteEnable(void)
{
		uint8_t write_enable = WRITE_ENABLE;
		nrf_drv_spi_transfer(&m_spi_master_0, &write_enable, sizeof(write_enable) , NULL, 0);
		SPI_SS_HIGH();
}

void w25x40_WaitForWriteEnd(void)
{
		uint8_t read_status_reg = READ_STATUS_REG;
		uint8_t miso_data = 0;
		uint32_t spi_timeout = SPI_FLAG_TIMEOUT;
		do
		{
				nrf_drv_spi_transfer(&m_spi_master_0, &read_status_reg, sizeof(read_status_reg), &miso_data, sizeof(miso_data));
				if(spi_timeout-- == 0)
				{
						return;
				}
		}while(miso_data & BUSY_FLAG); //write busy
		SPI_SS_HIGH();
}

void w25x40_PageWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
		uint8_t page_program[4] = {0};
		w25x40_WriteEnable();
//		m_tx_buf[0] = PAGE_PROGRAM;
//		m_tx_buf[1] = (WriteAddr >> 16) & 0xff;
//		m_tx_buf[2] = (WriteAddr >> 8) & 0xff;
//		m_tx_buf[3] = WriteAddr & 0xff;
		page_program[0] = PAGE_PROGRAM;
		page_program[1] = (WriteAddr >> 16) & 0xff;
		page_program[2] = (WriteAddr >> 8) & 0xff;
		page_program[3] = WriteAddr & 0xff;
		if(NumByteToWrite > PER_WRITE_PAGE_SIZE)
		{
				NumByteToWrite = PER_WRITE_PAGE_SIZE; //program too large
		}
		
		nrf_drv_spi_transfer(&m_spi_master_0, page_program, sizeof(page_program), NULL, 0);
//		unsigned int i = 0;
//		for(i = 0; i < NumByteToWrite; i++)
//		{
//				m_tx_buf[i] = pBuffer[i];
//		}
		while(NumByteToWrite--)
		{
				nrf_drv_spi_transfer(&m_spi_master_0, pBuffer, 1, NULL, 0);
			  pBuffer++;
		}
		SPI_SS_HIGH();	
		w25x40_WaitForWriteEnd();
}

void w25x40_BufferWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
		uint8_t NumOfPage = 0,NumOfSingle = 0,Addr = 0,count = 0,temp = 0;
		Addr = WriteAddr % PER_WRITE_PAGE_SIZE;
		NumOfPage = NumByteToWrite / PER_WRITE_PAGE_SIZE;
		NumOfSingle = NumByteToWrite % PER_WRITE_PAGE_SIZE;
		count = PER_WRITE_PAGE_SIZE - Addr;   //��count�����ݣ��պö��뵽ҳ��ַ
		if(Addr == 0)  //address align, ҳ����
		{
				if(NumOfPage == 0)
				{
						w25x40_PageWrite(pBuffer, WriteAddr, NumByteToWrite);
				}
				else
				{
						while(NumOfPage--)
						{
								w25x40_PageWrite(pBuffer, WriteAddr, PER_WRITE_PAGE_SIZE);
								WriteAddr += PER_WRITE_PAGE_SIZE;
								pBuffer += PER_WRITE_PAGE_SIZE;
						}
						w25x40_PageWrite(pBuffer, WriteAddr, NumOfSingle);
				}
		}
		else
		{
				if(NumOfPage == 0)
				{
						if(count >= NumOfSingle)
						{
								w25x40_PageWrite(pBuffer, WriteAddr, NumOfSingle);
						}
						else
						{
								w25x40_PageWrite(pBuffer, WriteAddr, count);
								WriteAddr += count;
								pBuffer += count;
								temp = NumOfSingle - count;
								w25x40_PageWrite(pBuffer, WriteAddr, temp);
						}
				}
				else
				{
						w25x40_PageWrite(pBuffer, WriteAddr, count);
						WriteAddr += count;
						pBuffer += count;
						temp = NumByteToWrite - count;
						NumOfPage = temp / PER_WRITE_PAGE_SIZE;
						NumOfSingle = temp % PER_WRITE_PAGE_SIZE;
						while(NumOfPage--)
						{
								w25x40_PageWrite(pBuffer, WriteAddr, PER_WRITE_PAGE_SIZE);
								WriteAddr += PER_WRITE_PAGE_SIZE;
								pBuffer += PER_WRITE_PAGE_SIZE;
						}
						if(NumOfSingle != 0)
						{
								w25x40_PageWrite(pBuffer, WriteAddr, NumOfSingle);
						}
						
				}
		}
}

void w25x40_BufferRead(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)//���������մӵ�4���ֽڿ�ʼ
{
//		spi_control_block_t *p_cb;
//		nrf_drv_spi_xfer_desc_t xfer_desc;
//    xfer_desc.p_tx_buffer = NULL;
//    xfer_desc.p_rx_buffer = pBuffer;
//    xfer_desc.tx_length   = 0;
//    xfer_desc.rx_length   = NumByteToWrite;
//		p_cb->evt.data.done = xfer_desc;
//    p_cb->tx_done = false;
//    p_cb->rx_done = false;
		NumByteToWrite = NumByteToWrite + 4;
		uint8_t read_date[4] = {0};
		read_date[0] = READ_DATE;
		read_date[1] = (WriteAddr >> 16) & 0xff;
		read_date[2] = (WriteAddr >> 8) & 0xff;
		read_date[3] = WriteAddr & 0xff;
		nrf_drv_spi_transfer(&m_spi_master_0, read_date, sizeof(read_date), pBuffer, NumByteToWrite);
		
//		nrf_drv_spi_transfer(&m_spi_master_0, NULL, 0, pBuffer, NumByteToWrite);
//		do {
//            while (!nrf_spi_event_check(m_spi_master_0.p_registers, NRF_SPI_EVENT_READY)) {}
//            nrf_spi_event_clear(m_spi_master_0.p_registers, NRF_SPI_EVENT_READY);
//           // NRF_LOG_DEBUG("SPI: Event: NRF_SPI_EVENT_READY.\r\n"); 
//							bsp_board_led_on(BSP_LED_GREEN);
//						*pBuffer = nrf_spi_rxd_get(m_spi_master_0.p_registers);
//						pBuffer++;
//        } while (NumByteToWrite--);
		SPI_SS_HIGH();	
//		bsp_board_led_on(BSP_LED_GREEN);
}

void w25x40_SectorErase(uint32_t SectorAddr)
{
		uint8_t sector_erase[4] = {0};
		sector_erase[0] = SECTOR_ERASE;
		sector_erase[1] = (SectorAddr >> 16) & 0xff;
		sector_erase[2] = (SectorAddr >> 8) & 0xff;
		sector_erase[3] = SectorAddr & 0xff;
		
		w25x40_WriteEnable();
		w25x40_WaitForWriteEnd();
		nrf_drv_spi_transfer(&m_spi_master_0, sector_erase, sizeof(sector_erase), NULL, 0);
		SPI_SS_HIGH();
		w25x40_WaitForWriteEnd();
}

void w25x40_Simulate_PageErase(uint32_t EraseAddr)
{
		uint16_t numsector = EraseAddr / PER_SECTOR_SIZE;
		uint16_t  singlebyte = EraseAddr % PER_SECTOR_SIZE;
		uint32_t sectoraddr = 0;
		if(singlebyte) //������sector��sector erase num + 1
		{
				numsector += 1;
		}
		for(uint16_t i = 0; i < numsector; i++)
		{
				sectoraddr = PER_SECTOR_SIZE * i;
				w25x40_SectorErase(sectoraddr);
		}
}

void w25x40_ChipErase(void)
{
		uint8_t chip_erase = CHIP_ERASE;
		w25x40_WriteEnable();
		w25x40_WaitForWriteEnd();
//		m_tx_buf[0] = CHIP_ERASE;
		nrf_drv_spi_transfer(&m_spi_master_0, &chip_erase, sizeof(chip_erase), NULL, 0);
		SPI_SS_HIGH();
		w25x40_WaitForWriteEnd();
}

bool flash_is_used(uint8_t *buf, uint8_t len) //oxffΪtrue
{
		bool flag = true;
		for(uint8_t i = 0; i < len; i++)
		{
				if(*buf == 0xff)
				{
						buf++;
				}
				else
				{
						flag = false;
						break;
				}
		}
		return flag;
}

uint32_t w25x40_Storage_Init(void)
{
		uint8_t buf[20] = {0};
		uint32_t addr = 0;
		uint32_t i;
		for(i = 0; i < TOTAL_PAGE; i++)
		{
				addr = PER_WRITE_PAGE_SIZE*i;
				w25x40_BufferRead(buf, addr, 8); //�ж�flash�Ƿ���
				if(flash_is_used(&buf[4], 8)) //δ��ʹ��
				{
						break;
				}
//		bsp_board_led_on(BSP_LED_GREEN);
//		PWM_on();
		}
		if(i == TOTAL_PAGE)
		{
				w25x40_ChipErase();
		}
//		if(flash_is_used(&buf[4], 8)) //���ڿ⺯��nrf_drv_spi_transfer���͵�ʱ��ͬʱ���գ�����ǰ4������ָ����յ����������塣�ӵ�5�����ݿ�ʼ���Ǵӻ����͵���Ч���ݡ�
//		{
//				while(addr < FLASH_LAST_ADDR)
//				{
//						w25x40_BufferRead(buf, addr, NumByteToRead);  //���ҵ�һ��δд�ĵ�ַ�������ء�
//						if(flash_is_used(&buf[4], 8))
//							break;
//						addr += NumByteToRead;
//				}
//		}
//		else                //flash��
//		{
////				bsp_board_led_on(BSP_LED_RED);
//				w25x40_ChipErase();
////				PWM_on();
//				addr = 0;
//		}
	
		return addr;
}

/*ֻ������һ�����ߣ�*/
offline_data w25x40_read_offline_data(uint32_t addr)  //���������ʱ���ݴ洢��ַ
{
		uint8_t buf[8] = {0};
		offline_data fl_data;
		uint32_t last_maeasure_count_addr = addr - 3;
		uint32_t offline_data_total_addr = 0;
		w25x40_BufferRead(buf, last_maeasure_count_addr, 2);
		fl_data.total_count = buf[4] << 8 | buf[5];
		fl_data.first_measure_count_addr = addr - fl_data.total_count * 16;
		return fl_data;
}

bool offline_is_tranfered(uint32_t addr)
{
		uint8_t buf[8] = {0};
		bool tranfer_flag = false;
		uint32_t last_end_addr = addr - 1;
		uint8_t end_data = 0;
		w25x40_BufferRead(buf, last_end_addr, 1);
		end_data = buf[4];
		if(end_data == 0xee)  //not tranfer .true
		{
				tranfer_flag = true;
		}
		return tranfer_flag;
}

uint32_t w25x40_sector_read_offline_data(uint32_t sector_first_addr, uint16_t NumByteToRead)
{
		uint8_t buf[8] = {0};
		uint32_t sector_first_flag_addr = (sector_first_addr - PER_SECTOR_SIZE) + NumByteToRead - 1;                  //��־λ����16Byte
		offline_data fl_data ={0,0};
		uint32_t offline_data_total_addr = 0;
		w25x40_BufferRead(buf, sector_first_flag_addr, 1);
		while(buf[4] == 0xee)      //not tranfer .true
		{
				sector_first_flag_addr -= PER_SECTOR_SIZE;
				w25x40_BufferRead(buf, sector_first_flag_addr, 1);
		}
		return (sector_first_flag_addr - NumByteToRead + 1);
}

uint32_t w25x40_page_read_offline_data(uint32_t page_first_addr, uint16_t NumByteToRead)
{
		uint8_t buf[8] = {0};
		uint32_t page_first_flag_addr = (page_first_addr - PER_WRITE_PAGE_SIZE) + NumByteToRead - 1;                  //��־λ����16Byte
		offline_data fl_data ={0,0};
		uint32_t offline_data_total_addr = 0;
		w25x40_BufferRead(buf, page_first_flag_addr, 1);
		while(buf[4] == 0xee)      //not tranfer .true
		{
				page_first_flag_addr -= PER_WRITE_PAGE_SIZE;
				w25x40_BufferRead(buf, page_first_flag_addr, 1);
		}
		return (page_first_flag_addr - NumByteToRead + 1);
}