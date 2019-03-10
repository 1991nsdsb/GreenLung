#include <stdio.h>
#include "nrf_delay.h"
#include "twi_master.h"
#include "nrf_gpio.h"
 
 
 
#define BMP180_SlaveAddr 0x77   //BMP180的器件地址
#define CHIP_ID				(0x55)

//BMP180校准系数
short AC1;
short AC2;
short AC3;
unsigned short AC4;
unsigned short AC5;
unsigned short AC6;
short B1;
short B2;
short MB;
short MC;
short MD;
 
uint8_t BMP180_ID=0;          //BMP180的ID

 
bool my_twi_write(uint8_t dev_addr, uint8_t reg, uint8_t len, uint8_t * data){
	static uint8_t temp_buf[50];
	temp_buf[0]=reg;
	for(int i=1;i<=len;i++)	temp_buf[i]=data[i-1];
	dev_addr =dev_addr<<1;
	if (twi_master_transfer(dev_addr,temp_buf,len+1,TWI_ISSUE_STOP)){
		return true;
	}else {
		return false;
	}
}


bool my_twi_read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf){
	static uint8_t temp_reg;
	temp_reg=reg;
	addr <<= 1;
	twi_master_transfer(addr,&temp_reg,1,TWI_DONT_ISSUE_STOP);
	if(twi_master_transfer(addr|0x01,buf,len,TWI_ISSUE_STOP)){
		return true;
	}else{
		return false;
	}
}

/*从BMP180中读1个字节的数据*/
uint8_t BMP180_ReadOneByte(uint8_t ReadAddr)
{
    uint8_t temp = 0;     

    my_twi_read(BMP180_SlaveAddr, ReadAddr, 1, &temp);
    return (temp);      
}
 
/*从BMP180中读2个字节的数据*/
short BMP180_ReadTwoByte(uint8_t ReadAddr)
{
	uint8_t data[2];
    short temp;
     
	my_twi_read(BMP180_SlaveAddr, ReadAddr, 2, data);
	
    temp = (data[0]<<8)|data[1];
 
    return temp;                                                    
}
 
/*向BMP180的寄存器写一个字节的数据*/
void Write_OneByteToBMP180(uint8_t RegAdd, uint8_t data)
{
	my_twi_write(BMP180_SlaveAddr, RegAdd, 1, &data);
}
 
 
/*读取BMP180的校准系数*/
void Read_CalibrationData(void)
{
    AC1 = BMP180_ReadTwoByte(0xaa);
    AC2 = BMP180_ReadTwoByte(0xac);
    AC3 = BMP180_ReadTwoByte(0xae);
    AC4 = BMP180_ReadTwoByte(0xb0);
    AC5 = BMP180_ReadTwoByte(0xb2);
    AC6 = BMP180_ReadTwoByte(0xb4);
    B1 = BMP180_ReadTwoByte(0xb6);
    B2 = BMP180_ReadTwoByte(0xb8);
    MB = BMP180_ReadTwoByte(0xba);
    MC = BMP180_ReadTwoByte(0xbc);
    MD = BMP180_ReadTwoByte(0xbe);
}
 
bool bmp180_init(void)
{
	uint8_t chip_id = 0;
	
	chip_id = BMP180_ReadOneByte(0xD0);
	if(chip_id != CHIP_ID){
		return false;
	}
	Read_CalibrationData();
	return true;
	
}
/*读BMP180没有经过补偿的温度值*/
long Get_BMP180UT(void)
{
    long UT;
 
    Write_OneByteToBMP180(0xf4,0x2e);       //write 0x2E into reg 0xf4
    nrf_delay_ms(10);                                   //wait 4.5ms
    UT = BMP180_ReadTwoByte(0xf6);          //read reg 0xF6(MSB),0xF7(LSB)
 
    return UT;
}
 
/*读BMP180没有经过补偿的压力值*/
long Get_BMP180UP(void)
{
    long UP=0;
	uint8_t data;
    Write_OneByteToBMP180(0xf4,0x34);       //write 0x34 into reg 0xf4 
    nrf_delay_ms(10);                                    //wait 4.5ms
    data = BMP180_ReadOneByte(0xf6); 
	UP |= data <<16;
	data = BMP180_ReadOneByte(0xf7);
    UP |= data <<8;
	data = BMP180_ReadOneByte(0xf8);
	UP |= data;
	UP >>= 8;

    return UP;      
}
 
/*把未经过补偿的温度和压力值转换为时间的温度和压力值
 *True_Temp:实际温度值,单位:℃
 *True_Press:时间压力值,单位:Pa
*/
void Convert_UncompensatedToTrue(long UT,long UP, long *true_tempe, long *true_press)
{
    long X1,X2,X3,B3,B5,B6,B7,T,P;
    unsigned long B4;
     
    X1 = ((UT-AC6)*AC5)>>15;      
    X2 = ((long)MC<<11)/(X1+MD);  
    B5 = X1+X2;                        
    T = (B5+8)>>4;                      
    *true_tempe = T;           
 
    B6 = B5-4000;                     
    X1 = (B2*B6*B6)>>23;            
    X2 = (AC2*B6)>>11;                
    X3 = X1+X2;                       
    B3 = (((long)AC1*4+X3)+2)/4;    
    X1 = (AC3*B6)>>13;              
    X2 = (B1*(B6*B6>>12))>>16;     
    X3 = ((X1+X2)+2)>>2;             
    B4 = AC4*(unsigned long)(X3+32768)>>15;  
    B7 = ((unsigned long)UP-B3)*50000;        
    if (B7 < 0x80000000)
    {
        P = (B7*2)/B4;  
    }
    else P=(B7/B4)*2;                         
    X1 = (P/256.0)*(P/256.0);      
    X1 = (X1*3038)>>16;              
    X2 = (-7357*P)>>16;            
    P = P+((X1+X2+3791)>>4);      
    *true_press = P;                

}
