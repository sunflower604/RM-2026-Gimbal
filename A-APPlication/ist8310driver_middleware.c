#include "ist8310driver_middleWare.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c3;

void ist8310_GPIO_init(void)
{

}

void ist8310_com_init(void)
{
}

uint8_t ist8310_IIC_read_single_reg(uint8_t reg)
{
    uint8_t res = 0;
    HAL_I2C_Mem_Read(&hi2c3, IST8310_IIC_ADDRESS <<1, reg,I2C_MEMADD_SIZE_8BIT,&res,1,10);
    return res;
}


void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c3, IST8310_IIC_ADDRESS <<1, reg,I2C_MEMADD_SIZE_8BIT,&data,1,10);
}

void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    HAL_I2C_Mem_Read(&hi2c3, IST8310_IIC_ADDRESS <<1, reg,I2C_MEMADD_SIZE_8BIT,buf,len,10);
}

void ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len)
{
    HAL_I2C_Mem_Write(&hi2c3, IST8310_IIC_ADDRESS <<1, reg,I2C_MEMADD_SIZE_8BIT,data,len,10);
}

void ist8310_delay_ms(uint16_t ms)
{
    HAL_Delay(ms);
}

void ist8310_delay_us(uint16_t us)
{
    uint32_t ticks = 0;
    uint32_t told = 0, tnow = 0, tcnt = 0;
    uint32_t reload = 0;
    reload = SysTick->LOAD;
    ticks = us * 72;
    told = SysTick->VAL;
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks)
            {
                break;
            }
        }
    }
}

void ist8310_RST_H(void)
{
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);
}

extern void ist8310_RST_L(void)
{
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);
}
