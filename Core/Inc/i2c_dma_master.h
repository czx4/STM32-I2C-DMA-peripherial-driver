#ifndef I2C_DMA_MASTER_H
#define I2C_DMA_MASTER_H
// #include "stm32f3xx_hal_i2c.h"
#include "main.h"

typedef enum __attribute__((packed)){
    OK,
    WRONG_ARGS,
    I2C_BUSY,
    DMA_BUSY
}I2C_DMA_RET;
void I2C1_EV_IRQHandler(void);
void I2C1_ER_IRQHandler(void);
extern I2C_DMA_RET I2C_DMA_master_tx(uint8_t *buf,uint16_t bufsize, uint8_t slvaddr, I2C_HandleTypeDef * hi2c);


#endif