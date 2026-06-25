#include "i2c_dma_master.h"
#include "cmsis_gcc.h"
#include "stm32f303xe.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_def.h"
#include "stm32f3xx_hal_dma.h"
#include "stm32f3xx_hal_i2c.h"
// #include "stm32f3xx_hal_i2c.h"
#include <stdint.h>

enum{
    MAX_I2C_BUF_SIZE = 0xFF,
    NBYTESSHIFT      = 16,
    START            = 1 << 13,
    SLVADDRSHIFT     = 1,
    CLEARISR         = 0x3F38,
    NACKFLAG         = 0x10,
    TCRFLAG          = 0x80,
    TCIEFLAG         = 0x40,
    STOPFLAG         = 0x20,
    ERRIEFLAG        = 0x80, 
    BERRFLAG         = 0x100,
    ARLOFLAG         = 0x200,
    TXIEFLAG         = 0x2,
    CLEARDMACCR      = ~0x7FFE,
    CLEARCR2         = ~0x7FFFFFF,
    STOPGENERATECR2  = 0x4000,
    TXDMAENFLAG      = 0x4000,
    DMAENABLEFLAG    = 0x1,
    DMATXERRORFLAG   = 0x8,
    DMATXHALFCPLT    = 0x4,
    DMATXCPLT        = 0x2
};

static void I2C_DMAmastertxcplt(DMA_HandleTypeDef *hdma);
static void I2C_DMAError(DMA_HandleTypeDef *hdma);
static void I2C_DMA_mastertxcont(I2C_HandleTypeDef *hi2c, uint32_t ITFlags, uint32_t ITSources);

inline static void __attribute__((always_inline)) disable_dma(DMA_HandleTypeDef * hdma){
    //disable dma
    hdma->Instance->CCR &= ~DMAENABLEFLAG;

    //clear dma config
    hdma->Instance->CCR &= CLEARDMACCR;

    //clear dma memory addr
    hdma->Instance->CMAR = 0;

    //clear dma peripherial addr
    hdma->Instance->CPAR = 0;

    //clear dma number of data to transfer
    hdma->Instance->CNDTR = 0;

    hdma->State = HAL_DMA_STATE_READY;
}

I2C_HandleTypeDef * hi2ci1;

void I2C1_EV_IRQHandler(void) /* Derogation MISRAC2012-Rule-8.13 */
{
  /* Get current IT Flags and IT sources value */
    uint32_t itflags   = hi2ci1->Instance->ISR;
    uint32_t itsources = hi2ci1->Instance->CR1;
    I2C_DMA_mastertxcont(hi2ci1, itflags, itsources);
}

// txdata can be written only when txe = 1


void I2C1_ER_IRQHandler(void) // only for master 
{
    I2C_HandleTypeDef *hi2c = hi2ci1;
    // check if errors enabled
    if(!(hi2c->Instance->CR1 & ERRIEFLAG))
        return;

    uint32_t tmperror  = hi2c->Instance->ISR & (BERRFLAG | ARLOFLAG);
    if(!tmperror)
        return;

    // Disable all relevant interrupts and dma except rxie and slave flags and smbus flags
    hi2c->Instance->CR1 &= ~(TXDMAENFLAG | ERRIEFLAG | TCIEFLAG | STOPFLAG | NACKFLAG | TXIEFLAG);

    disable_dma(hi2c->hdmatx);

    //clear cr2 regs
    hi2c->Instance->CR2 &= CLEARCR2;

    //clear relevant error interrupts
    hi2c->Instance->ICR |= tmperror;

    hi2c->State = HAL_I2C_STATE_READY;

    //give some info about error or smth
}

static void I2C_DMAmastertxcplt(DMA_HandleTypeDef *hdma)
{
    //clear interrupt
    hdma->DmaBaseAddress->IFCR = (DMA_FLAG_TC1 << hdma->ChannelIndex);
}

static void I2C_DMAError(DMA_HandleTypeDef *hdma)
{
    //clear interrupt
    hdma->DmaBaseAddress->IFCR = (DMA_FLAG_TE1 << hdma->ChannelIndex);

    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *) hdma->Parent;
    hi2c->Instance->CR1 &= ~TXDMAENFLAG;
    //enable stop interrupts
    hi2c->Instance->CR1 |= (STOPFLAG | TCIEFLAG);

    //stop generation
    hi2c->Instance->CR2 |= STOPGENERATECR2;
}

static void I2C_DMA_mastertxcont(I2C_HandleTypeDef *hi2c, uint32_t ITFlags, uint32_t ITSources)
{
    if(NACKFLAG & ITFlags & ITSources){
        //clear flag
        hi2c->Instance->ICR |= NACKFLAG;

        //enable stop interrupts
        hi2c->Instance->CR1 |= (STOPFLAG | TCIEFLAG);

        //stop generation
        hi2c->Instance->CR2 |= STOPGENERATECR2;

    }
    else if((TCRFLAG & ITFlags) && (ITSources & TCIEFLAG))
    {

        if (hi2c->XferCount != 0)
        {
            /* Prepare the new XferSize to transfer */
            if (hi2c->XferCount > MAX_I2C_BUF_SIZE)
            {
                hi2c->XferSize = MAX_I2C_BUF_SIZE;
                /* Set the new XferSize in Nbytes register */
                hi2c->Instance->CR2 |= (hi2c->XferSize << NBYTESSHIFT) | I2C_RELOAD_MODE;
            }
            else
            {
                hi2c->XferSize = hi2c->XferCount;
                /* Set the new XferSize in Nbytes register */
                hi2c->Instance->CR2 |= (hi2c->XferSize << NBYTESSHIFT) | I2C_AUTOEND_MODE;
            }

            /* Update XferCount value */
            hi2c->XferCount -= hi2c->XferSize;
        }
        else{
            //theoritically unreachable

            //enable stop interrupts
            hi2c->Instance->CR1 |= (STOPFLAG | TCIEFLAG);

            //stop generation
            hi2c->Instance->CR2 |= STOPGENERATECR2;
        }
    }
    else if(TCIEFLAG & ITFlags & ITSources){
        if (hi2c->XferCount == 0)
        {
            if (!(hi2c->Instance->CR2 & I2C_AUTOEND_MODE))
            {
                hi2c->Instance->CR1 |= (STOPFLAG | TCIEFLAG);
                /* Generate Stop */
                hi2c->Instance->CR2 |= STOPGENERATECR2;
            }
        }
        else
        {
            //theoritically unreachable
            hi2c->Instance->CR1 |= (STOPFLAG | TCIEFLAG);
            /* Generate Stop */
            hi2c->Instance->CR2 |= STOPGENERATECR2;

            /* Wrong size Status regarding TC flag event */
        }
    }
    else if(STOPFLAG & ITFlags & ITSources)
    {
        hi2c->Instance->ICR |= STOPFLAG;
        //disable dma requests and clear isr flags
        hi2c->Instance->CR1 &= ~(TXDMAENFLAG | ERRIEFLAG | TCIEFLAG | STOPFLAG | NACKFLAG);
        disable_dma(hi2c->hdmatx);

        //clear CR2 requests
        hi2c->Instance->CR2 &= CLEARCR2;

        hi2c->State = HAL_I2C_STATE_READY;
    }

}

I2C_DMA_RET I2C_DMA_master_tx(uint8_t *buf, uint16_t bufsize, uint8_t slvaddr, I2C_HandleTypeDef * hi2c)
{ // assuming timingr is set
    if(!bufsize | !buf | (slvaddr & 0x80) | !hi2c)
        return WRONG_ARGS;
    if(hi2c->State != HAL_I2C_STATE_READY) 
        return I2C_BUSY;
    hi2c->State = HAL_I2C_STATE_BUSY_TX;
    uint32_t tmpcr2 = 0;
    if(bufsize > MAX_I2C_BUF_SIZE){
        hi2c->XferSize = MAX_I2C_BUF_SIZE;
        tmpcr2 = I2C_RELOAD_MODE;
    }
    else{
        hi2c->XferSize = bufsize;
        tmpcr2 = I2C_AUTOEND_MODE;
    }
    
    tmpcr2 |= hi2c->XferSize << NBYTESSHIFT;
    tmpcr2 |= slvaddr << SLVADDRSHIFT;
    tmpcr2 |= START;

    hi2c->Instance->TXDR = *buf;
    hi2c->pBuffPtr = buf + 1;
    --hi2c->XferSize;
    hi2c->XferCount = bufsize - 1;

    // hi2c->XferISR = I2C_DMA_mastertxcont;
    hi2c->hdmatx->XferCpltCallback = I2C_DMAmastertxcplt;
    hi2c->hdmatx->XferErrorCallback = I2C_DMAError;

    DMA_HandleTypeDef * hdma = hi2c->hdmatx;

    if(hdma->State != HAL_DMA_STATE_READY){
        hi2c->State = HAL_I2C_STATE_READY;
        return DMA_BUSY;
    }
    hi2ci1 = hi2c;
    /* Change DMA peripheral state */
    hdma->State = HAL_DMA_STATE_BUSY;

    /* Disable the peripheral */
    hdma->Instance->CCR &= ~DMAENABLEFLAG;

    /* Configure the source, destination address and the data length */
    hdma->DmaBaseAddress->IFCR  = (DMA_FLAG_GL1 << hdma->ChannelIndex);

    /* Configure DMA Channel data length */
    hdma->Instance->CNDTR = hi2c->XferCount;

    /* Configure DMA Channel source address */
    hdma->Instance->CPAR =(uint32_t) hi2c->pBuffPtr;

    /* Configure DMA Channel destination address */
    hdma->Instance->CMAR =(uint32_t) &hi2c->Instance->TXDR;


    /* Enable the transfer complete, & transfer error interrupts */
    /* Half transfer interrupt is optional: enable it only if associated callback is available */
    if(NULL != hdma->XferHalfCpltCallback )
    {
        hdma->Instance->CCR |= (DMATXCPLT | DMATXHALFCPLT | DMATXERRORFLAG);
    }
    else
    {
        hdma->Instance->CCR |= (DMATXCPLT | DMATXERRORFLAG);
        hdma->Instance->CCR &= ~DMATXHALFCPLT;
    }
    /* Enable the Peripheral */
    hdma->Instance->CCR |= DMAENABLEFLAG;
    
    hi2c->XferCount -= hi2c->XferSize;

    /* Enable DMA Request */
    hi2c->Instance->CR1 |= TXDMAENFLAG;

    //enable isrs
    hi2c->Instance->CR1 |= (ERRIEFLAG | NACKFLAG);

    //start tx
    hi2c->Instance->CR2 = tmpcr2;

    return OK;
}


