#include "i2c_dma_master.h"
#include "cmsis_gcc.h"
#include "stm32f303xe.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_def.h"
#include "stm32f3xx_hal_dma.h"
#include "stm32f3xx_hal_i2c.h"
#include "stm32f3xx_hal_rcc_ex.h"
// #include "stm32f3xx_hal_i2c.h"
#include <stdint.h>
#include <stdbool.h>

I2C_info I2Cs[MAX_I2Cs];

static void I2C_DMA_mastercont(I2C_info *hi2c, uint32_t ITFlags, uint32_t ITSources);

inline static void __attribute__((always_inline)) set_gpio_afr(GPIO_TypeDef * GPIOX, uint8_t pinnum)
{
    uint32_t tempafr = GPIOX->AFR[pinnum >> 3];
    tempafr &= ~(GPIO_AFR_ALL_SET << ((pinnum & NORMALIZE_PINS_0_TO_7_MASK) * GPIO_AFR_BITCOUNT));
    tempafr |= (GPIO_AFR_I2CAF << ((pinnum & NORMALIZE_PINS_0_TO_7_MASK) * GPIO_AFR_BITCOUNT));
    GPIOX->AFR[pinnum >> 3] = tempafr;

    tempafr = GPIOX->MODER;
    tempafr &= ~(GPIO_MODER_SET << (pinnum * 2));
    tempafr |= GPIO_AF_MODE << (pinnum * 2);
    GPIOX->MODER = tempafr;
}


void I2C1_EV_IRQHandler(void)
{
  /* Get current IT Flags and IT sources value */
    uint32_t itflags   = I2Cs[I2C1_POS].Instance->ISR;
    uint32_t itsources = I2Cs[I2C1_POS].Instance->CR1;
    I2C_DMA_mastercont(&I2Cs[I2C1_POS], itflags, itsources);
}

// txdata can be written only when txe = 1


void I2C1_ER_IRQHandler(void) // only for master 
{
    //TODO Errata 2.6.6 states that spurious bus error is detected (BERRFLAG is set) in master mode and to just clear it  


    I2C_info *hi2c = &I2Cs[I2C1_POS];
    // check if errors enabled
    uint32_t tmperror = hi2c->Instance->CR1;
    if(!(tmperror & ERRIEFLAG))
        return;

    tmperror  = hi2c->Instance->ISR & (BERRFLAG | ARLOFLAG);
    if(!tmperror)
        return;

    if(tmperror == BERRFLAG){
        hi2c->Instance->ICR |= BERRFLAG;
        return;
    }

    uint32_t dmacheck = hi2c->Instance->CR1;
    // Disable all relevant interrupts and dma except rxie and slave flags and smbus flags
    // hi2c->Instance->CR1 &= ~(TXDMAENFLAG | RXDMAENFLAG | ERRIEFLAG | TCIEFLAG | STOPFLAG | NACKFLAG | TXIEFLAG);
    hi2c->Instance->CR1 &= ~(TXDMAENFLAG | RXDMAENFLAG);

    __asm__ volatile ("dmb\n");

    if(dmacheck & TXDMAENFLAG)
        disable_dma(&hi2c->txhdma);
    else
        disable_dma(&hi2c->rxhdma);

    //clear cr2 regs
    hi2c->Instance->CR2 &= CLEARCR2;

    //clear relevant error interrupts
    hi2c->Instance->ICR = tmperror;
    
    // while(hi2c->Instance->ISR & (1 << 15)); // busy bit set bug not mentioned in errata sheet :)))))))))))))

    __asm__ volatile ("dmb\n");
    hi2c->Instance->CR1 &= ~I2CENABLE;
    hi2c->Instance->CR1;
    __asm__ volatile ("dmb\n");
    hi2c->Instance->CR1 |= I2CENABLE;
    __asm__ volatile ("dmb\n");

    atomic_store(&hi2c->state, STATE_READY);
    /* 
    giving info about error would be useless,
    since we have to ignore BERRFLAG as per errata sheet and ARLO causes a busy bit set bug not mentioned in errata :)
    */
}

static void I2C_DMA_mastercont(I2C_info *hi2c, uint32_t ITFlags, uint32_t ITSources)
{
    if(NACKFLAG & ITFlags & ITSources){
        //clear flag
        hi2c->Instance->ICR |= NACKFLAG;

        //enable stop interrupts
        hi2c->Instance->CR1 |= (STOPFLAG | TCIEFLAG);

        __asm__ volatile ("dmb\n");
        //stop generation
        hi2c->Instance->CR2 |= STOPGENERATECR2;

    }
    if((TCRFLAG & ITFlags) && (ITSources & TCIEFLAG))
    {

        if (hi2c->XferCount != 0)
        {
            uint8_t XferSize = 0; 
            /* Prepare the new XferSize to transfer */
            if (hi2c->XferCount > MAX_I2C_BUF_SIZE)
            {
                XferSize = MAX_I2C_BUF_SIZE;
                /* Set the new XferSize in Nbytes register */
                hi2c->Instance->CR2 |= (MAX_I2C_BUF_SIZE << NBYTESSHIFT) | I2C_RELOAD_MODE;
            }
            else
            {
                XferSize = hi2c->XferCount;
                /* Set the new XferSize in Nbytes register */
                hi2c->Instance->CR2 |= (XferSize << NBYTESSHIFT) | I2C_AUTOEND_MODE;
            }

            /* Update XferCount value */
            hi2c->XferCount -= XferSize;
        }
        else{
            //theoritically unreachable

            //enable stop interrupts
            hi2c->Instance->CR1 |= (STOPFLAG | TCIEFLAG);

            __asm__ volatile ("dmb\n");

            //stop generation
            hi2c->Instance->CR2 |= STOPGENERATECR2;
        }
    }
    if(TCIEFLAG & ITFlags & ITSources){
        if (hi2c->XferCount == 0)
        {
            if (!(hi2c->Instance->CR2 & I2C_AUTOEND_MODE))
            {
                hi2c->Instance->CR1 |= (STOPFLAG | TCIEFLAG);
                __asm__ volatile ("dmb\n");
                /* Generate Stop */
                hi2c->Instance->CR2 |= STOPGENERATECR2;
            }
        }
        else
        {
            //theoritically unreachable
            hi2c->Instance->CR1 |= (STOPFLAG | TCIEFLAG);
            __asm__ volatile ("dmb\n");
            /* Generate Stop */
            hi2c->Instance->CR2 |= STOPGENERATECR2;

            /* Wrong size Status regarding TC flag event */
        }
    }
    if(STOPFLAG & ITFlags & ITSources)
    {
        hi2c->Instance->ICR |= STOPFLAG;
        //disable dma requests and clear isr flags
        uint32_t dmacheck = hi2c->Instance->CR1;
        hi2c->Instance->CR1 &= ~(TXDMAENFLAG | RXDMAENFLAG | ERRIEFLAG | TCIEFLAG | STOPFLAG | NACKFLAG);
        __asm__ volatile ("dmb\n");
        // hi2c->Instance->CR1 &= ~(TXDMAENFLAG | RXDMAENFLAG);

        if(dmacheck & TXDMAENFLAG)
            disable_dma(&hi2c->txhdma);
        else
            disable_dma(&hi2c->rxhdma);

        //clear CR2 requests
        hi2c->Instance->CR2 &= CLEARCR2;

        atomic_store(&hi2c->state, STATE_READY);
    }
}

I2C_DMA_RET I2C_DMA_Init(I2C_TypeDef          *I2C_instance,
                         GPIO_TypeDef         *GPIOONE,
                         uint8_t               pinonenum,
                         GPIO_TypeDef         *GPIOTWO, 
                         uint8_t               pintwonum, 
                         DMA_info              TX_DMA_info,
                         DMA_info              RX_DMA_info,
                         bool                  I2C_10bit_adressing)
{
    if(I2C_instance != I2C1 && I2C_instance != I2C2)
        return WRONG_ARGS;

    I2C_info * hi2c = NULL;
    if(I2C_instance == I2C1)
    {
        if(!atomic_cas(&I2Cs[I2C1_POS].state, STATE_BUSY, STATE_UNINIT))
            return ALREADY_INITILIZED;
        hi2c = &I2Cs[I2C1_POS];
    }
    else // as there can only be 2 I2Cs on my system and we pre checked on function entry
    {
        if(!atomic_cas(&I2Cs[I2C2_POS].state, STATE_BUSY, STATE_UNINIT))
            return ALREADY_INITILIZED;
        hi2c = &I2Cs[I2C2_POS];
    }
    if (hi2c == NULL)
        return I2C_INIT_ERROR;

    hi2c->Instance = I2C_instance;

    // set pin speed
    GPIOONE->OSPEEDR |= GPIO_PIN_HIGH_SPD << (pinonenum * 2);
    GPIOTWO->OSPEEDR |= GPIO_PIN_HIGH_SPD << (pintwonum * 2);
    
    // set up output open-drain mode
    GPIOONE->OTYPER |= GPIO_PIN_OPEN_DR << pinonenum;
    GPIOTWO->OTYPER |= GPIO_PIN_OPEN_DR << pintwonum;

    // // // disable pulldowns
    // GPIOONE->PUPDR &= ~(0x3 << (pinonenum * 2));
    // GPIOTWO->PUPDR &= ~(0x3 << (pintwonum * 2));

    // set AFR and MODER for I2C
    __asm__ volatile ("dmb\n");
    set_gpio_afr(GPIOONE, pinonenum);
    set_gpio_afr(GPIOTWO, pintwonum);

    if(hi2c->Instance == I2C1)//TODO replace
        __HAL_RCC_I2C1_CLK_ENABLE();
    else
        __HAL_RCC_I2C2_CLK_ENABLE();

    hi2c->txhdma = TX_DMA_info;
    hi2c->rxhdma = RX_DMA_info;

    if(hi2c->txhdma.Instance != NULL){
        DMA_info * txhdma = &hi2c->txhdma;//TODO replace by my own structure and callback

        if(!atomic_cas(&txhdma->state, DMA_BUSY_STATE, DMA_UNINIT_STATE)){
            atomic_store(&hi2c->state, STATE_UNINIT);
            return DMA_INIT_ERROR;
        }
        
        uint32_t tmpdmaccr = txhdma->Instance->CCR;

        //clear dma config
        tmpdmaccr &= CLEARDMACCR;

        tmpdmaccr |= DMAMEMTOPERIPH_DIR;

        tmpdmaccr |= DMAMEMINCENABLE; 

        tmpdmaccr |= DMADATAALIGNBYTE;

        tmpdmaccr |= DMADATAALIGNBYTE;

        tmpdmaccr |= DMANOTCIRCULAR;

        /* Write to DMA Channel CR register */
        __asm__ volatile ("dmb\n");
        txhdma->Instance->CCR = tmpdmaccr;

        /* calculation of the channel index */
        #if defined (DMA2)
            if ((uint32_t)(txhdma->Instance) < (uint32_t)(DMA2_Channel1))
            {
                /* DMA1 */
                txhdma->ChannelIndex = (((uint32_t)txhdma->Instance - (uint32_t)DMA1_Channel1) / ((uint32_t)DMA1_Channel2 - (uint32_t)DMA1_Channel1)) << 2U;
                txhdma->DmaBaseAddress = DMA1;
            }
            else
            {
                /* DMA2 */
                txhdma->ChannelIndex = (((uint32_t)txhdma->Instance - (uint32_t)DMA2_Channel1) / ((uint32_t)DMA2_Channel2 - (uint32_t)DMA2_Channel1)) << 2U;
                txhdma->DmaBaseAddress = DMA2;
            }
        #else
            /* DMA1 */
            txhdma->ChannelIndex = (((uint32_t)txhdma->Instance - (uint32_t)DMA1_Channel1) / ((uint32_t)DMA1_Channel2 - (uint32_t)DMA1_Channel1)) << 2U;
            txhdma->DmaBaseAddress = DMA1;
        #endif

        atomic_store(&txhdma->state, DMA_READY_STATE); 
    }

    if(hi2c->rxhdma.Instance != NULL){
        DMA_info * rxhdma = &hi2c->rxhdma; //TODO replace by my own structure and callback

        if(!atomic_cas(&rxhdma->state, DMA_BUSY_STATE, DMA_UNINIT_STATE)){
            atomic_store(&hi2c->state, STATE_UNINIT);
            return DMA_INIT_ERROR;
        }

        uint32_t tmpdmaccr = rxhdma->Instance->CCR;

        //clear dma config
        tmpdmaccr &= CLEARDMACCR;

        // default mode is reading from peripherial so we dont set it here

        tmpdmaccr |= DMAMEMINCENABLE; 

        tmpdmaccr |= DMADATAALIGNBYTE;

        tmpdmaccr |= DMADATAALIGNBYTE;

        tmpdmaccr |= DMANOTCIRCULAR;

        /* Write to DMA Channel CR register */
        rxhdma->Instance->CCR = tmpdmaccr;

        /* calculation of the channel index */
        #if defined (DMA2)
            if ((uint32_t)(rxhdma->Instance) < (uint32_t)(DMA2_Channel1))
            {
                /* DMA1 */
                rxhdma->ChannelIndex = (((uint32_t)rxhdma->Instance - (uint32_t)DMA1_Channel1) / ((uint32_t)DMA1_Channel2 - (uint32_t)DMA1_Channel1)) << 2U;
                rxhdma->DmaBaseAddress = DMA1;
            }
            else
            {
                /* DMA2 */
                rxhdma->ChannelIndex = (((uint32_t)rxhdma->Instance - (uint32_t)DMA2_Channel1) / ((uint32_t)DMA2_Channel2 - (uint32_t)DMA2_Channel1)) << 2U;
                rxhdma->DmaBaseAddress = DMA2;
            }
        #else
            /* DMA1 */
            rxhdma->ChannelIndex = (((uint32_t)rxhdma->Instance - (uint32_t)DMA1_Channel1) / ((uint32_t)DMA1_Channel2 - (uint32_t)DMA1_Channel1)) << 2U;
            rxhdma->DmaBaseAddress = DMA1;
        #endif

        atomic_store(&rxhdma->state, DMA_READY_STATE);
    }

    /* Disable the selected I2C peripheral */
    hi2c->Instance->CR1 &= ~I2CENABLE;
    __asm__ volatile ("dmb\n");

    /* Configure I2Cx: Frequency range */
    // hi2c->Instance->TIMINGR = Timing & TIMING_CLEAR_MASK;
    hi2c->Instance->TIMINGR = STANDARD_I2C_TIMING;

    // Disable Own Address1 
    hi2c->Instance->OAR1 &= ~OWNADDRENABLE;

    // disable own address 2
    hi2c->Instance->OAR2 &= ~DUALADDRESS_ENABLE;

    /* Configure I2Cx: Addressing Master mode */
    if (I2C_10bit_adressing)
    {
        hi2c->Instance->CR2 |= I2CADD10ENABLE;
    }
    else
    {
        /* Clear the I2C ADD10 bit */
        hi2c->Instance->CR2 &= ~I2CADD10ENABLE;
    }

    // Enable the AUTOEND by default, and enable NACK (should be disable only during Slave process)
    hi2c->Instance->CR2 |= (I2CAUTOENDCR2 | I2CNACKCR2);

    /*---------------------------- I2Cx CR1 Configuration ----------------------*/
    /* Configure I2Cx: Generalcall and NoStretch mode and enable analog filter */
    hi2c->Instance->CR1 = 0;

    /* Enable the selected I2C peripheral */
    hi2c->Instance->CR1 |= I2CENABLE;

    atomic_store(&hi2c->state, STATE_READY);
    return OK;
}

I2C_DMA_RET I2C_DMA_master_tx(uint8_t *buf, uint16_t bufsize, uint8_t slvaddr, I2C_info * hi2c)
{ // assuming timingr is set
    if(!bufsize | !buf | (slvaddr & 0x80) | !hi2c)
        return WRONG_ARGS;

    if(!atomic_cas(&hi2c->state, STATE_BUSY, STATE_READY))
        return I2C_BUSY;

    if(!atomic_cas(&hi2c->txhdma.state, DMA_BUSY_STATE, DMA_READY_STATE)){
        atomic_store(&hi2c->state, STATE_READY);
        return DMA_BUSY;
    }

    uint32_t tmpcr2 = 0;
    uint8_t XferSize = 0;
    if(bufsize > MAX_I2C_BUF_SIZE){
        XferSize = MAX_I2C_BUF_SIZE;
        tmpcr2 = I2C_RELOAD_MODE;
    }
    else{
        XferSize = bufsize;
        tmpcr2 = I2C_AUTOEND_MODE;
    }
    
    tmpcr2 |= XferSize << NBYTESSHIFT;
    tmpcr2 |= slvaddr << SLVADDRSHIFT;
    tmpcr2 |= START;

    hi2c->Instance->TXDR = *buf;
    hi2c->BuffPtr = buf + 1;
    --XferSize;
    hi2c->XferCount = bufsize - 1;

    //DMA config

    // hi2c->XferISR = I2C_DMA_mastertxcont;
    DMA_info * hdma = &hi2c->txhdma;

    /* Disable the peripheral */
    hdma->Instance->CCR &= ~DMAENABLEFLAG;
    __asm__ volatile ("dmb\n");

    /* Configure the source, destination address and the data length */
    hdma->DmaBaseAddress->IFCR  = (DMA_FLAG_GL1 << hdma->ChannelIndex);

    /* Configure DMA Channel data length */
    hdma->Instance->CNDTR = hi2c->XferCount;

    /* Configure DMA Channel memory address */
    hdma->Instance->CMAR =(uint32_t) hi2c->BuffPtr;

    /* Configure DMA Channel peripherial address */
    hdma->Instance->CPAR =(uint32_t) &hi2c->Instance->TXDR;


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
    __asm__ volatile ("dmb\n");
    hdma->Instance->CCR |= DMAENABLEFLAG;
    
    hi2c->XferCount -= XferSize;

    /* Enable DMA Request */
    hi2c->Instance->CR1 |= TXDMAENFLAG;

    //enable isrs
    hi2c->Instance->CR1 |= (ERRIEFLAG | NACKFLAG);
    __asm__ volatile ("dmb\n");

    //start tx
    hi2c->Instance->CR2 = tmpcr2;

    return OK;
}

I2C_DMA_RET I2C_DMA_master_rx(uint8_t *buf, uint16_t transfersize, uint8_t slvaddr, I2C_info * hi2c)
{
    if(!transfersize | !buf | (slvaddr & 0x80) | !hi2c)
        return WRONG_ARGS;

    if(!atomic_cas(&hi2c->state, STATE_BUSY, STATE_READY))
        return I2C_BUSY;

    if(!atomic_cas(&hi2c->rxhdma.state, DMA_BUSY_STATE, DMA_READY_STATE)){
        atomic_store(&hi2c->state, STATE_READY);
        return DMA_BUSY;
    }

    uint32_t tmpcr2 = 0;
    uint8_t XferSize = 0;
    if(transfersize > MAX_I2C_BUF_SIZE){
        XferSize = MAX_I2C_BUF_SIZE;
        tmpcr2 = I2C_RELOAD_MODE;
    }
    else{
        XferSize = transfersize;
        tmpcr2 = I2C_AUTOEND_MODE;
    }
    
    tmpcr2 |= XferSize << NBYTESSHIFT;
    tmpcr2 |= slvaddr << SLVADDRSHIFT;
    tmpcr2 |= I2C_7BITADDR_REC;
    tmpcr2 |= I2C_READREQCR2;

    tmpcr2 |= START;

    hi2c->BuffPtr = buf;
    hi2c->XferCount = transfersize;

    // DMA config
    // hi2c->XferISR = I2C_DMA_masterrxcont;
    DMA_info * hdma = &hi2c->rxhdma;

    /* Disable the peripheral */
    hdma->Instance->CCR &= ~DMAENABLEFLAG;
    __asm__ volatile ("dmb\n");

    /* Configure the source, destination address and the data length */
    hdma->DmaBaseAddress->IFCR  = (DMA_FLAG_GL1 << hdma->ChannelIndex);

    /* Configure DMA Channel data length */
    hdma->Instance->CNDTR = hi2c->XferCount;

    /* Configure DMA Channel peripherial address */
    hdma->Instance->CPAR =(uint32_t) &hi2c->Instance->RXDR;

    /* Configure DMA Channel memory address */
    hdma->Instance->CMAR =(uint32_t) hi2c->BuffPtr;


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
    __asm__ volatile ("dmb\n");
    hdma->Instance->CCR |= DMAENABLEFLAG;
    
    hi2c->XferCount -= XferSize;

    /* Enable DMA Request */
    hi2c->Instance->CR1 |= RXDMAENFLAG;

    //enable isrs
    hi2c->Instance->CR1 |= (ERRIEFLAG | NACKFLAG);
    __asm__ volatile ("dmb\n");

    //start tx
    hi2c->Instance->CR2 = tmpcr2;

    return OK;
}


