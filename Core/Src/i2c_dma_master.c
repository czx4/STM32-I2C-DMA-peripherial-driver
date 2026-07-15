#include "i2c_dma_master.h"
#include "i2c_dma_master_defines.h"
#include "stm32f303xe.h"
#include <stdint.h>

enum{
    GPIO_AFR_I2CAF              = 0x4,
    GPIO_PIN_HIGH_SPD           = 0x3,
    GPIO_PIN_OPEN_DR            = 0x1,
    GPIO_AF_MODE                = 0x2,
    GPIO_AFR_ALL_SET            = 0xF,
    GPIO_AFR_BITCOUNT           = 0x4,
    GPIO_MODER_SET              = 0x3,
    NORMALIZE_PINS_0_TO_7_MASK  = 0x7,

    GPIOA_AHBENRBIT             =(1 << 17),
    GPIOB_AHBENRBIT             =(1 << 18),
    GPIOC_AHBENRBIT             =(1 << 19),
    GPIOD_AHBENRBIT             =(1 << 20),
    GPIOE_AHBENRBIT             =(1 << 21),
    GPIOF_AHBENRBIT             =(1 << 22),
    GPIOG_AHBENRBIT             =(1 << 23),
    GPIOH_AHBENRBIT             =(1 << 16),

    DMAMEMINCENABLE             = (1 << 7), 
    DMADATAALIGNBYTE            = 0,
    DMANOTCIRCULAR              = 0,
    DMAMEMTOPERIPH_DIR          = (1 << 4),

    DMA1_AHBENRBIT              = 1,
    DMA2_AHBENRBIT              = (1 << 1),
    DMA1_NVICNUM_BASE           = 10,
    DMA2_NVICNUM_BASE           = 55,

    MAX_I2C_BUF_SIZE            = 0xFF,

    I2CENABLE                   = 0x1,
    NBYTESSHIFT                 = 16,
    START                       = (1 << 13),

    CLEARCR2                    = ~0x2FF67FF,
    I2CNACKCR2                  = (1 << 15),
    I2CAUTOENDCR2               = (1 << 25),
    I2CRELOADCR2                = (1 << 24),
    I2C_READREQCR2              = (1 << 10),

    SLVADDRSHIFT                = 1,
    DUALADDRESS_ENABLE          = (1 << 15),
    I2CADD10ENABLE              = (1 << 11),
    OWNADDRENABLE               = (1 << 15),
    I2C_7BITADDR_REC            = (1 << 12),

    STANDARD_I2C_TIMING         = 0x00201D2B //TIMINGR register value for stm32f303RE, I2C uses HSI 8Mhz clock by default
};

inline static void __attribute__((always_inline)) NVIC_setprio(uint32_t Interrupt_num, uint32_t PreemptPriority, uint32_t SubPriority)
{
    uint32_t prioritygroup = 0;
  
    // Check the parameters 
    if(SubPriority >= 10)
        SubPriority = 9;

    if(PreemptPriority >= 10)
        PreemptPriority = 9;
  
    // get priority group
    prioritygroup = (((SCBB->AIRCR & SCB_AIRCR_PRIOGR_MASK) >> SCB_AIRCR_PRIOGR_POS));

    prioritygroup &= 0x7;   // only values 0..7 are used          
    uint32_t PreemptPriorityBits;
    uint32_t SubPriorityBits;

    PreemptPriorityBits = 7 - prioritygroup > NVIC_PRIO_BITS ? NVIC_PRIO_BITS : 7 - prioritygroup;
    SubPriorityBits     = prioritygroup + NVIC_PRIO_BITS < 7 ?      0         : prioritygroup - 7 + NVIC_PRIO_BITS;

    uint32_t priority = (PreemptPriority & ((1 << PreemptPriorityBits) - 1)) << SubPriorityBits;
            priority |=  SubPriority     & ((1 << SubPriorityBits) - 1);
  
    NVICB->IP[Interrupt_num] = (uint8_t)((priority << (8 - NVIC_PRIO_BITS)) & 0xFF);
}

inline static void __attribute__((always_inline)) NVIC_enable(uint32_t Interrupt_num)
{
    NVICB->ISER[Interrupt_num >> 5] = (1 << (Interrupt_num & 0x1F));
}

I2C_info I2Cs[MAX_I2Cs];

inline static void __attribute__((always_inline)) set_gpio_afr(GPIO_Def * GPIOX, uint8_t pinnum)
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

inline static void __attribute__((always_inline)) set_if_unset_rcc_ahbenr_bit(uint32_t bit_to_check){
    uint32_t tmp = RCCB->AHBENR;
    if(!(tmp & bit_to_check))
        RCCB->AHBENR |= bit_to_check;
}

// sets up clock, nvic and inner structures
inline static void __attribute__((always_inline)) set_DMA_internals(DMA_info * hdma){
    #if defined (DMA2B)
        if ((uint32_t)(hdma->Instance) < (uint32_t)(DMA2Ch1))
        {
            // DMA1 
            set_if_unset_rcc_ahbenr_bit(DMA1_AHBENRBIT);
            uint8_t channel_num = 1 + (((uint32_t)hdma->Instance - (uint32_t)DMA1Ch1) / ((uint32_t)DMA1Ch2 - (uint32_t)DMA1Ch1));
            NVIC_setprio(DMA1_NVICNUM_BASE + channel_num, 0, 0);
            NVIC_enable(DMA1_NVICNUM_BASE + channel_num);
            hdma->ChannelIndex = (channel_num - 1) << 2U;
            hdma->DmaBaseAddress = DMA1B;
        }
        else
        {
            // DMA2 
            set_if_unset_rcc_ahbenr_bit(DMA2_AHBENRBIT);
            uint8_t channel_num = 1 + (((uint32_t)hdma->Instance - (uint32_t)DMA2Ch1) / ((uint32_t)DMA2Ch2 - (uint32_t)DMA2Ch1));
            NVIC_setprio(DMA2_NVICNUM_BASE + channel_num, 0, 0);
            NVIC_enable(DMA2_NVICNUM_BASE + channel_num);
            hdma->ChannelIndex = (channel_num - 1) << 2U;
            hdma->DmaBaseAddress = DMA2B;
        }
    #else
        // DMA1 
        set_if_unset_rcc_ahbenr_bit(DMA1_AHBENRBIT);
        uint8_t channel_num = 1 + (((uint32_t)hdma->Instance - (uint32_t)DMA1Ch1) / ((uint32_t)DMA1Ch2 - (uint32_t)DMA1Ch1));
        NVIC_setprio(DMA1_NVICNUM_BASE + channel_num, 0, 0);
        NVIC_enable(DMA1_NVICNUM_BASE + channel_num);
        hdma->ChannelIndex = (channel_num - 1) << 2U;
        hdma->DmaBaseAddress = DMA1B;
    #endif
}

inline static void __attribute__((always_inline)) enable_i2c_clock(uint32_t bit_to_enable){
    RCCB->APB1ENR |= bit_to_enable;
}

inline static void __attribute__((always_inline)) enable_gpio_clock(uint32_t GPIOX){
    switch (GPIOX) {
        case (uint32_t)GPIOA_BASE:
            set_if_unset_rcc_ahbenr_bit(GPIOA_AHBENRBIT);
            break;
        case (uint32_t)GPIOB_BASE:
            set_if_unset_rcc_ahbenr_bit(GPIOB_AHBENRBIT);
            break;
        case (uint32_t)GPIOC_BASE:
            set_if_unset_rcc_ahbenr_bit(GPIOC_AHBENRBIT);
            break;
        case (uint32_t)GPIOD_BASE:
            set_if_unset_rcc_ahbenr_bit(GPIOD_AHBENRBIT);
            break;
        case (uint32_t)GPIOE_BASE:
            set_if_unset_rcc_ahbenr_bit(GPIOE_AHBENRBIT);
            break;
        case (uint32_t)GPIOF_BASE:
            set_if_unset_rcc_ahbenr_bit(GPIOF_AHBENRBIT);
            break;
        case (uint32_t)GPIOG_BASE:
            set_if_unset_rcc_ahbenr_bit(GPIOG_AHBENRBIT);
            break;
        case (uint32_t)GPIOH_BASE:
            set_if_unset_rcc_ahbenr_bit(GPIOH_AHBENRBIT);
            break;
        default:
            break;
    }
}

inline static void __attribute__((always_inline)) I2C_DMA_mastercont(I2C_info *hi2c)
{
    uint32_t ITFlags   = hi2c->Instance->ISR;
    uint32_t ITSources = hi2c->Instance->CR1;
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
            // Prepare the new XferSize to transfer 
            if (hi2c->XferCount > MAX_I2C_BUF_SIZE)
            {
                XferSize = MAX_I2C_BUF_SIZE;
                // Set the new XferSize in Nbytes register 
                // hi2c->Instance->CR2 |= (MAX_I2C_BUF_SIZE << NBYTESSHIFT) | I2CRELOADCR2;
                hi2c->Instance->CR2 |= (MAX_I2C_BUF_SIZE << NBYTESSHIFT);
            }
            else
            {
                XferSize = hi2c->XferCount;
                // Set the new XferSize in Nbytes register 
                // hi2c->Instance->CR2 |= (XferSize << NBYTESSHIFT) | I2CAUTOENDCR2;
                uint32_t tmpcr2 = hi2c->Instance->CR2;

                tmpcr2 |= I2CAUTOENDCR2;
                tmpcr2 &= ~(MAX_I2C_BUF_SIZE << NBYTESSHIFT);
                tmpcr2 |= (XferSize << NBYTESSHIFT);
                tmpcr2 &= ~I2CRELOADCR2;

                hi2c->Instance->CR2 = tmpcr2;
            }

            // Update XferCount value 
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
            if (!(hi2c->Instance->CR2 & I2CAUTOENDCR2))
            {
                hi2c->Instance->CR1 |= (STOPFLAG | TCIEFLAG);
                __asm__ volatile ("dmb\n");
                // Generate Stop 
                hi2c->Instance->CR2 |= STOPGENERATECR2;
            }
        }
        else
        {
            //theoritically unreachable
            hi2c->Instance->CR1 |= (STOPFLAG | TCIEFLAG);
            __asm__ volatile ("dmb\n");
            // Generate Stop 
            hi2c->Instance->CR2 |= STOPGENERATECR2;

            // Wrong size Status regarding TC flag event 
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

        atomic_store(&hi2c->state, STATE_READY_I2C);
    }
}

inline static void __attribute__((always_inline)) I2C_DMA_errorhandle(I2C_info * hi2c)
{
    //Errata 2.6.6 states that spurious bus error is detected (BERRFLAG is set) in master mode and to just clear it  

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
    hi2c->Instance->CR1 &= ~(TXDMAENFLAG | RXDMAENFLAG | ERRIEFLAG | TCIEFLAG | STOPFLAG | NACKFLAG);
    // hi2c->Instance->CR1 &= ~(TXDMAENFLAG | RXDMAENFLAG);

    __asm__ volatile ("dmb\n");

    if(dmacheck & TXDMAENFLAG)
        disable_dma(&hi2c->txhdma);
    else
        disable_dma(&hi2c->rxhdma);

    //clear cr2 regs
    hi2c->Instance->CR2 &= CLEARCR2;

    //clear relevant error interrupts
    hi2c->Instance->ICR = tmperror;
    
    //reset sequence
    __asm__ volatile ("dmb\n");
    hi2c->Instance->CR1 &= ~I2CENABLE;
    if(hi2c->Instance->CR1 == 0){};
    __asm__ volatile ("dmb\n");
    hi2c->Instance->CR1 |= I2CENABLE;

    atomic_store(&hi2c->state, STATE_READY_I2C);
    /* 
    giving info about error would be useless,
    since we have to ignore BERRFLAG as per errata sheet and ARLO causes a busy bit set bug, not mentioned in errata :)
    */
}

#ifdef USING_I2C1
void I2C1_EV_IRQHandler(void)
{
    I2C_DMA_mastercont(&I2Cs[I2C1_POS]);
}

void I2C1_ER_IRQHandler(void) // only for master 
{
    I2C_DMA_errorhandle(&I2Cs[I2C1_POS]);
}
#endif

#ifdef USING_I2C2
void I2C2_EV_IRQHandler(void)
{
    I2C_DMA_mastercont(&I2Cs[I2C2_POS]);
}

void I2C2_ER_IRQHandler(void) // only for master 
{
    I2C_DMA_errorhandle(&I2Cs[I2C2_POS]);
}
#endif

#ifdef USING_I2C3
void I2C3_EV_IRQHandler(void)
{
    I2C_DMA_mastercont(&I2Cs[I2C3_POS]);
}

void I2C3_ER_IRQHandler(void) // only for master 
{
    I2C_DMA_errorhandle(&I2Cs[I2C3_POS]);
}
#endif

I2C_DMA_RET I2C_DMA_Init(I2C_Def              *I2C_instance,
                         GPIO_Def             *GPIOONE,
                         uint8_t               pinonenum,
                         GPIO_Def             *GPIOTWO, 
                         uint8_t               pintwonum, 
                         DMA_info              TX_DMA_info,
                         DMA_info              RX_DMA_info,
                         bool                  I2C_10bit_adressing)
{
    if(I2C_instance != I2C1B && I2C_instance != I2C2B && I2C_instance != I2C3B)
        return WRONG_ARGS;

    I2C_info * hi2c = NULL;
    if(I2C_instance == I2C1B)
    {
        if(!atomic_cas(&I2Cs[I2C1_POS].state, STATE_BUSY_I2C, STATE_UNINIT_I2C))
            return ALREADY_INITILIZED;
        hi2c = &I2Cs[I2C1_POS];
    }
    else if(I2C_instance == I2C2B)
    {
        if(!atomic_cas(&I2Cs[I2C2_POS].state, STATE_BUSY_I2C, STATE_UNINIT_I2C))
            return ALREADY_INITILIZED;
        hi2c = &I2Cs[I2C2_POS];
    }
    else
    {
        if(!atomic_cas(&I2Cs[I2C3_POS].state, STATE_BUSY_I2C, STATE_UNINIT_I2C))
            return ALREADY_INITILIZED;
        hi2c = &I2Cs[I2C3_POS];
    }

    if (hi2c == NULL)
        return I2C_INIT_ERROR;

    hi2c->Instance = I2C_instance;

    enable_gpio_clock((uint32_t) GPIOONE);
    if(GPIOONE != GPIOTWO)
        enable_gpio_clock((uint32_t) GPIOTWO);

    __asm__ volatile("dmb\n");

    // set pin speed
    GPIOONE->OSPEEDR |= GPIO_PIN_HIGH_SPD << (pinonenum * 2);
    GPIOTWO->OSPEEDR |= GPIO_PIN_HIGH_SPD << (pintwonum * 2);
    
    // set up output open-drain mode
    GPIOONE->OTYPER |= GPIO_PIN_OPEN_DR << pinonenum;
    GPIOTWO->OTYPER |= GPIO_PIN_OPEN_DR << pintwonum;

    // disable pulldowns
    GPIOONE->PUPDR &= ~(0x3 << (pinonenum * 2));
    GPIOTWO->PUPDR &= ~(0x3 << (pintwonum * 2));

    // set AFR and MODER for I2C
    __asm__ volatile ("dmb\n");
    set_gpio_afr(GPIOONE, pinonenum);
    set_gpio_afr(GPIOTWO, pintwonum);

    if (hi2c->Instance == I2C1B)
        enable_i2c_clock(RCCI2C1_ENABLE);
    else if (hi2c->Instance == I2C2B)
        enable_i2c_clock(RCCI2C2_ENABLE);
    else
        enable_i2c_clock(RCCI2C3_ENABLE);

    hi2c->txhdma = TX_DMA_info;
    hi2c->rxhdma = RX_DMA_info;

    if(hi2c->txhdma.Instance != NULL){
        DMA_info * txhdma = &hi2c->txhdma;

        if(!atomic_cas(&txhdma->state, STATE_BUSY_I2C,STATE_UNINIT_I2C)){
            atomic_store(&hi2c->state, STATE_UNINIT_I2C);
            return DMA_INIT_ERROR;
        }

        // set dma base address, channel index for isr, enable clock and nvic
        set_DMA_internals(txhdma);

        __asm__ volatile ("dmb\n");
        
        uint32_t tmpdmaccr = txhdma->Instance->CCR;

        //clear dma config
        tmpdmaccr &= CLEARDMACCR;

        tmpdmaccr |= DMAMEMTOPERIPH_DIR;

        tmpdmaccr |= DMAMEMINCENABLE; 

        tmpdmaccr |= DMADATAALIGNBYTE;

        tmpdmaccr |= DMADATAALIGNBYTE;

        tmpdmaccr |= DMANOTCIRCULAR;

        // Write to DMA Channel CR register 
        __asm__ volatile ("dmb\n");
        txhdma->Instance->CCR = tmpdmaccr;

        atomic_store(&txhdma->state, STATE_READY_I2C); 
    }

    if(hi2c->rxhdma.Instance != NULL){
        DMA_info * rxhdma = &hi2c->rxhdma; 

        if(!atomic_cas(&rxhdma->state, STATE_BUSY_I2C,STATE_UNINIT_I2C)){
            atomic_store(&hi2c->state, STATE_UNINIT_I2C);
            return DMA_INIT_ERROR;
        }

        // set dma base address, channel index for isr, enable clock and nvic
        set_DMA_internals(rxhdma);

        __asm__ volatile ("dmb\n");

        uint32_t tmpdmaccr = rxhdma->Instance->CCR;

        //clear dma config
        tmpdmaccr &= CLEARDMACCR;

        // default mode is reading from peripherial so we dont set it here

        tmpdmaccr |= DMAMEMINCENABLE; 

        tmpdmaccr |= DMADATAALIGNBYTE;

        tmpdmaccr |= DMADATAALIGNBYTE;

        tmpdmaccr |= DMANOTCIRCULAR;

        // Write to DMA Channel CR register 
        rxhdma->Instance->CCR = tmpdmaccr;

        atomic_store(&rxhdma->state, STATE_READY_I2C);
    }

    // Disable the selected I2C peripheral 
    hi2c->Instance->CR1 &= ~I2CENABLE;
    __asm__ volatile ("dmb\n");

    // Configure I2Cx: Frequency range 
    hi2c->Instance->TIMINGR = STANDARD_I2C_TIMING;

    // Disable Own Address1 
    hi2c->Instance->OAR1 &= ~OWNADDRENABLE;

    // disable own address 2
    hi2c->Instance->OAR2 &= ~DUALADDRESS_ENABLE;

    // Configure I2Cx: Addressing Master mode 
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

    //---------------------------- I2Cx CR1 Configuration ----------------------
    // Configure I2Cx: Generalcall and NoStretch mode and enable analog filter 
    hi2c->Instance->CR1 = 0;

    // Enable the selected I2C peripheral 
    hi2c->Instance->CR1 |= I2CENABLE;

    if(hi2c->Instance == I2C1B){
        NVIC_setprio(I2C1_EV_INTER_NUM, 0, 0);
        NVIC_setprio(I2C1_ER_INTER_NUM, 0, 0);
        NVIC_enable(I2C1_EV_INTER_NUM);
        NVIC_enable(I2C1_ER_INTER_NUM);
    }
    else if(hi2c->Instance == I2C2B){
        NVIC_setprio(I2C2_EV_INTER_NUM, 0, 0);
        NVIC_setprio(I2C2_ER_INTER_NUM, 0, 0);
        NVIC_enable(I2C2_EV_INTER_NUM);
        NVIC_enable(I2C2_ER_INTER_NUM);
    }
    else{
        NVIC_setprio(I2C3_EV_INTER_NUM, 0, 0);
        NVIC_setprio(I2C3_ER_INTER_NUM, 0, 0);
        NVIC_enable(I2C3_EV_INTER_NUM);
        NVIC_enable(I2C3_ER_INTER_NUM);
    }

    atomic_store(&hi2c->state, STATE_READY_I2C);
    return OK;
}

I2C_DMA_RET I2C_DMA_master_tx(uint8_t *buf, uint16_t bufsize, uint8_t slvaddr, I2C_info * hi2c)
{ // assuming timingr is set
    if(!bufsize | !buf | (slvaddr & 0x80) | !hi2c)
        return WRONG_ARGS;

    if(!atomic_cas(&hi2c->state, STATE_BUSY_I2C, STATE_READY_I2C))
        return I2C_BUSY;

    if(!atomic_cas(&hi2c->txhdma.state, STATE_BUSY_I2C, STATE_READY_I2C)){
        atomic_store(&hi2c->state, STATE_READY_I2C);
        return DMA_BUSY;
    }

    uint32_t tmpcr2 = hi2c->Instance->CR2;
    tmpcr2 &= CLEARCR2;

    uint8_t XferSize = 0;
    if(bufsize > MAX_I2C_BUF_SIZE){
        XferSize = MAX_I2C_BUF_SIZE;
        tmpcr2 = I2CRELOADCR2 | I2CAUTOENDCR2;
    }
    else{
        XferSize = bufsize;
        tmpcr2 = I2CAUTOENDCR2;
    }
    
    tmpcr2 |= XferSize << NBYTESSHIFT;
    tmpcr2 |= slvaddr << SLVADDRSHIFT;
    tmpcr2 |= START;

    hi2c->Instance->TXDR = *buf;
    hi2c->BuffPtr = buf + 1;
    --XferSize;
    hi2c->XferCount = bufsize - 1;

    //DMA config

    DMA_info * hdma = &hi2c->txhdma;

    // Disable the peripheral 
    hdma->Instance->CCR &= ~DMAENABLEFLAG;
    __asm__ volatile ("dmb\n");

    // Configure the source, destination address and the data length 
    hdma->DmaBaseAddress->IFCR  = (DMAGLISRFLAG << hdma->ChannelIndex);

    // Configure DMA Channel data length 
    hdma->Instance->CNDTR = hi2c->XferCount;

    // Configure DMA Channel memory address 
    hdma->Instance->CMAR =(uint32_t) hi2c->BuffPtr;

    // Configure DMA Channel peripherial address 
    hdma->Instance->CPAR =(uint32_t) &hi2c->Instance->TXDR;


    // Enable the transfer complete, & transfer error interrupts 
    // Half transfer interrupt is optional: enable it only if associated callback is available 
    if(NULL != hdma->XferHalfCpltCallback )
    {
        hdma->Instance->CCR |= (DMATXCPLT | DMATXHALFCPLT | DMATXERRORFLAG);
    }
    else
    {
        hdma->Instance->CCR |= (DMATXCPLT | DMATXERRORFLAG);
        hdma->Instance->CCR &= ~DMATXHALFCPLT;
    }
    // Enable the Peripheral 
    __asm__ volatile ("dmb\n");
    hdma->Instance->CCR |= DMAENABLEFLAG;
    
    hi2c->XferCount -= XferSize;

    // Enable DMA Request 
    hi2c->Instance->CR1 |= TXDMAENFLAG;

    //enable isrs
    hi2c->Instance->CR1 |= (TCIEFLAG | ERRIEFLAG | NACKFLAG | STOPFLAG);
    __asm__ volatile ("dmb\n");

    //start tx
    hi2c->Instance->CR2 = tmpcr2;

    return OK;
}

I2C_DMA_RET I2C_DMA_master_rx(uint8_t *buf, uint16_t transfersize, uint8_t slvaddr, I2C_info * hi2c)
{
    if(!transfersize | !buf | (slvaddr & 0x80) | !hi2c)
        return WRONG_ARGS;

    if(!atomic_cas(&hi2c->state, STATE_BUSY_I2C, STATE_READY_I2C))
        return I2C_BUSY;

    if(!atomic_cas(&hi2c->rxhdma.state, STATE_BUSY_I2C, STATE_READY_I2C)){
        atomic_store(&hi2c->state, STATE_READY_I2C);
        return DMA_BUSY;
    }

    uint32_t tmpcr2 = hi2c->Instance->CR2;
    tmpcr2 &= CLEARCR2;

    uint8_t XferSize = 0;
    if(transfersize > MAX_I2C_BUF_SIZE){
        XferSize = MAX_I2C_BUF_SIZE;
        tmpcr2 = I2CRELOADCR2 | I2CAUTOENDCR2;
    }
    else{
        XferSize = transfersize;
        tmpcr2 = I2CAUTOENDCR2;
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

    // Disable the peripheral 
    hdma->Instance->CCR &= ~DMAENABLEFLAG;
    __asm__ volatile ("dmb\n");

    // Configure the source, destination address and the data length 
    hdma->DmaBaseAddress->IFCR  = (DMAGLISRFLAG << hdma->ChannelIndex);

    // Configure DMA Channel data length 
    hdma->Instance->CNDTR = hi2c->XferCount;

    // Configure DMA Channel peripherial address 
    hdma->Instance->CPAR =(uint32_t) &hi2c->Instance->RXDR;

    // Configure DMA Channel memory address 
    hdma->Instance->CMAR =(uint32_t) hi2c->BuffPtr;


    // Enable the transfer complete, & transfer error interrupts 
    // Half transfer interrupt is optional: enable it only if associated callback is available 
    if(NULL != hdma->XferHalfCpltCallback )
    {
        hdma->Instance->CCR |= (DMATXCPLT | DMATXHALFCPLT | DMATXERRORFLAG);
    }
    else
    {
        hdma->Instance->CCR |= (DMATXCPLT | DMATXERRORFLAG);
        hdma->Instance->CCR &= ~DMATXHALFCPLT;
    }
    // Enable the Peripheral 
    __asm__ volatile ("dmb\n");
    hdma->Instance->CCR |= DMAENABLEFLAG;
    
    hi2c->XferCount -= XferSize;

    // Enable DMA Request 
    hi2c->Instance->CR1 |= RXDMAENFLAG;

    //enable isrs
    hi2c->Instance->CR1 |= (TCIEFLAG | ERRIEFLAG | NACKFLAG | STOPFLAG);
    __asm__ volatile ("dmb\n");

    //start tx
    hi2c->Instance->CR2 = tmpcr2;

    return OK;
}


