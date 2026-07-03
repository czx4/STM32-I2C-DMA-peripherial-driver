#ifndef I2C_DMA_MASTER_H
#define I2C_DMA_MASTER_H
// #include "stm32f3xx_hal_i2c.h"
#include "main.h"
#include "stm32f3xx_hal_dma.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum __attribute__((packed)){
    OK,
    WRONG_ARGS,
    ALREADY_INITILIZED,
    I2C_BUSY,
    DMA_BUSY,
    I2C_INIT_ERROR,
    DMA_INIT_ERROR
}I2C_DMA_RET;

enum{
    MAX_I2Cs = 2,
    I2C1_POS = 0,
    I2C2_POS = 1,
    STATE_UNINIT = 0,
    STATE_READY = 1,
    STATE_BUSY = 2,
    DMA_UNINIT_STATE = 0,
    DMA_READY_STATE  = 1,
    DMA_BUSY_STATE   = 2,
    CLEARDMACCR                 = ~0x7FFE,
    DMAENABLEFLAG               = 0x1
};

enum{
    MAX_I2C_BUF_SIZE            = 0xFF,
    NBYTESSHIFT                 = 16,
    START                       = (1 << 13),
    SLVADDRSHIFT                = 1,
    CLEARISR                    = 0x3F38,
    NACKFLAG                    = 0x10,
    TCRFLAG                     = 0x80,
    TCIEFLAG                    = 0x40,
    STOPFLAG                    = 0x20,
    ERRIEFLAG                   = 0x80, 
    BERRFLAG                    = 0x100,
    ARLOFLAG                    = 0x200,
    TXIEFLAG                    = 0x2,
    // CLEARDMACCR                 = ~0x7FFE,
    CLEARCR2                    = ~0x2FF67FF,
    STOPGENERATECR2             = 0x4000,
    TXDMAENFLAG                 = 0x4000,
    RXDMAENFLAG                 = (1 << 15),
    // DMAENABLEFLAG               = 0x1,
    DMATXERRORFLAG              = 0x8,
    DMATXHALFCPLT               = 0x4,
    DMATXCPLT                   = 0x2,
    I2CENABLE                   = 0x1,
    GPIO_AFR_I2CAF              = 0x4,
    I2C_PIN_POS                 = 0x8,
    GPIO_PIN_HIGH_SPD           = 0x3,
    GPIO_PIN_OPEN_DR            = 0x1,
    GPIO_AF_MODE                = 0x2,
    GPIO_AFR_RESET              = ~0xFF,
    GPIO_AFR_ALL_SET            = 0xF,
    NORMALIZE_PINS_0_TO_7_MASK  = 0x7,
    GPIO_AFR_BITCOUNT           = 0x4,
    GPIO_MODER_SET              = 0x3,
    DUALADDRESS_ENABLE          = 0x8000,
    I2CNACKCR2                  = (1 << 15),
    I2CAUTOENDCR2               = (1 << 25),
    I2CADD10ENABLE              = (1 << 11),
    OWNADDRENABLE               = (1 << 15),
    DMAMEMINCENABLE             = (1 << 7), 
    DMADATAALIGNBYTE            = 0,
    DMANOTCIRCULAR              = 0,
    DMAMEMTOPERIPH_DIR          = (1 << 4),
    I2C_READREQCR2              = (1 << 10),
    I2C_7BITADDR_REC            = (1 << 12),
    STANDARD_I2C_TIMING         = 0x00201D2B //TIMINGR register value for stm32f303RE, I2C uses HSI 8Mhz clock by default
};

typedef struct{
    DMA_Channel_TypeDef   *Instance; // register base address
    uint32_t               state;
    DMA_TypeDef           *DmaBaseAddress;                // DMA Channel Base Address               
    uint32_t               ChannelIndex;
    void                  (* XferCpltCallback)(void);     // DMA transfer complete callback         
    void                  (* XferHalfCpltCallback)(void); // DMA Half transfer complete callback    
    void                  (* XferErrorCallback)(void);    // DMA transfer error callback            
}DMA_info;

typedef struct{
    I2C_TypeDef       *Instance; // register base address
    uint8_t           *BuffPtr;
    volatile uint16_t XferCount;
    uint32_t          state;
    DMA_info          txhdma;
    DMA_info          rxhdma;
}I2C_info;

extern I2C_info I2Cs[MAX_I2Cs];

void I2C1_EV_IRQHandler(void);
void I2C1_ER_IRQHandler(void);
extern I2C_DMA_RET I2C_DMA_Init(I2C_TypeDef          *I2C_instance,
                                GPIO_TypeDef         *GPIOONE,
                                uint8_t               pinonenum,
                                GPIO_TypeDef         *GPIOTWO, 
                                uint8_t               pintwonum, 
                                DMA_info              TX_DMA_channel, // with only Instance and optional callbacks set
                                DMA_info              RX_DMA_channel,// with only Instance and optional callbacks set
                                bool                  I2C_10bit_adressing);
extern I2C_DMA_RET I2C_DMA_master_tx(uint8_t *buf,uint16_t bufsize, uint8_t slvaddr, I2C_info * hi2c);

inline void __attribute__((always_inline)) atomic_store(uint32_t * ptr, uint32_t value){
    __asm__ volatile(
        "1:\n"
        "ldrex r2, [%0]\n"
        "strex r3, %1, [%0]\n"
        "cmp r3, #0\n"
        "bne 1b\n"
        "dmb \n"
        :
        :"r"(ptr), "r"(value)
        :"r2", "r3", "cc", "memory"
    );
}

inline uint32_t __attribute__((always_inline)) atomic_cas(uint32_t * ptr,uint32_t value_to_store, uint32_t expected_value){
    uint32_t result;
    __asm__ volatile(
        "2:\n"
        "ldrex r2, [%1]\n"
        "cmp r2, %3\n"
        "bne 3f\n"
        "strex r3, %2, [%1]\n"
        "cmp r3, #0\n"
        "bne 2b\n"

        "dmb \n"
        "mov %0, #1\n"
        "b 4f\n"

        "3:\n"
        "clrex\n"
        "mov %0, #0\n"

        "4:\n"
        :"=&r"(result)
        :"r"(ptr), "r"(value_to_store), "r"(expected_value)
        :"r2","r3", "cc", "memory"
    );
    return result;
}

inline void __attribute__((always_inline)) disable_dma(DMA_info * hdma){
    //disable dma
    hdma->Instance->CCR &= ~DMAENABLEFLAG;
    __asm__ volatile ("dmb\n");

    //clear dma config
    hdma->Instance->CCR &= CLEARDMACCR;

    //clear dma memory addr
    hdma->Instance->CMAR = 0;

    //clear dma peripherial addr
    hdma->Instance->CPAR = 0;

    //clear dma number of data to transfer
    hdma->Instance->CNDTR = 0;

    __asm__ volatile ("dmb\n");
    atomic_store(&hdma->state, DMA_READY_STATE); //TODO change to my structure
}

inline void __attribute__((always_inline)) I2C_DMA_ISR_handler(DMA_info * hdma, I2C_info * hi2c){
    uint32_t flag_it = hdma->DmaBaseAddress->ISR;
    uint32_t source_it = hdma->Instance->CCR;

    // Half Transfer Complete Interrupt management
    if ((flag_it & (DMA_FLAG_HT1 << hdma->ChannelIndex)) && (source_it & DMA_IT_HT))
    {
        /* Disable the half transfer interrupt if the DMA mode is not CIRCULAR */
        if(!(source_it & DMA_CCR_CIRC))
        {
            /* Disable the half transfer interrupt */
            hdma->Instance->CCR &= ~DMA_IT_HT;
        }

        // Clear the half transfer complete flag 
        hdma->DmaBaseAddress->IFCR = DMA_FLAG_HT1 << hdma->ChannelIndex;

        // DMA peripheral state is not updated in Half Transfer 
        // State is updated only in Transfer Complete case 

        if(hdma->XferHalfCpltCallback != NULL)
        {
            hdma->XferHalfCpltCallback();
        }
    }

    // Transfer Complete Interrupt management
    else if ((flag_it & (DMA_FLAG_TC1 << hdma->ChannelIndex)) && (source_it & DMA_IT_TC))
    {
        if(!(source_it & DMA_CCR_CIRC))
        {
            /* Disable the transfer complete  & transfer error interrupts 
               if the DMA mode is not CIRCULAR */
            hdma->Instance->CCR &= ~(DMA_IT_TC | DMA_IT_TE);

        }

        // Clear the transfer complete flag 
        hdma->DmaBaseAddress->IFCR = DMA_FLAG_TC1 << hdma->ChannelIndex;

        if(hdma->XferCpltCallback != NULL)
        {
            hdma->XferCpltCallback();
        }
    }

    // Transfer Error Interrupt management
    else if ((flag_it & (DMA_FLAG_TE1 << hdma->ChannelIndex)) && (source_it & DMA_IT_TE))
    {
        /*  When a DMA transfer error occurs 
            A hardware clear of its EN bits is performed 
            Then, disable all DMA interrupts */
        hdma->Instance->CCR &= ~(DMA_IT_TC | DMA_IT_HT | DMA_IT_TE);

        // Clear all flags 
        hdma->DmaBaseAddress->IFCR = DMA_FLAG_GL1 << hdma->ChannelIndex;
        
        //disable dma requests
        hi2c->Instance->CR1 &= ~(TXDMAENFLAG | RXDMAENFLAG);
        //enable stop interrupts
        hi2c->Instance->CR1 |= (STOPFLAG | TCIEFLAG);

        //stop generation
        hi2c->Instance->CR2 |= STOPGENERATECR2;

        if(hdma->XferErrorCallback != NULL)
        {
            hdma->XferErrorCallback();
        }
    }
}


#endif