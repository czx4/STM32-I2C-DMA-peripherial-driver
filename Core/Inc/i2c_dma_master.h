#ifndef I2C_DMA_MASTER_H
#define I2C_DMA_MASTER_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

// if specific USING_I2CX commented out its ISR handler wont be defined

#define USING_I2C1
#define USING_I2C2
#define USING_I2C3

#include "i2c_dma_master_defines.h"

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
    MAX_I2Cs = 3,
    I2C1_POS = 0,
    I2C2_POS = 1,
    I2C3_POS = 2,
    STATE_UNINIT_I2C = 0,
    STATE_READY_I2C = 1,
    STATE_BUSY_I2C = 2,

    I2C1_EV_INTER_NUM                = 31,     // I2C1 Event Interrupt & EXTI Line23 Interrupt
    I2C1_ER_INTER_NUM                = 32,     // I2C1 Error Interrupt                        
    I2C2_EV_INTER_NUM                = 33,     // I2C2 Event Interrupt & EXTI Line24 Interrupt
    I2C2_ER_INTER_NUM                = 34,     // I2C2 Error Interrupt                        
    I2C3_EV_INTER_NUM                = 72,     // I2C3 event interrupt                        
    I2C3_ER_INTER_NUM                = 73,     // I2C3 Error Interrupt                        

    RCCI2C1_ENABLE              = (1 << 21),
    RCCI2C2_ENABLE              = (1 << 22),
    RCCI2C3_ENABLE              = (1 << 30),

    STOPGENERATECR2             = (1 << 14),

    NACKFLAG                    = (1 << 4),
    TCRFLAG                     = (1 << 7),
    TCIEFLAG                    = (1 << 6),
    STOPFLAG                    = (1 << 5),
    ERRIEFLAG                   = (1 << 7), 
    BERRFLAG                    = (1 << 8),
    ARLOFLAG                    = (1 << 9),

    DMATXERRORFLAG              = (1 << 3),
    DMATXHALFCPLT               = (1 << 2),
    DMATXCPLT                   = (1 << 1),
    DMACCRCIRC                  = (1 << 5),
    DMAGLISRFLAG                = 1,
    CLEARDMACCR                 = ~0x7FFE,
    DMAENABLEFLAG               = 0x1,
    TXDMAENFLAG                 = (1 << 14),
    RXDMAENFLAG                 = (1 << 15),
};

typedef struct{
    DMA_Channel_Def       *Instance; // register base address
    uint32_t               state;
    DMA_Def               *DmaBaseAddress;                // DMA Channel Base Address               
    uint32_t               ChannelIndex;
    void                (* XferCpltCallback)(void);     // DMA transfer complete user callback         
    void                (* XferHalfCpltCallback)(void); // DMA Half transfer complete user callback    
    void                (* XferErrorCallback)(void);    // DMA transfer error user callback            
}DMA_info; // User should only set Instance and optionally set callbacks

typedef struct{
    I2C_Def           *Instance; // register base address
    uint8_t           *BuffPtr;
    volatile uint16_t  XferCount;
    uint32_t           state;
    DMA_info           txhdma;
    DMA_info           rxhdma;
}I2C_info;

extern I2C_info I2Cs[MAX_I2Cs];

#ifdef USING_I2C1
extern void I2C1_EV_IRQHandler(void);
extern void I2C1_ER_IRQHandler(void);
#endif

#ifdef USING_I2C2
extern void I2C2_EV_IRQHandler(void);
extern void I2C2_ER_IRQHandler(void);
#endif

#ifdef USING_I2C3
extern void I2C3_EV_IRQHandler(void);
extern void I2C3_ER_IRQHandler(void);
#endif

extern I2C_DMA_RET I2C_DMA_Init(I2C_Def              *I2C_instance,
                                GPIO_Def             *GPIOONE,
                                uint8_t               pinonenum,
                                GPIO_Def             *GPIOTWO, 
                                uint8_t               pintwonum, 
                                DMA_info              TX_DMA_channel, // with only Instance and optional callbacks set
                                DMA_info              RX_DMA_channel,// with only Instance and optional callbacks set
                                bool                  I2C_10bit_adressing);
extern I2C_DMA_RET I2C_DMA_master_tx(uint8_t *buf,uint16_t bufsize, uint8_t slvaddr, I2C_info * hi2c);
extern I2C_DMA_RET I2C_DMA_master_rx(uint8_t *buf, uint16_t transfersize, uint8_t slvaddr, I2C_info * hi2c);

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

    atomic_store(&hdma->state, STATE_READY_I2C);
}

inline void __attribute__((always_inline)) I2C_DMA_ISR_handler(DMA_info * hdma, I2C_info * hi2c){
    uint32_t flag_it = hdma->DmaBaseAddress->ISR;
    uint32_t source_it = hdma->Instance->CCR;

    // Half Transfer Complete Interrupt management
    if ((flag_it & (DMATXHALFCPLT << hdma->ChannelIndex)) && (source_it & DMATXHALFCPLT))
    {
        // Disable the half transfer interrupt if the DMA mode is not CIRCULAR 
        if(!(source_it & DMACCRCIRC))
        {
            // Disable the half transfer interrupt 
            hdma->Instance->CCR &= ~DMATXHALFCPLT;
        }

        // Clear the half transfer complete flag 
        hdma->DmaBaseAddress->IFCR = DMATXHALFCPLT << hdma->ChannelIndex;

        // DMA peripheral state is not updated in Half Transfer 
        // State is updated only in Transfer Complete case 

        if(hdma->XferHalfCpltCallback != NULL)
        {
            hdma->XferHalfCpltCallback();
        }
    }

    // Transfer Complete Interrupt management
    else if ((flag_it & (DMATXCPLT << hdma->ChannelIndex)) && (source_it & DMATXCPLT))
    {
        if(!(source_it & DMACCRCIRC))
        {
            /* Disable the transfer complete  & transfer error interrupts 
               if the DMA mode is not CIRCULAR */
            hdma->Instance->CCR &= ~(DMATXCPLT | DMATXERRORFLAG);

        }

        // Clear the transfer complete flag 
        hdma->DmaBaseAddress->IFCR = DMATXCPLT << hdma->ChannelIndex;

        if(hdma->XferCpltCallback != NULL)
        {
            hdma->XferCpltCallback();
        }
    }

    // Transfer Error Interrupt management
    else if ((flag_it & (DMATXERRORFLAG << hdma->ChannelIndex)) && (source_it & DMATXERRORFLAG))
    {
        /*  When a DMA transfer error occurs 
            A hardware clear of its EN bits is performed 
            Then, disable all DMA interrupts */
        hdma->Instance->CCR &= ~(DMATXCPLT | DMATXHALFCPLT | DMATXERRORFLAG);

        // Clear all flags 
        hdma->DmaBaseAddress->IFCR = DMAGLISRFLAG << hdma->ChannelIndex;
        
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