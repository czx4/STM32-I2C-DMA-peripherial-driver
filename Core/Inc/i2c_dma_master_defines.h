#ifndef I2C_DMA_MASTER_DEFS
#define I2C_DMA_MASTER_DEFS

#define NVIC_PRIO_BITS 4

#define SCS_BASE            (0xE000E000UL)                            // System Control Space Base Address 
#define NVIC_BASE           (SCS_BASE +  0x0100UL)                    // NVIC Base Address
#define SCB_BASE            (SCS_BASE +  0x0D00UL)                    // System Control Block Base Address

#define SCB_AIRCR_PRIOGR_POS              8                                            // SCB AIRCR: PRIGROUP Position 
#define SCB_AIRCR_PRIOGR_MASK             (7 << SCB_AIRCR_PRIGROUP_Pos)                // SCB AIRCR: PRIGROUP Mask 
typedef struct
{
    volatile const  uint32_t CPUID;           // Offset: 0x000 (R/ )  CPUID Base Register 
    volatile uint32_t ICSR;                   // Offset: 0x004 (R/W)  Interrupt Control and State Register 
    volatile uint32_t VTOR;                   // Offset: 0x008 (R/W)  Vector Table Offset Register 
    volatile uint32_t AIRCR;                  // Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register 
    volatile uint32_t SCR;                    // Offset: 0x010 (R/W)  System Control Register 
    volatile uint32_t CCR;                    // Offset: 0x014 (R/W)  Configuration Control Register 
    volatile uint8_t  SHP[12U];               // Offset: 0x018 (R/W)  System Handlers Priority Registers (4-7, 8-11, 12-15) 
    volatile uint32_t SHCSR;                  // Offset: 0x024 (R/W)  System Handler Control and State Register 
    volatile uint32_t CFSR;                   // Offset: 0x028 (R/W)  Configurable Fault Status Register 
    volatile uint32_t HFSR;                   // Offset: 0x02C (R/W)  HardFault Status Register 
    volatile uint32_t DFSR;                   // Offset: 0x030 (R/W)  Debug Fault Status Register 
    volatile uint32_t MMFAR;                  // Offset: 0x034 (R/W)  MemManage Fault Address Register 
    volatile uint32_t BFAR;                   // Offset: 0x038 (R/W)  BusFault Address Register 
    volatile uint32_t AFSR;                   // Offset: 0x03C (R/W)  Auxiliary Fault Status Register 
    volatile const uint32_t PFR[2U];          // Offset: 0x040 (R/ )  Processor Feature Register 
    volatile const uint32_t DFR;              // Offset: 0x048 (R/ )  Debug Feature Register 
    volatile const uint32_t ADR;              // Offset: 0x04C (R/ )  Auxiliary Feature Register 
    volatile const uint32_t MMFR[4U];         // Offset: 0x050 (R/ )  Memory Model Feature Register 
    volatile const uint32_t ISAR[5U];         // Offset: 0x060 (R/ )  Instruction Set Attributes Register 
        uint32_t RESERVED0[5U];
    volatile uint32_t CPACR;                  // Offset: 0x088 (R/W)  Coprocessor Access Control Register 
} SCB_Def;

typedef struct
{
    volatile uint32_t ISER[8U];               // Offset: 0x000 (R/W)  Interrupt Set Enable Register 
        uint32_t RESERVED0[24U];
    volatile uint32_t ICER[8U];               // Offset: 0x080 (R/W)  Interrupt Clear Enable Register 
        uint32_t RSERVED1[24U];
    volatile uint32_t ISPR[8U];               // Offset: 0x100 (R/W)  Interrupt Set Pending Register 
        uint32_t RESERVED2[24U];
    volatile uint32_t ICPR[8U];               // Offset: 0x180 (R/W)  Interrupt Clear Pending Register 
        uint32_t RESERVED3[24U];
    volatile uint32_t IABR[8U];               // Offset: 0x200 (R/W)  Interrupt Active bit Register 
        uint32_t RESERVED4[56U];
    volatile uint8_t  IP[240U];               // Offset: 0x300 (R/W)  Interrupt Priority Register (8Bit wide) 
        uint32_t RESERVED5[644U];
    volatile uint32_t STIR;                   // Offset: 0xE00 ( /W)  Software Trigger Interrupt Register 
}  NVIC_Def;

#define NVICB                ((NVIC_Def      *)     NVIC_BASE     )   // NVIC configuration struct 
#define SCBB                 ((SCB_Def       *)     SCB_BASE      )   // SCB configuration struct 

#define PERIPH_BASE           0x40000000UL // Peripheral base address in the alias region 
#define APB1PERIPH_BASE       PERIPH_BASE
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000UL)
#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x08000000UL)

#define RCC_BASE              (AHB1PERIPH_BASE + 0x00001000UL)

#define DMA1_BASE             (AHB1PERIPH_BASE + 0x00000000UL)
#define DMA1_Channel1_BASE    (AHB1PERIPH_BASE + 0x00000008UL)
#define DMA1_Channel2_BASE    (AHB1PERIPH_BASE + 0x0000001CUL)
#define DMA1_Channel3_BASE    (AHB1PERIPH_BASE + 0x00000030UL)
#define DMA1_Channel4_BASE    (AHB1PERIPH_BASE + 0x00000044UL)
#define DMA1_Channel5_BASE    (AHB1PERIPH_BASE + 0x00000058UL)
#define DMA1_Channel6_BASE    (AHB1PERIPH_BASE + 0x0000006CUL)
#define DMA1_Channel7_BASE    (AHB1PERIPH_BASE + 0x00000080UL)
#define DMA2_BASE             (AHB1PERIPH_BASE + 0x00000400UL)
#define DMA2_Channel1_BASE    (AHB1PERIPH_BASE + 0x00000408UL)
#define DMA2_Channel2_BASE    (AHB1PERIPH_BASE + 0x0000041CUL)
#define DMA2_Channel3_BASE    (AHB1PERIPH_BASE + 0x00000430UL)
#define DMA2_Channel4_BASE    (AHB1PERIPH_BASE + 0x00000444UL)
#define DMA2_Channel5_BASE    (AHB1PERIPH_BASE + 0x00000458UL)

#define I2C1_BASE             (APB1PERIPH_BASE + 0x00005400UL)
#define I2C2_BASE             (APB1PERIPH_BASE + 0x00005800UL)
#define I2C3_BASE             (APB1PERIPH_BASE + 0x00007800UL)

// AHB2 peripherals 
#define GPIOA_BASE            (AHB2PERIPH_BASE + 0x00000000UL)
#define GPIOB_BASE            (AHB2PERIPH_BASE + 0x00000400UL)
#define GPIOC_BASE            (AHB2PERIPH_BASE + 0x00000800UL)
#define GPIOD_BASE            (AHB2PERIPH_BASE + 0x00000C00UL)
#define GPIOE_BASE            (AHB2PERIPH_BASE + 0x00001000UL)
#define GPIOF_BASE            (AHB2PERIPH_BASE + 0x00001400UL)
#define GPIOG_BASE            (AHB2PERIPH_BASE + 0x00001800UL)
#define GPIOH_BASE            (AHB2PERIPH_BASE + 0x00001C00UL)

typedef struct
{
    volatile uint32_t CR;         // RCC clock control register,                                  Address offset: 0x00 
    volatile uint32_t CFGR;       // RCC clock configuration register,                            Address offset: 0x04 
    volatile uint32_t CIR;        // RCC clock interrupt register,                                Address offset: 0x08 
    volatile uint32_t APB2RSTR;   // RCC APB2 peripheral reset register,                          Address offset: 0x0C 
    volatile uint32_t APB1RSTR;   // RCC APB1 peripheral reset register,                          Address offset: 0x10 
    volatile uint32_t AHBENR;     // RCC AHB peripheral clock register,                           Address offset: 0x14 
    volatile uint32_t APB2ENR;    // RCC APB2 peripheral clock enable register,                   Address offset: 0x18 
    volatile uint32_t APB1ENR;    // RCC APB1 peripheral clock enable register,                   Address offset: 0x1C 
    volatile uint32_t BDCR;       // RCC Backup domain control register,                          Address offset: 0x20 
    volatile uint32_t CSR;        // RCC clock control & status register,                         Address offset: 0x24 
    volatile uint32_t AHBRSTR;    // RCC AHB peripheral reset register,                           Address offset: 0x28 
    volatile uint32_t CFGR2;      // RCC clock configuration register 2,                          Address offset: 0x2C 
    volatile uint32_t CFGR3;      // RCC clock configuration register 3,                          Address offset: 0x30 
}RCC_Def;

typedef struct
{
    volatile uint32_t ISR;          // DMA interrupt status register,     Address offset: 0x00 
    volatile uint32_t IFCR;         // DMA interrupt flag clear register, Address offset: 0x04 
}DMA_Def;

typedef struct
{
    volatile uint32_t CCR;          // DMA channel x configuration register                                           
    volatile uint32_t CNDTR;        // DMA channel x number of data register                                          
    volatile uint32_t CPAR;         // DMA channel x peripheral address register                                      
    volatile uint32_t CMAR;         // DMA channel x memory address register                                          
}DMA_Channel_Def;


typedef struct
{
    volatile uint32_t CR1;      // I2C Control register 1,            Address offset: 0x00 
    volatile uint32_t CR2;      // I2C Control register 2,            Address offset: 0x04 
    volatile uint32_t OAR1;     // I2C Own address 1 register,        Address offset: 0x08 
    volatile uint32_t OAR2;     // I2C Own address 2 register,        Address offset: 0x0C 
    volatile uint32_t TIMINGR;  // I2C Timing register,               Address offset: 0x10 
    volatile uint32_t TIMEOUTR; // I2C Timeout register,              Address offset: 0x14 
    volatile uint32_t ISR;      // I2C Interrupt and status register, Address offset: 0x18 
    volatile uint32_t ICR;      // I2C Interrupt clear register,      Address offset: 0x1C 
    volatile uint32_t PECR;     // I2C PEC register,                  Address offset: 0x20 
    volatile uint32_t RXDR;     // I2C Receive data register,         Address offset: 0x24 
    volatile uint32_t TXDR;     // I2C Transmit data register,        Address offset: 0x28 
}I2C_Def;

#define RCCB                 ((RCC_Def *) RCC_BASE)

#define DMA1B                ((DMA_Def *) DMA1_BASE)
#define DMA1Ch1       ((DMA_Channel_Def *) DMA1_Channel1_BASE)
#define DMA1Ch2       ((DMA_Channel_Def *) DMA1_Channel2_BASE)
#define DMA1Ch3       ((DMA_Channel_Def *) DMA1_Channel3_BASE)
#define DMA1Ch4       ((DMA_Channel_Def *) DMA1_Channel4_BASE)
#define DMA1Ch5       ((DMA_Channel_Def *) DMA1_Channel5_BASE)
#define DMA1Ch6       ((DMA_Channel_Def *) DMA1_Channel6_BASE)
#define DMA1Ch7       ((DMA_Channel_Def *) DMA1_Channel7_BASE)
#define DMA2B                ((DMA_Def *) DMA2_BASE)
#define DMA2Ch1       ((DMA_Channel_Def *) DMA2_Channel1_BASE)
#define DMA2Ch2       ((DMA_Channel_Def *) DMA2_Channel2_BASE)
#define DMA2Ch3       ((DMA_Channel_Def *) DMA2_Channel3_BASE)
#define DMA2Ch4       ((DMA_Channel_Def *) DMA2_Channel4_BASE)
#define DMA2Ch5       ((DMA_Channel_Def *) DMA2_Channel5_BASE)

#define I2C1B                ((I2C_Def *) I2C1_BASE)
#define I2C2B                ((I2C_Def *) I2C2_BASE)
#define I2C3B                ((I2C_Def *) I2C3_BASE)

#define SCS_BASE            (0xE000E000UL)                            // System Control Space Base Address 

typedef struct
{
    volatile uint32_t MODER;        // GPIO port mode register,               Address offset: 0x00      
    volatile uint32_t OTYPER;       // GPIO port output type register,        Address offset: 0x04      
    volatile uint32_t OSPEEDR;      // GPIO port output speed register,       Address offset: 0x08      
    volatile uint32_t PUPDR;        // GPIO port pull-up/pull-down register,  Address offset: 0x0C      
    volatile uint32_t IDR;          // GPIO port input data register,         Address offset: 0x10      
    volatile uint32_t ODR;          // GPIO port output data register,        Address offset: 0x14      
    volatile uint32_t BSRR;         // GPIO port bit set/reset register,      Address offset: 0x1A 
    volatile uint32_t LCKR;         // GPIO port configuration lock register, Address offset: 0x1C      
    volatile uint32_t AFR[2];       // GPIO alternate function registers,     Address offset: 0x20-0x24 
    volatile uint32_t BRR;          // GPIO bit reset register,               Address offset: 0x28 
}GPIO_Def;

#define GPIO_A               ((GPIO_Def *) GPIOA_BASE)
#define GPIO_B               ((GPIO_Def *) GPIOB_BASE)
#define GPIO_C               ((GPIO_Def *) GPIOC_BASE)
#define GPIO_D               ((GPIO_Def *) GPIOD_BASE)
#define GPIO_E               ((GPIO_Def *) GPIOE_BASE)
#define GPIO_F               ((GPIO_Def *) GPIOF_BASE)
#define GPIO_G               ((GPIO_Def *) GPIOG_BASE)
#define GPIO_H               ((GPIO_Def *) GPIOH_BASE)

#endif