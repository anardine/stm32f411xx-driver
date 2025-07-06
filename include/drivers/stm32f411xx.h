/*
 * stm32f411xx.h
 *
 *  Created on: Apr 5, 2025
 *      Author: anardinelli
 */

 #ifndef INC_STM32F411XX_H_
 #define INC_STM32F411XX_H_
 
 #include <stdint.h>
 
 #define ENABLE						1
 #define DISABLE 					0
 #define SET						ENABLE
 #define RESET						DISABLE


// definition of Core M4
//------------------------------------------

#define NVIC_BASEADDR               0xE000E100UL


typedef struct {
    volatile uint32_t NVIC_ISER[7];
    volatile uint32_t NVIC_ICER[7];
    volatile uint32_t NVIC_ISPR[7];
    volatile uint32_t NVIC_IABR[7];
    volatile uint32_t NVIC_IPR[59];
    volatile uint32_t NVIC_STIR;
}NVIC_MapR_t;

typedef enum
{
  /******  Cortex-M4 Processor Exceptions Numbers ****************************************************************/
  NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                                          */
  MemoryManagement_IRQn       = -12,    /*!< 4 Cortex-M4 Memory Management Interrupt                           */
  BusFault_IRQn               = -11,    /*!< 5 Cortex-M4 Bus Fault Interrupt                                   */
  UsageFault_IRQn             = -10,    /*!< 6 Cortex-M4 Usage Fault Interrupt                                 */
  SVCall_IRQn                 = -5,     /*!< 11 Cortex-M4 SV Call Interrupt                                    */
  DebugMonitor_IRQn           = -4,     /*!< 12 Cortex-M4 Debug Monitor Interrupt                              */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M4 Pend SV Interrupt                                    */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M4 System Tick Interrupt                                */
  /******  STM32 specific Interrupt Numbers **********************************************************************/
  WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                                         */
  PVD_IRQn                    = 1,      /*!< PVD through EXTI Line detection Interrupt                         */
  TAMP_STAMP_IRQn             = 2,      /*!< Tamper and TimeStamp interrupts through the EXTI line             */
  RTC_WKUP_IRQn               = 3,      /*!< RTC Wakeup interrupt through the EXTI line                        */
  FLASH_IRQn                  = 4,      /*!< FLASH global Interrupt                                            */
  RCC_IRQn                    = 5,      /*!< RCC global Interrupt                                              */
  EXTI0_IRQn                  = 6,      /*!< EXTI Line0 Interrupt                                              */
  EXTI1_IRQn                  = 7,      /*!< EXTI Line1 Interrupt                                              */
  EXTI2_IRQn                  = 8,      /*!< EXTI Line2 Interrupt                                              */
  EXTI3_IRQn                  = 9,      /*!< EXTI Line3 Interrupt                                              */
  EXTI4_IRQn                  = 10,     /*!< EXTI Line4 Interrupt                                              */
  DMA1_Stream0_IRQn           = 11,     /*!< DMA1 Stream 0 global Interrupt                                    */
  DMA1_Stream1_IRQn           = 12,     /*!< DMA1 Stream 1 global Interrupt                                    */
  DMA1_Stream2_IRQn           = 13,     /*!< DMA1 Stream 2 global Interrupt                                    */
  DMA1_Stream3_IRQn           = 14,     /*!< DMA1 Stream 3 global Interrupt                                    */
  DMA1_Stream4_IRQn           = 15,     /*!< DMA1 Stream 4 global Interrupt                                    */
  DMA1_Stream5_IRQn           = 16,     /*!< DMA1 Stream 5 global Interrupt                                    */
  DMA1_Stream6_IRQn           = 17,     /*!< DMA1 Stream 6 global Interrupt                                    */
  ADC_IRQn                    = 18,     /*!< ADC1, ADC2 and ADC3 global Interrupts                             */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                                     */
  TIM1_BRK_TIM9_IRQn          = 24,     /*!< TIM1 Break interrupt and TIM9 global interrupt                    */
  TIM1_UP_TIM10_IRQn          = 25,     /*!< TIM1 Update Interrupt and TIM10 global interrupt                  */
  TIM1_TRG_COM_TIM11_IRQn     = 26,     /*!< TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                                    */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                             */
  TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                             */
  TIM4_IRQn                   = 30,     /*!< TIM4 global Interrupt                                             */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                              */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                              */
  I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                              */
  I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                              */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                             */
  SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                             */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                                           */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                                           */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                                   */
  RTC_Alarm_IRQn              = 41,     /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
  OTG_FS_WKUP_IRQn            = 42,     /*!< USB OTG FS Wakeup through EXTI line interrupt                     */
  DMA1_Stream7_IRQn           = 47,     /*!< DMA1 Stream7 Interrupt                                            */
  SDIO_IRQn                   = 49,     /*!< SDIO global Interrupt                                             */
  TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                                             */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                             */
  DMA2_Stream0_IRQn           = 56,     /*!< DMA2 Stream 0 global Interrupt                                    */
  DMA2_Stream1_IRQn           = 57,     /*!< DMA2 Stream 1 global Interrupt                                    */
  DMA2_Stream2_IRQn           = 58,     /*!< DMA2 Stream 2 global Interrupt                                    */
  DMA2_Stream3_IRQn           = 59,     /*!< DMA2 Stream 3 global Interrupt                                    */
  DMA2_Stream4_IRQn           = 60,     /*!< DMA2 Stream 4 global Interrupt                                    */
  OTG_FS_IRQn                 = 67,     /*!< USB OTG FS global Interrupt                                       */
  DMA2_Stream5_IRQn           = 68,     /*!< DMA2 Stream 5 global interrupt                                    */
  DMA2_Stream6_IRQn           = 69,     /*!< DMA2 Stream 6 global interrupt                                    */
  DMA2_Stream7_IRQn           = 70,     /*!< DMA2 Stream 7 global interrupt                                    */
  USART6_IRQn                 = 71,     /*!< USART6 global interrupt                                           */
  I2C3_EV_IRQn                = 72,     /*!< I2C3 event interrupt                                              */
  I2C3_ER_IRQn                = 73,     /*!< I2C3 error interrupt                                              */
  FPU_IRQn                    = 81,     /*!< FPU global interrupt                                              */
  SPI4_IRQn                   = 84,     /*!< SPI4 global Interrupt                                             */
  SPI5_IRQn                   = 85      /*!< SPI5 global Interrupt                                              */
} IRQn_MapR_t;

typedef struct {
    IRQn_MapR_t IRQn;
    uint16_t posArray[6];
} IRQn_Handler_t;


#define NVIC                        ((NVIC_MapR_t *) NVIC_BASEADDR)


 
// definition of STM32
//------------------------------------------

 // keep all the base addresses of the microcontroller
 
 #define FLASH_BASEADDR				0x08000000UL
 #define FLASH						FLASH_BASEADDR
 
 #define SRAM1_BASEADDR				0x20000000UL
 #define SRAM 						SRAM1_BASEADDR
 
 #define ROM_BASEADDR				0x1FFF0000UL
 #define ROM						ROM_BASEADDR
 
 #define OTP_BASEADDR				0x1FFF7800UL
 
 // base addresses of each bus domains
 #define PERIPH_BASEADDR			0x40000000UL
 #define APB1PERIPH_BASE			0x40000000UL
 #define APB2PERIPH_BASE			0x40010000UL
 #define AHB1PERIPH_BASE			0x40020000UL
 #define AHB2PERIPH_BASE			0x50000000UL

 // base address for EXT interrupt line
 #define EXTI_BASEADDR              (APB2PERIPH_BASE + 0x3C00)
 
 // base address for RTC Clock
 #define RTC_BASEADDR				(APB1PERIPH_BASE + 0x2800)
 
 // base address for RCC Clock
 #define RCC_BASEADDR				(AHB1PERIPH_BASE + 0x3800)
 
 // base addresses of GPIOx on AHB1 peripheral
 #define GPIOA_BASEADDR				(AHB1PERIPH_BASE + 0x0000)
 #define GPIOB_BASEADDR				(AHB1PERIPH_BASE + 0x0400)
 #define GPIOC_BASEADDR				(AHB1PERIPH_BASE + 0x0800)
 #define GPIOD_BASEADDR				(AHB1PERIPH_BASE + 0x0C00)
 #define GPIOE_BASEADDR				(AHB1PERIPH_BASE + 0x1000)
 #define GPIOH_BASEADDR				(AHB1PERIPH_BASE + 0x1C00)
 
 // base addresses of components for the APB1 peripheral
 #define SPI2_BASEADDR				(APB1PERIPH_BASE + 0x3800)
 #define SPI3_BASEADDR				(APB1PERIPH_BASE + 0x3C00)
 #define USART2_BASEADDR			(APB1PERIPH_BASE + 0x4400)
 #define I2C1_BASEADDR				(APB1PERIPH_BASE + 0x5400)
 #define I2C2_BASEADDR				(APB1PERIPH_BASE + 0x5800)
 #define I2C3_BASEADDR				(APB1PERIPH_BASE + 0x5C00)
 
 // base addresses of components for the APB2 peripheral
 #define USART1_BASEADDR			(APB2PERIPH_BASE + 0x1000)
 #define USART6_BASEADDR			(APB2PERIPH_BASE + 0x1400)
 #define SPI1_BASEADDR				(APB2PERIPH_BASE + 0x3000)
 #define SPI4_BASEADDR				(APB2PERIPH_BASE + 0x3400)
 #define SPI5_BASEADDR				(APB2PERIPH_BASE + 0x5000)
 #define SYSCONFIG_BASEADDR			(APB2PERIPH_BASE + 0x3800)
 
 // base addresses of components for the APB2 peripheral
 #define USBOTG_BASEADDR			(AHB2PERIPH_BASE + 0x0000)
 

 // Struct definition for RTC clock for all peripheral buses
 typedef struct {
     volatile uint32_t RTC_TR; // time register
     volatile uint32_t RTC_DR; // date register
     volatile uint32_t RTC_CR; // control register
     volatile uint32_t RTC_ISR; // initialization and status register
     volatile uint32_t RTC_PRER; // prescaler register
     volatile uint32_t RTC_WUTR; // wakeup timer register
     volatile uint32_t RTC_CALIBR; // calibration register
     volatile uint32_t RTC_ALRMAR; // alarm A register
     volatile uint32_t RTC_ALRMBR; // alarm B register
     volatile uint32_t RTC_WPR; // write protection register
     volatile uint32_t RTC_SSR; // sub second register
     volatile uint32_t RTC_SHIFTR; // shift control register
     volatile uint32_t RTC_TSTR; // time stamp time register
     volatile uint32_t RTC_TSDR; // time stamp date register
     volatile uint32_t RTC_TSSSR; // time-stamp sub second register
     volatile uint32_t RTC_CARLR; // calibration register
     volatile uint32_t RTC_TAFCR; // tamper and alternate function configuration register
     volatile uint32_t RTC_ALRMASSR; // alarm A sub second register
     volatile uint32_t RTC_ALRMBSSR; // alarm B sub second register
     volatile uint32_t RTC_BKP0R; // backup register 0
     volatile uint32_t RTC_BKP19R; // backup register 19
 
 }RTC_MapR_t;
 
 // Struct definitopn for RCC clock for all peripheral buses
 typedef struct {
     volatile uint32_t RCC_CR; // clock control register
     volatile uint32_t RCC_PLLCFGR; // PLL configuration register
     volatile uint32_t RCC_CFGR; // clock configuration register
     volatile uint32_t RCC_CIR; // clock interrupt register
     volatile uint32_t RCC_AHB1RSTR; // AHB1 peripheral reset register
     volatile uint32_t RCC_AHB2RSTR; // AHB2 peripheral reset register  
     volatile uint32_t RCC_RESERVED1[2]; 
     volatile uint32_t RCC_APB1RSTR; // APB1 peripheral reset register
     volatile uint32_t RCC_APB2RSTR; // APB2 peripheral reset register
     volatile uint32_t RCC_RESERVED2[2];
     volatile uint32_t RCC_AHB1ENR; // AHB1 peripheral clock enable register
     volatile uint32_t RCC_AHB2ENR; // AHB2 peripheral clock enable register
     volatile uint32_t RCC_RESERVED3[2];
     volatile uint32_t RCC_APB1ENR; // APB1 peripheral clock enable register
     volatile uint32_t RCC_APB2ENR; // APB2 peripheral clock enable register
     volatile uint32_t RCC_RESERVED4[2];
     volatile uint32_t RCC_AHB1LPENR; // AHB1 peripheral clock enable in low power mode register
     volatile uint32_t RCC_AHB2LPENR; // AHB2 peripheral clock enable in low power mode register
     volatile uint32_t RCC_RESERVED5[2];
     volatile uint32_t RCC_APB1LPENR; // APB1 peripheral clock enable in low power mode register
     volatile uint32_t RCC_APB2LPENR; // APB2 peripheral clock enable in low power mode register
     volatile uint32_t RCC_BDCR; // backup domain control register
     volatile uint32_t RCC_CSR; // clock control & status register
     volatile uint32_t RCC_RESERVED6[2];
     volatile uint32_t RCC_SSCGR; // spread spectrum clock generation register
     volatile uint32_t RCC_PLLI2SCFGR; // PLLI2S configuration register
     volatile uint32_t RCC_DCKCFGR; // dedicated clock configuration register
 
 }RCC_Map_t;
 
 // Struct def for all common used peripherals.
 
 //struct definition for GPIOx
 typedef struct {
     volatile uint32_t MODER; 		//mode select typer register
     volatile uint32_t OTYPER;		//output type register
     volatile uint32_t OSPEEDR;		//output speed register
     volatile uint32_t PUPDR;		//pull-up or pull-down register
     volatile uint32_t IDR; 		//input data register
     volatile uint32_t ODR;			//output data register
     volatile uint32_t BSSR;		//bit set/reset register
     volatile uint32_t LCKR;		//port config lock register
     volatile uint32_t AFRL; 		//alternate function low register
     volatile uint32_t AFRH;        //alternate function high register
 
 }GPIOx_MapR_t;
 
 
 // struct definition for USARTx
 typedef struct {
     volatile uint32_t USART_SR; // status register
     volatile uint32_t USART_DR; // data register
     volatile uint32_t USART_BRR; // baud rate register
     volatile uint32_t USART_CR1; // control register 1
     volatile uint32_t USART_CR2; // control register 2
     volatile uint32_t USART_CR3; // control register 3
     volatile uint32_t USART_GTPR; // guard time and prescaler register
 
 }USARTx_MapR_t;
 
 // struct definition for SPIx
 typedef struct {
     volatile uint32_t SPI_CR1; // control register 1
     volatile uint32_t SPI_CR2; // control register 2
     volatile uint32_t SPI_SR; // status register
     volatile uint32_t SPI_DR; // data register
     volatile uint32_t SPI_CRCPR; // CRC polynomial register
     volatile uint32_t SPI_RXCRCR; // RX CRC register
     volatile uint32_t SPI_TXCRCR; // TX CRC register
     volatile uint32_t SPI_I2S_CFGR; // configuration register
     volatile uint32_t SPI_I2S_PR; // prescaler register
 
 }SPIx_MapR_t;
 
 // struct definition for I2Cx
 typedef struct {
     volatile uint32_t I2C_CR1; // control register 1
     volatile uint32_t I2C_CR2; // control register 2
     volatile uint32_t I2C_OAR1; // own address register 1
     volatile uint32_t I2C_OAR2; // own address register 2
     volatile uint32_t I2C_DR; // data register
     volatile uint32_t I2C_SR1; // status register 1
     volatile uint32_t I2C_SR2; // status register 2
     volatile uint32_t I2C_CCR; // clock control register
     volatile uint32_t I2C_TRISE; // rise time register
     volatile uint32_t I2C_FLTR; // fall time register
 }I2Cx_MapR_t;

 // struct definition for EXTI
 typedef struct {
    volatile uint32_t EXTI_IMR; // interrupt mask register
    volatile uint32_t EXTI_EMR; // event mask register
    volatile uint32_t EXTI_RTSR; //rising trigger selection register
    volatile uint32_t EXTI_FTSR; // falling trigger selection register
    volatile uint32_t EXTI_SWIER; //software interrupt event register
    volatile uint32_t EXTI_PR; // pending register
} EXTI_MapR_t;

typedef struct {
    volatile uint32_t SYSCFG_MEMRMP; // memory remap register
    volatile uint32_t SYSCFG_PMC; // peripheral mode config register
    volatile uint32_t SYSCFG_EXTCRx[4]; // external interrupt config register
    volatile uint32_t SYSCFG_CMPCR; // comparator control register
}SYSCFG_MapR_t;
 
 //pointer definition to GPIOx Base address
 #define GPIOA 						((GPIOx_MapR_t*) GPIOA_BASEADDR)
 #define GPIOB 						((GPIOx_MapR_t*) GPIOB_BASEADDR)
 #define GPIOC 						((GPIOx_MapR_t*) GPIOC_BASEADDR)
 #define GPIOD 						((GPIOx_MapR_t*) GPIOD_BASEADDR)
 #define GPIOE 						((GPIOx_MapR_t*) GPIOE_BASEADDR)
 #define GPIOH 						((GPIOx_MapR_t*) GPIOH_BASEADDR)
 
 #define RCC						((RCC_Map_t *) RCC_BASEADDR)

 #define EXTI						((EXTI_MapR_t *) EXTI_BASEADDR)
 #define SYSCFG					    ((SYSCFG_MapR_t *) SYSCONFIG_BASEADDR)

 #define SPI1                       ((SPIx_MapR_t*) SPI1_BASEADDR)
 #define SPI2                       ((SPIx_MapR_t*) SPI2_BASEADDR)
 #define SPI3                       ((SPIx_MapR_t*) SPI3_BASEADDR)
 #define SPI4                       ((SPIx_MapR_t*) SPI4_BASEADDR)
 #define SPI5                       ((SPIx_MapR_t*) SPI5_BASEADDR)


#define I2C1                       ((I2Cx_MapR_t*) I2C1_BASEADDR)
#define I2C2                       ((I2Cx_MapR_t*) I2C2_BASEADDR)
#define I2C3                       ((I2Cx_MapR_t*) I2C3_BASEADDR)

 // clock enable for GPIOx
 #define GPIOA_CLK_EN()				((RCC->RCC_AHB1ENR) |= (1 << 0))
 #define GPIOB_CLK_EN()				((RCC->RCC_AHB1ENR) |= (1 << 1))
 #define GPIOC_CLK_EN()				((RCC->RCC_AHB1ENR) |= (1 << 2))
 #define GPIOD_CLK_EN()				((RCC->RCC_AHB1ENR) |= (1 << 3))
 #define GPIOE_CLK_EN()				((RCC->RCC_AHB1ENR) |= (1 << 4))
 #define GPIOH_CLK_EN()				((RCC->RCC_AHB1ENR) |= (1 << 7))
 
 // clock disable for GPIOx
 #define GPIOA_CLK_DIS()			((RCC->RCC_AHB1ENR) &= ~(1 << 0))
 #define GPIOB_CLK_DIS()			((RCC->RCC_AHB1ENR) &= ~(1 << 1))
 #define GPIOC_CLK_DIS()			((RCC->RCC_AHB1ENR) &= ~(1 << 2))
 #define GPIOD_CLK_DIS()			((RCC->RCC_AHB1ENR) &= ~(1 << 3))
 #define GPIOE_CLK_DIS()			((RCC->RCC_AHB1ENR) &= ~(1 << 4))
 #define GPIOH_CLK_DIS()			((RCC->RCC_AHB1ENR) &= ~(1 << 7))
 
 //clock enable for USART
 
 #define USART2_CLK_EN()			((RCC->RCC_APB1ENR) |= (1 << 17))
 #define USART1_CLK_EN()			((RCC->RCC_APB2ENR) |= (1 << 4))
 #define USART6_CLK_EN()			((RCC->RCC_APB2ENR) |= (1 << 5))
 
 
 //clock enable for I2C
 #define I2C1_CLK_EN()				((RCC->RCC_APB1ENR) |= (1 << 21))
 #define I2C2_CLK_EN()				((RCC->RCC_APB1ENR) |= (1 << 22))
 #define I2C3_CLK_EN()				((RCC->RCC_APB1ENR) |= (1 << 23))
 
 
 // clock enable for SPI
 #define SPI1_CLK_EN()				((RCC->RCC_APB2ENR) |= (1 << 12))
 #define SPI2_CLK_EN()				((RCC->RCC_APB1ENR) |= (1 << 14))
 #define SPI3_CLK_EN()				((RCC->RCC_APB1ENR) |= (1 << 15))
 #define SPI4_CLK_EN()				((RCC->RCC_APB2ENR) |= (1 << 13))
 #define SPI5_CLK_EN()				((RCC->RCC_APB2ENR) |= (1 << 20))

  // clock disable for SPI
 #define SPI1_CLK_DIS()				((RCC->RCC_APB2ENR) &= ~(1 << 12))
 #define SPI2_CLK_DIS()				((RCC->RCC_APB1ENR) &= ~(1 << 14))
 #define SPI3_CLK_DIS()				((RCC->RCC_APB1ENR) &= ~(1 << 15))
 #define SPI4_CLK_DIS()				((RCC->RCC_APB2ENR) &= ~(1 << 13))
 #define SPI5_CLK_DIS()				((RCC->RCC_APB2ENR) &= ~(1 << 20))
 
 // clock enable for SYSCONFIG
 #define SYSCOFG_CLK_EN()			((RCC->RCC_APB2ENR) |= (1 << 14))
 
 
 // reset the gpio port config using RCC reset register
 #define RST_GPIOA()				do{((RCC->RCC_AHB1RSTR) |= (1 << 0)); ((RCC->RCC_AHB1RSTR) &= ~(1 << 0));} while(0)
 #define RST_GPIOB()				do{((RCC->RCC_AHB1RSTR) |= (1 << 1)); ((RCC->RCC_AHB1RSTR) &= ~(1 << 1));} while(0)
 #define RST_GPIOC()				do{((RCC->RCC_AHB1RSTR) |= (1 << 2)); ((RCC->RCC_AHB1RSTR) &= ~(1 << 2));} while(0)
 #define RST_GPIOD()				do{((RCC->RCC_AHB1RSTR) |= (1 << 3)); ((RCC->RCC_AHB1RSTR) &= ~(1 << 3));} while(0)
 #define RST_GPIOE()				do{((RCC->RCC_AHB1RSTR) |= (1 << 4)); ((RCC->RCC_AHB1RSTR) &= ~(1 << 4));} while(0)
 #define RST_GPIOH()				do{((RCC->RCC_AHB1RSTR) |= (1 << 7)); ((RCC->RCC_AHB1RSTR) &= ~(1 << 7));} while(0)

//reset all the SPI registers using RCC reset
#define SPI1_REG_RESET()            do{ (RCC->RCC_APB2RSTR |= (1 << 12)); (RCC->RCC_APB2RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET()            do{ (RCC->RCC_APB1RSTR |= (1 << 14)); (RCC->RCC_APB1RSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET()            do{ (RCC->RCC_APB1RSTR |= (1 << 15)); (RCC->RCC_APB1RSTR &= ~(1 << 15)); }while(0)
#define SPI4_REG_RESET()            do{ (RCC->RCC_APB2RSTR |= (1 << 13)); (RCC->RCC_APB2RSTR &= ~(1 << 13)); }while(0)
#define SPI5_REG_RESET()            do{ (RCC->RCC_APB2RSTR |= (1 << 20)); (RCC->RCC_APB2RSTR &= ~(1 << 20)); }while(0)

// reset all the I2C registers using the RCC reset
#define I2C1_REG_RESET()             do{ (RCC->RCC_APB1RSTR |= (1 << 21)); (RCC->RCC_APB1RSTR &= ~(1 << 21)); }while(0)
#define I2C2_REG_RESET()             do{ (RCC->RCC_APB1RSTR |= (1 << 22)); (RCC->RCC_APB1RSTR &= ~(1 << 22)); }while(0)
#define I2C3_REG_RESET()             do{ (RCC->RCC_APB1RSTR |= (1 << 23)); (RCC->RCC_APB1RSTR &= ~(1 << 23)); }while(0)

// reset all the USART register using the RCC reset
#define USART1_REG_RESET()           do{ (RCC->RCC_APB2RSTR |= (1 << 4));  (RCC->RCC_APB2RSTR &= ~(1 << 4)); }while(0)
#define USART2_REG_RESET()           do{ (RCC->RCC_APB1RSTR |= (1 << 17)); (RCC->RCC_APB1RSTR &= ~(1 << 17)); }while(0)
#define USART6_REG_RESET()           do{ (RCC->RCC_APB2RSTR |= (1 << 5));  (RCC->RCC_APB2RSTR &= ~(1 << 5)); }while(0)

 //Define GPIO operations
 #define GPIO_MODE_INPUT            0U
 #define GPIO_MODE_OUTPUT           1U
 #define GPIO_MODE_AF               2U
 #define GPIO_MODE_ANALOG           3U

 #define GPIO_OTYPE_PP              0U
 #define GPIO_OTYPE_OD              1U

 #define GPIO_OSPEED_LOW            0U
 #define GPIO_OSPEED_MED            1U
 #define GPIO_OSPEED_FAST           2U
 #define GPIO_OSPEED_HIGH           3U

 #define GPIO_PUPD_OFF              0U
 #define GPIO_PUPD_PU               1U
 #define GPIO_PUPD_PD               2U

 // Define GPIO Alternate Function Low (AFL) values
#define GPIO_AFL_AF0                0x0U 
#define GPIO_AFL_AF1                0x1U 
#define GPIO_AFL_AF2                0x2U 
#define GPIO_AFL_AF3                0x3U 
#define GPIO_AFL_AF4                0x4U 
#define GPIO_AFL_AF5                0x5U 
#define GPIO_AFL_AF6                0x6U 
#define GPIO_AFL_AF7                0x7U 
#define GPIO_AFL_AF8                0x8U 
#define GPIO_AFL_AF9                0x9U 
#define GPIO_AFL_AF10               0xAU 
#define GPIO_AFL_AF11               0xBU 
#define GPIO_AFL_AF12               0xCU 
#define GPIO_AFL_AF13               0xDU 
#define GPIO_AFL_AF14               0xEU 
#define GPIO_AFL_AF15               0xFU 

// Define GPIO Alternate Function High (AFH) values
#define GPIO_AFH_AF0                0x0U  
#define GPIO_AFH_AF1                0x1U  
#define GPIO_AFH_AF2                0x2U  
#define GPIO_AFH_AF3                0x3U  
#define GPIO_AFH_AF4                0x4U  
#define GPIO_AFH_AF5                0x5U  
#define GPIO_AFH_AF6                0x6U  
#define GPIO_AFH_AF7                0x7U  
#define GPIO_AFH_AF8                0x8U  
#define GPIO_AFH_AF9                0x9U  
#define GPIO_AFH_AF10               0xAU 
#define GPIO_AFH_AF11               0xBU 
#define GPIO_AFH_AF12               0xCU
#define GPIO_AFH_AF13               0xDU
#define GPIO_AFH_AF14               0xEU  
#define GPIO_AFH_AF15               0xFU  


// Define SPI configuration values

// SPI Device Modes
#define SPI_DEVICE_MODE_SLAVE        0U
#define SPI_DEVICE_MODE_MASTER       1U

// SPI Bus Configurations
#define SPI_BUS_CONFIG_FULL_DUPLEX   0U  // Full Duplex
#define SPI_BUS_CONFIG_HALF_DUPLEX   1U  // Half Duplex
#define SPI_BUS_CONFIG_SIMPLEX_RX    1U  // Simplex RX only.

// SPI Serial Clock Speeds (Baud Rate Prescaler)
#define SPI_SCLK_SPEED_HALF_CLOCK    0U
#define SPI_SCLK_SPEED_QUARTER_CLOCK 1U
#define SPI_SCLK_SPEED_DIV8          2U
#define SPI_SCLK_SPEED_DIV16         3U
#define SPI_SCLK_SPEED_DIV32         4U
#define SPI_SCLK_SPEED_DIV64         5U
#define SPI_SCLK_SPEED_DIV128        6U
#define SPI_SCLK_SPEED_DIV256        7U

// SPI Data Frame Format
#define SPI_DFF_8BITS                0U
#define SPI_DFF_16BITS               1U

// SPI Clock Polarity
#define SPI_CPOL_LOW                 0U
#define SPI_CPOL_HIGH                1U

// SPI Clock Phase
#define SPI_CPHA_FIRST_EDGE          0U
#define SPI_CPHA_SECOND_EDGE         1U

// SPI Software Slave Management
#define SPI_SSM_DISABLE              0U
#define SPI_SSM_ENABLE               1U

// SPI Enable/Disable
#define SPI_ENABLE                   1U
#define SPI_DISABLE                  0U

// SPI Status Flags
#define SPI_TXE_FLAG                 (1 << 1)   // Transmit buffer empty
#define SPI_RXNE_FLAG                (1 << 0)   // Receive buffer not empty
#define SPI_BUSY_FLAG                (1 << 7)   // Busy flag


// Define I2C configuration values
/* I2C_CR1 Register Flags */
#define I2C_CR1_PE                   (1 << 0)    // Peripheral Enable
#define I2C_CR1_SMBUS                (1 << 1)    // SMBus Mode
#define I2C_CR1_SMBTYPE              (1 << 3)    // SMBus Type
#define I2C_CR1_ENARP                (1 << 4)    // ARP Enable
#define I2C_CR1_ENPEC                (1 << 5)    // PEC Enable
#define I2C_CR1_ENGC                 (1 << 6)    // General Call Enable
#define I2C_CR1_NOSTRETCH            (1 << 7)    // Clock Stretching Disable (Slave mode)
#define I2C_CR1_START                (1 << 8)    // Start Generation
#define I2C_CR1_STOP                 (1 << 9)    // Stop Generation
#define I2C_CR1_ACK                  (1 << 10)   // Acknowledge Enable
#define I2C_CR1_POS                  (1 << 11)   // Acknowledge/PEC Position (for data reception)
#define I2C_CR1_PEC                  (1 << 12)   // PEC Request
#define I2C_CR1_ALERT                (1 << 13)   // SMBus Alert
#define I2C_CR1_SWRST                (1 << 15)   // Software Reset

/* I2C_CR2 Register Flags */
#define I2C_CR2_FREQ_MASK            0x3F        // Peripheral Clock Frequency Mask (bits 5:0)
#define I2C_CR2_ITERREN              (1 << 8)    // Error Interrupt Enable
#define I2C_CR2_ITEVTEN              (1 << 9)    // Event Interrupt Enable
#define I2C_CR2_ITBUFEN              (1 << 10)   // Buffer Interrupt Enable
#define I2C_CR2_DMAEN                (1 << 11)   // DMA Requests Enable
#define I2C_CR2_LAST                 (1 << 12)   // DMA Last Transfer

/* I2C_SR1 Register Flags */
#define I2C_SR1_SB                   (1 << 0)    // Start Bit (Master mode)
#define I2C_SR1_ADDR                 (1 << 1)    // Address sent (Master) / matched (Slave)
#define I2C_SR1_BTF                  (1 << 2)    // Byte Transfer Finished
#define I2C_SR1_ADD10                (1 << 3)    // 10-bit header sent (Master mode)
#define I2C_SR1_STOPF                (1 << 4)    // Stop detection (Slave mode)
#define I2C_SR1_RXNE                 (1 << 6)    // Data Register not Empty (Receive)
#define I2C_SR1_TXE                  (1 << 7)    // Data Register Empty (Transmit)
#define I2C_SR1_BERR                 (1 << 8)    // Bus Error
#define I2C_SR1_ARLO                 (1 << 9)    // Arbitration Lost
#define I2C_SR1_AF                   (1 << 10)   // Acknowledge Failure
#define I2C_SR1_OVR                  (1 << 11)   // Overrun/Underrun
#define I2C_SR1_PECERR               (1 << 12)   // PEC Error in reception
#define I2C_SR1_TIMEOUT              (1 << 14)   // Timeout or Tlow Error
#define I2C_SR1_SMBALERT             (1 << 15)   // SMBus Alert

/* I2C_SR2 Register Flags */
#define I2C_SR2_MSL                  (1 << 0)    // Master/Slave
#define I2C_SR2_BUSY                 (1 << 1)    // Bus Busy
#define I2C_SR2_TRA                  (1 << 2)    // Transmitter/Receiver
#define I2C_SR2_GENCALL              (1 << 4)    // General Call Address (Slave mode)
#define I2C_SR2_SMBDEFAULT           (1 << 5)    // SMBus Device Default Address (Slave mode)
#define I2C_SR2_SMBHOST              (1 << 6)    // SMBus Host Header (Slave mode)
#define I2C_SR2_DUALF                (1 << 7)    // Dual Flag (Slave mode)
#define I2C_SR2_PEC_MASK             (0xFF << 8) // Packet Error Checking Register (bits 15:8)

/* I2C_CCR Register Flags */
#define I2C_CCR_CCR_MASK             0x0FFF      // Clock Control Register (bits 11:0)
#define I2C_CCR_DUTY                 (1 << 14)   // Fast Mode Duty Cycle
#define I2C_CCR_FS                   (1 << 15)   // I2C Master Mode Selection (Standard/Fast mode)

#define I2C_SLC_SPEED_NORMAL             100000U
#define I2C_SLC_SPEED_FAST               400000U

#define I2C_ACK_ENABLE              1
#define I2C_ACK_DISABLE             0

#define I2C_DUTY_CYCLE_2            0
#define I2C_DUTY_CYCLE_16_9         1

#endif /* INC_STM32F411XX_H_ */