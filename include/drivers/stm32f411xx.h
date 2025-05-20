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
     volatile uint32_t I2C_FRTR; // fall time register
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
 
 // clock enable for SYSCONFIG
 #define SYSCOFG_CLK_EN()			((RCC->RCC_APB2ENR) |= (1 << 14))
 
 
 // reset the gpio port config using RCC reset register
 #define RST_GPIOA()				do{((RCC->RCC_AHB1RSTR) |= (1 << 0)); ((RCC->RCC_AHB1RSTR) &= ~(1 << 0));} while(0)
 #define RST_GPIOB()				do{((RCC->RCC_AHB1RSTR) |= (1 << 1)); ((RCC->RCC_AHB1RSTR) &= ~(1 << 1));} while(0)
 #define RST_GPIOC()				do{((RCC->RCC_AHB1RSTR) |= (1 << 2)); ((RCC->RCC_AHB1RSTR) &= ~(1 << 2));} while(0)
 #define RST_GPIOD()				do{((RCC->RCC_AHB1RSTR) |= (1 << 3)); ((RCC->RCC_AHB1RSTR) &= ~(1 << 3));} while(0)
 #define RST_GPIOE()				do{((RCC->RCC_AHB1RSTR) |= (1 << 4)); ((RCC->RCC_AHB1RSTR) &= ~(1 << 4));} while(0)
 #define RST_GPIOH()				do{((RCC->RCC_AHB1RSTR) |= (1 << 7)); ((RCC->RCC_AHB1RSTR) &= ~(1 << 7));} while(0)

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



#endif /* INC_STM32F411XX_H_ */