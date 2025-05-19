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
 
 // Struct definition for RTC clock for all peripheral buses
 typedef struct {
     volatile uint32_t RTC_TR;
     volatile uint32_t RTC_DR;
     volatile uint32_t RTC_CR;
     volatile uint32_t RTC_ISR;
     volatile uint32_t RTC_PRER;
     volatile uint32_t RTC_WUTR;
     volatile uint32_t RTC_CALIBR;
     volatile uint32_t RTC_ALRMAR;
     volatile uint32_t RTC_ALRMBR;
     volatile uint32_t RTC_WPR;
     volatile uint32_t RTC_SSR;
     volatile uint32_t RTC_SHIFTR;
     volatile uint32_t RTC_TSTR;
     volatile uint32_t RTC_TSDR;
     volatile uint32_t RTC_TSSSR;
     volatile uint32_t RTC_CARLR;
     volatile uint32_t RTC_TAFCR;
     volatile uint32_t RTC_ALRMASSR;
     volatile uint32_t RTC_ALRMBSSR;
     volatile uint32_t RTC_BKP0R;
     volatile uint32_t RTC_BKP19R;
 
 }RTC_MapR_t;
 
 // Struct definitopn for RCC clock for all peripheral buses
 typedef struct {
     volatile uint32_t RCC_CR;
     volatile uint32_t RCC_PLLCFGR;
     volatile uint32_t RCC_CFGR;
     volatile uint32_t RCC_CIR;
     volatile uint32_t RCC_AHB1RSTR;
     volatile uint32_t RCC_AHB2RSTR;
     volatile uint32_t RCC_RESERVED1[2];
     volatile uint32_t RCC_APB1RSTR;
     volatile uint32_t RCC_APB2RSTR;
     volatile uint32_t RCC_RESERVED2[2];
     volatile uint32_t RCC_AHB1ENR;
     volatile uint32_t RCC_AHB2ENR;
     volatile uint32_t RCC_RESERVED3[2];
     volatile uint32_t RCC_APB1ENR;
     volatile uint32_t RCC_APB2ENR;
     volatile uint32_t RCC_RESERVED4[2];
     volatile uint32_t RCC_AHB1LPENR;
     volatile uint32_t RCC_AHB2LPENR;
     volatile uint32_t RCC_RESERVED5[2];
     volatile uint32_t RCC_APB1LPENR;
     volatile uint32_t RCC_APB2LPENR;
     volatile uint32_t RCC_BDCR;
     volatile uint32_t RCC_CSR;
     volatile uint32_t RCC_RESERVED6[2];
     volatile uint32_t RCC_SSCGR;
     volatile uint32_t RCC_PLLI2SCFGR;
     volatile uint32_t RCC_DCKCFGR;
 
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
     volatile uint32_t AFRH;		//alternate function high register
 
 }GPIOx_MapR_t;
 
 
 // struct definition for USARTx
 typedef struct {
     volatile uint32_t USART_SR;
     volatile uint32_t USART_DR;
     volatile uint32_t USART_BRR;
     volatile uint32_t USART_CR1;
     volatile uint32_t USART_CR2;
     volatile uint32_t USART_CR3;
     volatile uint32_t USART_GTPR;
 
 }USARTx_MapR_t;
 
 // struct definition for SPIx
 typedef struct {
     volatile uint32_t SPI_CR1;
     volatile uint32_t SPI_SR;
     volatile uint32_t SPI_DR;
     volatile uint32_t SPI_CRCPR;
     volatile uint32_t SPI_RXCRCR;
     volatile uint32_t SPI_TXCRCR;
     volatile uint32_t SPI_I2S_CFGR;
     volatile uint32_t SPI_I2S_PR;
 
 }SPIx_MapR_t;
 
 // struct definition for I2Cx
 typedef struct {
     volatile uint32_t I2C_CR1;
     volatile uint32_t I2C_CR2;
     volatile uint32_t I2C_OAR1;
     volatile uint32_t I2C_OAR2;
     volatile uint32_t I2C_DR;
     volatile uint32_t I2C_SR1;
     volatile uint32_t I2C_SR2;
     volatile uint32_t I2C_CCR;
     volatile uint32_t I2C_TRISE;
     volatile uint32_t I2C_FRTR;
 }I2Cx_MapR_t;

 // struct definition for EXTI
 typedef struct {
    volatile uint32_t EXTI_IMR;
    volatile uint32_t EXTI_EMR;
    volatile uint32_t EXTI_RTSR;
    volatile uint32_t EXTI_FTSR;
    volatile uint32_t EXTI_SWIER;
    volatile uint32_t EXTI_PR;
} EXTI_MapR_t;

typedef struct {
    volatile uint32_t SYSCFG_MEMRMP;
    volatile uint32_t SYSCFG_PMC;
    volatile uint32_t SYSCFG_EXTCRx[4];
    volatile uint32_t SYSCFG_CMPCR;
}SYSCFG_MapR_t;
 
 //pointer definition to GPIOx Base address
 #define GPIOA 						((GPIOx_MapR_t*) GPIOA_BASEADDR)
 #define GPIOB 						((GPIOx_MapR_t*) GPIOB_BASEADDR)
 #define GPIOC 						((GPIOx_MapR_t*) GPIOC_BASEADDR)
 #define GPIOD 						((GPIOx_MapR_t*) GPIOD_BASEADDR)
 #define GPIOE 						((GPIOx_MapR_t*) GPIOE_BASEADDR)
 #define GPIOH 						((GPIOx_MapR_t*) GPIOH_BASEADDR)
 
 #define RCC						((RCC_Map_t *) RCC_BASEADDR)
 
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