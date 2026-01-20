#ifndef INC_STM32F407XX_DMA_H_
#define INC_STM32F407XX_DMA_H_

#include <stdint.h>

#define __vo volatile

/******************************** DMA Base Addresses ********************************/

#define DMA1_BASE_ADDR   (0x40026000U)   /* AHB1 + 0x6000 */
#define DMA2_BASE_ADDR   (0x40026400U)   /* AHB1 + 0x6400 */

/******************************** DMA IRQ Numbers ********************************/

#define IRQ_NO_DMA1_STREAM0   11
#define IRQ_NO_DMA1_STREAM1   12
#define IRQ_NO_DMA1_STREAM2   13
#define IRQ_NO_DMA1_STREAM3   14
#define IRQ_NO_DMA1_STREAM4   15
#define IRQ_NO_DMA1_STREAM5   16
#define IRQ_NO_DMA1_STREAM6   17
#define IRQ_NO_DMA1_STREAM7   47

#define IRQ_NO_DMA2_STREAM0   56
#define IRQ_NO_DMA2_STREAM1   57
#define IRQ_NO_DMA2_STREAM2   58
#define IRQ_NO_DMA2_STREAM3   59
#define IRQ_NO_DMA2_STREAM4   60
#define IRQ_NO_DMA2_STREAM5   68
#define IRQ_NO_DMA2_STREAM6   69
#define IRQ_NO_DMA2_STREAM7   70

/******************************** DMA Register Definitions ********************************/

/* DMA controller registers */
typedef struct
{
    __vo uint32_t LISR;    /* Low Interrupt Status Register  */
    __vo uint32_t HISR;    /* High Interrupt Status Register */
    __vo uint32_t LIFCR;   /* Low Interrupt Flag Clear       */
    __vo uint32_t HIFCR;   /* High Interrupt Flag Clear      */
} DMA_RegDef_t;

/* DMA stream registers */
typedef struct
{
    __vo uint32_t CR;      /* Stream configuration register */
    __vo uint32_t NDTR;    /* Number of data register       */
    __vo uint32_t PAR;     /* Peripheral address register  */
    __vo uint32_t M0AR;    /* Memory 0 address register    */
    __vo uint32_t M1AR;    /* Memory 1 address register    */
    __vo uint32_t FCR;     /* FIFO control register        */
} DMA_StreamRegDef_t;

/******************************** DMA Peripheral Definitions ********************************/

#define DMA1    ((DMA_RegDef_t *)DMA1_BASE_ADDR)
#define DMA2    ((DMA_RegDef_t *)DMA2_BASE_ADDR)

/* DMA1 Streams */
#define DMA1_STREAM0   ((DMA_StreamRegDef_t *)(DMA1_BASE_ADDR + 0x0010))
#define DMA1_STREAM1   ((DMA_StreamRegDef_t *)(DMA1_BASE_ADDR + 0x0028))
#define DMA1_STREAM2   ((DMA_StreamRegDef_t *)(DMA1_BASE_ADDR + 0x0040))
#define DMA1_STREAM3   ((DMA_StreamRegDef_t *)(DMA1_BASE_ADDR + 0x0058))
#define DMA1_STREAM4   ((DMA_StreamRegDef_t *)(DMA1_BASE_ADDR + 0x0070))
#define DMA1_STREAM5   ((DMA_StreamRegDef_t *)(DMA1_BASE_ADDR + 0x0088))
#define DMA1_STREAM6   ((DMA_StreamRegDef_t *)(DMA1_BASE_ADDR + 0x00A0))
#define DMA1_STREAM7   ((DMA_StreamRegDef_t *)(DMA1_BASE_ADDR + 0x00B8))

/* DMA2 Streams */
#define DMA2_STREAM0   ((DMA_StreamRegDef_t *)(DMA2_BASE_ADDR + 0x0010))
#define DMA2_STREAM1   ((DMA_StreamRegDef_t *)(DMA2_BASE_ADDR + 0x0028))
#define DMA2_STREAM2   ((DMA_StreamRegDef_t *)(DMA2_BASE_ADDR + 0x0040))
#define DMA2_STREAM3   ((DMA_StreamRegDef_t *)(DMA2_BASE_ADDR + 0x0058))
#define DMA2_STREAM4   ((DMA_StreamRegDef_t *)(DMA2_BASE_ADDR + 0x0070))
#define DMA2_STREAM5   ((DMA_StreamRegDef_t *)(DMA2_BASE_ADDR + 0x0088))
#define DMA2_STREAM6   ((DMA_StreamRegDef_t *)(DMA2_BASE_ADDR + 0x00A0))
#define DMA2_STREAM7   ((DMA_StreamRegDef_t *)(DMA2_BASE_ADDR + 0x00B8))

/******************************** DMA Clock Control ********************************/

#define DMA1_PCLK_EN()   (RCC->AHB1ENR |= (1 << 21))
#define DMA1_PCLK_DI()   (RCC->AHB1ENR &= ~(1 << 21))

#define DMA2_PCLK_EN()   (RCC->AHB1ENR |= (1 << 22))
#define DMA2_PCLK_DI()   (RCC->AHB1ENR &= ~(1 << 22))

#endif /* INC_STM32F407XX_DMA_H_ */
