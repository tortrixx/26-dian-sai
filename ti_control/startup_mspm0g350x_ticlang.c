/*****************************************************************************

  Copyright (C) 2023 Texas Instruments Incorporated - http://www.ti.com/

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

   Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

   Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the
   distribution.

   Neither the name of Texas Instruments Incorporated nor the names of
   its contributors may be used to endorse or promote products derived
   from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*****************************************************************************/

#include <stdint.h>
#include <ti/devices/msp/msp.h>

extern unsigned long __STACK_END;
extern __NO_RETURN void __PROGRAM_START(void);

void Default_Handler(void) __attribute__((weak));
extern void Reset_Handler(void) __attribute__((weak));

extern void NMI_Handler(void) __attribute__((weak, alias("Default_Handler")));
extern void HardFault_Handler(void)
    __attribute__((weak, alias("Default_Handler")));
extern void SVC_Handler(void) __attribute__((weak, alias("Default_Handler")));
extern void PendSV_Handler(void)
    __attribute__((weak, alias("Default_Handler")));
extern void SysTick_Handler(void)
    __attribute__((weak, alias("Default_Handler")));

extern void GROUP0_IRQHandler(void)
    __attribute__((weak, alias("Default_Handler")));
extern void GROUP1_IRQHandler(void)
    __attribute__((weak, alias("Default_Handler")));
extern void TIMG8_IRQHandler(void)
    __attribute__((weak, alias("Default_Handler")));
extern void UART3_IRQHandler(void)
    __attribute__((weak, alias("Default_Handler")));
extern void ADC0_IRQHandler(void)
    __attribute__((weak, alias("Default_Handler")));
extern void ADC1_IRQHandler(void)
    __attribute__((weak, alias("Default_Handler")));
extern void CANFD0_IRQHandler(void)
    __attribute__((weak, alias("Default_Handler")));
extern void DAC0_IRQHandler(void)
    __attribute__((weak, alias("Default_Handler")));
extern void SPI0_IRQHandler(void)
    __attribute__((weak, alias("Default_Handler")));
extern void SPI1_IRQHandler(void)
    __attribute__((weak, alias("Default_Handler")));
extern void UART1_IRQHandler(void)
    __attribute__((weak, alias("Default_Handler")));
extern void UART2_IRQHandler(void)
    __attribute__((weak, alias("Default_Handler")));
extern void UART0_IRQHandler(void)
    __attribute__((weak, alias("Default_Handler")));
extern void TIMG0_IRQHandler(void)
    __attribute__((weak, alias("Default_Handler")));
extern void TIMG6_IRQHandler(void)
    __attribute__((weak, alias("Default_Handler")));
extern void TIMA0_IRQHandler(void)
    __attribute__((weak, alias("Default_Handler")));
extern void TIMA1_IRQHandler(void)
    __attribute__((weak, alias("Default_Handler")));
extern void TIMG7_IRQHandler(void)
    __attribute__((weak, alias("Default_Handler")));
extern void TIMG12_IRQHandler(void)
    __attribute__((weak, alias("Default_Handler")));
extern void I2C0_IRQHandler(void)
    __attribute__((weak, alias("Default_Handler")));
extern void I2C1_IRQHandler(void)
    __attribute__((weak, alias("Default_Handler")));
extern void AES_IRQHandler(void)
    __attribute__((weak, alias("Default_Handler")));
extern void RTC_IRQHandler(void)
    __attribute__((weak, alias("Default_Handler")));
extern void DMA_IRQHandler(void)
    __attribute__((weak, alias("Default_Handler")));

#if defined(__ARM_ARCH) && (__ARM_ARCH != 0)
void (*const interruptVectors[])(void) __attribute((used))
__attribute__((section(".intvecs"))) =
#elif defined(__TI_ARM__)
#pragma RETAIN(interruptVectors)
#pragma DATA_SECTION(interruptVectors, ".intvecs")
void (*const interruptVectors[])(void) =
#else
#error "Compiler not supported"
#endif
    {
        (void (*)(void))((uint32_t) &__STACK_END),
        Reset_Handler,
        NMI_Handler,
        HardFault_Handler,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        SVC_Handler,
        0,
        0,
        PendSV_Handler,
        SysTick_Handler,
        GROUP0_IRQHandler,
        GROUP1_IRQHandler,
        TIMG8_IRQHandler,
        UART3_IRQHandler,
        ADC0_IRQHandler,
        ADC1_IRQHandler,
        CANFD0_IRQHandler,
        DAC0_IRQHandler,
        0,
        SPI0_IRQHandler,
        SPI1_IRQHandler,
        0,
        0,
        UART1_IRQHandler,
        UART2_IRQHandler,
        UART0_IRQHandler,
        TIMG0_IRQHandler,
        TIMG6_IRQHandler,
        TIMA0_IRQHandler,
        TIMA1_IRQHandler,
        TIMG7_IRQHandler,
        TIMG12_IRQHandler,
        0,
        0,
        I2C0_IRQHandler,
        I2C1_IRQHandler,
        0,
        0,
        AES_IRQHandler,
        0,
        RTC_IRQHandler,
        DMA_IRQHandler
    };

void Reset_Handler(void)
{
    __asm(
        "    .global _c_int00\n"
        "    b       _c_int00");
}

void Default_Handler(void)
{
    while (1) {
    }
}
