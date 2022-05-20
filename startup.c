//*****************************************************************************
// Declaration of the default fault handlers
//*****************************************************************************

#define WEAK __attribute__ ((weak))

void WEAK  ResetHandler(void);
void WEAK  NMI_Int_Handler(void);
void WEAK  HardFault_Int_Handler(void);
void WEAK  MemManage_Int_Handler(void);
void WEAK  BusFault_Int_Handler(void);
void WEAK  UsageFault_Int_Handler(void);
void WEAK  SVC_Int_Handler(void);
void WEAK  DebugMon_Int_Handler(void);
void WEAK  PendSV_Int_Handler(void);
void WEAK  SysTick_Int_Handler(void);
void WEAK  DMA0_Int_Handler(void);
void WEAK  DMA1_Int_Handler(void);
void WEAK  DMA2_Int_Handler(void);
void WEAK  DMA3_Int_Handler(void);
void WEAK  FTFA_Int_Handler(void);
void WEAK  LVD_Int_Handler(void);
void WEAK  LLWU_Int_Handler(void);
void WEAK  I2C0_Int_Handler(void);
void WEAK  I2C1_Int_Handler(void);
void WEAK  SPI0_Int_Handler(void);
void WEAK  SPI1_Int_Handler(void);
void WEAK  UART0SE_Int_Handler(void);
void WEAK  UART1SE_Int_Handler(void);
void WEAK  UART2SE_Int_Handler(void);
void WEAK  ADC_Int_Handler(void);
void WEAK  ACMP_Int_Handler(void);
void WEAK  FTM0_Int_Handler(void);
void WEAK  FTM1_Int_Handler(void);
void WEAK  FTM2_Int_Handler(void);
void WEAK  RTCA_Int_Handler(void);
void WEAK  RTCS_Int_Handler(void);
void WEAK  PIT_Int_Handler(void);
void WEAK  USBOTG_Int_Handler(void);
void WEAK  DAC_Int_Handler(void);
void WEAK  TSI_Int_Handler(void);
void WEAK  MCG_Int_Handler(void);
void WEAK  LPTMR_Int_Handler(void);
void WEAK  PORTA_Int_Handler(void);
void WEAK  PORTD_Int_Handler(void);

//*****************************************************************************
// Symbols defined in linker script
//*****************************************************************************

// Start address for the initialization values of the .data section.
extern unsigned long _sidata;

// Start address for the .data section
extern unsigned long _sdata;

// End address for the .data section
extern unsigned long _edata;

// Start address for the .bss section
extern unsigned long _sbss;

// End address for the .bss section
extern unsigned long _ebss;

// End address for ram
extern void _eram;

// Stack start address
extern unsigned long _start_of_stack;

//*****************************************************************************
// Function prototypes
//*****************************************************************************
extern int main(void);
void ResetHandler(void);
static void Default_Int_Handler(void);

//*****************************************************************************
// The minimal vector table for a Cortex M0+.  Note that the proper constructs
// must be placed on this to ensure that it ends up at physical address
// 0x00000000.
//*****************************************************************************
__attribute__ ((section(".isr_vector")))
void (* const g_pfnVectors[])(void) =
  {
    (void *)&_start_of_stack,
    ResetHandler,                           // The reset handler
    NMI_Int_Handler,                          // The NMI handler
    HardFault_Int_Handler,                    // The hard fault handler
    0, 0, 0, 0, 0, 0, 0,                    // Reserved
    SVC_Int_Handler,                          // SVCall handler
    0, 0,                                   // Reserved
    PendSV_Int_Handler,                       // The PendSV handler
    SysTick_Int_Handler,                      // The SysTick handler

    DMA0_Int_Handler,                         // DMA channel 0 transfer complete
                                            // and error handler
    DMA1_Int_Handler,                         // DMA channel 1 transfer complete
                                            // and error handler
    DMA2_Int_Handler,                         // DMA channel 2 transfer complete
                                            // and error handler
    DMA3_Int_Handler,                         // DMA channel 3 transfer complete
                                            // and error handler
    0,                                      // Reserved
    FTFA_Int_Handler,                         // Command complete and read collision
    LVD_Int_Handler,                          // Low-voltage detect, low-voltage warning
    LLWU_Int_Handler,                         // Low Leakage Wakeup
    I2C0_Int_Handler,                         // I2C0 handler
    I2C1_Int_Handler,                         // I2C1 handler
    SPI0_Int_Handler,                         // SPI0 handler
    SPI1_Int_Handler,                         // SPI1 handler
    UART0SE_Int_Handler,                      // UART0SE handler
    UART1SE_Int_Handler,                      // UART1SE handler
    UART2SE_Int_Handler,                      // UART2SE handler
    ADC_Int_Handler,                          // ADC handler
    ACMP_Int_Handler,                         // ACMP handler
    FTM0_Int_Handler,                         // FTM0 handler
    FTM1_Int_Handler,                         // FTM1 handler
    FTM2_Int_Handler,                         // FTM2 handler
    RTCA_Int_Handler,                         // RTCA handler
    RTCS_Int_Handler,                         // RTCS handler
    PIT_Int_Handler,                          // PIT handler
    0,                                      // Reserved
    USBOTG_Int_Handler,                       // USBOTG handler
    DAC_Int_Handler,                          // DAC handler
    TSI_Int_Handler,                          // TSI handler
    MCG_Int_Handler,                          // MCG handler
    LPTMR_Int_Handler,                        // PIT handler
    0,                                      // Reserved
    PORTA_Int_Handler,                        // PORTA handler
    PORTD_Int_Handler,                        // PORTC/PORTD handler
  };

//*****************************************************************************
//! \brief This is the code that gets called when the processor first
//! starts execution following a reset event.
//!
//! \param None.
//!
//! Only the absolutely necessary set is performed, after which the
//! application supplied main() routine is called.
//!
//! \return None.
//*****************************************************************************

// Defined in system_MKL46Z4.c: disables watchdog
extern void SystemInit(void);

void Default_ResetHandler(void)
{
  unsigned long *pulSrc, *pulDest;

  SystemInit();

  /* copy the data segment initializers from flash to SRAM */
  pulSrc = &_sidata;
  for(pulDest = &_sdata; pulDest < &_edata; )
    {
      *(pulDest++) = *(pulSrc++);
    }

  /* zero fill the bss segment */
  for(pulDest = &_sbss; pulDest < &_ebss; )
    {
      *(pulDest++) = 0;
    }

  /* call the application's entry point */
  main();
}

//*****************************************************************************
// Provide weak aliases for each Exception handler to the Default_Int_Handler.
// As they are weak aliases, any function with the same name will override
// this definition.
//*****************************************************************************
#pragma weak ResetHandler = Default_ResetHandler
#pragma weak NMI_Int_Handler = Default_Int_Handler
#pragma weak HardFault_Int_Handler = Default_Int_Handler
#pragma weak MemManage_Int_Handler = Default_Int_Handler
#pragma weak BusFault_Int_Handler = Default_Int_Handler
#pragma weak UsageFault_Int_Handler = Default_Int_Handler
#pragma weak SVC_Int_Handler = Default_Int_Handler
#pragma weak DebugMon_Int_Handler = Default_Int_Handler
#pragma weak PendSV_Int_Handler = Default_Int_Handler
#pragma weak SysTick_Int_Handler = Default_Int_Handler
#pragma weak DMA0_Int_Handler = Default_ResetHandler
#pragma weak DMA1_Int_Handler = Default_ResetHandler
#pragma weak DMA2_Int_Handler = Default_ResetHandler
#pragma weak DMA3_Int_Handler = Default_ResetHandler
#pragma weak FTFA_Int_Handler = Default_ResetHandler
#pragma weak LVD_Int_Handler = Default_ResetHandler
#pragma weak LLWU_Int_Handler = Default_ResetHandler
#pragma weak I2C0_Int_Handler = Default_ResetHandler
#pragma weak I2C1_Int_Handler = Default_ResetHandler
#pragma weak SPI0_Int_Handler = Default_ResetHandler
#pragma weak SPI1_Int_Handler = Default_ResetHandler
#pragma weak UART0SE_Int_Handler = Default_ResetHandler
#pragma weak UART1SE_Int_Handler = Default_ResetHandler
#pragma weak UART2SE_Int_Handler = Default_ResetHandler
#pragma weak ADC_Int_Handler = Default_ResetHandler
#pragma weak ACMP_Int_Handler = Default_ResetHandler
#pragma weak FTM0_Int_Handler = Default_ResetHandler
#pragma weak FTM1_Int_Handler = Default_ResetHandler
#pragma weak FTM2_Int_Handler = Default_ResetHandler
#pragma weak RTCA_Int_Handler = Default_ResetHandler
#pragma weak RTCS_Int_Handler = Default_ResetHandler
#pragma weak PIT_Int_Handler = Default_ResetHandler
#pragma weak USBOTG_Int_Handler = Default_ResetHandler
#pragma weak DAC_Int_Handler = Default_ResetHandler
#pragma weak TSI_Int_Handler = Default_ResetHandler
#pragma weak MCG_Int_Handler = Default_ResetHandler
#pragma weak LPTMR_Int_Handler = Default_ResetHandler
#pragma weak PORTA_Int_Handler = Default_ResetHandler
#pragma weak PORTD_Int_Handler = Default_ResetHandler

//*****************************************************************************
//! \brief This is the code that gets called when the processor receives an
//! unexpected interrupt.
//!
//! \param None.
//!
//! This simply enters an infinite loop, preserving the system state for
//! examination by a debugger.
//!
//! \return None.
//*****************************************************************************
static void Default_Int_Handler(void)
{
  for(;;);
}

