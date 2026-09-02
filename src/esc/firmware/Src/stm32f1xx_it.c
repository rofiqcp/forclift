#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"
#include "util.h"

extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern UART_HandleTypeDef huart3;

/**
 * Mematikan Main Output Enable TIM1/TIM8 secepat mungkin pada fault CPU.
 * Ini memaksa PWM high-side dan complementary low-side kembali ke idle state
 * yang telah dikunci di setup.c (high OFF=LOW, low OFF=HIGH).
 */
static void EmergencyPwmOff(void)
{
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM8->BDTR &= ~TIM_BDTR_MOE;
}

/** Handler NMI. PWM dimatikan terlebih dahulu, kemudian CPU berhenti. */
void NMI_Handler(void) { EmergencyPwmOff(); while (1) {} }

/** HardFault: matikan gate PWM terlebih dahulu lalu berhenti permanen. */
void HardFault_Handler(void) { EmergencyPwmOff(); while (1) {} }
/** Memory management fault: matikan gate PWM terlebih dahulu lalu berhenti. */
void MemManage_Handler(void) { EmergencyPwmOff(); while (1) {} }
/** Bus fault: matikan gate PWM terlebih dahulu lalu berhenti. */
void BusFault_Handler(void) { EmergencyPwmOff(); while (1) {} }
/** Usage fault: matikan gate PWM terlebih dahulu lalu berhenti. */
void UsageFault_Handler(void) { EmergencyPwmOff(); while (1) {} }
/** Handler SVC standar Cortex-M3. */
void SVC_Handler(void) {}
/** Handler debug monitor standar Cortex-M3. */
void DebugMon_Handler(void) {}
/** Handler PendSV standar Cortex-M3. */
void PendSV_Handler(void) {}

/** Menambah tick HAL 1 ms dan meneruskan event SysTick ke HAL. */
void SysTick_Handler(void)
{
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
}

/** DMA1 channel 2 adalah TX USART3. */
void DMA1_Channel2_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart3_tx);
}

/** DMA1 channel 3 adalah RX USART3 circular DMA. */
void DMA1_Channel3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart3_rx);
}

/**
 * Interrupt USART3. IDLE-line dipakai sebagai penanda adanya paket baru sehingga
 * parser dapat mengecek panjang frame, START_FRAME, dan checksum.
 */
void USART3_IRQHandler(void)
{
    /* IDLE harus dicek sebagai FLAG, bukan hanya status enable interrupt. */
    if ((__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE) != RESET) &&
        (__HAL_UART_GET_IT_SOURCE(&huart3, UART_IT_IDLE) != RESET)) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart3);
        usart3_rx_check();
    }

    HAL_UART_IRQHandler(&huart3);
}
