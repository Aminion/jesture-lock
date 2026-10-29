#include "hal/uart.hpp"

#include <initializer_list>

#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_gpio.h"
#include "stm32g0xx_ll_usart.h"

namespace {

USART_TypeDef* const uart = USART2;

GPIO_TypeDef* const port = GPIOA;
constexpr std::uint32_t tx_pin = LL_GPIO_PIN_2;
constexpr std::uint32_t rx_pin = LL_GPIO_PIN_3;

// RCC_CCIPR.USART2SEL is reserved on G031 (RM0444 5.4.21 note 1): the kernel clock is PCLK.
constexpr std::uint32_t pclk_hz = 16'000'000;

} // namespace

namespace hal {

void uart_init(std::uint32_t baud) {
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

    // AF1 = USART2 on PA2/PA3 (DS12992 Table 13). Pull-up on RX keeps it idle-high if the
    // VCP is disconnected.
    for (const auto pin : {tx_pin, rx_pin}) {
        LL_GPIO_SetAFPin_0_7(port, pin, LL_GPIO_AF_1);
        LL_GPIO_SetPinOutputType(port, pin, LL_GPIO_OUTPUT_PUSHPULL);
        LL_GPIO_SetPinSpeed(port, pin, LL_GPIO_SPEED_FREQ_LOW);
        LL_GPIO_SetPinMode(port, pin, LL_GPIO_MODE_ALTERNATE);
    }
    LL_GPIO_SetPinPull(port, tx_pin, LL_GPIO_PULL_NO);
    LL_GPIO_SetPinPull(port, rx_pin, LL_GPIO_PULL_UP);

    // CR1/BRR are writable only with UE = 0. Reset values: 8 data bits, no parity,
    // 1 stop bit, OVER8 = 0. Oversampling by 16: BRR = USARTDIV = f_ck / baud (RM0444 33.5.7).
    LL_USART_Disable(uart);
    uart->BRR = (pclk_hz + baud / 2) / baud;
    LL_USART_SetTransferDirection(uart, LL_USART_DIRECTION_TX_RX);
    LL_USART_Enable(uart);
    while (!LL_USART_IsActiveFlag_TEACK(uart)) {
    }
}

void uart_write(std::span<const std::uint8_t> data) {
    for (const std::uint8_t byte : data) {
        while (!LL_USART_IsActiveFlag_TXE_TXFNF(uart)) {
        }
        LL_USART_TransmitData8(uart, byte);
    }
}

} // namespace hal
