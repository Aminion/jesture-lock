#include "hal/uart.hpp"

#include <initializer_list>

#include "hal/ring.hpp"

#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_gpio.h"
#include "stm32g0xx_ll_usart.h"

namespace {

USART_TypeDef* const uart = USART2;

GPIO_TypeDef* const port = GPIOA;
constexpr std::uint32_t tx_pin = LL_GPIO_PIN_2;
constexpr std::uint32_t rx_pin = LL_GPIO_PIN_3;

// 64 bytes hold ~5.5 ms of input at 115200 baud: longer than a blocking status line.
constexpr std::size_t rx_size = 64;
hal::Ring<rx_size> g_rx;
volatile std::uint32_t g_rx_errors = 0;

} // namespace

extern "C" void USART2_IRQHandler() {
    const std::uint32_t isr = uart->ISR;
    // RXNEIE also raises the interrupt on ORE (RM0444 USART_ISR): clear it or the IRQ
    // never ends. FE/NE come with RXNE; the byte is still read and stored.
    constexpr std::uint32_t errors = USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE;
    if ((isr & errors) != 0) {
        uart->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF;
        g_rx_errors = g_rx_errors + 1;
    }
    if ((isr & USART_ISR_RXNE_RXFNE) != 0) {
        if (!hal::ring_push(g_rx, static_cast<std::uint8_t>(uart->RDR))) {
            g_rx_errors = g_rx_errors + 1;
        }
    }
}

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
    // 1 stop bit, OVER8 = 0 (oversampling by 16, see uart_brr()).
    LL_USART_Disable(uart);
    uart->BRR = uart_brr(baud);
    LL_USART_SetTransferDirection(uart, LL_USART_DIRECTION_TX_RX);
    LL_USART_EnableIT_RXNE_RXFNE(uart);
    LL_USART_Enable(uart);
    while (LL_USART_IsActiveFlag_TEACK(uart) == 0U || LL_USART_IsActiveFlag_REACK(uart) == 0U) {
    }
    NVIC_EnableIRQ(USART2_IRQn);
}

void uart_write(std::span<const std::uint8_t> data) {
    for (const std::uint8_t byte : data) {
        while (LL_USART_IsActiveFlag_TXE_TXFNF(uart) == 0U) {
        }
        LL_USART_TransmitData8(uart, byte);
    }
}

std::optional<std::uint8_t> uart_read() { return hal::ring_pop(g_rx); }

std::uint32_t uart_rx_errors() { return g_rx_errors; }

} // namespace hal
