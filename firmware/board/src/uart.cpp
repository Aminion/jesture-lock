#include "gesture/board/uart.hpp"

#include "gesture/board/ring.hpp"
#include "pins.hpp"
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_gpio.h"
#include "stm32g0xx_ll_usart.h"

namespace {

USART_TypeDef* const uart = USART2;

// 64 bytes hold ~5.5 ms of input at 115200 baud: longer than a blocking status line.
constexpr std::size_t rx_size = 64;
board::Ring<rx_size> g_rx;
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
        if (!board::ring_push(g_rx, static_cast<std::uint8_t>(uart->RDR))) {
            g_rx_errors = g_rx_errors + 1;
        }
    }
}

namespace board {

void uart_init(std::uint32_t baud) {
    LL_IOP_GRP1_EnableClock(pins::uart_port_clock);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

    // Pull-up on RX keeps it idle-high if the VCP is disconnected.
    LL_GPIO_InitTypeDef cfg{.Pin = pins::uart_tx,
                            .Mode = LL_GPIO_MODE_ALTERNATE,
                            .Speed = LL_GPIO_SPEED_FREQ_LOW,
                            .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
                            .Pull = LL_GPIO_PULL_NO,
                            .Alternate = pins::uart_af};
    LL_GPIO_Init(pins::uart_port, &cfg);
    cfg.Pin = pins::uart_rx;
    cfg.Pull = LL_GPIO_PULL_UP;
    LL_GPIO_Init(pins::uart_port, &cfg);

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

std::optional<std::uint8_t> uart_read() {
    return board::ring_pop(g_rx);
}

std::uint32_t uart_rx_errors() {
    return g_rx_errors;
}

} // namespace board
