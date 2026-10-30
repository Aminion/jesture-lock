#include "gesture/board/uart.hpp"

#include "gesture/board/ring.hpp"
#include "pins.hpp"
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_dma.h"
#include "stm32g0xx_ll_gpio.h"
#include "stm32g0xx_ll_usart.h"

#include <algorithm>
#include <array>

namespace {

USART_TypeDef* const uart = USART2;

// 64 bytes hold ~5.5 ms of input at 115200 baud: longer than any main loop pass.
constexpr std::size_t rx_size = 64;
board::Ring<rx_size> g_rx;
volatile std::uint32_t g_rx_errors = 0;

// TX: DMA1 channel 1 (DMAMUX channel 0, request USART2_TX) sends one buffer while writes
// go into the other. 2 x 64 B: each holds a whole frame (36 B max) with room to spare.
constexpr std::uint32_t tx_channel = LL_DMA_CHANNEL_1;
constexpr IRQn_Type tx_irq = DMA1_Channel1_IRQn;
constexpr std::size_t tx_size = 64;
struct Tx {
    std::array<std::array<std::uint8_t, tx_size>, 2> buf{};
    std::size_t fill = 0; // index of the buffer taking writes
    std::size_t len = 0;  // bytes in it
    bool busy = false;    // DMA is sending the other buffer
};
// Shared with DMA1_Channel1_IRQHandler. uart_write touches it only with tx_irq masked;
// NVIC_DisableIRQ/EnableIRQ are compiler barriers, so no volatile is needed.
Tx g_tx;

// Sends the fill buffer and switches writes to the other one. Called with tx_irq masked
// or from its handler, while the channel is idle.
void tx_start() {
    // CMAR and CNDTR are writable only with the channel disabled (RM0444 DMA_CCRx); in
    // normal mode EN stays set after the last transfer.
    LL_DMA_DisableChannel(DMA1, tx_channel);
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast): CMAR takes the address
    const auto addr = reinterpret_cast<std::uint32_t>(g_tx.buf[g_tx.fill].data());
    LL_DMA_SetMemoryAddress(DMA1, tx_channel, addr);
    LL_DMA_SetDataLength(DMA1, tx_channel, g_tx.len);
    // RM0444 "Transmission using DMA": clear TC before enabling the channel.
    LL_USART_ClearFlag_TC(uart);
    LL_DMA_EnableChannel(DMA1, tx_channel);
    g_tx.fill ^= 1U;
    g_tx.len = 0;
    g_tx.busy = true;
}

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

extern "C" void DMA1_Channel1_IRQHandler() {
    // A transfer error (bad address, not expected) disables the channel; treat it as the
    // end of the buffer so TX does not stall. Clear TCIF/TEIF one by one, not with GIF:
    // ES0486 2.3.1.
    if (LL_DMA_IsActiveFlag_TC1(DMA1) == 0U && LL_DMA_IsActiveFlag_TE1(DMA1) == 0U) {
        return;
    }
    LL_DMA_ClearFlag_TC1(DMA1);
    LL_DMA_ClearFlag_TE1(DMA1);
    if (g_tx.len > 0) {
        tx_start();
    } else {
        g_tx.busy = false;
    }
}

namespace board {

void uart_init(std::uint32_t baud) {
    LL_IOP_GRP1_EnableClock(pins::uart_port_clock);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1); // DMA1 and DMAMUX (RM0444 RCC_AHBENR)

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

    LL_DMA_ConfigTransfer(DMA1, tx_channel,
                          LL_DMA_DIRECTION_MEMORY_TO_PERIPH | LL_DMA_MODE_NORMAL |
                              LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT |
                              LL_DMA_PDATAALIGN_BYTE | LL_DMA_MDATAALIGN_BYTE |
                              LL_DMA_PRIORITY_LOW);
    LL_DMA_SetPeriphAddress(DMA1, tx_channel,
                            LL_USART_DMA_GetRegAddr(uart, LL_USART_DMA_REG_DATA_TRANSMIT));
    LL_DMA_SetPeriphRequest(DMA1, tx_channel, LL_DMAMUX_REQ_USART2_TX);
    LL_DMA_EnableIT_TC(DMA1, tx_channel);
    LL_DMA_EnableIT_TE(DMA1, tx_channel);
    NVIC_EnableIRQ(tx_irq);

    // CR1/BRR are writable only with UE = 0. Reset values: 8 data bits, no parity,
    // 1 stop bit, OVER8 = 0 (oversampling by 16, see uart_brr()).
    LL_USART_Disable(uart);
    uart->BRR = uart_brr(baud);
    LL_USART_SetTransferDirection(uart, LL_USART_DIRECTION_TX_RX);
    LL_USART_EnableDMAReq_TX(uart);
    LL_USART_EnableIT_RXNE_RXFNE(uart);
    LL_USART_Enable(uart);
    while (LL_USART_IsActiveFlag_TEACK(uart) == 0U || LL_USART_IsActiveFlag_REACK(uart) == 0U) {
    }
    NVIC_EnableIRQ(USART2_IRQn);
}

void uart_write(std::span<const std::uint8_t> data) {
    // Masks only the DMA interrupt: the IMU edge interrupt keeps its timing.
    while (!data.empty()) {
        NVIC_DisableIRQ(tx_irq);
        const std::size_t n = std::min(data.size(), tx_size - g_tx.len);
        std::ranges::copy(data.first(n), std::span{g_tx.buf[g_tx.fill]}.subspan(g_tx.len).begin());
        g_tx.len += n;
        if (!g_tx.busy && g_tx.len > 0) {
            tx_start();
        }
        NVIC_EnableIRQ(tx_irq);
        // n == 0: both buffers full; the pending DMA interrupt runs here and frees one.
        data = data.subspan(n);
    }
}

std::optional<std::uint8_t> uart_read() {
    return board::ring_pop(g_rx);
}

std::uint32_t uart_rx_errors() {
    return g_rx_errors;
}

} // namespace board
