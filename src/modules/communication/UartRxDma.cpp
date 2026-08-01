#if defined(MACHINE_Z1)

#include "UartRxDma.h"

#include <cstring>

#include "DmaRxRing.h"
#include "libs/compiler.h"
#include "lpc17xx_clkpwr.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_uart.h"

namespace {

constexpr uint8_t kChannel = 1;
constexpr uint32_t kChannelMask = 1U << kChannel;
constexpr std::size_t kRingSize = 0x220;
constexpr std::size_t kDmaBufferSize = 0x180;

struct RxStorage {
    DmaRxRing<kRingSize> ring;
    alignas(4) uint8_t dma_buffer[kDmaBufferSize];
};

LOCATED_IN_AHBSRAM ALIGNED_TO(4) RxStorage storage;
volatile uint32_t consumed;
volatile bool transport_error;

bool dma_irq_enabled()
{
    const uint32_t irq = static_cast<uint32_t>(DMA_IRQn);
    return (NVIC->ISER[irq >> 5] & (1UL << (irq & 0x1f))) != 0;
}

void restore_dma_irq(bool enabled)
{
    if (enabled) NVIC_EnableIRQ(DMA_IRQn);
}

void append_locked(const uint8_t* source, std::size_t length)
{
    if (storage.ring.push(source, length) != length) {
        transport_error = true;
    }
}

void capture_partial_locked()
{
    const uintptr_t base = reinterpret_cast<uintptr_t>(storage.dma_buffer);
    const uintptr_t destination = LPC_GPDMACH1->DMACCDestAddr;
    if (destination < base || destination > base + kDmaBufferSize) {
        transport_error = true;
        return;
    }

    const uint32_t position = static_cast<uint32_t>(destination - base);
    if (position < consumed) {
        transport_error = true;
        return;
    }
    if (position != consumed) {
        append_locked(storage.dma_buffer + consumed, position - consumed);
        consumed = position;
    }
}

void start_transfer()
{
    LPC_GPDMACH1->DMACCConfig = 0;
    LPC_GPDMACH1->DMACCSrcAddr = reinterpret_cast<uint32_t>(&LPC_UART2->RBR);
    LPC_GPDMACH1->DMACCDestAddr = reinterpret_cast<uint32_t>(storage.dma_buffer);
    LPC_GPDMACH1->DMACCLLI = 0;
    LPC_GPDMACH1->DMACCControl =
        GPDMA_DMACCxControl_TransferSize(kDmaBufferSize) |
        GPDMA_DMACCxControl_DI |
        GPDMA_DMACCxControl_I;

    // Request inputs 8-15 are multiplexed with timer matches. Clear only the
    // UART2 RX selector bit; leave every unrelated DMA route untouched.
    LPC_SC->DMAREQSEL &= ~(1UL << (GPDMA_CONN_UART2_Rx - 8));
    LPC_GPDMACH1->DMACCConfig =
        GPDMA_DMACCxConfig_SrcPeripheral(GPDMA_CONN_UART2_Rx) |
        GPDMA_DMACCxConfig_TransferType(GPDMA_TRANSFERTYPE_P2M) |
        GPDMA_DMACCxConfig_IE |
        GPDMA_DMACCxConfig_ITC |
        GPDMA_DMACCxConfig_E;
}

} // namespace

namespace uart_rx_dma {

void initialize()
{
    NVIC_DisableIRQ(DMA_IRQn);
    storage.ring.reset();
    std::memset(storage.dma_buffer, 0, sizeof(storage.dma_buffer));
    consumed = 0;
    transport_error = false;

    CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCGPDMA, ENABLE);
    LPC_GPDMACH1->DMACCConfig = 0;
    LPC_GPDMA->DMACIntTCClear = kChannelMask;
    LPC_GPDMA->DMACIntErrClr = kChannelMask;
    LPC_GPDMA->DMACConfig = GPDMA_DMACConfig_E;
    while ((LPC_GPDMA->DMACConfig & GPDMA_DMACConfig_E) == 0) {}

    UART_FIFO_CFG_Type fifo{};
    UART_FIFOConfigStructInit(&fifo);
    fifo.FIFO_DMAMode = ENABLE;
    UART_FIFOConfig(LPC_UART2, &fifo);

    start_transfer();
    NVIC_SetPriority(DMA_IRQn, 5);
    NVIC_EnableIRQ(DMA_IRQn);
}

bool ready()
{
    const bool restore = dma_irq_enabled();
    NVIC_DisableIRQ(DMA_IRQn);
    capture_partial_locked();
    const bool result = storage.ring.size() != 0;
    restore_dma_irq(restore);
    return result;
}

int get()
{
    const bool restore = dma_irq_enabled();
    NVIC_DisableIRQ(DMA_IRQn);
    capture_partial_locked();
    uint8_t byte = 0;
    const bool available = storage.ring.pop(&byte, 1) == 1;
    restore_dma_irq(restore);
    return available ? byte : -1;
}

bool take_error()
{
    const bool restore = dma_irq_enabled();
    NVIC_DisableIRQ(DMA_IRQn);
    const bool overflow = storage.ring.take_overflow();
    const bool error = transport_error || overflow;
    transport_error = false;
    restore_dma_irq(restore);
    return error;
}

} // namespace uart_rx_dma

extern "C" void DMA_IRQHandler()
{
    const uint32_t terminal_count = LPC_GPDMA->DMACIntTCStat;
    const uint32_t errors = LPC_GPDMA->DMACIntErrStat;

    if ((errors & kChannelMask) != 0) {
        capture_partial_locked();
        LPC_GPDMA->DMACIntErrClr = kChannelMask;
        if ((terminal_count & kChannelMask) != 0) {
            LPC_GPDMA->DMACIntTCClear = kChannelMask;
        }
        transport_error = true;
        consumed = 0;
        start_transfer();
        return;
    }

    if ((terminal_count & kChannelMask) != 0) {
        LPC_GPDMA->DMACIntTCClear = kChannelMask;
        append_locked(storage.dma_buffer + consumed,
                      kDmaBufferSize - consumed);
        consumed = 0;
        start_transfer();
    }
}

#endif
