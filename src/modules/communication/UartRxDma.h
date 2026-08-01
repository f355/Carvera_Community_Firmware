#ifndef UARTRXDMA_H
#define UARTRXDMA_H

#if defined(MACHINE_Z1)

#include <cstdint>

namespace uart_rx_dma {

void initialize();
bool ready();
int get();
bool take_error();

}

#endif
#endif
