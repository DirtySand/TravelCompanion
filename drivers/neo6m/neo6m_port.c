#include "neo6m_port.h"
#include "board_config.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/uart.h"

#define GPS_RX_BUFFER_SIZE 1024u
#define GPS_RX_BUFFER_MASK (GPS_RX_BUFFER_SIZE - 1u)

#define GPS_UART_ERROR_BITS                                                    \
  (UART_UARTDR_OE_BITS | UART_UARTDR_BE_BITS | UART_UARTDR_PE_BITS |           \
   UART_UARTDR_FE_BITS)

static volatile uint8_t rx_buffer[GPS_RX_BUFFER_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile uint32_t dropped_bytes;

static void gps_uart_irq_handler(void) {
  while (uart_is_readable(GPS_UART)) {
    uint32_t dr = uart_get_hw(GPS_UART)->dr;
    if (dr & GPS_UART_ERROR_BITS) {
      dropped_bytes++;
      continue;
    }
    uint16_t next = (uint16_t)((rx_head + 1u) & GPS_RX_BUFFER_MASK);
    if (next == rx_tail) {
      dropped_bytes++;
      continue;
    }
    rx_buffer[rx_head] = (uint8_t)dr;
    rx_head = next;
  }
}

void neo6m_port_hw_init(void) {
  uint irq = UART_IRQ_NUM(GPS_UART);
  irq_set_enabled(irq, false);

  uart_init(GPS_UART, GPS_BAUDRATE);
  gpio_set_function(GPS_UART_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(GPS_UART_RX_PIN, GPIO_FUNC_UART);
  gpio_pull_up(GPS_UART_RX_PIN);

  rx_head = 0;
  rx_tail = 0;

  irq_set_exclusive_handler(irq, gps_uart_irq_handler);
  uart_set_irqs_enabled(GPS_UART, true, false);
  irq_set_enabled(irq, true);
}

int neo6m_port_uart_read(void *ctx, uint8_t *data, size_t max_length) {
  (void)ctx;
  size_t count = 0;
  while (count < max_length && rx_tail != rx_head) {
    data[count++] = rx_buffer[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1u) & GPS_RX_BUFFER_MASK);
  }
  return (int)count;
}

int neo6m_port_uart_write(void *ctx, const uint8_t *data, size_t length) {
  (void)ctx;
  uart_write_blocking(GPS_UART, data, length);
  return (int)length;
}

uint32_t neo6m_port_dropped_bytes(void) { return dropped_bytes; }