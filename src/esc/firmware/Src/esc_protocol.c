#include "esc_protocol.h"
#include "config.h"
#include "setup.h"
#include "runtime_control.h"
#include "stm32f1xx_hal.h"
#include <string.h>

extern UART_HandleTypeDef huart3;

static uint8_t rx_dma_buffer[SERIAL_BUFFER_SIZE];
static uint32_t rx_dma_length = sizeof(rx_dma_buffer);
static uint32_t rx_old_position = 0;

static uint8_t parser_buffer[ESC_FRAME_SIZE];
static uint16_t parser_index = 0;
static uint32_t valid_frame_count = 0;
static uint32_t bad_frame_count = 0;

/*
 * Command tidak boleh dieksekusi langsung dari USART3_IRQHandler. Khususnya
 * SAVE EEPROM dapat melakukan program/erase flash yang terlalu berat untuk ISR.
 * ISR hanya mem-parse dan memasukkan frame ke ring queue; main-loop yang
 * memanggil RuntimeControl_OnCommand(). SPSC queue: producer=USART3 ISR,
 * consumer=main-loop. Depth 8 cukup untuk burst HELLO+CONFIG+CONTROL.
 */
#define ESC_COMMAND_QUEUE_DEPTH 8U
static EscCommandFrame command_queue[ESC_COMMAND_QUEUE_DEPTH];
static volatile uint8_t command_queue_head = 0U;
static volatile uint8_t command_queue_tail = 0U;

/*
 * Buffer TX DMA harus memiliki lifetime permanen. Jangan pernah memberikan
 * alamat variabel lokal/stack ke HAL_UART_Transmit_DMA(), karena DMA masih
 * membaca buffer setelah fungsi pemanggil kembali. Buffer statik ini mencegah
 * frame telemetry berubah di tengah transmisi dan menghindari CRC rusak.
 */
static EscFeedbackFrame tx_dma_frame;

/*
 * CRC16-CCITT-FALSE: polynomial 0x1021, init 0xFFFF.
 * Dipakai agar frame command/telemetry lebih tahan terhadap byte corrupt
 * dibanding checksum XOR sederhana firmware lama.
 */
uint16_t EscProtocol_Crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    for (uint16_t i = 0; i < length; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0; bit < 8U; ++bit) {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* Memulai circular DMA USART3. Parser tidak mengasumsikan boundary frame DMA. */
void EscProtocol_StartRx(void)
{
    memset(rx_dma_buffer, 0, sizeof(rx_dma_buffer));
    parser_index = 0;
    rx_old_position = 0;
    command_queue_head = 0U;
    command_queue_tail = 0U;
    HAL_UART_Receive_DMA(&huart3, rx_dma_buffer, sizeof(rx_dma_buffer));
}

/*
 * Memasukkan satu byte ke parser fixed-frame 64 byte. START_FRAME little-endian
 * adalah CD AB. Jika CRC gagal parser melakukan resynchronization otomatis.
 */
static void EscProtocol_FeedByte(uint8_t byte)
{
    const uint8_t start_low = (uint8_t)(ESC_FRAME_START & 0xFFU);
    const uint8_t start_high = (uint8_t)(ESC_FRAME_START >> 8);

    if (parser_index == 0U) {
        if (byte == start_low) {
            parser_buffer[0] = byte;
            parser_index = 1U;
        }
        return;
    }

    if (parser_index == 1U) {
        if (byte == start_high) {
            parser_buffer[1] = byte;
            parser_index = 2U;
        } else if (byte == start_low) {
            parser_buffer[0] = byte;
            parser_index = 1U;
        } else {
            parser_index = 0U;
        }
        return;
    }

    parser_buffer[parser_index++] = byte;
    if (parser_index < ESC_FRAME_SIZE) {
        return;
    }

    EscCommandFrame frame;
    memcpy(&frame, parser_buffer, sizeof(frame));
    const uint16_t expected = EscProtocol_Crc16(parser_buffer, ESC_FRAME_SIZE - sizeof(uint16_t));

    if (frame.start == ESC_FRAME_START &&
        frame.version == ESC_PROTOCOL_VERSION &&
        frame.checksum == expected) {
        const uint8_t next_head = (uint8_t)((command_queue_head + 1U) % ESC_COMMAND_QUEUE_DEPTH);
        if (next_head != command_queue_tail) {
            command_queue[command_queue_head] = frame;
            /* Publish head terakhir setelah seluruh struct selesai ditulis. */
            command_queue_head = next_head;
            ++valid_frame_count;
        } else {
            /* Queue overflow diperlakukan sebagai bad/drop frame, bukan eksekusi di ISR. */
            ++bad_frame_count;
        }
    } else {
        ++bad_frame_count;
    }

    /* Reset lalu biarkan frame berikutnya mencari start baru. */
    parser_index = 0U;
}

/**
 * Mengeksekusi semua command valid yang sudah diantrekan USART3 ISR.
 * Fungsi ini WAJIB dipanggil dari main-loop, bukan interrupt, sehingga operasi
 * berat seperti EEPROM SAVE/LOAD tidak memblok ISR komunikasi/motor.
 */
void EscProtocol_ServiceCommands(void)
{
    while (command_queue_tail != command_queue_head) {
        EscCommandFrame frame = command_queue[command_queue_tail];
        command_queue_tail = (uint8_t)((command_queue_tail + 1U) % ESC_COMMAND_QUEUE_DEPTH);
        RuntimeControl_OnCommand(&frame);
    }
}

/*
 * Dipanggil oleh USART3_IRQHandler ketika IDLE line terdeteksi. Semua byte baru
 * dari circular DMA diteruskan ke parser, termasuk saat buffer wrap.
 */
void usart3_rx_check(void)
{
    uint32_t position = rx_dma_length - __HAL_DMA_GET_COUNTER(huart3.hdmarx);
    if (position == rx_old_position) {
        return;
    }

    if (position > rx_old_position) {
        for (uint32_t i = rx_old_position; i < position; ++i) {
            EscProtocol_FeedByte(rx_dma_buffer[i]);
        }
    } else {
        for (uint32_t i = rx_old_position; i < rx_dma_length; ++i) {
            EscProtocol_FeedByte(rx_dma_buffer[i]);
        }
        for (uint32_t i = 0; i < position; ++i) {
            EscProtocol_FeedByte(rx_dma_buffer[i]);
        }
    }

    rx_old_position = (position == rx_dma_length) ? 0U : position;
}

/*
 * Mengirim frame telemetry 64 byte menggunakan DMA bila UART siap.
 *
 * Penting: data lebih dahulu disalin ke tx_dma_frame yang bersifat statik.
 * HAL_UART_Transmit_DMA() bekerja asynchronous, sehingga pointer ke frame lokal
 * RuntimeControl_ServiceTelemetry() tidak boleh dipakai langsung.
 */
bool EscProtocol_SendFeedback(const EscFeedbackFrame *frame)
{
    if (frame == NULL || huart3.hdmatx == NULL) {
        return false;
    }

    /* gState adalah indikator resmi HAL bahwa TX sebelumnya telah selesai. */
    if (huart3.gState != HAL_UART_STATE_READY) {
        return false;
    }

    memcpy(&tx_dma_frame, frame, sizeof(tx_dma_frame));

    if (HAL_UART_Transmit_DMA(&huart3, (uint8_t *)&tx_dma_frame, sizeof(tx_dma_frame)) != HAL_OK) {
        return false;
    }

    return true;
}

uint32_t EscProtocol_GetValidFrameCount(void) { return valid_frame_count; }
uint32_t EscProtocol_GetBadFrameCount(void) { return bad_frame_count; }

/* Compile-time guard agar protokol host dan STM32 tidak diam-diam beda ukuran. */
typedef char EscCommandFrameMustBe64Bytes[(sizeof(EscCommandFrame) == ESC_FRAME_SIZE) ? 1 : -1];
typedef char EscFeedbackFrameMustBe64Bytes[(sizeof(EscFeedbackFrame) == ESC_FRAME_SIZE) ? 1 : -1];

/* Guard payload agar perubahan telemetry tidak melebihi 46 byte. */
typedef char EscTelemetryBasicMustFitPayload[(sizeof(EscTelemetryBasic) <= 46U) ? 1 : -1];
typedef char EscTelemetryFocMustFitPayload[(sizeof(EscTelemetryFoc) <= 46U) ? 1 : -1];
typedef char EscTelemetryPidMustFitPayload[(sizeof(EscTelemetryPid) <= 46U) ? 1 : -1];
typedef char EscTelemetryRawMustFitPayload[(sizeof(EscTelemetryRaw) <= 46U) ? 1 : -1];
typedef char EscTelemetryConfigMustFitPayload[(sizeof(EscTelemetryConfig) <= 46U) ? 1 : -1];
