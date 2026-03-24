#pragma once

#include <stdint.h>
#include <stddef.h>

#define AHOI_HEADER_SIZE 6
#define AHOI_MAX_PAYLOAD_SIZE 127
#define AHOI_MAX_PACKET_SIZE (AHOI_MAX_PAYLOAD_SIZE + AHOI_HEADER_SIZE)
#define AHOI_DLE  0x10
#define AHOI_STX  0x02
#define AHOI_ETX  0x03
#define AHOI_ACK_TYPE 0x7F
#define AHOI_SERIAL_NACK_TYPE 0xFE
#define AHOI_SERIAL_ACK_TYPE 0xFF
#define AHOI_ID_CMD 0x84
#define AHOI_IS_DATA_PACKET(pkt) ((pkt)->type < 0x7F)
#define AHOI_IS_ACK(pkt) ((pkt)->type == AHOI_ACK_TYPE)
#define AHOI_IS_COMMAND_PACKET(pkt) ((pkt)->type >= 0x80 && (pkt)->type <= 0xFD)
#define AHOI_IS_SERIAL_ACK(pkt) ((pkt)->type >= AHOI_SERIAL_ACK_TYPE)
#define AHOI_IS_SERIAL_NACK(pkt) ((pkt)->type >= AHOI_SERIAL_NACK_TYPE)
#define AHOI_IS_FOOTER_CARRIER(pkt) (AHOI_IS_DATA_PACKET(pkt) || AHOI_IS_ACK(pkt))
#define AHOI_BROADCAST_ADDR 255
#define AHOI_IS_BROADCAST(pkt) ((pkt)->dst == AHOI_BROADCAST_ADDR)

// A and R flags are incompatible
typedef enum ahoi_packet_flags {
    AHOI_NO_FLAGS = 0x00,
    AHOI_A_FLAG = 0x01,
    AHOI_R_FLAG = 0x02,
    AHOI_E_FLAG = 0x04,
    AHOI_AE_FLAGS = 0x05,
    AHOI_RE_FLAGS = 0x06,
} ahoi_packet_flags_t;

typedef struct {
    uint8_t power;
    uint8_t rssi;
    uint8_t biterrors;
    uint8_t agcMean;
    uint8_t agcMin;
    uint8_t agcMax;
} ahoi_footer_t;

typedef struct {
    uint8_t src;
    uint8_t dst;
    uint8_t type;
    uint8_t flags;
    uint8_t seq;
    uint8_t pl_size;
    uint8_t payload[AHOI_MAX_PAYLOAD_SIZE];
    ahoi_footer_t footer;
} ahoi_packet_t;

typedef void (*msg_handler_t) (ahoi_packet_t* p);

int open_serial_port(const char *port, int baudrate);

int set_ahoi_id(int fd, uint8_t id);

void set_msg_handler(msg_handler_t h);

/**
 * Should be called in a loop
 * @return
 */
void handle_receive(int fd);

void handle_receive_silent(int fd);

void trigger_send(int fd, const ahoi_packet_t* ahoi);

void print_packet(const ahoi_packet_t *pkt);
