#define _GNU_SOURCE

#include "core.h"

#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <termios.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#define RECV_BUF_SIZE (2*sizeof(ahoi_packet_t))
#define SEND_BUF_SIZE (2*sizeof(ahoi_packet_t))

typedef uint16_t buffer_len_t;

typedef struct buffer {
    buffer_len_t length;
    uint8_t* value;
} buffer_t;

static uint8_t recv_mem[RECV_BUF_SIZE] = {0};
static buffer_t recv_buf = {
        RECV_BUF_SIZE,
        recv_mem
};
static uint8_t send_mem[SEND_BUF_SIZE] = {0};
static buffer_t send_buf = {
        SEND_BUF_SIZE,
        send_mem
};
static ahoi_packet_t staging_packet = {0};

static uint8_t snack[] = {
    AHOI_DLE, AHOI_STX, 0, AHOI_SERIAL_NACK_TYPE, 0, 0, 0, AHOI_DLE, AHOI_ETX
};

static uint8_t sack[] = {
        AHOI_DLE, AHOI_STX,0, 0, AHOI_SERIAL_ACK_TYPE, 0, 0, 0, AHOI_DLE, AHOI_ETX
};

typedef struct {
    const buffer_t *buffer;
    size_t pos;
} mem_reader_t;

typedef mem_reader_t mem_writer_t;

static int mem_read(mem_reader_t *r, void *dst, size_t n)
{
    if (r->pos >= r->buffer->length)
        return 0; // EOF

    size_t remaining = r->buffer->length - r->pos;
    if (n > remaining)
        n = remaining;

    memcpy(dst, r->buffer->value + r->pos, n);
    r->pos += n;

    return (int) n;
}

static int mem_write(mem_writer_t *w, const buffer_t *src, size_t n)
{
    if (w->buffer == NULL) {
        w->pos += n;
        return (int) n;
    }

    if (w->pos >= w->buffer->length)
        return 0; // buffer full

    size_t remaining = w->buffer->length - w->pos;
    if (n > remaining)
        n = remaining;

    memcpy(w->buffer->value + w->pos, src->value, n);
    w->pos += n;

    return (int) n;
}

typedef enum packet_decode_status {
    AHOI_DECODE_OK,
    AHOI_DECODE_NOT_COMPLETE,
    AHOI_DECODE_KO
} packet_decode_status_t;

typedef enum packet_encode_status {
    AHOI_ENCODE_OK,
    AHOI_ENCODE_KO,
} packet_encode_status_t;

static msg_handler_t msg_handler = NULL;

static packet_encode_status_t ahoi_serial_protocol_encode(const ahoi_packet_t* ahoi, mem_writer_t* writer, size_t* n) {
    const uint8_t* raw = (const uint8_t*) ahoi;
    size_t packet_size = AHOI_HEADER_SIZE + ahoi->pl_size;

    if (ahoi->pl_size > AHOI_MAX_PAYLOAD_SIZE) {
        // TODO: Log error
        return AHOI_ENCODE_KO;
    }

    // Write STX
    uint8_t stx[2] = {AHOI_DLE, AHOI_STX};
    if (mem_write(writer, &(buffer_t){2, stx}, 2) != 2) return AHOI_ENCODE_KO;

    // Write packet bytes with byte-stuffing: escape 0x10 as 0x10 0x10
    for (size_t i = 0; i < packet_size; i++) {
        uint8_t byte = raw[i];
        if (byte == AHOI_DLE) {
            uint8_t esc[2] = {AHOI_DLE, AHOI_DLE};
            if (mem_write(writer, &(buffer_t){2, esc}, 2) != 2) return AHOI_ENCODE_KO;
        } else {
            if (mem_write(writer, &(buffer_t){1, &byte}, 1) != 1) return AHOI_ENCODE_KO;
        }
    }

    // Write ETX
    uint8_t etx[2] = {AHOI_DLE, AHOI_ETX};
    if (mem_write(writer, &(buffer_t){2, etx}, 2) != 2) return AHOI_ENCODE_KO;

    if (n != NULL) {
        *n = packet_size;
    }
    return AHOI_ENCODE_OK;
}

static int put_byte(uint8_t byte, buffer_t* ahoi_buff, uint16_t* put_pos) {
    if (*put_pos >= ahoi_buff->length) {
        // TODO: Log error buffer overflow
        return -1;
    }

    ahoi_buff->value[(*put_pos)++] = byte;

    if (*put_pos == AHOI_HEADER_SIZE && ahoi_buff->value[AHOI_HEADER_SIZE - 1] > AHOI_MAX_PAYLOAD_SIZE) {
        // TODO: Log error
        return -1;
    }
    if (*put_pos >= AHOI_HEADER_SIZE && *put_pos == AHOI_HEADER_SIZE + ahoi_buff->value[AHOI_HEADER_SIZE - 1]) {
        *put_pos = AHOI_MAX_PACKET_SIZE;
    }
    return 1;
}

static packet_decode_status_t ahoi_serial_protocol_decode(mem_reader_t* reader, ahoi_packet_t* ahoi, size_t* n) {
    packet_decode_status_t ret = AHOI_DECODE_OK;
    uint8_t byte;
    bool in_packet = false;
    bool packet_received = false;
    const buffer_t* stream_in = reader->buffer;
    memset(ahoi, 0, sizeof(ahoi_packet_t));
    buffer_t stream_out = {
            sizeof(ahoi_packet_t),
            (uint8_t*) ahoi
    };
    uint16_t put_pos = 0;
    uint16_t actual_size = 0;
    for (int i = 0; i < stream_in->length; ++i) {
        if (mem_read(reader, &byte, 1) != 1) continue;
        if (!in_packet && byte == AHOI_DLE) {
            if (mem_read(reader, &byte, 1) == 1 && byte == AHOI_STX) {
                in_packet = 1;
//                buf_pos = 0;
                actual_size = 0;
            }
            continue;
        }

        if (in_packet) {
            if (byte == AHOI_DLE) {
                if (mem_read(reader, &byte, 1) == 1) {
                    if (byte == AHOI_ETX) {
                        if (put_pos < AHOI_HEADER_SIZE) {
//                          TODO: Log Error
                            in_packet = 0;
                            ret = AHOI_DECODE_KO;
                            goto finish;
                        }
                        in_packet = 0;
                        packet_received = 1;
                        break;
                    } else if (byte != AHOI_DLE) {
                        // TODO: Log error: malformed packet
                        ret = AHOI_DECODE_KO;
                        goto finish;
                    }
                } else {
                    // TODO: Log stream error
                    ret = AHOI_DECODE_KO;
                    goto finish;
                }
            }

            if (put_byte(byte, &stream_out, &put_pos) < 0) {
                ret = AHOI_DECODE_KO;
                goto finish;
            }
            actual_size++;
            if (actual_size > AHOI_MAX_PACKET_SIZE && !AHOI_IS_FOOTER_CARRIER(ahoi)) {
                // TODO: Log error
                return -1;
            }
        }
    }

    if (!packet_received) {
        if (in_packet) {
            ret = AHOI_DECODE_NOT_COMPLETE;
            goto finish;
        }
        // TODO: Log error
        ret = AHOI_DECODE_KO;
        goto finish;
    }

    finish:
    if (n != NULL) {
        *n = actual_size;
    }
    return ret;
}

// TODO: Log
// TODO: Something ugly here. Need to refactor
static int send_ahoi_cmd(int fd, uint8_t* rsp_buf, size_t buf_len, size_t* rsp_len) {
    int ret = -1;
    if (!AHOI_IS_COMMAND_PACKET(&staging_packet)) {
        fprintf(stderr, "Expecting ahoi command packet\n");
        return ret;
    }

    // TODO: Ensure that there is no data in read in the socket

    mem_writer_t writer = {
            NULL,
            0
    };
    ahoi_serial_protocol_encode(&staging_packet, &writer, NULL);
    uint8_t cmd_buf[writer.pos];
    buffer_t b = {
            writer.pos,
            cmd_buf
    };
    writer.buffer = &b;
    writer.pos = 0;
    ahoi_serial_protocol_encode(&staging_packet, &writer, NULL);

    ssize_t n = write(fd, b.value, b.length);
    if (n < 0) {
        fprintf(stderr, "Error writing to serial port\n");
        goto finish;
    }
    if (n != writer.pos) {
        fprintf(stderr, "Warning: Partial write (%zd of %lu bytes)\n", n, writer.pos);
        goto finish;
    }

    // Ensuring response arrival
    usleep(100 * 1000);

    uint8_t cmd_resp_buf[256];
    b.value = cmd_resp_buf;
    b.length = sizeof(cmd_resp_buf);

    if ((n = read(fd, b.value, b.length)) < 1) {
        // TODO: Log
        goto finish;
    }

    b.length = n;

    mem_reader_t resp_reader = {
            &b,
            0
    };
    if (ahoi_serial_protocol_decode(&resp_reader, &staging_packet, NULL) != AHOI_DECODE_OK) {
        goto finish;

    }

    if (!AHOI_IS_SERIAL_ACK(&staging_packet)) {
        fprintf(stderr, "Ahoi cmd is malformed\n");
        goto finish;
    }

    if (rsp_buf != NULL) {
        if (buf_len < staging_packet.pl_size) {
            fprintf(stderr, "Response buffer is too small\n");
            goto finish;
        }
        memcpy(rsp_buf, staging_packet.payload, staging_packet.pl_size);
        *rsp_len = staging_packet.pl_size;
    }
    finish:
    return ret;
}

static int ahoi_set_command(int fd, const uint8_t type, const uint8_t* payload, const size_t pl_len) {
    staging_packet.type = type;
    staging_packet.pl_size = pl_len;
    memcpy(staging_packet.payload, payload, pl_len);

    return send_ahoi_cmd(fd, NULL, 0, NULL);
}

int set_ahoi_id(int fd, const uint8_t id) {
    return ahoi_set_command(fd, AHOI_ID_CMD, &id, 1);
}

int open_serial_port(const char *port, int baudrate) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd == -1) {
        return -1;
    }

    struct termios options;
    tcgetattr(fd, &options);

    cfsetispeed(&options, baudrate);
    cfsetospeed(&options, baudrate);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;

    tcsetattr(fd, TCSANOW, &options);
    return fd;
}

void set_msg_handler(msg_handler_t h) {
    msg_handler = h;
}

void handle_receive_silent(int fd) {
    ssize_t n = read(fd, recv_buf.value, recv_buf.length);
    if (n > 0) {
        mem_reader_t r = {
                &recv_buf,
                0
        };
        if (ahoi_serial_protocol_decode(&r, &staging_packet, NULL) != AHOI_DECODE_OK) {
            // TODO: Log error
            return;
        }

        if (msg_handler != NULL) {
            msg_handler(&staging_packet);
        }
    }
}

void trigger_send(int fd, const ahoi_packet_t* ahoi) {
    mem_writer_t w = {
        &send_buf,
        0
    };
    if (ahoi_serial_protocol_encode(ahoi, &w, NULL) != AHOI_ENCODE_OK) {
        return;
    }

    printf("Wrote %zu bytes\n", w.pos);

    if (write(fd, send_buf.value, w.pos) < 0) {
        // TODO: error
        return;
    }

    sleep(1);
    uint8_t sack_buf[sizeof(sack)];
    if (read(fd, sack_buf, sizeof(sack)) != sizeof(sack)) {
        perror("sack read");
    }
}

/* ---------- IPv6 / UDP payload printer ---------------------------------- */

static void print_ipv6_addr(const uint8_t *a) {
    for (int i = 0; i < 16; i += 2) {
        if (i) printf(":");
        printf("%02x%02x", a[i], a[i+1]);
    }
}

static uint16_t calc_udp6_checksum(const uint8_t *src, const uint8_t *dst,
                                    const uint8_t *udp, uint16_t udp_len) {
    uint32_t sum = 0;
    for (int i = 0; i < 16; i += 2) sum += (uint16_t)((src[i] << 8) | src[i+1]);
    for (int i = 0; i < 16; i += 2) sum += (uint16_t)((dst[i] << 8) | dst[i+1]);
    sum += (uint32_t)udp_len;
    sum += 17; /* next-header UDP */
    for (uint16_t i = 0; i < udp_len; i += 2) {
        uint16_t w = (i + 1 < udp_len)
                     ? (uint16_t)((udp[i] << 8) | udp[i+1])
                     : (uint16_t)(udp[i] << 8);
        sum += w;
    }
    while (sum >> 16) sum = (sum & 0xFFFF) + (sum >> 16);
    return (uint16_t)(~sum);
}

/* Returns 1 if payload looks like an IPv6/UDP datagram and prints it */
static int try_print_ipv6_udp(const uint8_t *pl, uint8_t pl_size) {
    /* Minimum: 40 (IPv6) + 8 (UDP) = 48 bytes */
    if (pl_size < 48) return 0;

    /* Check IPv6 version nibble */
    if ((pl[0] >> 4) != 6) return 0;

    uint8_t  next_hdr   = pl[6];
    uint8_t  hop_limit  = pl[7];
    uint16_t pay_len    = (uint16_t)((pl[4] << 8) | pl[5]);
    const uint8_t *src6 = pl + 8;
    const uint8_t *dst6 = pl + 24;

    /* We only decode UDP (17) for now */
    if (next_hdr != 17) return 0;

    /* Sanity: pay_len must cover the UDP header */
    if (pay_len < 8 || (uint16_t)(40 + pay_len) > pl_size) return 0;

    const uint8_t *udp    = pl + 40;
    uint16_t sport        = (uint16_t)((udp[0] << 8) | udp[1]);
    uint16_t dport        = (uint16_t)((udp[2] << 8) | udp[3]);
    uint16_t udp_len      = (uint16_t)((udp[4] << 8) | udp[5]);
    uint16_t udp_cksum    = (uint16_t)((udp[6] << 8) | udp[7]);
    uint16_t expected_ck  = calc_udp6_checksum(src6, dst6, udp, udp_len);
    int      cksum_ok     = (expected_ck == 0 || udp_cksum == 0); /* 0 means not computed */
    if (!cksum_ok) cksum_ok = (expected_ck == udp_cksum);

    /* Traffic class + flow label */
    uint8_t  tc   = (uint8_t)(((pl[0] & 0x0F) << 4) | (pl[1] >> 4));
    uint32_t flow = (uint32_t)(((pl[1] & 0x0F) << 16) | (pl[2] << 8) | pl[3]);

    printf("│  ╔═══ IPv6 Header ════════════════════════════╗\n");
    printf("│  ║  version    : 6\n");
    printf("│  ║  traffic cl : 0x%02X\n", tc);
    printf("│  ║  flow label : 0x%05X\n", flow);
    printf("│  ║  payload len: %u\n", pay_len);
    printf("│  ║  next hdr   : %u (UDP)\n", next_hdr);
    printf("│  ║  hop limit  : %u\n", hop_limit);
    printf("│  ║  src        : "); print_ipv6_addr(src6); printf("\n");
    printf("│  ║  dst        : "); print_ipv6_addr(dst6); printf("\n");
    printf("│  ╠═══ UDP Header ═════════════════════════════╣\n");
    printf("│  ║  src port   : %u\n", sport);
    printf("│  ║  dst port   : %u\n", dport);
    printf("│  ║  length     : %u\n", udp_len);
    printf("│  ║  checksum   : 0x%04X %s\n", udp_cksum, cksum_ok ? "[OK]" : "[BAD]");

    uint16_t data_len = (uint16_t)(udp_len > 8 ? udp_len - 8 : 0);
    if (data_len > 0) {
        const uint8_t *data = udp + 8;
        printf("│  ╠═══ UDP Payload (%u byte%s) ══════════════════╣\n",
               data_len, data_len == 1 ? "" : "s");
        printf("│  ║  hex  : ");
        for (uint16_t i = 0; i < data_len; i++) printf("%02X ", data[i]);
        printf("\n");
        printf("│  ║  ascii: ");
        for (uint16_t i = 0; i < data_len; i++)
            printf("%c", (data[i] >= 0x20 && data[i] < 0x7F) ? data[i] : '.');
        printf("\n");
    }
    printf("│  ╚════════════════════════════════════════════╝\n");
    return 1;
}

/* ------------------------------------------------------------------------ */

void print_packet(const ahoi_packet_t *pkt) {
    if (!pkt) {
        printf("[ahoi_packet] (null)\n");
        return;
    }

    const char *flags_str[] = {"NONE", "A", "R", "?", "E", "AE", "RE", "?"};
    uint8_t f = pkt->flags & 0x07;

    printf("┌─── AHOI Packet ───────────────────────────────┐\n");
    printf("│  src      : 0x%02X (%3u)\n", pkt->src, pkt->src);
    printf("│  dst      : 0x%02X (%3u)%s\n", pkt->dst, pkt->dst,
           pkt->dst == AHOI_BROADCAST_ADDR ? "  [BROADCAST]" : "");
    printf("│  type     : 0x%02X", pkt->type);
    if      (AHOI_IS_SERIAL_ACK(pkt))  printf("  [SERIAL ACK]");
    else if (AHOI_IS_SERIAL_NACK(pkt)) printf("  [SERIAL NACK]");
    else if (AHOI_IS_ACK(pkt))         printf("  [ACK]");
    else if (pkt->type == AHOI_ID_CMD) printf("  [ID CMD]");
    else if (AHOI_IS_COMMAND_PACKET(pkt)) printf("  [COMMAND]");
    else if (AHOI_IS_DATA_PACKET(pkt))    printf("  [DATA]");
    printf("\n");
    printf("│  flags    : 0x%02X  [%s]\n", pkt->flags, flags_str[f]);
    printf("│  seq      : %u\n", pkt->seq);
    printf("│  pl_size  : %u\n", pkt->pl_size);

    if (pkt->pl_size > 0) {
        uint8_t len = pkt->pl_size <= AHOI_MAX_PAYLOAD_SIZE ? pkt->pl_size : AHOI_MAX_PAYLOAD_SIZE;

        if (!try_print_ipv6_udp(pkt->payload, pkt->pl_size)) {
            printf("│  payload  : ");
            for (uint8_t i = 0; i < len; i++)
                printf("%02X ", pkt->payload[i]);
            printf("\n");
            printf("│  as ASCII : ");
            for (uint8_t i = 0; i < len; i++)
                printf("%c", (pkt->payload[i] >= 0x20 && pkt->payload[i] < 0x7F) ? pkt->payload[i] : '.');
            printf("\n");
        }
    }

    if (AHOI_IS_FOOTER_CARRIER(pkt)) {
        printf("│  footer   :\n");
        printf("│    power      : %u\n", pkt->footer.power);
        printf("│    rssi       : %u\n", pkt->footer.rssi);
        printf("│    bit_errors : %u\n", pkt->footer.biterrors);
        printf("│    agc_mean   : %u\n", pkt->footer.agcMean);
        printf("│    agc_min    : %u\n", pkt->footer.agcMin);
        printf("│    agc_max    : %u\n", pkt->footer.agcMax);
    }

    printf("└───────────────────────────────────────────────┘\n");
    fflush(stdout);
}
