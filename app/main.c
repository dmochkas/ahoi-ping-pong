#include <errno.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include "core.h"

static int g_fd = -1;

static void send_pong_packet(const ahoi_packet_t *request)
{
    ahoi_packet_t response = {0};
    const size_t pong_len = 1;
    const uint8_t pong_rule_id = (uint8_t) PONG_RULE_ID;

    response.src = request->dst;
    response.dst = request->src;
    response.type = request->type;
    response.flags = request->flags;
    response.seq = request->seq;
    response.pl_size = (uint8_t)pong_len;
    response.payload[0] = pong_rule_id;

    trigger_send(g_fd, &response);
}

static void on_packet_received(ahoi_packet_t *pkt)
{
    const size_t ping_len = 1;
    const uint8_t ping_rule_id = (uint8_t) PING_RULE_ID;
    if (pkt == NULL || pkt->pl_size != ping_len) {
        return;
    }

    if (pkt->payload[0] != ping_rule_id) {
        return;
    }

    send_pong_packet(pkt);
}

int main(void) {
    struct pollfd fds = {
        .fd = -1,
        .events = POLLIN,
        .revents = 0,
    };

    g_fd = open_serial_port(AHOI_PORT, B115200);
    if (g_fd < 0) {
        perror("open_serial_port");
        return -1;
    }

    fds.fd = g_fd;
    set_ahoi_id(g_fd, AHOI_ID);
    set_msg_handler(on_packet_received);

    while (1) {
        int ret = poll(&fds, 1, -1);
        if (ret < 0) {
            if (errno == EINTR) {
                continue;
            }

            perror("poll");
            break;
        }

        if (fds.revents & POLLIN) {
            handle_receive_silent(g_fd);
        }

        if (fds.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            fprintf(stderr, "serial port error\n");
            break;
        }
    }

    close(g_fd);
    return 0;
}