//
// Created by tumap on 5/7/20.
//

#include "isotp/isotp.h"
#include <errno.h>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/un.h>
#include <unistd.h>

/*
 * Parameters
 */
char *can_interface;
char *local_socket_filename;
unsigned can_tx_id;
unsigned can_rx_id;

#define VERBOSITY_INFO 1
#define VERBOSITY_ERROR 2
int verbosity_level = 1;

/*
 * CAN/ISO-TP related variables
 */
int can_socket;
long time_offset;
IsoTpLink g_link;
uint8_t g_isotpRecvBuf[1024];
uint8_t g_isotpSendBuf[1024];
char message[1024];

#define CONNECTION_REFUSAL_PERIOD 5000
uint32_t last_reported_conref_time = 0;
unsigned last_reported_conref_count = 0;

/*
 * Bridging buffers
 */
#define CAN2SOCK_LENGTH 4096
char can2sock_buffer[CAN2SOCK_LENGTH];
int can2sock_head = 0;
int can2sock_tail = 0;

#define SOCK2CAN_LENGTH 64
struct tCANmessage {
    unsigned id;
    int dlc;
    unsigned char data[8];
} sock2can_buffer[SOCK2CAN_LENGTH];
int sock2can_head = 0;
int sock2can_tail = 0;

/*
 * Local socket related variables
 */
// server socket
int local_socket;
// accepted connection socket
int local_socket_client = 0;

void print_msg(int level, char *msg) {
    if (level > verbosity_level) {
        write(STDOUT_FILENO, msg, strlen(msg));
        write(STDOUT_FILENO, "\r\n", 2);
    }
}

void print_error(char *msg) {
    if (verbosity_level <= VERBOSITY_ERROR) {
        write(STDOUT_FILENO, "Error: ", 7);
        write(STDOUT_FILENO, msg, strlen(msg));
        write(STDOUT_FILENO, ": ", 2);
        char *err_text = strerror(errno);
        write(STDOUT_FILENO, err_text, strlen(err_text));
        write(STDOUT_FILENO, "\r\n", 2);
    }
}

void isotp_user_debug(const char *message, ...) {
    write(STDOUT_FILENO, message, strlen(message));

}

int isotp_user_send_can(const uint32_t arbitration_id,
                        const uint8_t *data, const uint8_t size) {

    // space available?
    unsigned next_head = sock2can_head + 1;
    if (next_head >= SOCK2CAN_LENGTH)
        next_head = 0;
    if (next_head == sock2can_tail)
        return 1;

    // copy data
    sock2can_buffer[sock2can_head].id = arbitration_id;
    sock2can_buffer[sock2can_head].dlc = size;
    memcpy(sock2can_buffer[sock2can_head].data, data, size);

    // update pointer
    sock2can_head = next_head;

    return 0;
}

uint32_t isotp_user_get_ms(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (tv.tv_sec - time_offset) * 1000 + tv.tv_usec / 1000;
}


static unsigned fromHex(char *text) {
    unsigned value = 0;
    while (*text) {
        value <<= 4;
        if (*text >= '0' && *text <= '9')
            value |= *text - '0';
        else if (*text >= 'a' && *text <= 'f')
            value |= *text - 'a' + 10;
        else if (*text >= 'A' && *text <= 'F')
            value |= *text - 'A' + 10;
        text++;
    }
    return value;
}

static int parse_arguments(int argc, char **argv) {
    int arg_count = 0;
    while (--argc > 0) {
        argv++;
        if (strcmp(*argv, "-v") == 0) {
            verbosity_level = 0;
            continue;
        }
        if (strcmp(*argv, "-q") == 0) {
            verbosity_level = 2;
            continue;
        }
        if (argv[0][0] == '-') {
            printf("Unknown option '%s'\r\n", argv[0]);
            return 1;
        }
        if (arg_count == 0) {
            can_interface = strdup(*argv);
            arg_count++;
            continue;
        }
        if (arg_count == 1) {
            local_socket_filename = strdup(*argv);
            arg_count++;
            continue;
        }
        if (arg_count == 2) {
            can_rx_id = fromHex(*argv);
            arg_count++;
            continue;
        }
        if (arg_count == 3) {
            can_tx_id = fromHex(*argv);
            arg_count++;
            continue;
        }
        arg_count++;
    }

    return arg_count != 4;
}

void termination_handler(int signum) {
    // delete open socket
    unlink(local_socket_filename);
}

void pipe_handler(int signum) { puts("Pipe handler"); }

int main(int argc, char **argv) {

    /* Parse arguments */
    if (parse_arguments(argc, argv)) {
        puts("Usage: isotp-socket-bridge [options] <CAN interface> <Socket name> "
             "<RX msg id> <TX msg id>");
        puts("Options:");
        puts(" -v             increase verbosity");
        puts(" -q             be quiet");
        puts("<RX msg id>    CAN message identifier");
        puts("<TX msg id>    CAN message identifier");
        exit(EXIT_FAILURE);
    }

    if (verbosity_level <= VERBOSITY_INFO) {
        printf("* Started, CAN=%s, socket=%s, RX.id=%08X, TX.id=%08X\r\n",
               can_interface, local_socket_filename, can_rx_id, can_tx_id);
    }

    /* Catch signals */
    if (signal(SIGINT, termination_handler) == SIG_IGN)
        signal(SIGINT, SIG_IGN);
    if (signal(SIGHUP, termination_handler) == SIG_IGN)
        signal(SIGHUP, SIG_IGN);
    if (signal(SIGTERM, termination_handler) == SIG_IGN)
        signal(SIGTERM, SIG_IGN);
    signal(SIGPIPE, pipe_handler);

    /* Save start time */
    struct timeval tv;
    gettimeofday(&tv, NULL);
    time_offset = tv.tv_sec;

    /* Initialize CAN */
    struct sockaddr_can addr;
    struct ifreq ifr;

    can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    int resval;
    strcpy(ifr.ifr_name, can_interface);
    resval = ioctl(can_socket, SIOCGIFINDEX, &ifr);
    if (resval < 0) {
        print_error("ioctl");
        exit(EXIT_FAILURE);
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    resval = bind(can_socket, (struct sockaddr *) &addr, sizeof(addr));
    if (resval < 0) {
        print_error("bind on CAN");
        exit(EXIT_FAILURE);
    }

    struct can_filter rfilter[1];

    rfilter[1].can_id = can_rx_id | CAN_EFF_FLAG;
    rfilter[1].can_mask = (CAN_EFF_FLAG | CAN_EFF_MASK);

    resval = setsockopt(can_socket, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter,
                        sizeof(rfilter));
    if (resval < 0) {
        print_error("CAN filtering");
        exit(EXIT_FAILURE);
    }
    print_msg(VERBOSITY_INFO, "* CAN device open");

    /* Remove already existing local socket */
    resval = unlink("local-test.socket");
    if (resval) {
        if (errno != ENOENT) {
            print_error("unlink");
            exit(EXIT_FAILURE);
        }
    } else {
        print_msg(VERBOSITY_INFO, "* Old local socket removed");
    }

    /* Initialize local socket */
    local_socket = socket(PF_LOCAL, SOCK_STREAM, 0);
    struct sockaddr_un local_addr;
    local_addr.sun_family = AF_LOCAL;

    strncpy(local_addr.sun_path, local_socket_filename,
            sizeof(local_addr.sun_path));
    local_addr.sun_path[sizeof(local_addr.sun_path) - 1] = '\0';
    int local_addr_size = SUN_LEN(&local_addr);
    if (bind(local_socket, (struct sockaddr *) &local_addr, local_addr_size) < 0) {
        print_error("bind on local socket");
        exit(EXIT_FAILURE);
    }
    if (listen(local_socket, 5) == -1) {
        print_error("listen on local socket");
        exit(EXIT_FAILURE);
    }
    print_msg(VERBOSITY_INFO, "* Local socket created");

    /* Initialize ISO-TP layer */
    isotp_init_link(&g_link, can_tx_id, g_isotpSendBuf, sizeof(g_isotpSendBuf),
                    g_isotpRecvBuf, sizeof(g_isotpRecvBuf));
    print_msg(VERBOSITY_INFO, "* ISO-TP layer initialized");

    while (1) {

        /* Event timeout = 1ms, we can't wait for data forever because of handling
         * ISO-TP timeouts */
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 1000;

        /* Make a list of sockets to wait on */
        int nfds;
        fd_set rfds;
        fd_set wfds;
        FD_ZERO(&rfds);
        FD_ZERO(&wfds);
        // always read on CAN socket
        FD_SET(can_socket, &rfds);
        // if we have anything to send, wait on available CAN write
        if (sock2can_head != sock2can_tail)
            FD_SET(can_socket, &wfds);
        // wait for incoming connection
        FD_SET(local_socket, &rfds);
        nfds = local_socket;
        // if we have connection on local socket, read/write on it
        if (local_socket_client) {
            // always wait for read from local socket
            FD_SET(local_socket_client, &rfds);
            // wait for available write to local socket only if we have anything
            // to write there
            if (can2sock_head != can2sock_tail)
                FD_SET(local_socket_client, &wfds);
            nfds = local_socket_client;
        }

        /* Wait for events */
        resval = select(nfds + 1, &rfds, &wfds, NULL, &tv);
        if (resval < 0) {
            print_error("select");
            exit(EXIT_FAILURE);
        }


        /* Incoming connection? */
        if (resval && FD_ISSET(local_socket, &rfds)) {
            // already have open connection?
            if (local_socket_client) {
                // refuse connection
                int socket = accept(local_socket, NULL, NULL);
                close(socket);

                // report refused connection
                uint32_t now = isotp_user_get_ms();
                if (!last_reported_conref_time || (now - last_reported_conref_time) >= CONNECTION_REFUSAL_PERIOD) {
                    print_msg(VERBOSITY_INFO, "* Incoming connection refused");
                    last_reported_conref_time = now;
                    last_reported_conref_count = 0;
                } else {
                    last_reported_conref_count++;
                }
            } else {
                // accept connection
                local_socket_client = accept(local_socket, NULL, NULL);
                if (local_socket_client > 0)
                    print_msg(VERBOSITY_INFO, "* Incoming connection accepted");
                else
                    local_socket_client = 0;

                // make socket non-blocking
                resval =
                        fcntl(local_socket_client, F_SETFL,
                              fcntl(local_socket_client, F_GETFL, 0) | O_NONBLOCK);

                if (resval < 0) {
                    print_error("fcntl");
                }
            }
        }

        /* Incoming data on CAN? */
        if (resval && FD_ISSET(can_socket, &rfds)) {
            struct can_frame frame;

            int nbytes = read(can_socket, &frame, sizeof(struct can_frame));

            if (nbytes < 0) {
                print_error("read on CAN");
                exit(EXIT_FAILURE);
            }

            /* paranoid check ... */
            if (nbytes < sizeof(struct can_frame)) {
                print_msg(VERBOSITY_ERROR, "read on CAN: incomplete CAN frame");
                exit(EXIT_FAILURE);
            }

            if ((frame.can_id & 0x3fffff) == can_rx_id) {
                isotp_on_can_message(&g_link, frame.data, frame.can_dlc);
            }
        }

        /* Incoming data on local socket? */
        if (resval && FD_ISSET(local_socket_client, &rfds)) {
            char buf;
            int nbytes = read(local_socket_client, &buf, 1);
            if (nbytes < 0) {
                print_error("read on local socket");
                exit(EXIT_FAILURE);
            }
            if (nbytes == 0) {
                close(local_socket_client);
                local_socket_client = 0;
                print_msg(VERBOSITY_INFO, "* Local socket closed");
            } else {
                isotp_send(&g_link, &buf, 1);
            }
        }

        /* Write available on CAN? */
        if (resval && sock2can_head != sock2can_tail && FD_ISSET(can_socket, &wfds)) {
            struct can_frame frame;
            frame.can_dlc = sock2can_buffer[sock2can_tail].dlc;
            frame.can_id = sock2can_buffer[sock2can_tail].id | CAN_EFF_FLAG;
            memcpy(frame.data, sock2can_buffer[sock2can_tail].data,
                   frame.can_dlc);

            if (++sock2can_tail >= SOCK2CAN_LENGTH)
                sock2can_tail = 0;

            int resval = write(can_socket, &frame, sizeof(frame));
            if (resval < 0 && errno != ENOBUFS) {
                print_error("send on CAN");
                exit(EXIT_FAILURE);
            }
        }

        /* Write available on local socket */
        if (resval && local_socket_client && can2sock_head != can2sock_tail &&
            FD_ISSET(local_socket_client, &wfds)) {
            // write as much as possible
            int length = (can2sock_head > can2sock_tail)
                         ? (can2sock_head - can2sock_tail)
                         : (CAN2SOCK_LENGTH - can2sock_tail);
            length = write(local_socket_client, can2sock_buffer + can2sock_tail,
                           length);
            if (length < 0) {
                print_error("write on local socket");
                exit(EXIT_FAILURE);
            } else if (length == 0) {
                close(local_socket_client);
                local_socket_client = 0;
                print_msg(VERBOSITY_INFO, "* Local socket closed");
            } else {
                can2sock_tail += length;
                if (can2sock_tail >= CAN2SOCK_LENGTH)
                    can2sock_tail = 0;
            }
        }

        /* Poll link to handle multiple frame transmition */
        isotp_poll(&g_link);

        unsigned short length;
        unsigned ret = isotp_receive(&g_link, message, 1024, &length);
        if (ISOTP_RET_OK == ret) {
            // check available buffer
            int avail =
                    (can2sock_head >= can2sock_tail)
                    ? (CAN2SOCK_LENGTH - 1 - can2sock_head + can2sock_tail)
                    : (can2sock_tail - can2sock_head - 1);
            if (length <= avail) {
                if (can2sock_head > can2sock_tail) {
                    avail = CAN2SOCK_LENGTH - can2sock_head;
                    if (avail > length)
                        avail = length;
                    memcpy(can2sock_buffer + can2sock_head, message, avail);
                    can2sock_head += avail;
                    if (can2sock_head >= CAN2SOCK_LENGTH)
                        can2sock_head = 0;
                    length -= avail;
                    if (length) {
                        memcpy(can2sock_buffer, memcpy + avail, length);
                        can2sock_head = length;
                    }
                } else {
                    memcpy(can2sock_buffer + can2sock_head, message, length);
                    can2sock_head += length;
                }
            }
        }
    }

    return 0;
}
