//
// Created by tumap on 5/7/20.
//

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include "isotp/isotp.h"
#include <linux/can.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <sys/time.h>

static int can_socket;
static int local_socket;
static int local_socket_client;
static int time_offset;
static IsoTpLink g_link;
static uint8_t g_isotpRecvBuf[1024];
static uint8_t g_isotpSendBuf[1024];
static char message[1024];

#define CAN2SOCK_LENGTH     1024
static char can2sock_buffer[CAN2SOCK_LENGTH];
static unsigned can2sock_head=0;
static unsigned can2sock_tail=0;


void isotp_user_debug(const char* message, ...) {
    write(STDOUT_FILENO, message, strlen(message));

}

int  isotp_user_send_can(const uint32_t arbitration_id,
                         const uint8_t* data, const uint8_t size) {

    struct can_frame frame;
    frame.can_dlc=size;
    frame.can_id=arbitration_id|CAN_EFF_FLAG;
    memcpy(frame.data, data, size);

    int resval=write(can_socket, &frame, sizeof(frame));
    if(resval<0) {
        perror("send");
        exit(1);
    }
    return 0;
}

uint32_t isotp_user_get_ms(void) {
    struct timeval  tv;
    gettimeofday(&tv, NULL);
    return (tv.tv_sec-time_offset)*1000+tv.tv_usec/1000;
}



static unsigned fromHex(char* text) {
    unsigned value=0;
    while(*text) {
        value<<=4;
        if(*text>='0' && *text<='9')
            value|=*text-'0';
        else if(*text>='a' && *text<='f')
            value|=*text-'a'+10;
        else if(*text>='A' && *text<='F')
            value|=*text-'A'+10;
        text++;
    }
    return value;
}



int main(int argc, char** argv) {

    unsigned rx_id=0x00150200;
    unsigned tx_id=0x00150201;

    struct timeval  tv;
    gettimeofday(&tv, NULL);
    time_offset=tv.tv_sec;

    /* Initialize CAN and other peripherals */
    struct sockaddr_can addr;
    struct ifreq ifr;

    can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    int resval;
    strcpy(ifr.ifr_name, "can0" );
    resval=ioctl(can_socket, SIOCGIFINDEX, &ifr);
    if(resval<0) {
        perror("ioctl");
        return 1;
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    resval=bind(can_socket, (struct sockaddr *)&addr, sizeof(addr));
    if(resval<0) {
        perror("bind");
        return 1;
    }


    /* Initialize local socket */
    local_socket=socket(PF_LOCAL, SOCK_STREAM, 0);
    struct sockaddr_un local_addr;
    local_addr.sun_family=AF_LOCAL;
    unlink("local-test.socket");
    strncpy(local_addr.sun_path, "local-test.socket", sizeof (local_addr.sun_path));
    local_addr.sun_path[sizeof (local_addr.sun_path) - 1] = '\0';
    int local_addr_size = SUN_LEN (&local_addr);
    if (bind (local_socket, (struct sockaddr *) &local_addr, local_addr_size) < 0) {
        perror ("bind");
        exit (EXIT_FAILURE);
    }
    if (listen(local_socket, 5) == -1) {
        perror("listen error");
        exit(-1);
    }
    local_socket_client=accept(local_socket, NULL, NULL);

    /* Initialize link, 0x7TT is the CAN ID you send with */
    isotp_init_link(&g_link, tx_id, g_isotpSendBuf, sizeof(g_isotpSendBuf), g_isotpRecvBuf, sizeof(g_isotpRecvBuf));

    while(1) {
//        struct can_frame f;
//        int i=read(can_socket, &f, sizeof(f));

        struct timeval tv;
        tv.tv_sec=0;
        tv.tv_usec=500;

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(can_socket, &rfds);
        FD_SET(local_socket_client, &rfds);
        resval=select(local_socket_client+1, &rfds, NULL, NULL, &tv);

        if(resval<0) {
            perror("select");
            return 1;
        }
        if(resval>0 && FD_ISSET(can_socket, &rfds)) {
            struct can_frame frame;

            int nbytes = read(can_socket, &frame, sizeof(struct can_frame));

            if (nbytes < 0) {
                perror("can raw socket read");
                return 1;
            }

            /* paranoid check ... */
            if (nbytes < sizeof(struct can_frame)) {
                fprintf(stderr, "read: incomplete CAN frame\n");
                return 1;
            }

            if((frame.can_id&0x3fffff)==rx_id) {
                isotp_on_can_message(&g_link, frame.data, frame.can_dlc);
            }

        }
        if(resval>0 && FD_ISSET(local_socket_client, &rfds)) {
            char buf;
            int nbytes=read(local_socket_client, &buf, 1);
            if(nbytes<0) {
                perror("*nix socket read");
                return 1;
            }
            isotp_send(&g_link, &buf, 1);

        }

        /* Poll link to handle multiple frame transmition */
        isotp_poll(&g_link);

        unsigned short length;
        unsigned ret = isotp_receive(&g_link, message, 1024, &length);
        if (ISOTP_RET_OK == ret) {
            write(local_socket_client, message, length);
        }
    }

    return 0;


}