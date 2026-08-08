//
// Created by 24158 on 2026/6/17.
//

#include "fs_ia6b.h"
#include <stdio.h>

int ibus_parse( ibus_msg *msg, uint8_t *src ){

    uint16_t	calc_sum = 0;
    uint16_t 	ori_sum;
    uint8_t  	i;

    if( src[0] != 0x20 )
        return -1;
    if( src[1] != 0x40 )
        return -1;

    for( i = 0; i < 30; i++ )
        calc_sum += src[i];
    calc_sum ^= 0xFFFF;
    ori_sum = src[30] + (src[31]<<8);

    // printf( "calc_sum: %04x %04x\n", (unsigned)calc_sum, (unsigned)ori_sum );
    if( calc_sum != ori_sum )
        return -1;

    // IBUS 协议：通道值 12 位 (0~4095)
    for (i = 0; i < 14; i++)
    {
        msg->ch[i] = src[i * 2 + 2] + ((src[i * 2 + 3] & 0x0F) << 8);
    }

    // 通道 15~18（FS-IA6B 无输出）
    for (i = 14; i < 18; i++)
    {
        msg->ch[i] = 0;
    }

    return 0;
}

void ibus_show( ibus_msg *msg ){
    uint8_t	i;
    for( i = 0; i < 14; i++ ){
        printf( "ch%d(%hu)  ", i, (int)msg->ch[i] );
    }
    printf( "\r\n" );
}
