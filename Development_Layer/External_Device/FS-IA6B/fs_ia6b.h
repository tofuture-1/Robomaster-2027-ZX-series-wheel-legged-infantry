//
// Created by 24158 on 2026/6/17.
//

#ifndef RC_ROBOTDOG_REBUILD20260605_FS_I6X_H
#define RC_ROBOTDOG_REBUILD20260605_FS_I6X_H
#include <stdint.h>

typedef struct{
    int16_t	ch[18];
}ibus_msg;

extern int ibus_parse( ibus_msg *msg, uint8_t *src );
extern void ibus_show( ibus_msg *msg );



#endif //RC_ROBOTDOG_REBUILD20260605_FS_I6X_H
