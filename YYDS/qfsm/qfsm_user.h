#ifndef __QFSM_USER_H
#define __QFSM_USER_H

#include "qep.h"
#include "qevent.h"
#include "main.h"

typedef struct CarTag {
    uint8_t timeout; /* number of seconds till explosion */
    uint8_t code;    /* 目前输入的密码 */
    uint8_t defuse;  /* 解锁需要的密码 */

} Car;


#endif
