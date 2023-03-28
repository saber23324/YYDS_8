#ifndef __QEVENT_H
#define __QEVENT_H

#include "main.h"

typedef uint8_t QSignal;
typedef struct QEventTag {
    QSignal sig;
    uint8_t dynamic_;
} QEvent;

#endif
