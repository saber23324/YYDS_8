#ifndef __QEP_H
#define __QEP_H

#include "qevent.h"

typedef uint8_t QState;  /* 状态返回值，状态机状态处理函数返回值 */
typedef QState(*QStateHandler) (void *me, QEvent const *e); /* 声明一个函数指针 */
typedef struct QFsmTag {
    QStateHandler state; /* 状态变量，当前活动状态，也就是经常用到的 me->state */
} QFsm;  /* 平面状态机FSM数据结构 */

#define QFsm_ctor(me_, initial_)  \
    ((me_)->state = (initial_))
void QFsm_init(QFsm *me, QEvent const *e);
void QFsm_dispatch(QFsm *me, QEvent const *e);

#define Q_RET_HANDLED ((QState)0)
#define Q_RET_IGNORED ((QState)1)
#define Q_RET_TRAN ((QState)2)

#define Q_HANDLED() (Q_RET_HANDLED)  /* 已处理事件 */
#define Q_IGNORED() (Q_RET_IGNORED)  /* 忽略事件 */
#define Q_TRAN(target_)  \
    (((QFsm *)me)->state = (QStateHandler)(target_), Q_RET_TRAN)  /* 状态转移 */


/* 状态机的全部信号    一个工程中的多个状态机公用一个信号枚举  */
enum QReservedSignals {
    Q_ENTRY_SIG = 1,
    Q_EXIT_SIG,
    Q_INIT_SIG,
    Q_USER_SIG,  /* 下面是用户自定义信号  Q_USER_SIG占一个空的位置 */
		
		
		MID_SIG,
		LW_SIG,
		BW_SIG,
		FW_SIG,
		RW_SIG,
		TR_SIG,
		TL_SIG,
		TM_SIG,
		Take_Site_SIG,
		Catch_SIG,
		Put1_SIG,
		Put2_SIG,
		Return_SIG,
		GetCode_SIG,
		Find_Bridge_SIG,
		Find_Put_SIG,
		Find_put2_SIG,
		Get_num_SIG,
		Put_bridge_SIG,
		Put_end_SIG,
		Put_end2_SIG,
};

#endif
