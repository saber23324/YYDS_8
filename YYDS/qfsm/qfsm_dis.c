#include "qep.h"


extern QEvent const QEP_reservedEvt_[100];

/**
 * @brief QFsm_dispatch FSM状态机派遣函数
 *
 * @param me
 * @param e
 */
void QFsm_dispatch(QFsm *me, QEvent const *e)
{
    QStateHandler s = me->state;  /* 保存当前的函数指针 */
    QState r = (*s)(me, e); /* 调用当前状态函数，并获取事件 */
    if(r == Q_RET_TRAN) {  /* 判断是否有状态转换事件 */
        (void)(*s)(me, &QEP_reservedEvt_[Q_EXIT_SIG]); /* 执行当前状态退出动作 */
        (void)(*me->state)(me, &QEP_reservedEvt_[Q_ENTRY_SIG]);/* 执行终状态进入动作 */
    }
}
