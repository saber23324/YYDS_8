#include "qep.h"


extern QEvent const QEP_reservedEvt_[100];

/**
 * @brief QFsm_init 初始化FSM
 *
 * @param me
 * @param e
 */
void QFsm_init(QFsm *me, QEvent const *e)
{
    (*me->state)(me, e); /* 执行最顶端的初始转换 */
    (void)(*me->state)(me , &QEP_reservedEvt_[Q_ENTRY_SIG]);  /* 执行进入动作 */
}
