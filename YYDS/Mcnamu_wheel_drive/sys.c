
#include "sys.h"



/* 初始化小车状态机 */
extern QFsm car_state;  /* 实例化函数指针 */
extern QEvent car_signal;  /* 实例化事件结构体 */

uint16_t set_speed = 1500;  // 1300 最低速   (2000 1.5m/s)

void QFsm_init(QFsm *me, QEvent const *e);
QState Car_initial(void *me, QEvent const *e);


QEvent const QEP_reservedEvt_[] = { /* 定义事件QEvent结构体数组并初始化   新的事件在qep.h中的QReservedSignals中定义  */
        [Q_ENTRY_SIG] = {
                .sig = Q_ENTRY_SIG,
        },
        [Q_EXIT_SIG] = {
                .sig = Q_EXIT_SIG,
        },
        [Q_INIT_SIG] = {
                .sig = Q_INIT_SIG,
        },
				
				
				
				
        [MID_SIG] = {
                .sig = MID_SIG,
        },
        [FW_SIG] = {
                .sig = FW_SIG,
        },
        [BW_SIG] = {
                .sig = BW_SIG,
        },
				[LW_SIG] = {
                .sig = LW_SIG,
        },
				[RW_SIG] = {
                .sig = RW_SIG,
        },
				
				[TL_SIG] = {
                .sig = TL_SIG,
        },
				[TR_SIG] = {
                .sig = TR_SIG,
        },
				[TM_SIG] = {
                .sig = TM_SIG,
        },
				[Take_Site_SIG] = {
                .sig = Take_Site_SIG,
        },
				[Put1_SIG] = {
                .sig = Put1_SIG,
        },
				[Put2_SIG] = {
                .sig = Put2_SIG,
        },
				[Catch_SIG] = {
                .sig = Catch_SIG,
        },
				[Return_SIG] = {
                .sig = Return_SIG,
        },
				[GetCode_SIG] = {
                .sig = GetCode_SIG,
        },
				[Find_Bridge_SIG] = {
                .sig = Find_Bridge_SIG,
        },
				[Find_Put_SIG] = {
                .sig = Find_Put_SIG,
        },
				[Find_put2_SIG] = {
                .sig = Find_put2_SIG,
        },
				[Get_num_SIG] = {
                .sig = Get_num_SIG,
        },
				
				[Put_bridge_SIG] = {
                .sig = Put_bridge_SIG,
        },
				[Put_end_SIG] = {
                .sig = Put_end_SIG,
        },
				[Put_end2_SIG] = {
                .sig = Put_end2_SIG,
        },
};

/* 状态机0状态 执行空指令 */
QState qfsm_temp(void *me, QEvent const *e)
{
    __NOP();
    return Q_IGNORED();
}

void System_Init(void)
{
	
	
	
	
	
	car_state.state = qfsm_temp;         // 防止空的函数指针
	car_signal.sig = Q_ENTRY_SIG;        // 事件为初始化事件
	QFsm_init(&car_state, &car_signal);
	Car_initial(&car_state, &car_signal);

	
	//HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
	
	


}
