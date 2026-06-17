#ifndef __PC_TX_TASK_H
#define __PC_TX_TASK_H

#include "Control_Task.h"
#include "Data_Analysis.h"

/* ── 状态请求接口（由 Data_Analysis GET_STATUS 命令触发） ── */
void PC_TX_ReqSysStatus(void);
void PC_TX_ReqChsStatus(void);
void PC_TX_ReqArmStatus(void);
void PC_TX_ReqToolStatus(void);
void PC_TX_ReqRobotStatus(void);
void PC_TX_ReqClimbStatus(void);

#endif
