#ifndef __REMOTE_H
#define __REMOTE_H
#include <stdint.h>
#include "remote_control.h"

const RC_ctrl_t *local_rc_ctrl;	//26-1-23：这里存疑，h文件不要存在定义变量	
void Remote_Init(void);//遥控器初始化

#endif