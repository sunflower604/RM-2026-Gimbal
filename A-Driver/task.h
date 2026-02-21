#ifndef TASK_H
#define TASK_H
//简介：任务调度器，提供一个简单的定时器功能，方便在主循环中调用不同频率的任务。
//使用方法：在需要定时执行的任务函数中调用PROCESSOR宏，传入任务的周期（单位毫秒）。例如，PROCESSOR(10)表示该任务每10毫秒执行一次。
#define PROCESSOR(TASK_T)\
  static uint32_t lasttick,thistick;\
  thistick = HAL_GetTick();\
  if(thistick - lasttick < TASK_T) return;\
  lasttick = thistick;



#endif // TASK_H
