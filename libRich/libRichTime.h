/*******************************************************************************************
  ZH星辰51单片机
  技术支持QQ群：975748331

* 文件: libRichTime.h
* 作者: 朱昊源
* 审核：洪灿墅
* 日期：2025-02-09
* 描述：软件延时库。本库中的所有函数都将独占CPU，直至满足延时时长！
* 警告：此文件中的任何修改都将被覆盖。如需修改代码，请新建工程并复制代码，切勿直接在此文件中进行修改！
*******************************************************************************************/

#ifndef LIB_RICH_TIME_H
#define LIB_RICH_TIME_H

void libRich_Delay5us();
void libRich_Delay10us();
void libRich_Delay50us();
void libRich_Delay100us();
void libRich_Delay500us();
void libRich_Delay1000us();

/// @brief 阻塞延时。此函数运行期间将独占CPU。
/// @param ms 需延时的毫秒数。最小值为1ms。
void libRich_DelayMs(unsigned int ms);

#endif