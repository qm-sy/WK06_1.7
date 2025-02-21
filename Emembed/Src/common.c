#include "common.h"

/**
 * @brief	Timer1中断处理函数
 *
 * @param   
 *
 * @return  void
**/
void Tim2_ISR( void ) interrupt 12   //10ms
{
    static uint8_t temp_scan_cnt = 0;

    if( temp.temp_scan_flag == 0 )
    {
        temp_scan_cnt++;
        if( temp_scan_cnt == 100)
        {
            temp_scan_cnt = 0;
            temp.temp_scan_flag = 1;
        }
    }

}
