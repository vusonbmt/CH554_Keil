
/********************************** (C) COPYRIGHT ******************************
 * File Name          : CH554IAPDemo.C
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2017/01/20
 * Description        : After power-on and running, the P17 LED light flashes. When the "EnableIAP" pin is detected to be low, it will jump from the user program to BOOT, and upgrade the user program through BOOT
 *******************************************************************************/
#include "./Public/CH554.H"
#include "./Public/Debug.H"

sbit EnableIAP = P1 ^ 6;
#define BOOT_ADDR 0x3800

#pragma NOAREGS

/*******************************************************************************
 * Function Name  : main
 * Description    : main function
 *                ：
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
typedef void (*pTaskFn)(void);

pTaskFn tasksArr[1];

void main(void)
{
  UINT16 i = 0;
  while (1)
  {
    SCK = ~SCK; // P17 blinks
    mDelaymS(50);
    if (EnableIAP == 0)
    { // P16 pin detects a low level jump
      break;
    }
  }
  EA = 0; // Turn off the total interrupt, must add
  tasksArr[0] = BOOT_ADDR;
  mDelaymS(100);
  (tasksArr[0])(); // Skip to BOOT upgrade procedure, use ISP tool to upgrade
  while (1)
    ;
}
