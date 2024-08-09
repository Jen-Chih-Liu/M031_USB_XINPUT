/******************************************************************************
 * @file     hid_mouse.c
 * @version  V1.00
 * $Revision: 11 $
 * $Date: 18/07/18 4:54p $
 * @brief    M031 series USBD driver Sample file
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
/*!<Includes */
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "hid_transfer.h"

uint8_t volatile g_u8EP2Ready = 0;
uint8_t volatile g_u8Suspend = 0;
uint8_t g_u8Idle = 0, g_u8Protocol = 0;

void USBD_IRQHandler(void)
{
    uint32_t volatile u32IntSts = USBD_GET_INT_FLAG();
    uint32_t volatile u32State = USBD_GET_BUS_STATE();

    if (u32IntSts & USBD_INTSTS_FLDET)
    {
        /* Floating detect */
        USBD_CLR_INT_FLAG(USBD_INTSTS_FLDET);

        if (USBD_IS_ATTACHED())
        {
            /* USB Plug In */
            USBD_ENABLE_USB();
        }
        else
        {
            /* USB Un-plug */
            USBD_DISABLE_USB();
        }
    }

    if (u32IntSts & USBD_INTSTS_BUS)
    {
        /* Clear event flag */
        USBD_CLR_INT_FLAG(USBD_INTSTS_BUS);

        if (u32State & USBD_STATE_USBRST)
        {
            /* Bus reset */
            USBD_ENABLE_USB();
            USBD_SwReset();
            g_u8Suspend = 0;
        }
        if (u32State & USBD_STATE_SUSPEND)
        {
            /* Enter power down to wait USB attached */
            g_u8Suspend = 1;

            /* Enable USB but disable PHY */
            USBD_DISABLE_PHY();
        }
        if (u32State & USBD_STATE_RESUME)
        {
            /* Enable USB and enable PHY */
            USBD_ENABLE_USB();
            g_u8Suspend = 0;
        }
    }

    if(u32IntSts & USBD_INTSTS_SOF)
    {
        /* Clear SOF flag */
        USBD_CLR_INT_FLAG(USBD_INTSTS_SOF);
    }

    if(u32IntSts & USBD_INTSTS_WAKEUP)
    {
        /* Clear event flag */
        USBD_CLR_INT_FLAG(USBD_INTSTS_WAKEUP);
    }

    if (u32IntSts & USBD_INTSTS_USB)
    {
        /* USB event */
        if (u32IntSts & USBD_INTSTS_SETUP)
        {
            /* Setup packet */
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_SETUP);

            /* Clear the data IN/OUT ready flag of control end-points */
            USBD_STOP_TRANSACTION(EP0);
            USBD_STOP_TRANSACTION(EP1);

            USBD_ProcessSetupPacket();
        }

        /* EP events */
        if (u32IntSts & USBD_INTSTS_EP0)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP0);
            /* control IN */
            USBD_CtrlIn();
        }

        if (u32IntSts & USBD_INTSTS_EP1)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP1);
            /* control OUT */
            USBD_CtrlOut();
        }

        if (u32IntSts & USBD_INTSTS_EP2)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP2);
            /* Interrupt IN */
            EP2_Handler();
        }

        if (u32IntSts & USBD_INTSTS_EP3)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP3);
            /* Interrupt OUT */
            EP3_Handler();
        }

        if (u32IntSts & USBD_INTSTS_EP4)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP4);
        }

        if (u32IntSts & USBD_INTSTS_EP5)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP5);
        }

        if (u32IntSts & USBD_INTSTS_EP6)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP6);
        }

        if (u32IntSts & USBD_INTSTS_EP7)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP7);
        }
    }
}

void EP2_Handler(void)  /* Interrupt IN handler */
{
	  uint8_t *ptrin,i;
    ptrin = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2));
     
	for(i=0;i<20;i++)
			{
			ptrin[i]=0x0;
			}
			ptrin[1]=0x14;
			ptrin[3]=0x10;
			ptrin[6]=0x80;
			ptrin[8]=0x80;
			ptrin[10]=0x83;
			ptrin[11]=0x3;
			ptrin[12]=0x80;
   USBD_SET_PAYLOAD_LEN(EP2, 20);
}

void EP3_Handler(void)  /* Interrupt OUT handler */
{
    uint8_t *ptr;
    /* Interrupt OUT */
    ptr = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP3));
    USBD_SET_PAYLOAD_LEN(EP3, 3);
}


/*--------------------------------------------------------------------------*/
/**
  * @brief  USBD Endpoint Config.
  * @param  None.
  * @retval None.
  */
void HID_Init(void)
{
    /* Init setup packet buffer */
    /* Buffer range for setup packet -> [0 ~ 0x7] */
    USBD->STBUFSEG = SETUP_BUF_BASE;

    /*****************************************************/
    /* EP0 ==> control IN endpoint, address 0 */
    USBD_CONFIG_EP(EP0, USBD_CFG_CSTALL | USBD_CFG_EPMODE_IN | 0);
    /* Buffer range for EP0 */
    USBD_SET_EP_BUF_ADDR(EP0, EP0_BUF_BASE);

    /* EP1 ==> control OUT endpoint, address 0 */
    USBD_CONFIG_EP(EP1, USBD_CFG_CSTALL | USBD_CFG_EPMODE_OUT | 0);
    /* Buffer range for EP1 */
    USBD_SET_EP_BUF_ADDR(EP1, EP1_BUF_BASE);

    /*****************************************************/
    /* EP2 ==> Interrupt IN endpoint, address 1 */
    USBD_CONFIG_EP(EP2, USBD_CFG_EPMODE_IN | INT_IN_EP_NUM);
    /* Buffer range for EP2 */
    USBD_SET_EP_BUF_ADDR(EP2, EP2_BUF_BASE);

    /* EP3 ==> Interrupt OUT endpoint, address 2 */
    USBD_CONFIG_EP(EP3, USBD_CFG_EPMODE_OUT | INT_OUT_EP_NUM);
    /* Buffer range for EP3 */
    USBD_SET_EP_BUF_ADDR(EP3, EP3_BUF_BASE);
    /* trigger to receive OUT data */
    USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
    USBD_SET_PAYLOAD_LEN(EP2, 20);
}

void HID_ClassRequest(void)
{
    uint8_t buf[8];
    uint8_t tempbuf[2]={0x12, 0x0};
    USBD_GetSetupPacket(buf);

    if(buf[0] & 0x80)    /* request data transfer direction */
    {
        /* Device to host */
        switch(buf[1])
        {
            case GET_IDLE:
            {
                USBD_SET_PAYLOAD_LEN(EP1, buf[6]);
                /* Data stage */
                USBD_PrepareCtrlIn(&g_u8Idle, buf[6]);
                /* Status stage */
                USBD_PrepareCtrlOut(0, 0); 
                break;
            }
            case GET_PROTOCOL:
            {
                USBD_SET_PAYLOAD_LEN(EP1, buf[6]);
                /* Data stage */
                USBD_PrepareCtrlIn(&g_u8Protocol, buf[6]);
                /* Status stage */
                USBD_PrepareCtrlOut(0, 0); 
                break;
            }
            case GET_REPORT:
             {
							 
							   
                
                USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)), (uint8_t *)&tempbuf, 2);
                
                /* Data stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 2);
                /* Status stage */
                USBD_PrepareCtrlOut(0,0);
							 
                 break;
             }
            default:
            {
                /* Setup error, stall the device */
                USBD_SetStall(EP0);
                USBD_SetStall(EP1);
                break;
            }
        }
    }
    else
    {
        /* Host to device */
        switch(buf[1])
        {
            case SET_REPORT:
            {
                if(buf[3] == 3)
                {
                    /* Request Type = Feature */
                    USBD_SET_DATA1(EP1);
                    USBD_SET_PAYLOAD_LEN(EP1, 0);
                }
								if(buf[3] == 0)
                {
                    /* Request Type = Feature */
                    USBD_SET_DATA1(EP1);
                    USBD_SET_PAYLOAD_LEN(EP1, 0);
                }
                break;
            }
            case SET_IDLE:
            {
                g_u8Idle = buf[3]; 
                /* Status stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 0);
                break;
            }
            case SET_PROTOCOL:
            {
                g_u8Protocol = buf[2]; 
                /* Status stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 0);
                break;
            }
            default:
            {
                /* Stall */
                /* Setup error, stall the device */
                USBD_SetStall(EP0);
                USBD_SetStall(EP1);
                break;
            }
        }
    }
}
