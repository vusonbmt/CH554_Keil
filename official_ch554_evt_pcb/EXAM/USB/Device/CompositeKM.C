
/********************************** (C) COPYRIGHT *******************************
 * File Name          :CompositeKM.C
 * Author             : WCH
 * Version            : V1.1
 * Date               : 2017/07/05
 * Description        : CH559 simulates USB composite device, keyboard and mouse, supports class commands
 *******************************************************************************/

#include "./Public/CH554.H"
#include "./Public/Debug.H"
#include <string.h>
#include <stdio.h>

#define THIS_ENDP0_SIZE DEFAULT_ENDP0_SIZE

UINT8X Ep0Buffer[8 > (THIS_ENDP0_SIZE + 2) ? 8 : (THIS_ENDP0_SIZE + 2)] _at_ 0x0000;   // Endpoint 0 OUT&IN buffer, must be an even address
UINT8X Ep1Buffer[64 > (MAX_PACKET_SIZE + 2) ? 64 : (MAX_PACKET_SIZE + 2)] _at_ 0x000a; // Endpoint 1 IN buffer, must be an even address
UINT8X Ep2Buffer[64 > (MAX_PACKET_SIZE + 2) ? 64 : (MAX_PACKET_SIZE + 2)] _at_ 0x0050; // Endpoint 2 IN buffer, must be an even address
UINT8 SetupReq, SetupLen, Ready, Count, FLAG, UsbConfig;
PUINT8 pDescr;             // USB Configuration Flags
USB_SETUP_REQ SetupReqBuf; // Staging the Setup package
sbit Ep2InKey = P1 ^ 5;
#define UsbSetupBuf ((PUSB_SETUP_REQ)Ep0Buffer)
#define DEBUG 0
#pragma NOAREGS
/*Device descriptor*/
UINT8C DevDesc[18] = {0x12, 0x01, 0x10, 0x01, 0x00, 0x00, 0x00, 0x08,
                      0x3d, 0x41, 0x07, 0x21, 0x00, 0x00, 0x00, 0x00,
                      0x00, 0x01};
UINT8C CfgDesc[59] =
    {
        0x09, 0x02, 0x3b, 0x00, 0x02, 0x01, 0x00, 0xA0, 0x32, // configuration descriptor
        0x09, 0x04, 0x00, 0x00, 0x01, 0x03, 0x01, 0x01, 0x00, // interface descriptor, keyboard
        0x09, 0x21, 0x11, 0x01, 0x00, 0x01, 0x22, 0x3e, 0x00, // HID Class Descriptor
        0x07, 0x05, 0x81, 0x03, 0x08, 0x00, 0x0a,             // endpoint descriptor
        0x09, 0x04, 0x01, 0x00, 0x01, 0x03, 0x01, 0x02, 0x00, // interface descriptor, mouse
        0x09, 0x21, 0x10, 0x01, 0x00, 0x01, 0x22, 0x34, 0x00, // HID Class Descriptor
        0x07, 0x05, 0x82, 0x03, 0x04, 0x00, 0x0a              // endpoint descriptor
};
/* String descriptor */
/* HID class report descriptor */
UINT8C KeyRepDesc[62] =
    {
        0x05, 0x01, 0x09, 0x06, 0xA1, 0x01, 0x05, 0x07,
        0x19, 0xe0, 0x29, 0xe7, 0x15, 0x00, 0x25, 0x01,
        0x75, 0x01, 0x95, 0x08, 0x81, 0x02, 0x95, 0x01,
        0x75, 0x08, 0x81, 0x01, 0x95, 0x03, 0x75, 0x01,
        0x05, 0x08, 0x19, 0x01, 0x29, 0x03, 0x91, 0x02,
        0x95, 0x05, 0x75, 0x01, 0x91, 0x01, 0x95, 0x06,
        0x75, 0x08, 0x26, 0xff, 0x00, 0x05, 0x07, 0x19,
        0x00, 0x29, 0x91, 0x81, 0x00, 0xC0};
UINT8C MouseRepDesc[52] =
    {
        0x05, 0x01, 0x09, 0x02, 0xA1, 0x01, 0x09, 0x01,
        0xA1, 0x00, 0x05, 0x09, 0x19, 0x01, 0x29, 0x03,
        0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x03,
        0x81, 0x02, 0x75, 0x05, 0x95, 0x01, 0x81, 0x01,
        0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x38,
        0x15, 0x81, 0x25, 0x7f, 0x75, 0x08, 0x95, 0x03,
        0x81, 0x06, 0xC0, 0xC0};
/*mouse data*/
UINT8 HIDMouse[4] = {0x0, 0x0, 0x0, 0x0};
/*keyboard data*/
UINT8 HIDKey[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

/*******************************************************************************
 * Function Name  : USBDeviceInit()
 * Description    : USB device mode configuration, device mode startup, transceiver endpoint configuration, interrupt enable
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void USBDeviceInit()
{
    IE_USB = 0;
    USB_CTRL = 0x00;                                        // First set the USB device mode
    UEP2_DMA = Ep2Buffer;                                   // Endpoint 2 Data Transfer Address
    UEP2_3_MOD = UEP2_3_MOD & ~bUEP2_BUF_MOD | bUEP2_TX_EN; // Endpoint 2 send enable 64 byte buffer
    UEP2_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK;              // Endpoint 2 automatically flips the synchronization flag bit, and the IN transaction returns NAK
    UEP0_DMA = Ep0Buffer;                                   // Endpoint 0 data transfer address
    UEP4_1_MOD &= ~(bUEP4_RX_EN | bUEP4_TX_EN);             // Endpoint 0 single 64-byte transmit and receive buffer
    UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;              // OUT transaction returns ACK, IN transaction returns NAK
    UEP1_DMA = Ep1Buffer;                                   // Endpoint 1 data transfer address
    UEP4_1_MOD = UEP4_1_MOD & ~bUEP1_BUF_MOD | bUEP1_TX_EN; // Endpoint 1 send enable 64 byte buffer
    UEP1_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK;              // Endpoint 1 automatically flips the synchronization flag bit, and the IN transaction returns NAK

    USB_DEV_AD = 0x00;
    UDEV_CTRL = bUD_PD_DIS;                               // Disable DP/DM pull-down resistor
    USB_CTRL = bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN; // Start the USB device and DMA, and return to NAK automatically before the interrupt flag is cleared during the interrupt
    UDEV_CTRL |= bUD_PORT_EN;                             // Allow USB port
    USB_INT_FG = 0xFF;                                    // clear interrupt flag
    USB_INT_EN = bUIE_SUSPEND | bUIE_TRANSFER | bUIE_BUS_RST;
    IE_USB = 1;
}
/*******************************************************************************
 * Function Name  : Enp1IntIn()
 * Description    : Interrupt upload for USB device mode endpoint 1
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void Enp1IntIn()
{
    memcpy(Ep1Buffer, HIDKey, sizeof(HIDKey));               // load upload data
    UEP1_T_LEN = sizeof(HIDKey);                             // Upload data length
    UEP1_CTRL = UEP1_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_ACK; // Upload data and answer ACK when there is data
}
/*******************************************************************************
 * Function Name  : Enp2IntIn()
 * Description    : USB设备模式端点2的中断上传
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void Enp2IntIn()
{
    memcpy(Ep2Buffer, HIDMouse, sizeof(HIDMouse));           // load upload data
    UEP2_T_LEN = sizeof(HIDMouse);                           // Upload data length
    UEP2_CTRL = UEP2_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_ACK; // Upload data and answer ACK when there is data
}
/*******************************************************************************
 * Function Name  : DeviceInterrupt()
 * Description    : CH559USB interrupt processing function
 *******************************************************************************/
void DeviceInterrupt(void) interrupt INT_NO_USB using 1 // USB interrupt service routine, using register set 1
{
    UINT8 len;
    if (UIF_TRANSFER) // USB transfer complete flag
    {
        switch (USB_INT_ST & (MASK_UIS_TOKEN | MASK_UIS_ENDP))
        {
        case UIS_TOKEN_IN | 2: // endpoint 2# interrupt endpoint upload
            UEP2_T_LEN = 0;    // The pre-used sending length must be cleared
            //            UEP1_CTRL ^= bUEP_T_TOG;                                          //If you do not set automatic flip, you need to flip manually
            UEP2_CTRL = UEP2_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_NAK; // NAK by default
            break;
        case UIS_TOKEN_IN | 1: // endpoint 1# 中断端点上传
            UEP1_T_LEN = 0;    // The pre-used sending length must be cleared
            //            UEP2_CTRL ^= bUEP_T_TOG;                                          //If you do not set automatic flip, you need to flip manually
            UEP1_CTRL = UEP1_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_NAK; // NAK by default
            FLAG = 1;                                                /*transfer complete flag*/
            break;
        case UIS_TOKEN_SETUP | 0: // SETUP transaction
            len = USB_RX_LEN;
            if (len == (sizeof(USB_SETUP_REQ)))
            {
                SetupLen = UsbSetupBuf->wLengthL;
                if (UsbSetupBuf->wLengthH || SetupLen > 0x7F)
                {
                    SetupLen = 0x7F; // limit the total length
                }
                len = 0; // Default is success and upload 0 length
                SetupReq = UsbSetupBuf->bRequest;
                if ((UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK) != USB_REQ_TYP_STANDARD) /* HID class commands */
                {
                    switch (SetupReq)
                    {
                    case 0x01: // GetReport
                        break;
                    case 0x02: // GetIdle
                        break;
                    case 0x03: // GetProtocol
                        break;
                    case 0x09: // SetReport
                        break;
                    case 0x0A: // SetIdle
                        break;
                    case 0x0B: // SetProtocol
                        break;
                    default:
                        len = 0xFF; /*command not supported*/
                        break;
                    }
                }
                else
                {                     // standard request
                    switch (SetupReq) // request code
                    {
                    case USB_GET_DESCRIPTOR:
                        switch (UsbSetupBuf->wValueH)
                        {
                        case 1:               // device descriptor
                            pDescr = DevDesc; // Send the device descriptor to the buffer to be sent
                            len = sizeof(DevDesc);
                            break;
                        case 2:               // configuration descriptor
                            pDescr = CfgDesc; // Send the device descriptor to the buffer to be sent
                            len = sizeof(CfgDesc);
                            break;
                        case 0x22:                         // report descriptor
                            if (UsbSetupBuf->wIndexL == 0) // Interface 0 Report Descriptor
                            {
                                pDescr = KeyRepDesc; // Data ready to upload
                                len = sizeof(KeyRepDesc);
                            }
                            else if (UsbSetupBuf->wIndexL == 1) // Interface 1 Report Descriptor
                            {
                                pDescr = MouseRepDesc; // Data ready to upload
                                len = sizeof(MouseRepDesc);
                                Ready = 1; // If there are more interfaces, this standard bit should be valid after the last interface is configured
                            }
                            else
                            {
                                len = 0xff; // This program has only 2 interfaces, it is impossible to execute this sentence normally
                            }
                            break;
                        default:
                            len = 0xff; // Unsupported command or error
                            break;
                        }
                        if (SetupLen > len)
                        {
                            SetupLen = len; // limit the total length
                        }
                        len = SetupLen >= 8 ? 8 : SetupLen; // The length of this transfer
                        memcpy(Ep0Buffer, pDescr, len);     // load upload data
                        SetupLen -= len;
                        pDescr += len;
                        break;
                    case USB_SET_ADDRESS:
                        SetupLen = UsbSetupBuf->wValueL; // Temporary USB device address
                        break;
                    case USB_GET_CONFIGURATION:
                        Ep0Buffer[0] = UsbConfig;
                        if (SetupLen >= 1)
                        {
                            len = 1;
                        }
                        break;
                    case USB_SET_CONFIGURATION:
                        UsbConfig = UsbSetupBuf->wValueL;
                        break;
                    case 0x0A:
                        break;
                    case USB_CLEAR_FEATURE:                                                         // Clear Feature
                        if ((UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP) // endpoint
                        {
                            switch (UsbSetupBuf->wIndexL)
                            {
                            case 0x82:
                                UEP2_CTRL = UEP2_CTRL & ~(bUEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK;
                                break;
                            case 0x81:
                                UEP1_CTRL = UEP1_CTRL & ~(bUEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK;
                                break;
                            case 0x01:
                                UEP1_CTRL = UEP1_CTRL & ~(bUEP_R_TOG | MASK_UEP_R_RES) | UEP_R_RES_ACK;
                                break;
                            default:
                                len = 0xFF; // unsupported endpoint
                                break;
                            }
                        }
                        else
                        {
                            len = 0xFF; // not endpoint does not support
                        }
                        break;
                    case USB_SET_FEATURE:                               /* Set Feature */
                        if ((UsbSetupBuf->bRequestType & 0x1F) == 0x00) /* set up device */
                        {
                            if ((((UINT16)UsbSetupBuf->wValueH << 8) | UsbSetupBuf->wValueL) == 0x01)
                            {
                                if (CfgDesc[7] & 0x20)
                                {
                                    /* Set wakeup enable flag */
                                }
                                else
                                {
                                    len = 0xFF; /* operation failed */
                                }
                            }
                            else
                            {
                                len = 0xFF; /* operation failed */
                            }
                        }
                        else if ((UsbSetupBuf->bRequestType & 0x1F) == 0x02) /* set endpoint */
                        {
                            if ((((UINT16)UsbSetupBuf->wValueH << 8) | UsbSetupBuf->wValueL) == 0x00)
                            {
                                switch (((UINT16)UsbSetupBuf->wIndexH << 8) | UsbSetupBuf->wIndexL)
                                {
                                case 0x82:
                                    UEP2_CTRL = UEP2_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL; /* set endpoint2 IN STALL */
                                    break;
                                case 0x02:
                                    UEP2_CTRL = UEP2_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL; /* set endpoint2 OUT Stall */
                                    break;
                                case 0x81:
                                    UEP1_CTRL = UEP1_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL; /* set endpoint1 IN STALL */
                                    break;
                                default:
                                    len = 0xFF; // operation failed
                                    break;
                                }
                            }
                            else
                            {
                                len = 0xFF; // operation failed
                            }
                        }
                        else
                        {
                            len = 0xFF; // operation failed
                        }
                        break;
                    case USB_GET_STATUS:
                        Ep0Buffer[0] = 0x00;
                        Ep0Buffer[1] = 0x00;
                        if (SetupLen >= 2)
                        {
                            len = 2;
                        }
                        else
                        {
                            len = SetupLen;
                        }
                        break;
                    default:
                        len = 0xff; // operation failed
                        break;
                    }
                }
            }
            else
            {
                len = 0xff; // packet length error
            }
            if (len == 0xff)
            {
                SetupReq = 0xFF;
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL; // STALL
            }
            else if (len <= 8) // Upload data or return a 0-length packet in the status stage
            {
                UEP0_T_LEN = len;
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK; // The default data packet is DATA1, and the response ACK is returned
            }
            else
            {
                UEP0_T_LEN = 0;                                                      // Although it has not yet reached the state stage, the upload of 0-length data packets is preset in advance to prevent the host from entering the state stage in advance
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK; // The default data packet is DATA1, and the response ACK is returned
            }
            break;
        case UIS_TOKEN_IN | 0: // endpoint0 IN
            switch (SetupReq)
            {
            case USB_GET_DESCRIPTOR:
                len = SetupLen >= 8 ? 8 : SetupLen; // The length of this transfer
                memcpy(Ep0Buffer, pDescr, len);     // load upload data
                SetupLen -= len;
                pDescr += len;
                UEP0_T_LEN = len;
                UEP0_CTRL ^= bUEP_T_TOG; // Sync flag flip
                break;
            case USB_SET_ADDRESS:
                USB_DEV_AD = USB_DEV_AD & bUDA_GP_BIT | SetupLen;
                UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                break;
            default:
                UEP0_T_LEN = 0; // The status stage completes the interrupt or forces the upload of a 0-length data packet to end the control transmission
                UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                break;
            }
            break;
        case UIS_TOKEN_OUT | 0: // endpoint0 OUT
            len = USB_RX_LEN;
            if (SetupReq == 0x09)
            {
                if (Ep0Buffer[0])
                {
                    printf("Light on Num Lock LED!\n");
                }
                else if (Ep0Buffer[0] == 0)
                {
                    printf("Light off Num Lock LED!\n");
                }
            }
            UEP0_T_LEN = 0;                            // Although it has not yet reached the state stage, the upload of 0-length data packets is preset in advance to prevent the host from entering the state stage in advance
            UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_ACK; // The default data packet is DATA0, and the response ACK is returned
            break;
        default:
            break;
        }
        UIF_TRANSFER = 0; // Write 0 to clear interrupt
    }
    if (UIF_BUS_RST) // Device Mode USB Bus Reset Interrupt
    {
        UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
        UEP1_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK;
        UEP2_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK;
        USB_DEV_AD = 0x00;
        UIF_SUSPEND = 0;
        UIF_TRANSFER = 0;
        UIF_BUS_RST = 0; // clear interrupt flag
    }
    if (UIF_SUSPEND) // USB bus suspend/wake complete
    {
        UIF_SUSPEND = 0;
        if (USB_MIS_ST & bUMS_SUSPEND) // hang up
        {
#if DEBUG
            printf("zz"); // sleep state
#endif
            while (XBUS_AUX & bUART0_TX)
                ; // Wait for the send to complete
            SAFE_MOD = 0x55;
            SAFE_MOD = 0xAA;
            WAKE_CTRL = bWAK_BY_USB | bWAK_RXD0_LO; // Can be woken up when USB or RXD0 has a signal
            PCON |= PD;                             // to sleep
            SAFE_MOD = 0x55;
            SAFE_MOD = 0xAA;
            WAKE_CTRL = 0x00;
        }
    }
    else
    {                      // unexpected interruption, impossible situation
        USB_INT_FG = 0xFF; // clear interrupt flag
        //      printf("UnknownInt  N");
    }
}
void HIDValueHandle()
{
    UINT8 i;
    i = getkey();
    printf("%c", (UINT8)i);
    switch (i)
    {
        // Mouse data upload example
    case 'L': // left button
        HIDMouse[0] = 0x01;
        Enp2IntIn();
        HIDMouse[0] = 0;
        break;
    case 'R': // right click
        HIDMouse[0] = 0x02;
        Enp2IntIn();
        HIDMouse[0] = 0;
        break;
        // Example of keyboard data upload
    case 'A': // A key
        FLAG = 0;
        HIDKey[2] = 0x04; // key to start
        Enp1IntIn();
        HIDKey[2] = 0; // key end
        while (FLAG == 0)
        {
            ; /*Wait for the previous packet transfer to complete*/
        }
        Enp1IntIn();
        break;
    case 'P': // P键
        FLAG = 0;
        HIDKey[2] = 0x13;
        Enp1IntIn();
        HIDKey[2] = 0; // key end
        while (FLAG == 0)
        {
            ; /*Wait for the previous packet transfer to complete*/
        }
        Enp1IntIn();
        break;
    case 'Q': // Num Lock键
        FLAG = 0;
        HIDKey[2] = 0x53;
        Enp1IntIn();
        HIDKey[2] = 0; // key end
        while (FLAG == 0)
        {
            ; /*Wait for the previous packet transfer to complete*/
        }
        Enp1IntIn();
        break;
    default:                                                     // other
        UEP1_CTRL = UEP1_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_NAK; // NAK by default
        UEP2_CTRL = UEP2_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_NAK; // NAK by default
        break;
    }
}

main()
{
    CfgFsys();    // CH559 clock selection configuration
    mDelaymS(5);  // Modify the main frequency and wait for the internal crystal oscillator to stabilize, it must be added
    mInitSTDIO(); // Serial port 0 initialization
#ifdef DE_PRINTF
    printf("start ...\n");
#endif
    USBDeviceInit(); // USB device mode initialization
    EA = 1;          // Allow microcontroller interrupt
    UEP1_T_LEN = 0;  // The pre-used sending length must be cleared
    UEP2_T_LEN = 0;  // The pre-used sending length must be cleared
    FLAG = 0;
    Ready = 0;
    while (1)
    {
        if (Ready)
        {
            HIDValueHandle();
        }
        if (Ready && (Ep2InKey == 0))
        {
#ifdef DE_PRINTF // Read chip ID number
            printf("ID0 = %02x %02x \n", (UINT16) * (PUINT8C)(0x3FFA), (UINT16) * (PUINT8C)(0x3FFB));
            printf("ID1 = %02x %02x \n", (UINT16) * (PUINT8C)(0x3FFC), (UINT16) * (PUINT8C)(0x3FFD));
            printf("ID2 = %02x %02x \n", (UINT16) * (PUINT8C)(0x3FFE), (UINT16) * (PUINT8C)(0x3FFF));
#endif
            mDelaymS(100);
        }
        mDelaymS(100); // Simulate MCU to do other things
    }
}
