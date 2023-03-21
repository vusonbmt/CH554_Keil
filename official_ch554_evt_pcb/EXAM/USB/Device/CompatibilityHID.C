
/********************************** (C) COPYRIGHT *******************************
 * File Name          :CompatibilityHID.C
 * Author             : WCH
 * Version            : V1.1
 * Date               : 2017/07/05
 * Description        : CH554 analog HID compatible device, support interrupt up and down transmission, support setting full speed, low speed
 *******************************************************************************/

#include "./Public/CH554.H"
#include "./Public/Debug.H"
#include <stdio.h>
#include <string.h>

#define Fullspeed 1
#ifdef Fullspeed
#define THIS_ENDP0_SIZE 64
#else
#define THIS_ENDP0_SIZE DEFAULT_ENDP0_SIZE
#endif
UINT8X Ep0Buffer[8 > (THIS_ENDP0_SIZE + 2) ? 8 : (THIS_ENDP0_SIZE + 2)] _at_ 0x0000;             // Endpoint 0 OUT & IN buffer, which must be an even address
UINT8X Ep2Buffer[128 > (2 * MAX_PACKET_SIZE + 4) ? 128 : (2 * MAX_PACKET_SIZE + 4)] _at_ 0x0044; // ENDPOINT 2 IN & OUT BUFFER, WHICH MUST BE AN EVEN ADDRESS
UINT8 SetupReq, SetupLen, Ready, Count, FLAG, UsbConfig;
PUINT8 pDescr;             // USB configuration flag
USB_SETUP_REQ SetupReqBuf; // Stage the Setup package
#define UsbSetupBuf ((PUSB_SETUP_REQ)Ep0Buffer)

sbit Ep2InKey = P1 ^ 5; // K1 key
#pragma NOAREGS
/*Device descriptor*/
UINT8C DevDesc[18] = {0x12, 0x01, 0x10, 0x01, 0x00, 0x00, 0x00, THIS_ENDP0_SIZE,
                      0x31, 0x51, 0x07, 0x20, 0x00, 0x00, 0x00, 0x00,
                      0x00, 0x01};
UINT8C CfgDesc[41] =
    {
        0x09, 0x02, 0x29, 0x00, 0x01, 0x01, 0x04, 0xA0, 0x23, // Configuration descriptors
        0x09, 0x04, 0x00, 0x00, 0x02, 0x03, 0x00, 0x00, 0x05, // interface descriptors
        0x09, 0x21, 0x00, 0x01, 0x00, 0x01, 0x22, 0x22, 0x00, // HID class descriptors
        0x07, 0x05, 0x82, 0x03, THIS_ENDP0_SIZE, 0x00, 0x18,  // endpoint descriptors
        0x07, 0x05, 0x02, 0x03, THIS_ENDP0_SIZE, 0x00, 0x18,  // endpoint descriptors
};
/*String descriptor omitted*/

/*HID class report descriptor*/
UINT8C HIDRepDesc[] =
    {
        0x06, 0x00, 0xff,
        0x09, 0x01,
        0xa1, 0x01,            // The collection starts
        0x09, 0x02,            // Usage Page  usage
        0x15, 0x00,            // Logical  Minimun
        0x26, 0x00, 0xff,      // Logical  Maximun
        0x75, 0x08,            // Report Size
        0x95, THIS_ENDP0_SIZE, // Report Counet
        0x81, 0x06,            // Input
        0x09, 0x02,            // Usage Page  usage
        0x15, 0x00,            // Logical  Minimun
        0x26, 0x00, 0xff,      // Logical  Maximun
        0x75, 0x08,            // Report Size
        0x95, THIS_ENDP0_SIZE, // Report Counet
        0x91, 0x06,            // Output
        0xC0};
// unsigned char  code LangDes[]={0x04,0x03,0x09,0x04};           //Language descriptors
// unsigned char  code SerDes[]={
//                           0x28,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
//                           0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
//                           0x00,0x00,0x00,0x00,0x00,0x49,0x00,0x43,0x00,0x42,
//                           0x00,0x43,0x00,0x31,0x00,0x00,0x00,0x00,0x00,0x00
//                           };                                   //String descriptor

UINT8X UserEp2Buf[64]; // User data definitions

/*******************************************************************************
 * Function Name  : USBDeviceInit()
 * Description    : USB device mode configuration, device mode start, transceiver endpoint configuration, interrupt enablement
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void USBDeviceInit()
{
    IE_USB = 0;
    USB_CTRL = 0x00; // Set the USB device mode first
#ifndef Fullspeed
    UDEV_CTRL |= bUD_LOW_SPEED; // Select low-speed 1.5M mode
#else
    UDEV_CTRL &= ~bUD_LOW_SPEED; // Select full-speed 12M mode, the default way
#endif
    UEP2_DMA = Ep2Buffer;                                      // Endpoint 2: Data transfer address
    UEP2_3_MOD |= bUEP2_TX_EN;                                 // Endpoint 2 sends enabled
    UEP2_3_MOD |= bUEP2_RX_EN;                                 // Endpoint 2 receive enable
    UEP2_3_MOD &= ~bUEP2_BUF_MOD;                              // Endpoint 2 sends and receives buffers of 64 bytes each
    UEP2_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK | UEP_R_RES_ACK; // Endpoint 2 automatically flips the sync flag bits, with IN transactions returning NAK and OUT returning ACK
    UEP0_DMA = Ep0Buffer;                                      // Endpoint 0 data transfer address
    UEP4_1_MOD &= ~(bUEP4_RX_EN | bUEP4_TX_EN);                // Endpoint 0 sends and receives buffers of 64 bytes
    UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;                 // OUT transactions return ACKs and IN transactions return NAKs

    USB_DEV_AD = 0x00;
    UDEV_CTRL = bUD_PD_DIS;                               // Disables DP/DM pull-down resistors
    USB_CTRL = bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN; // Start the USB device and DMA, and automatically return the NAK before the interrupt flag is cleared during the interrupt
    UDEV_CTRL |= bUD_PORT_EN;                             // Allow USB ports
    USB_INT_FG = 0xFF;                                    // Clear the interrupt flag
    USB_INT_EN = bUIE_SUSPEND | bUIE_TRANSFER | bUIE_BUS_RST;
    IE_USB = 1;
}

/*******************************************************************************
 * Function Name  : Enp2BlukIn()
 * Description    : Bulk upload of USB device mode endpoint 2
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void Enp2BlukIn()
{
    memcpy(Ep2Buffer + MAX_PACKET_SIZE, UserEp2Buf, sizeof(UserEp2Buf)); // Load the feed
    UEP2_T_LEN = THIS_ENDP0_SIZE;                                        // Maximum package length for upload
    UEP2_CTRL = UEP2_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_ACK;             // Upload the data and answer the ACK when available
    while (UEP2_CTRL & UEP_T_RES_ACK)
        ; // Wait for the transfer to complete
}

/*******************************************************************************
 * Function Name  : DeviceInterrupt()
 * Description    : CH559USB interrupt handling functions
 *******************************************************************************/
void DeviceInterrupt(void) interrupt INT_NO_USB using 1 // USB interrupt service program, using register bank 1
{
    UINT8 len, i;
    if (UIF_TRANSFER) // USB transfer complete flag
    {
        switch (USB_INT_ST & (MASK_UIS_TOKEN | MASK_UIS_ENDP))
        {
        case UIS_TOKEN_IN | 2: // endpoint 2# Endpoint Bulk Upload
            UEP2_T_LEN = 0;    // The pre-used sending length must be cleared
            //            UEP1_CTRL ^= bUEP_T_TOG;                                          //If you do not set automatic flip, you need to flip manually
            UEP2_CTRL = UEP2_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_NAK; // NAK by default
            break;
        case UIS_TOKEN_OUT | 2: // endpoint 2# Endpoint batch download
            if (U_TOG_OK)       // Out-of-sync packets will be dropped
            {
                len = USB_RX_LEN; // The length of the received data, the data is stored from the first address of Ep2Buffer
                for (i = 0; i < len; i++)
                {
                    Ep2Buffer[MAX_PACKET_SIZE + i] = Ep2Buffer[i] ^ 0xFF; // OUT data is reversed to IN and verified by computer
                }
                UEP2_T_LEN = len;
                UEP2_CTRL = UEP2_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_ACK; // allow upload
            }
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
                if ((UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK) != USB_REQ_TYP_STANDARD) /*HID class commands*/
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
                else // standard request
                {
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
                        case 0x22:               // report descriptor
                            pDescr = HIDRepDesc; // Data ready to upload
                            len = sizeof(HIDRepDesc);
                            Ready = 1; // If there are more interfaces, this standard bit should be valid after the last interface is configured
                            break;
                        default:
                            len = 0xff; // Unsupported command or error
                            break;
                        }
                        if (SetupLen > len)
                        {
                            SetupLen = len; // limit the total length
                        }
                        len = SetupLen >= THIS_ENDP0_SIZE ? THIS_ENDP0_SIZE : SetupLen; // The length of this transfer
                        memcpy(Ep0Buffer, pDescr, len);                                 // load upload data
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
                            case 0x02:
                                UEP2_CTRL = UEP2_CTRL & ~(bUEP_R_TOG | MASK_UEP_R_RES) | UEP_R_RES_ACK;
                                break;
                            default:
                                len = 0xFF; // unsupported endpoint
                                break;
                            }
                        }
                        else
                        {
                            len = 0xFF; // endpoint does not support
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
                                    UEP2_CTRL = UEP2_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL; /* set endpoint 2 IN STALL */
                                    break;
                                case 0x02:
                                    UEP2_CTRL = UEP2_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL; /* set endpoint 2 OUT Stall */
                                    break;
                                case 0x81:
                                    UEP1_CTRL = UEP1_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL; /* set endpoint 1 IN STALL */
                                    break;
                                default:
                                    len = 0xFF; /* operation failed */
                                    break;
                                }
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
            else if (len <= THIS_ENDP0_SIZE) // Upload data or return a 0-length packet in the status stage
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
                len = SetupLen >= THIS_ENDP0_SIZE ? THIS_ENDP0_SIZE : SetupLen; // The length of this transfer
                memcpy(Ep0Buffer, pDescr, len);                                 // load upload data
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
#ifdef DE_PRINTF
            printf("zz"); // sleep state
#endif
            while (XBUS_AUX & bUART0_TX)
            {
                ; // Wait for the send to complete
            }
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

main()
{
    UINT8 i;
    CfgFsys();    // CH559 clock selection configuration
    mDelaymS(5);  // Modify the main frequency and wait for the internal crystal oscillator to stabilize, it must be added
    mInitSTDIO(); // Serial port 0 initialization
#ifdef DE_PRINTF
    printf("start ...\n");
#endif
    for (i = 0; i < 64; i++) // Prepare demo data
    {
        UserEp2Buf[i] = i;
    }
    USBDeviceInit(); // USB device mode initialization
    EA = 1;          // Allow microcontroller interrupt
    UEP1_T_LEN = 0;  // The pre-used sending length must be cleared
    UEP2_T_LEN = 0;  // The pre-used sending length must be cleared
    FLAG = 0;
    Ready = 0;
    while (1)
    {
        if (Ready && (Ep2InKey == 0))
        {
            Enp2BlukIn();
            mDelaymS(100);
        }
        mDelaymS(100); // Simulate MCU to do other things
    }
}
