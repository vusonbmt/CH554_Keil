
/********************************** (C) COPYRIGHT *******************************
* File Name          : USBHostHUB_KM.C
* Author             : WCH
* Version            : V1.0
* Date               : 2017/01/20
* Description        :
 USB host example for CH554, start USB device under DP/DM and HP/HM port
 USB host application example, initialize and enumerate the devices connected to the USB port, support up to 1 USB device at the same time, support a first-level external HUB,
 Can operate USB keyboard and mouse and HUB, including HID class command processing
 If you need to operate the U disk, please refer to the example under the U_DISK folder
 Supports simple USB printer operations, does not handle USB printer commands
*******************************************************************************/

#include "..\..\Public\CH554.H"
#include "..\..\Public\Debug.H"
#include "..\USB_LIB\USBHOST.H"
#include <stdio.h>
#include <string.h>

#pragma NOAREGS

/*Get device descriptor*/
UINT8C SetupGetDevDescr[] = {USB_REQ_TYP_IN, USB_GET_DESCRIPTOR, 0x00, USB_DESCR_TYP_DEVICE, 0x00, 0x00, sizeof(USB_DEV_DESCR), 0x00};

/*Get configuration descriptor*/
UINT8C SetupGetCfgDescr[] = {USB_REQ_TYP_IN, USB_GET_DESCRIPTOR, 0x00, USB_DESCR_TYP_CONFIG, 0x00, 0x00, 0x04, 0x00};

/*Set USB address*/
UINT8C SetupSetUsbAddr[] = {USB_REQ_TYP_OUT, USB_SET_ADDRESS, USB_DEVICE_ADDR, 0x00, 0x00, 0x00, 0x00, 0x00};

/* Set USB configuration */
UINT8C SetupSetUsbConfig[] = {USB_REQ_TYP_OUT, USB_SET_CONFIGURATION, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/*Set USB interface configuration*/
UINT8C SetupSetUsbInterface[] = {USB_REQ_RECIP_INTERF, USB_SET_INTERFACE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/*Clear endpoint STALL*/
UINT8C SetupClrEndpStall[] = {USB_REQ_TYP_OUT | USB_REQ_RECIP_ENDP, USB_CLEAR_FEATURE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/*Get HUB descriptor*/
UINT8C SetupGetHubDescr[] = {HUB_GET_HUB_DESCRIPTOR, HUB_GET_DESCRIPTOR, 0x00, USB_DESCR_TYP_HUB, 0x00, 0x00, sizeof(USB_HUB_DESCR), 0x00};

/*Get HID device report descriptor*/
UINT8C SetupGetHIDDevReport[] = {0x81, USB_GET_DESCRIPTOR, 0x00, USB_DESCR_TYP_REPORT, 0x00, 0x00, 0x41, 0x00};

/*printer commands*/
UINT8C XPrinterReport[] = {0xA1, 0, 0x00, 0, 0x00, 0x00, 0xF1, 0x03};

UINT8X UsbDevEndp0Size;                       //* The maximum packet size of endpoint 0 of the USB device */
UINT8X RxBuffer[MAX_PACKET_SIZE] _at_ 0x0000; // IN, must even address
UINT8X TxBuffer[MAX_PACKET_SIZE] _at_ 0x0040; // OUT, must even address

UINT8 Set_Port = 0;

_RootHubDev xdata ThisUsbDev;                    // ROOT mouth
_DevOnHubPort xdata DevOnHubPort[HUB_MAX_PORTS]; // Assumption: no more than 1 external HUB, each external HUB does not exceed HUB_MAX_PORTS ports (don’t care if there are more)

bit RootHubId; // The currently operating root-hub port number: 0=HUB0, 1=HUB1
bit FoundNewDev;

main()
{
    UINT8 i, s, len, endp;
    UINT16 loc;
    CfgFsys();
    mDelaymS(5);  // Modify the main frequency and wait for the clock to stabilize
    mInitSTDIO(); // In order to let the computer monitor the demonstration process through the serial port
    printf("Start @ChipID=%02X\n", (UINT16)CHIP_ID);
    InitUSB_Host();
    FoundNewDev = 0;
    printf("Wait Device In\n");
    while (1)
    {
        s = ERR_SUCCESS;
        if (UIF_DETECT)
        {                         // Handle if there is a USB host detection interrupt
            UIF_DETECT = 0;       // clear interrupt flag
            s = AnalyzeRootHub(); // Analyze ROOT-HUB status
            if (s == ERR_USB_CONNECT)
                FoundNewDev = 1;
        }
        if (FoundNewDev)
        { // There is a new USB device plugged in
            FoundNewDev = 0;
            mDelaymS(200);           // Since the USB device has just been inserted and is not yet stable, wait for the USB device for hundreds of milliseconds to eliminate plugging and unplugging jitter
            s = EnumAllRootDevice(); // Enumerate USB devices of all ROOT-HUB ports
            if (s != ERR_SUCCESS)
            {
                printf("EnumAllRootDev err = %02X\n", (UINT16)s);
            }
        }
        if (RI == 0)
            continue;
        i = getkey();
        printf("%c", (UINT8)i);
        if (i == 'E')
        {                         // Every once in a while, usually 100mS to 1000mS, enumerate the ports of the external HUB, and do it when the microcontroller is free.
            s = EnumAllHubPort(); // Enumerate all secondary USB devices behind the external HUB under the ROOT-HUB port
            if (s != ERR_SUCCESS)
            { // Maybe the HUB is disconnected
                printf("EnumAllHubPort err = %02X\n", (UINT16)s);
            }
        }
        switch (i)
        {                                           // Simulate subjective requests to operate on a USB device
        case 'M':                                   // Use timing to simulate subjective needs, you need to operate the mouse
            loc = SearchTypeDevice(DEV_TYPE_MOUSE); // Search for the port number of the specified type of device on each port of ROOT-HUB and external HUB
            if (loc != 0xFFFF)
            { // Found it, how to deal with two MOUSE?
                printf("Query Mouse @%04X\n", loc);
                i = (UINT8)(loc >> 8);
                len = (UINT8)loc;
                SelectHubPort(len);                                          // Select to operate the designated ROOT-HUB port, set the current USB speed and the USB address of the operated device
                endp = len ? DevOnHubPort[len - 1].GpVar : ThisUsbDev.GpVar; // The address of the interrupt endpoint, bit 7 is used for the synchronization flag bit
                if (endp & USB_ENDP_ADDR_MASK)
                {                                                                                                   // endpoint valid
                    s = USBHostTransact(USB_PID_IN << 4 | endp & 0x7F, endp & 0x80 ? bUH_R_TOG | bUH_T_TOG : 0, 0); // CH554 transmit transaction, get data, NAK does not retry
                    if (s == ERR_SUCCESS)
                    {
                        endp ^= 0x80; // sync flag flip
                        if (len)
                            DevOnHubPort[len - 1].GpVar = endp; // save synchronization flag
                        else
                            ThisUsbDev.GpVar = endp;
                        len = USB_RX_LEN; // Received data length
                        if (len)
                        {
                            printf("Mouse data: ");
                            for (i = 0; i < len; i++)
                            {
                                printf("x%02X ", (UINT16)(RxBuffer[i]));
                            }
                            printf("\n");
                        }
                    }
                    else if (s != (USB_PID_NAK | ERR_USB_TRANSFER))
                    {
                        printf("Mouse error %02x\n", (UINT16)s); // may be disconnected
                    }
                }
                else
                {
                    printf("Mouse no interrupt endpoint\n");
                }
                SetUsbSpeed(1); // Default is full speed
            }
            break;
        case 'K':                                      // Use timing to simulate subjective needs, need to operate the keyboard
            loc = SearchTypeDevice(DEV_TYPE_KEYBOARD); // Search for the port number of the specified type of device on each port of ROOT-HUB and external HUB
            if (loc != 0xFFFF)
            { // Found it, how to deal with it if there are two KeyBoards?
                printf("Query Keyboard @%04X\n", loc);
                i = (UINT8)(loc >> 8);
                len = (UINT8)loc;
                SelectHubPort(len);                                          // Select to operate the designated ROOT-HUB port, set the current USB speed and the USB address of the operated device
                endp = len ? DevOnHubPort[len - 1].GpVar : ThisUsbDev.GpVar; // The address of the interrupt endpoint, bit 7 is used for the synchronization flag bit
                printf("%02X  ", endp);
                if (endp & USB_ENDP_ADDR_MASK)
                {                                                                                                   // endpoint valid
                    s = USBHostTransact(USB_PID_IN << 4 | endp & 0x7F, endp & 0x80 ? bUH_R_TOG | bUH_T_TOG : 0, 0); // CH554 transmit transaction, get data, NAK does not retry
                    if (s == ERR_SUCCESS)
                    {
                        endp ^= 0x80; // sync flag flip
                        if (len)
                            DevOnHubPort[len - 1].GpVar = endp; // save synchronization flag
                        else
                            ThisUsbDev.GpVar = endp;
                        len = USB_RX_LEN; // Received data length
                        if (len)
                        {
                            SETorOFFNumLock(RxBuffer);
                            printf("keyboard data: ");
                            for (i = 0; i < len; i++)
                            {
                                printf("x%02X ", (UINT16)(RxBuffer[i]));
                            }
                            printf("\n");
                        }
                    }
                    else if (s != (USB_PID_NAK | ERR_USB_TRANSFER))
                    {
                        printf("keyboard error %02x\n", (UINT16)s); // may be disconnected
                    }
                }
                else
                {
                    printf("keyboard no interrupt endpoint\n");
                }
                SetUsbSpeed(1); // Default is full speed
            }
            break;
        case 'H':                                      // Operation HUB
            loc = SearchTypeDevice(DEV_TYPE_KEYBOARD); // Search for the port number of the specified type of device on each port of ROOT-HUB and external HUB
            if (loc != 0xFFFF)
            { // Found it, how to deal with it if there are two KeyBoards?
                printf("Query Keyboard @%04X\n", loc);
                i = (UINT8)(loc >> 8);
                len = (UINT8)loc;
                SelectHubPort(len);                                          // Select to operate the designated ROOT-HUB port, set the current USB speed and the USB address of the operated device
                endp = len ? DevOnHubPort[len - 1].GpVar : ThisUsbDev.GpVar; // The address of the interrupt endpoint, bit 7 is used for the synchronization flag bit
                printf("%02X  ", endp);
                if (endp & USB_ENDP_ADDR_MASK)
                {                                                                                                   // endpoint valid
                    s = USBHostTransact(USB_PID_IN << 4 | endp & 0x7F, endp & 0x80 ? bUH_R_TOG | bUH_T_TOG : 0, 0); // CH554 transmit transaction, get data, NAK does not retry
                    if (s == ERR_SUCCESS)
                    {
                        endp ^= 0x80; // sync flag flip
                        if (len)
                            DevOnHubPort[len - 1].GpVar = endp; // save synchronization flag
                        else
                            ThisUsbDev.GpVar = endp;
                        len = USB_RX_LEN; // Received data length
                        if (len)
                        {
                            // SETorOFFNumLock(RxBuffer);
                            printf("keyboard data: ");
                            for (i = 0; i < len; i++)
                            {
                                printf("x%02X ", (UINT16)(RxBuffer[i]));
                            }
                            printf("\n");
                        }
                    }
                    else if (s != (USB_PID_NAK | ERR_USB_TRANSFER))
                    {
                        printf("keyboard error %02x\n", (UINT16)s); // may be disconnected
                    }
                }
                else
                    printf("keyboard no interrupt endpoint\n");
            }
            for (i = 0; i < 2; i++)
            {
                if ((ThisUsbDev.DeviceStatus == ROOT_DEV_SUCCESS) && (ThisUsbDev.DeviceType == USB_DEV_CLASS_HUB))
                {
                    SelectHubPort(0);         // Select to operate the designated ROOT-HUB port, set the current USB speed and the USB address of the operated device
                    endp = ThisUsbDev.GpVar1; // The address of the interrupt endpoint, bit 7 is used for the synchronization flag bit
                    if (endp & USB_ENDP_ADDR_MASK)
                    {                                                                                                   // endpoint valid
                        s = USBHostTransact(USB_PID_IN << 4 | endp & 0x7F, endp & 0x80 ? bUH_R_TOG | bUH_T_TOG : 0, 0); // CH554 transmit transaction, get data, NAK does not retry
                        if (s == ERR_SUCCESS)
                        {
                            endp ^= 0x80;             // sync flag flip
                            ThisUsbDev.GpVar1 = endp; // save synchronization flag
                            len = USB_RX_LEN;         // Received data length
                            if (len)
                            {
                                EnumHubPort();
                                for (i = 0; i < len; i++)
                                {
                                    printf("x%02X ", (UINT16)(RxBuffer[i]));
                                }
                                printf("\n");
                            }
                        }
                        else if (s != (USB_PID_NAK | ERR_USB_TRANSFER))
                        {
                            printf("HUB error %02x\n", (UINT16)s); // may be disconnected
                        }
                    }
                    else
                        printf("HUB %02d no interrupt endpoint\n", i);
                }
                else
                    printf("ROOTHUB %02d not HUB\n", i);
            }
            break;
        case 'P': // Operate a USB printer
            if (TIN0 == 0)
            { // P10to low to start printing
                memset(TxBuffer, 0, sizeof(TxBuffer));
                TxBuffer[0] = 0x1B;
                TxBuffer[1] = 0x40;
                TxBuffer[2] = 0x1D;
                TxBuffer[3] = 0x55;
                TxBuffer[4] = 0x42;
                TxBuffer[5] = 0x02;
                TxBuffer[6] = 0x18;
                TxBuffer[7] = 0x1D;
                TxBuffer[8] = 0x76;
                TxBuffer[9] = 0x30;
                TxBuffer[10] = 0x00;
                TxBuffer[11] = 0x30;
                TxBuffer[12] = 0x00;
                TxBuffer[13] = 0x18;
                loc = SearchTypeDevice(USB_DEV_CLASS_PRINTER); // Search for the port number of the specified type of device on each port of ROOT-HUB and external HUB
                if (loc != 0xFFFF)
                { // Found it, what to do if there are two printers?
                    printf("Query Printer @%04X\n", loc);
                    i = (UINT8)(loc >> 8);
                    len = (UINT8)loc;
                    SelectHubPort(len);                                          // Select to operate the designated ROOT-HUB port, set the current USB speed and the USB address of the operated device
                    endp = len ? DevOnHubPort[len - 1].GpVar : ThisUsbDev.GpVar; // The address of the endpoint, bit 7 is used for the synchronization flag bit
                    printf("%02X  ", endp);
                    if (endp & USB_ENDP_ADDR_MASK)
                    {                                                                                                         // endpoint valid
                        UH_TX_LEN = 64;                                                                                       // The default state of no data is IN
                        s = USBHostTransact(USB_PID_OUT << 4 | endp & 0x7F, endp & 0x80 ? bUH_R_TOG | bUH_T_TOG : 0, 0xffff); // CH554 transmit transaction, get data, NAK retry
                        if (s == ERR_SUCCESS)
                        {
                            endp ^= 0x80; // sync flag flip
                            memset(TxBuffer, 0, sizeof(TxBuffer));
                            UH_TX_LEN = 64;                                                                                       // The default state of no data is IN
                            s = USBHostTransact(USB_PID_OUT << 4 | endp & 0x7F, endp & 0x80 ? bUH_R_TOG | bUH_T_TOG : 0, 0xffff); // CH554 transmit transaction, get data, NAK retry
                        }
                        else if (s != (USB_PID_NAK | ERR_USB_TRANSFER))
                            printf("Printer error %02x\n", (UINT16)s); // may be disconnected
                    }
                }
            }
            break;
        default:
            break;
        }
    }
}
