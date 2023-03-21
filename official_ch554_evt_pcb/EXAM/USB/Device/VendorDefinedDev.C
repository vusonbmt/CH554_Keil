
/********************************** (C) COPYRIGHT *******************************
* File Name          : VendorDefinedDev.C
* Author             : WCH
* Version            : V1.0
* Date               : 2017/01/20
* Description        : CH554 simulates USB Module (CH554), which is a manufacturer-defined interface device. Drivers need to be installed. Searching for CH37XDRV or installing ISPTool will automatically install the driver for this device class. In addition to controlling transmission, this device class is also directly plugged into the endpoint 2 for batch uploading and uploading and endpoints 1 Interrupt the upload, you can obtain other USB debugging tools through 372DEBUG.EXE to demonstrate sending and receiving data

*******************************************************************************/

#include "./Public/CH554.H"
#include "./Public/Debug.H"
#include <stdio.h>
#include <string.h>

#define THIS_ENDP0_SIZE DEFAULT_ENDP0_SIZE

// device descriptor
UINT8C MyDevDescr[] = {0x12, 0x01, 0x10, 0x01,
					   0xFF, 0x80, 0x55, THIS_ENDP0_SIZE,
					   0x48, 0x43, 0x37, 0x55, // Vendor ID and Product ID
					   0x00, 0x01, 0x01, 0x02,
					   0x00, 0x01};
// configuration descriptor
UINT8C MyCfgDescr[] = {0x09, 0x02, 0x27, 0x00, 0x01, 0x01, 0x00, 0x80, 0x32,
					   0x09, 0x04, 0x00, 0x00, 0x03, 0xFF, 0x80, 0x55, 0x00,
					   0x07, 0x05, 0x82, 0x02, 0x40, 0x00, 0x00,
					   0x07, 0x05, 0x02, 0x02, 0x40, 0x00, 0x00,
					   0x07, 0x05, 0x81, 0x03, 0x40, 0x00, 0x00};
// language descriptor
UINT8C MyLangDescr[] = {0x04, 0x03, 0x09, 0x04};
// Manufacturer information
UINT8C MyManuInfo[] = {0x0E, 0x03, 'w', 0, 'c', 0, 'h', 0, '.', 0, 'c', 0, 'n', 0};
// product information
UINT8C MyProdInfo[] = {0x0C, 0x03, 'C', 0, 'H', 0, '5', 0, '5', 0, '4', 0};

UINT8 UsbConfig = 0; // USB Configuration Flags

UINT8X Ep0Buffer[THIS_ENDP0_SIZE + 2 >= MAX_PACKET_SIZE ? MAX_PACKET_SIZE : THIS_ENDP0_SIZE + 2]; // OUT&IN
UINT8X Ep1Buffer[MAX_PACKET_SIZE];																  // IN
UINT8X Ep2Buffer[2 * MAX_PACKET_SIZE];															  // OUT+IN

#define UsbSetupBuf ((PUSB_SETUP_REQ)Ep0Buffer)

#pragma NOAREGS

void USB_DeviceInterrupt(void) interrupt INT_NO_USB using 1 /* USB interrupt service routine, using register set 1 */
{
	UINT8 i, len;
	static UINT8 SetupReqCode, SetupLen;
	static PUINT8 pDescr;
	if (UIF_TRANSFER)
	{ // USB transfer completed
		if (U_IS_NAK)
		{ // not enable for this example
		  //			switch ( USB_INT_ST & ( MASK_UIS_TOKEN | MASK_UIS_ENDP ) ) {  // Analyzing Action Tokens and Endpoint Numbers
		  //				case UIS_TOKEN_OUT | 2:                                     // endpoint 2# Bulk endpoint download
		  //					break;
		  //				case UIS_TOKEN_IN | 2:                                      // endpoint 2# Bulk endpoint upload
		  //					break;
		  //				case UIS_TOKEN_IN | 1:                                      // endpoint 1# interrupt endpoint upload
		  //					break;
		  //				default:
		  //					break;
		  //			}
		}
		else
		{
			switch (USB_INT_ST & (MASK_UIS_TOKEN | MASK_UIS_ENDP))
			{						// Analyzing Action Tokens and Endpoint Numbers
			case UIS_TOKEN_OUT | 2: // endpoint 2# Bulk endpoint download
				if (U_TOG_OK)
				{ // Out-of-sync packets will be dropped
					//						UEP2_CTRL ^= bUEP_R_TOG;                               // auto flipped
					len = USB_RX_LEN;
					for (i = 0; i < len; i++)
					{
						Ep2Buffer[MAX_PACKET_SIZE + i] = Ep2Buffer[i] ^ 0xFF; // OUT data is reversed to IN and verified by computer
					}
					UEP2_T_LEN = len;
					UEP2_CTRL = UEP2_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_ACK; // allow upload
				}
				break;
			case UIS_TOKEN_IN | 2:										 // endpoint 2# Bulk endpoint upload
																		 //					UEP2_CTRL ^= bUEP_T_TOG;                                 // auto flipped
				UEP2_CTRL = UEP2_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_NAK; // pause upload
				break;
			case UIS_TOKEN_IN | 1:										 // endpoint 1# interrupt endpoint upload
																		 //					UEP1_CTRL ^= bUEP_T_TOG;                                 // auto flipped
				UEP1_CTRL = UEP1_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_NAK; // pause upload
				break;
			case UIS_TOKEN_SETUP | 0: // endpoint 0# SETUP
				len = USB_RX_LEN;
				if (len == sizeof(USB_SETUP_REQ))
				{ // SETUP包长度
					SetupLen = UsbSetupBuf->wLengthL;
					if (UsbSetupBuf->wLengthH || SetupLen > 0x7F)
						SetupLen = 0x7F; // limit the total length
					len = 0;			 // Default is success and upload 0 length
					SetupReqCode = UsbSetupBuf->bRequest;
					if ((UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK) != USB_REQ_TYP_STANDARD)
					{ /* non-standard request */
						len = 0xFF;
					}
					else
					{ // standard request
						switch (SetupReqCode)
						{ // request code
						case USB_GET_DESCRIPTOR:
							switch (UsbSetupBuf->wValueH)
							{
							case 1: // device descriptor
								pDescr = (PUINT8)(&MyDevDescr[0]);
								len = sizeof(MyDevDescr);
								break;
							case 2: // configuration descriptor
								pDescr = (PUINT8)(&MyCfgDescr[0]);
								len = sizeof(MyCfgDescr);
								break;
							case 3: // string descriptor
								switch (UsbSetupBuf->wValueL)
								{
								case 1:
									pDescr = (PUINT8)(&MyManuInfo[0]);
									len = sizeof(MyManuInfo);
									break;
								case 2:
									pDescr = (PUINT8)(&MyProdInfo[0]);
									len = sizeof(MyProdInfo);
									break;
								case 0:
									pDescr = (PUINT8)(&MyLangDescr[0]);
									len = sizeof(MyLangDescr);
									break;
								default:
									len = 0xFF; // unsupported string descriptor
									break;
								}
								break;
							default:
								len = 0xFF; // Unsupported descriptor type
								break;
							}
							if (SetupLen > len)
								SetupLen = len;												// limit the total length
							len = SetupLen >= THIS_ENDP0_SIZE ? THIS_ENDP0_SIZE : SetupLen; // The length of this transfer
							memcpy(Ep0Buffer, pDescr, len);									/* load upload data */
							SetupLen -= len;
							pDescr += len;
							break;
						case USB_SET_ADDRESS:
							SetupLen = UsbSetupBuf->wValueL; // Temporary USB device address
							break;
						case USB_GET_CONFIGURATION:
							Ep0Buffer[0] = UsbConfig;
							if (SetupLen >= 1)
								len = 1;
							break;
						case USB_SET_CONFIGURATION:
							UsbConfig = UsbSetupBuf->wValueL;
							break;
						case USB_CLEAR_FEATURE:
							if ((UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP)
							{ // endpoint
								switch (UsbSetupBuf->wIndexL)
								{
								case 0x82:
									UEP2_CTRL = UEP2_CTRL & ~(bUEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK;
									break;
								case 0x02:
									UEP2_CTRL = UEP2_CTRL & ~(bUEP_R_TOG | MASK_UEP_R_RES) | UEP_R_RES_ACK;
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
								len = 0xFF; // not endpoint does not support
							break;
						case USB_GET_INTERFACE:
							Ep0Buffer[0] = 0x00;
							if (SetupLen >= 1)
								len = 1;
							break;
						case USB_GET_STATUS:
							Ep0Buffer[0] = 0x00;
							Ep0Buffer[1] = 0x00;
							if (SetupLen >= 2)
								len = 2;
							else
								len = SetupLen;
							break;
						default:
							len = 0xFF; // operation failed
#ifdef DE_PRINTF
							printf("ErrEp0ReqCode=%02X\n", (UINT16)SetupReqCode);
#endif
							break;
						}
					}
				}
				else
				{
					len = 0xFF; // SETUP packet length error
#ifdef DE_PRINTF
					printf("ErrEp0ReqSize\n");
#endif
				}
				if (len == 0xFF)
				{ // operation failed
					SetupReqCode = 0xFF;
					UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL; // STALL
				}
				else if (len <= THIS_ENDP0_SIZE)
				{ // Upload data or return a 0-length packet in the status stage
					UEP0_T_LEN = len;
					UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK; // The default data packet is DATA1
				}
				else
				{																		 // Download data or other
					UEP0_T_LEN = 0;														 // Although it has not yet reached the state stage, the upload of 0-length data packets is preset in advance to prevent the host from entering the state stage in advance
					UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK; // The default data packet is DATA1
				}
				break;
			case UIS_TOKEN_IN | 0: // endpoint 0# IN
				switch (SetupReqCode)
				{
				case USB_GET_DESCRIPTOR:
					len = SetupLen >= THIS_ENDP0_SIZE ? THIS_ENDP0_SIZE : SetupLen; // The length of this transfer
					memcpy(Ep0Buffer, pDescr, len);									/* load upload data */
					SetupLen -= len;
					pDescr += len;
					UEP0_T_LEN = len;
					UEP0_CTRL ^= bUEP_T_TOG; // turn over
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
			case UIS_TOKEN_OUT | 0: // endpoint 0# OUT
				switch (SetupReqCode)
				{
					//						case download:
					//							if ( U_TOG_OK ) {                                  // Out-of-sync packets will be dropped
					//								UEP0_CTRL ^= bUEP_R_TOG;                         // turn over
					//								                                                 //Get download data;
					//								//UEP0_CTRL = UEP0_CTRL & bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK; // Preset to upload 0-length data packet DATA1 to prevent the host from entering the status stage ahead of time
					//							}
					//							break;
				case USB_GET_DESCRIPTOR:
				default:
					if (U_TOG_OK)
					{ // Out-of-sync packets will be dropped
					  //								if ( USB_RX_LEN ) control_status_error;
					  //								else control_ok;                                 // Receiving a 0-length packet indicates that the control read operation/upload is OK
					}
					//							else control_status_error;
					UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK; // Prepare for next control transfer
					break;
				}
				break;
			default:
				break;
			}
		}
		UIF_TRANSFER = 0; // clear interrupt flag
	}
	else if (UIF_BUS_RST)
	{ // USB总线复位
		UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
		UEP1_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK;
		UEP2_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK;
		USB_DEV_AD = 0x00;
		UIF_SUSPEND = 0;
		UIF_TRANSFER = 0;
		UIF_BUS_RST = 0; // clear interrupt flag
	}
	else if (UIF_SUSPEND)
	{ // USB bus suspend/wake complete
		UIF_SUSPEND = 0;
		if (USB_MIS_ST & bUMS_SUSPEND)
		{ // hang up
#ifdef DE_PRINTF
			printf("zz"); // sleep state
#endif
			while (XBUS_AUX & bUART0_TX)
				; // Wait for the send to complete
			SAFE_MOD = 0x55;
			SAFE_MOD = 0xAA;
			WAKE_CTRL = bWAK_BY_USB | bWAK_RXD0_LO; // Can be woken up when USB or RXD0 has a signal
			PCON |= PD;								// to sleep
			SAFE_MOD = 0x55;
			SAFE_MOD = 0xAA;
			WAKE_CTRL = 0x00;
		}
		else
		{ // wake
		}
	}
	else
	{ // unexpected interruption, impossible situation

		USB_INT_FG = 0xFF; // clear interrupt flag
	}
}

/*******************************************************************************
 * Function Name  : InitUSB_Device()
 * Description    : USB device mode configuration, device mode startup, transceiver endpoint configuration, interrupt enable
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void InitUSB_Device(void) // Initialize the USB device
{
	IE_USB = 0;
	USB_CTRL = 0x00;						// set mode first
	UEP4_1_MOD = bUEP1_TX_EN;				// Endpoint 1 upload IN
	UEP2_3_MOD = bUEP2_RX_EN | bUEP2_TX_EN; // Endpoint 2 download OUT and upload IN
	UEP0_DMA = Ep0Buffer;
	UEP1_DMA = Ep1Buffer;
	UEP2_DMA = Ep2Buffer;
	UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
	UEP1_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK;
	UEP2_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK;
	USB_DEV_AD = 0x00;
	UDEV_CTRL = bUD_PD_DIS;								  // Disable DP/DM pull-down resistor
	USB_CTRL = bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN; // Start the USB device and DMA, and return to NAK automatically before the interrupt flag is cleared during the interrupt
	UDEV_CTRL |= bUD_PORT_EN;							  // Allow USB port
	USB_INT_FG = 0xFF;									  // clear interrupt flag
	USB_INT_EN = bUIE_SUSPEND | bUIE_TRANSFER | bUIE_BUS_RST;
	IE_USB = 1;
}

main()
{
	UINT8 i;
	CfgFsys();
	mDelaymS(5);  // Modify the main frequency, and wait for the main frequency to stabilize with a slight delay
	mInitSTDIO(); /* Initialize serial port 0 in order to allow the computer to monitor the demonstration process through the serial port */
#ifdef DE_PRINTF
	printf("Start @ChipID=%02X\n", (UINT16)CHIP_ID);
#endif
	InitUSB_Device();
	EA = 1;
	while (1)
	{
		i = getkey();
		printf("%c", (UINT8)i);
		if (i >= '0' && i <= 'z')
		{
			memcpy(Ep1Buffer, (PUINT8C)(i - '0'), MAX_PACKET_SIZE); /* load upload data */
			UEP1_T_LEN = i - '0' > 8 ? 8 : i - '0';
			UEP1_CTRL = UEP1_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_ACK;
		}
	}
}
