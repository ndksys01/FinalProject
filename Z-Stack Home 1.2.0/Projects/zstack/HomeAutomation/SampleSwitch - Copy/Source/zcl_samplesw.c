//git main_code branch
/**************************************************************************************************
Filename:       zcl_samplesw.c
Revised:        $Date: 2013-08-02 16:56:53 -0700 (Fri, 02 Aug 2013) $
Revision:       $Revision: 34876 $

Description:    Zigbee Cluster Library - sample device application.


Copyright 2006-2013 Texas Instruments Incorporated. All rights reserved.

IMPORTANT: Your use of this Software is limited to those specific rights
granted under the terms of a software license agreement between the user
who downloaded the software, his/her employer (which must be your employer)
and Texas Instruments Incorporated (the "License").  You may not use this
Software unless you agree to abide by the terms of the License. The License
limits your use, and you acknowledge, that the Software may not be modified,
copied or distributed unless embedded on a Texas Instruments microcontroller
or used solely and exclusively in conjunction with a Texas Instruments radio
frequency transceiver, which is integrated into your product.  Other than for
the foregoing purpose, you may not use, reproduce, copy, prepare derivative
works of, modify, distribute, perform, display or sell this Software and/or
its documentation for any purpose.

YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

Should you have any questions regarding your right to use this Software,
contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
This device will be like an On/Off Switch device. This application
is not intended to be a On/Off Switch device, but will use the device
description to implement this sample code.

----------------------------------------
Main:
- SW1: Toggle remote light
- SW2: Invoke EZMode
- SW4: Enable/Disable Permit Join
- SW5: Go to Help screen
----------------------------------------
*********************************************************************/

/*********************************************************************
* INCLUDES
*/
#include "ZComDef.h"
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"
#include "MT_SYS.h"
#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ha.h"
#include "zcl_samplesw.h"
#include "zcl_ezmode.h"

#include "onboard.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "bsp.h"//khai bao them
#include "acc_bma250.h"//khai bao them
#include "als_sfh5711.h"
#include "adc.h"
#include "hw_cctest.h"
#include "hw_rfcore_xreg.h"

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "hw_memmap.h"
#include "gpio.h"
#include "hw_ioc.h"
#include "ioc.h"
#include "interrupt.h"
#include "sys_ctrl.h"
#include "hw_sys_ctrl.h"
#include "systick.h"
#include "bsp_led.h"
/*********************************************************************
* MACROS
*/

/*********************************************************************
* CONSTANTS
*/
/*********************************************************************
* TYPEDEFS
*/

/*********************************************************************
* GLOBAL VARIABLES
*/
byte zclSampleSw_TaskID;

uint8 zclSampleSwSeqNum;

uint8 zclSampleSw_OnOffSwitchType = ON_OFF_SWITCH_TYPE_TOGGLE;

uint8 zclSampleSw_OnOffSwitchActions = ON_OFF_SWITCH_ACTIONS_2;   // Toggle -> Toggle

/*********************************************************************
* GLOBAL FUNCTIONS
*/

/*********************************************************************
* LOCAL VARIABLES
*/
#ifdef ZCL_ON_OFF
afAddrType_t zclSampleSw_DstAddr;
#endif

#ifdef ZCL_EZMODE
static void zclSampleSw_ProcessZDOMsgs( zdoIncomingMsg_t *pMsg );
static void zclSampleSw_EZModeCB( zlcEZMode_State_t state, zclEZMode_CBData_t *pData );

static const zclEZMode_RegisterData_t zclSampleSw_RegisterEZModeData =
{
  &zclSampleSw_TaskID,
  SAMPLESW_EZMODE_NEXTSTATE_EVT,
  SAMPLESW_EZMODE_TIMEOUT_EVT,
  &zclSampleSwSeqNum,
  zclSampleSw_EZModeCB
};

// NOT ZLC_EZMODE, Use EndDeviceBind
#else

static cId_t bindingOutClusters[] =
{
  ZCL_CLUSTER_ID_GEN_ON_OFF
};
#define ZCLSAMPLESW_BINDINGLIST   (sizeof(bindingOutClusters)/sizeof(bindingOutClusters[0]))
#endif  // ZLC_EZMODE

// Endpoint to allow SYS_APP_MSGs
static endPointDesc_t sampleSw_TestEp =
{
  SAMPLESW_ENDPOINT,                  // endpoint
  &zclSampleSw_TaskID,
  (SimpleDescriptionFormat_t *)NULL,  // No Simple description for this test endpoint
  (afNetworkLatencyReq_t)0            // No Network Latency req
};

uint8 giSwScreenMode = SW_MAINMODE;   // display the main screen mode first

static uint8 aProcessCmd[] = { 1, 0, 0, 0 }; // used for reset command, { length + cmd0 + cmd1 + data }

uint8 gPermitDuration = 0;    // permit joining default to disabled

devStates_t zclSampleSw_NwkState = DEV_INIT;


/*********************************************************************
* LOCAL FUNCTIONS
*/
static void zclSampleSw_HandleKeys( byte shift, byte keys );
static void zclSampleSw_BasicResetCB( void );
static void zclSampleSw_IdentifyCB( zclIdentify_t *pCmd );
static void zclSampleSw_IdentifyQueryRspCB(  zclIdentifyQueryRsp_t *pRsp );
static void zclSampleSw_ProcessIdentifyTimeChange( void );

// app display functions
void zclSampleSw_LcdDisplayUpdate(void);
void zclSampleSw_LcdDisplayMainMode(void);
void zclSampleSw_LcdDisplayHelpMode(void);

// Functions to process ZCL Foundation incoming Command/Response messages
static void zclSampleSw_ProcessIncomingMsg( zclIncomingMsg_t *msg );
#ifdef ZCL_READ
static uint8 zclSampleSw_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg );
#endif
#ifdef ZCL_WRITE
static uint8 zclSampleSw_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg );
#endif
static uint8 zclSampleSw_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg );
#ifdef ZCL_DISCOVER
static uint8 zclSampleSw_ProcessInDiscCmdsRspCmd( zclIncomingMsg_t *pInMsg );
static uint8 zclSampleSw_ProcessInDiscAttrsRspCmd( zclIncomingMsg_t *pInMsg );
static uint8 zclSampleSw_ProcessInDiscAttrsExtRspCmd( zclIncomingMsg_t *pInMsg );
#endif

/*********************************************************************
* ZCL General Profile Callback table
*/
static zclGeneral_AppCallbacks_t zclSampleSw_CmdCallbacks =
{
  zclSampleSw_BasicResetCB,               // Basic Cluster Reset command
  zclSampleSw_IdentifyCB,                 // Identify command
#ifdef ZCL_EZMODE
  NULL,                                   // Identify EZ-Mode Invoke command
  NULL,                                   // Identify Update Commission State command
#endif
  NULL,                                   // Identify Trigger Effect command
  zclSampleSw_IdentifyQueryRspCB,         // Identify Query Response command
  NULL,                                   // On/Off cluster commands
  NULL,                                   // On/Off cluster enhanced command Off with Effect
  NULL,                                   // On/Off cluster enhanced command On with Recall Global Scene
  NULL,                                   // On/Off cluster enhanced command On with Timed Off
#ifdef ZCL_LEVEL_CTRL
  NULL,                                   // Level Control Move to Level command
  NULL,                                   // Level Control Move command
  NULL,                                   // Level Control Step command
  NULL,                                   // Level Control Stop command
#endif
#ifdef ZCL_GROUPS
  NULL,                                   // Group Response commands
#endif
#ifdef ZCL_SCENES
  NULL,                                   // Scene Store Request command
  NULL,                                   // Scene Recall Request command
  NULL,                                   // Scene Response command
#endif
#ifdef ZCL_ALARMS
  NULL,                                   // Alarm (Response) commands
#endif
#ifdef SE_UK_EXT
  NULL,                                   // Get Event Log command
  NULL,                                   // Publish Event Log command
#endif
  NULL,                                   // RSSI Location command
  NULL                                    // RSSI Location Response command
};


/*********************************************************************
* STATUS STRINGS
*/
#ifdef LCD_SUPPORTED
const char sDeviceName[]   = "  Sample Switch";
const char sClearLine[]    = " ";
const char sSwLight[]      = "SW1: ToggleLight";
const char sSwEZMode[]     = "SW2: EZ-Mode";
const char sSwHelp[]       = "SW5: Help";
const char sCmdSent[]      = "  COMMAND SENT";
#endif





//**************************Cac Khai bao duoc them vao
#define CONST1 0.58134*3.8 //(VREF / 2047) = (1190 / 2047), VREF from Datasheet
#define OFFSET_DATASHEET_25C 827 // 1422*, from Datasheet 
#define TEMP_COEFF (CONST1 * 4.2) // From Datasheet
#define OFFSET_0C (OFFSET_DATASHEET_25C - (25 * TEMP_COEFF))

endPointDesc_t zclSampleSw_epDesc;
void SendData(int CluID);
void ProcessData( afIncomingMSGPacket_t *pkt);
char dataTest[];
char data[];
byte zclSampleSw_TransID;//Them vao
static void sendAccelermeter();
uint8_t ui8RegVal;//cam bien gia toc
int16_t i16X, i16Y, i16Z;//cam bien gia toc
uint16_t ui16AlsValue = 0;//cam bien anh sang
static void initAccelerometer();
int iCount=1;
int jCount=1;
int flag =0;
static uint8 enableBaotrom=0;
static void sendLight();
uint16_t ui16Dummy;
double dOutputVoltage;
char pcTemp[10];
void readTemperature();
void SendDataFull(char *dataSend,int cluID, char *info);
void connect2ZC();
uint8 autoLight = 0;
static void initUart();
static void rxCB( uint8 port, uint8 event );
int ic=0;
char* tempData;
static uint8 bufTemp[8];
//**************************Ket thuc cac khai bao duoc them vao



// them ham nay de lay trang thai led
uint32_t
bspLedGet(uint8_t ui8Leds)
{
    //
    // Turn on specified LEDs
    //
    return GPIOPinRead(BSP_LED_BASE, ui8Leds);
}


/*********************************************************************
* @fn          zclSampleSw_Init
*
* @brief       Initialization function for the zclGeneral layer.
*
* @param       none
*
* @return      none
*/
void zclSampleSw_Init( byte task_id )
{
  zclSampleSw_TaskID = task_id;
  
  initAccelerometer();
  alsInit(); //khoi ta cam bien anh sang
  bspLedInit();
  initUart();
  
  osal_set_event(zclSampleSw_TaskID,TIME_SEND_ACCE_EVT);
  osal_set_event(zclSampleSw_TaskID,TIME_SEND_LIGH_EVT);
  
  zclSampleSw_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  zclSampleSw_DstAddr.endPoint = 0x01;       //end point 20 dia chi muon gui
  zclSampleSw_DstAddr.addr.shortAddr = 0xFFFF;
  
  zclSampleSw_epDesc.endPoint = 0x01;
  zclSampleSw_epDesc.task_id = &zclSampleSw_TaskID;
  zclSampleSw_epDesc.simpleDesc
    = (SimpleDescriptionFormat_t *)&zclSampleSw_SimpleDesc;
  zclSampleSw_epDesc.latencyReq = noLatencyReqs;
  
  // This app is part of the Home Automation Profile
  zclHA_Init( &zclSampleSw_SimpleDesc );
  
  // Register the ZCL General Cluster Library callback functions
  zclGeneral_RegisterCmdCallbacks( SAMPLESW_ENDPOINT, &zclSampleSw_CmdCallbacks );
  
  // Register the application's attribute list
  zcl_registerAttrList( SAMPLESW_ENDPOINT, SAMPLESW_MAX_ATTRIBUTES, zclSampleSw_Attrs );
  
  // Register the Application to receive the unprocessed Foundation command/response messages
  zcl_registerForMsg( zclSampleSw_TaskID );
  
#ifdef ZCL_EZMODE
  // Register EZ-Mode
  zcl_RegisterEZMode( &zclSampleSw_RegisterEZModeData );
  
  // Register with the ZDO to receive Match Descriptor Responses
  ZDO_RegisterForZDOMsg(task_id, Match_Desc_rsp);
#endif
  
  // Register for all key events - This app will handle all key events
  RegisterForKeys( zclSampleSw_TaskID );
  
  // Register for a test endpoint
  afRegister( &sampleSw_TestEp );
  
  ZDO_RegisterForZDOMsg( zclSampleSw_TaskID, End_Device_Bind_rsp );
  ZDO_RegisterForZDOMsg( zclSampleSw_TaskID, Match_Desc_rsp );
  
#ifdef LCD_SUPPORTED
  HalLcdWriteString ( (char *)sDeviceName, HAL_LCD_LINE_3 );
#endif
  //connect2ZC();
}

/*********************************************************************
* @fn          zclSample_event_loop
*
* @brief       Event Loop Processor for zclGeneral.
*
* @param       none
*
* @return      none
*/
uint16 zclSampleSw_event_loop( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  (void)task_id;  // Intentionally unreferenced parameter
  
  if ( events & SYS_EVENT_MSG )
  {
    while ( (MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( zclSampleSw_TaskID )) )
    {
      switch ( MSGpkt->hdr.event )
      {
#ifdef ZCL_EZMODE
      case ZDO_CB_MSG:
        zclSampleSw_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
        break;
#endif
      case ZCL_INCOMING_MSG:
        // Incoming ZCL Foundation command/response messages
        zclSampleSw_ProcessIncomingMsg( (zclIncomingMsg_t *)MSGpkt );
        break;
        
      case KEY_CHANGE:
        zclSampleSw_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
        break;
        
      case ZDO_STATE_CHANGE:
        zclSampleSw_NwkState = (devStates_t)(MSGpkt->hdr.status);
        
        // now on the network
        if ( (zclSampleSw_NwkState == DEV_ZB_COORD) ||
            (zclSampleSw_NwkState == DEV_ROUTER)   ||
              (zclSampleSw_NwkState == DEV_END_DEVICE) )
        {
#ifndef HOLD_AUTO_START
          giSwScreenMode = SW_MAINMODE;
          zclSampleSw_LcdDisplayUpdate();
#endif
#ifdef ZCL_EZMODE
          zcl_EZModeAction( EZMODE_ACTION_NETWORK_STARTED, NULL );
#endif
        }
        break;
        
      default:
        break;
      }
      
      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );
    }
    
    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  
  if ( events & SAMPLESW_IDENTIFY_TIMEOUT_EVT )
  {
    zclSampleSw_IdentifyTime = 10;
    zclSampleSw_ProcessIdentifyTimeChange();
    
    return ( events ^ SAMPLESW_IDENTIFY_TIMEOUT_EVT );
  }
  //Them vao su kien kiem tra gia toc ke
  if ( events & SAMPLESW_MAIN_SCREEN_EVT )
  {
    giSwScreenMode = SW_MAINMODE;
    
    zclSampleSw_LcdDisplayUpdate();
    //________________-send data ACCE
    if ( events & TIME_SEND_ACCE_EVT )
    {
      //readTemperature();
      if(enableBaotrom)
      {
        iCount--;jCount--;
        if(!iCount)
        {
          sendAccelermeter();
          iCount=1;
        }
      }
      if(autoLight)
      {
        ui16AlsValue = alsRead();
        if(ui16AlsValue>400)
        {
          bspLedClear(BSP_LED_1);
        }
        else
        {
          bspLedSet(BSP_LED_1);
        }
      }
      osal_start_timerEx(task_id,TIME_SEND_ACCE_EVT, 100);
      return (events ^ TIME_SEND_ACCE_EVT);  
    }
    
#ifdef LCD_SUPPORTED
    HalLcdWriteString( (char *)sClearLine, HAL_LCD_LINE_2 );
#endif
    return ( events ^ SAMPLESW_MAIN_SCREEN_EVT );
  }
  
#ifdef ZCL_EZMODE
  if ( events & SAMPLESW_EZMODE_NEXTSTATE_EVT )
  {
    zcl_EZModeAction ( EZMODE_ACTION_PROCESS, NULL );   // going on to next state
    return ( events ^ SAMPLESW_EZMODE_NEXTSTATE_EVT );
  }
  
  if ( events & SAMPLESW_EZMODE_TIMEOUT_EVT )
  {
    zcl_EZModeAction ( EZMODE_ACTION_TIMED_OUT, NULL ); // EZ-Mode timed out
    return ( events ^ SAMPLESW_EZMODE_TIMEOUT_EVT );
  }
  
  
#endif // ZLC_EZMODE
  
  // Discard unknown events
  return 0;
}

/*********************************************************************
* @fn      zclSampleSw_HandleKeys
*
* @brief   Handles all key events for this device.
*
* @param   shift - true if in shift/alt.
* @param   keys - bit field for key events. Valid entries:
*                 HAL_KEY_SW_5
*                 HAL_KEY_SW_4
*                 HAL_KEY_SW_2
*                 HAL_KEY_SW_1
*
* @return  none
*/
static void zclSampleSw_HandleKeys( byte shift, byte keys )
{
  // toggle remote light
  if ( keys & HAL_KEY_SW_1 )
  {
    giSwScreenMode = SW_MAINMODE;   // remove help screen if there
    
    // Using this as the "Light Switch"
#ifdef ZCL_ON_OFF
    zclGeneral_SendOnOff_CmdToggle( SAMPLESW_ENDPOINT, &zclSampleSw_DstAddr, FALSE, 0 );
    //HalLcdWriteString( "Hung", HAL_LCD_LINE_6 );
#endif
#ifdef LCD_SUPPORTED
    HalLcdWriteString( (char *)sCmdSent, HAL_LCD_LINE_2 );
    
    // clear message on screen after 3 seconds
    osal_start_timerEx( zclSampleSw_TaskID, SAMPLESW_MAIN_SCREEN_EVT, 3000 );
#endif
  }
  
  // invoke EZ-Mode
  if ( keys & HAL_KEY_SW_2 )
  {
    giSwScreenMode = SW_MAINMODE;   // remove help screen if there
    
#ifdef ZCL_EZMODE
    {
      zclEZMode_InvokeData_t ezModeData;
      static uint16 clusterIDs[] = { ZCL_CLUSTER_ID_GEN_ON_OFF };   // only bind on the on/off cluster
      
      // Invoke EZ-Mode
      ezModeData.endpoint = SAMPLESW_ENDPOINT; // endpoint on which to invoke EZ-Mode
      if ( (zclSampleSw_NwkState == DEV_ZB_COORD) ||
          (zclSampleSw_NwkState == DEV_ROUTER)   ||
            (zclSampleSw_NwkState == DEV_END_DEVICE) )
      {
        ezModeData.onNetwork = TRUE;      // node is already on the network
      }
      else
      {
        ezModeData.onNetwork = FALSE;     // node is not yet on the network
      }
      ezModeData.initiator = TRUE;        // OnOffSwitch is an initiator
      ezModeData.numActiveOutClusters = 1;   // active output cluster
      ezModeData.pActiveOutClusterIDs = clusterIDs;
      ezModeData.numActiveInClusters = 0;  // no active input clusters
      ezModeData.pActiveInClusterIDs = NULL;
      zcl_InvokeEZMode( &ezModeData );
      
#ifdef LCD_SUPPORTED
      HalLcdWriteString( "EZMode2222", HAL_LCD_LINE_2 );
#endif
    }
    
#else // NOT ZCL_EZMODE
    // bind to remote light
    zAddrType_t dstAddr;
    HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );
    
    // Initiate an End Device Bind Request, this bind request will
    // only use a cluster list that is important to binding.
    dstAddr.addrMode = afAddr16Bit;
    dstAddr.addr.shortAddr = 0;   // Coordinator makes the match
    ZDP_EndDeviceBindReq( &dstAddr, NLME_GetShortAddr(),
                         SAMPLESW_ENDPOINT,
                         ZCL_HA_PROFILE_ID,
                         0, NULL,   // No incoming clusters to bind
                         ZCLSAMPLESW_BINDINGLIST, bindingOutClusters,
                         TRUE );
    HalLcdWriteString( "EZMode1111111", HAL_LCD_LINE_6 );
#endif // ZCL_EZMODE
  }
  
  // toggle permit join
  if ( keys & HAL_KEY_SW_4 )
  {
    giSwScreenMode = SW_MAINMODE;   // remove help screen if there
    
    if ( ( zclSampleSw_NwkState == DEV_ZB_COORD ) ||
        ( zclSampleSw_NwkState == DEV_ROUTER ) )
    {
      // toggle permit join
      gPermitDuration = gPermitDuration ? 0 : 0xff;
      NLME_PermitJoiningRequest( gPermitDuration  );
    }
  }
  
  if ( shift && ( keys & HAL_KEY_SW_5 ) )
  {
    zclSampleSw_BasicResetCB();
  }
  else if ( keys & HAL_KEY_SW_5 )
  {
    giSwScreenMode = giSwScreenMode ? SW_MAINMODE : SW_HELPMODE;
#ifdef LCD_SUPPORTED
    HalLcdWriteString( (char *)sClearLine, HAL_LCD_LINE_2 );
#endif
  }
  
  // update the display
  zclSampleSw_LcdDisplayUpdate();
}

/*********************************************************************
* @fn      zclSampleSw_LcdDisplayUpdate
*
* @brief   Called to update the LCD display.
*
* @param   none
*
* @return  none
*/
void zclSampleSw_LcdDisplayUpdate(void)
{
  if ( giSwScreenMode == SW_HELPMODE )
  {
    zclSampleSw_LcdDisplayHelpMode();
  }
  else
  {
    zclSampleSw_LcdDisplayMainMode();
  }
}

/*********************************************************************
* @fn      zclSampleSw_LcdDisplayMainMode
*
* @brief   Called to display the main screen on the LCD.
*
* @param   none
*
* @return  none
*/
void zclSampleSw_LcdDisplayMainMode(void)
{
  if ( zclSampleSw_NwkState == DEV_ZB_COORD )
  {
    zclHA_LcdStatusLine1(0);
  }
  else if ( zclSampleSw_NwkState == DEV_ROUTER )
  {
    zclHA_LcdStatusLine1(1);
  }
  else if ( zclSampleSw_NwkState == DEV_END_DEVICE )
  {
    zclHA_LcdStatusLine1(2);
  }
  
#ifdef LCD_SUPPORTED
  if ( ( zclSampleSw_NwkState == DEV_ZB_COORD ) ||
      ( zclSampleSw_NwkState == DEV_ROUTER ) )
  {
    // display help key with permit join status
    if ( gPermitDuration )
    {
      HalLcdWriteString("SW5: Help      *", HAL_LCD_LINE_3);
    }
    else
    {
      HalLcdWriteString("SW5: Help       ", HAL_LCD_LINE_3);
    }
  }
  else
  {
    // display help key
    HalLcdWriteString((char *)sSwHelp, HAL_LCD_LINE_3);
  }
#endif
}

/*********************************************************************
* @fn      zclSampleSw_LcdDisplayHelpMode
*
* @brief   Called to display the SW options on the LCD.
*
* @param   none
*
* @return  none
*/
void zclSampleSw_LcdDisplayHelpMode(void)
{
#ifdef LCD_SUPPORTED
  HalLcdWriteString( (char *)sSwLight, HAL_LCD_LINE_1 );
  HalLcdWriteString( (char *)sSwEZMode, HAL_LCD_LINE_2 );
  HalLcdWriteString( (char *)sSwHelp, HAL_LCD_LINE_3 );
#endif
}

/*********************************************************************
* @fn      zclSampleSw_ProcessIdentifyTimeChange
*
* @brief   Called to process any change to the IdentifyTime attribute.
*
* @param   none
*
* @return  none
*/
static void zclSampleSw_ProcessIdentifyTimeChange( void )
{
  if ( zclSampleSw_IdentifyTime > 0 )
  {
    osal_start_timerEx( zclSampleSw_TaskID, SAMPLESW_IDENTIFY_TIMEOUT_EVT, 1000 );
    HalLedBlink ( HAL_LED_4, 0xFF, HAL_LED_DEFAULT_DUTY_CYCLE, HAL_LED_DEFAULT_FLASH_TIME );
  }
  else
  {
    if ( zclSampleSw_OnOff )
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_ON );
    else
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );
    osal_stop_timerEx( zclSampleSw_TaskID, SAMPLESW_IDENTIFY_TIMEOUT_EVT );
  }
}

/*********************************************************************
* @fn      zclSampleSw_BasicResetCB
*
* @brief   Callback from the ZCL General Cluster Library
*          to set all the Basic Cluster attributes to  default values.
*
* @param   none
*
* @return  none
*/
static void zclSampleSw_BasicResetCB( void )
{
  // Put device back to factory default settings
  zgWriteStartupOptions( ZG_STARTUP_SET, 3 );   // bit set both default configuration and default network
  
  // restart device
  MT_SysCommandProcessing( aProcessCmd );
}

/*********************************************************************
* @fn      zclSampleSw_IdentifyCB
*
* @brief   Callback from the ZCL General Cluster Library when
*          it received an Identity Command for this application.
*
* @param   srcAddr - source address and endpoint of the response message
* @param   identifyTime - the number of seconds to identify yourself
*
* @return  none
*/
static void zclSampleSw_IdentifyCB( zclIdentify_t *pCmd )
{
  zclSampleSw_IdentifyTime = pCmd->identifyTime;
  zclSampleSw_ProcessIdentifyTimeChange();
}

/*********************************************************************
* @fn      zclSampleSw_IdentifyQueryRspCB
*
* @brief   Callback from the ZCL General Cluster Library when
*          it received an Identity Query Response Command for this application.
*
* @param   srcAddr - source address
* @param   timeout - number of seconds to identify yourself (valid for query response)
*
* @return  none
*/
static void zclSampleSw_IdentifyQueryRspCB(  zclIdentifyQueryRsp_t *pRsp )
{
  (void)pRsp;
#ifdef ZCL_EZMODE
  {
    zclEZMode_ActionData_t data;
    data.pIdentifyQueryRsp = pRsp;
    zcl_EZModeAction ( EZMODE_ACTION_IDENTIFY_QUERY_RSP, &data );
  }
#endif
}

/******************************************************************************
*
*  Functions for processing ZCL Foundation incoming Command/Response messages
*
*****************************************************************************/

/*********************************************************************
* @fn      zclSampleSw_ProcessIncomingMsg
*
* @brief   Process ZCL Foundation incoming message
*
* @param   pInMsg - pointer to the received message
*
* @return  none
*/
static void zclSampleSw_ProcessIncomingMsg( zclIncomingMsg_t *pInMsg )
{
  switch ( pInMsg->zclHdr.commandID )
  {
#ifdef ZCL_READ
  case ZCL_CMD_READ_RSP:
    zclSampleSw_ProcessInReadRspCmd( pInMsg );
    break;
#endif
#ifdef ZCL_WRITE
  case ZCL_CMD_WRITE_RSP:
    zclSampleSw_ProcessInWriteRspCmd( pInMsg );
    break;
#endif
#ifdef ZCL_REPORT
    // See ZCL Test Applicaiton (zcl_testapp.c) for sample code on Attribute Reporting
  case ZCL_CMD_CONFIG_REPORT:
    //zclSampleSw_ProcessInConfigReportCmd( pInMsg );
    break;
    
  case ZCL_CMD_CONFIG_REPORT_RSP:
    //zclSampleSw_ProcessInConfigReportRspCmd( pInMsg );
    break;
    
  case ZCL_CMD_READ_REPORT_CFG:
    //zclSampleSw_ProcessInReadReportCfgCmd( pInMsg );
    break;
    
  case ZCL_CMD_READ_REPORT_CFG_RSP:
    //zclSampleSw_ProcessInReadReportCfgRspCmd( pInMsg );
    break;
    
  case ZCL_CMD_REPORT:
    //zclSampleSw_ProcessInReportCmd( pInMsg );
    break;
#endif
  case ZCL_CMD_DEFAULT_RSP:
    zclSampleSw_ProcessInDefaultRspCmd( pInMsg );
    break;
#ifdef ZCL_DISCOVER
  case ZCL_CMD_DISCOVER_CMDS_RECEIVED_RSP:
    zclSampleSw_ProcessInDiscCmdsRspCmd( pInMsg );
    break;
    
  case ZCL_CMD_DISCOVER_CMDS_GEN_RSP:
    zclSampleSw_ProcessInDiscCmdsRspCmd( pInMsg );
    break;
    
  case ZCL_CMD_DISCOVER_ATTRS_RSP:
    zclSampleSw_ProcessInDiscAttrsRspCmd( pInMsg );
    break;
    
  case ZCL_CMD_DISCOVER_ATTRS_EXT_RSP:
    zclSampleSw_ProcessInDiscAttrsExtRspCmd( pInMsg );
    break;
#endif
  default:
    break;
  }
  
  if ( pInMsg->attrCmd )
    osal_mem_free( pInMsg->attrCmd );
}

#ifdef ZCL_READ
/*********************************************************************
* @fn      zclSampleSw_ProcessInReadRspCmd
*
* @brief   Process the "Profile" Read Response Command
*
* @param   pInMsg - incoming message to process
*
* @return  none
*/
static uint8 zclSampleSw_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclReadRspCmd_t *readRspCmd;
  uint8 i;
  
  readRspCmd = (zclReadRspCmd_t *)pInMsg->attrCmd;
  for (i = 0; i < readRspCmd->numAttr; i++)
  {
    // Notify the originator of the results of the original read attributes
    // attempt and, for each successfull request, the value of the requested
    // attribute
  }
  
  return TRUE;
}
#endif // ZCL_READ

#ifdef ZCL_WRITE
/*********************************************************************
* @fn      zclSampleSw_ProcessInWriteRspCmd
*
* @brief   Process the "Profile" Write Response Command
*
* @param   pInMsg - incoming message to process
*
* @return  none
*/
static uint8 zclSampleSw_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclWriteRspCmd_t *writeRspCmd;
  uint8 i;
  
  writeRspCmd = (zclWriteRspCmd_t *)pInMsg->attrCmd;
  for (i = 0; i < writeRspCmd->numAttr; i++)
  {
    // Notify the device of the results of the its original write attributes
    // command.
  }
  
  return TRUE;
}
#endif // ZCL_WRITE

/*********************************************************************
* @fn      zclSampleSw_ProcessInDefaultRspCmd
*
* @brief   Process the "Profile" Default Response Command
*
* @param   pInMsg - incoming message to process
*
* @return  none
*/
static uint8 zclSampleSw_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg )
{
  // zclDefaultRspCmd_t *defaultRspCmd = (zclDefaultRspCmd_t *)pInMsg->attrCmd;
  // Device is notified of the Default Response command.
  (void)pInMsg;
  return TRUE;
}

#ifdef ZCL_DISCOVER
/*********************************************************************
* @fn      zclSampleSw_ProcessInDiscCmdsRspCmd
*
* @brief   Process the Discover Commands Response Command
*
* @param   pInMsg - incoming message to process
*
* @return  none
*/
static uint8 zclSampleSw_ProcessInDiscCmdsRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverCmdsCmdRsp_t *discoverRspCmd;
  uint8 i;
  
  discoverRspCmd = (zclDiscoverCmdsCmdRsp_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numCmd; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }
  
  return TRUE;
}

/*********************************************************************
* @fn      zclSampleSw_ProcessInDiscAttrsRspCmd
*
* @brief   Process the "Profile" Discover Attributes Response Command
*
* @param   pInMsg - incoming message to process
*
* @return  none
*/
static uint8 zclSampleSw_ProcessInDiscAttrsRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverAttrsRspCmd_t *discoverRspCmd;
  uint8 i;
  
  discoverRspCmd = (zclDiscoverAttrsRspCmd_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numAttr; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }
  
  return TRUE;
}

/*********************************************************************
* @fn      zclSampleSw_ProcessInDiscAttrsExtRspCmd
*
* @brief   Process the "Profile" Discover Attributes Extended Response Command
*
* @param   pInMsg - incoming message to process
*
* @return  none
*/
static uint8 zclSampleSw_ProcessInDiscAttrsExtRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverAttrsExtRsp_t *discoverRspCmd;
  uint8 i;
  
  discoverRspCmd = (zclDiscoverAttrsExtRsp_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numAttr; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }
  
  return TRUE;
}
#endif // ZCL_DISCOVER

#if ZCL_EZMODE
/*********************************************************************
* @fn      zclSampleSw_ProcessZDOMsgs
*
* @brief   Called when this node receives a ZDO/ZDP response.
*
* @param   none
*
* @return  status
*/
static void zclSampleSw_ProcessZDOMsgs( zdoIncomingMsg_t *pMsg )
{
  
  // Let EZ-Mode know of the Match Descriptor Reponse (same as ActiveEP Response)
  if ( pMsg->clusterID == Match_Desc_rsp )
  {
    zclEZMode_ActionData_t data;
    ZDO_ActiveEndpointRsp_t *pRsp = ZDO_ParseEPListRsp( pMsg );
    data.pMatchDescRsp = pRsp;
    zcl_EZModeAction( EZMODE_ACTION_MATCH_DESC_RSP, &data );
    osal_mem_free(pRsp);
  }
}

/*********************************************************************
* @fn      zclSampleSw_EZModeCB
*
* @brief   The Application is informed of events. This can be used to show on the UI what is
*           going on during EZ-Mode steering/finding/binding.
*
* @param   state - an
*
* @return  none
*/
static void zclSampleSw_EZModeCB( zlcEZMode_State_t state, zclEZMode_CBData_t *pData )
{
#ifdef LCD_SUPPORTED
  char szLine[20];
  char *pStr;
  uint8 err;
#endif
  
  // time to go into identify mode
  if ( state == EZMODE_STATE_IDENTIFYING )
  {
    zclSampleSw_IdentifyTime = (EZMODE_TIME / 1000);  // convert to seconds
    zclSampleSw_ProcessIdentifyTimeChange();
  }
  
  // autoclosing, show what happened (success, cancelled, etc...)
  if( state == EZMODE_STATE_AUTOCLOSE )
  {
#ifdef LCD_SUPPORTED
    pStr = NULL;
    err = pData->sAutoClose.err;
    if ( err == EZMODE_ERR_SUCCESS )
    {
      pStr = "EZMode: Success";
    }
    else if ( err == EZMODE_ERR_NOMATCH )
    {
      pStr = "EZMode: NoMatch"; // not a match made in heaven
    }
    if ( pStr )
    {
      if ( giSwScreenMode == SW_MAINMODE )
        HalLcdWriteString ( pStr, HAL_LCD_LINE_2 );
    }
#endif
  }
  
  // finished, either show DstAddr/EP, or nothing (depending on success or not)
  if( state == EZMODE_STATE_FINISH )
  {
    // turn off identify mode
    zclSampleSw_IdentifyTime = 0;
    zclSampleSw_ProcessIdentifyTimeChange();
    
#ifdef LCD_SUPPORTED
    // if successful, inform user which nwkaddr/ep we bound to
    pStr = NULL;
    err = pData->sFinish.err;
    if( err == EZMODE_ERR_SUCCESS )
    {
      // "EZDst:1234 EP:34"
      osal_memcpy(szLine, "EZDst:", 6);
      zclHA_uint16toa( pData->sFinish.nwkaddr, &szLine[6]);
      osal_memcpy(&szLine[10], " EP:", 4);
      _ltoa( pData->sFinish.ep, (void *)(&szLine[14]), 16 );  // _ltoa NULL terminates
      pStr = szLine;
    }
    else if ( err == EZMODE_ERR_BAD_PARAMETER )
    {
      pStr = "EZMode: BadParm";
    }
    else if ( err == EZMODE_ERR_CANCELLED )
    {
      pStr = "EZMode: Cancel";
    }
    else
    {
      pStr = "EZMode: TimeOut";
    }
    if ( pStr )
    {
      if ( giSwScreenMode == SW_MAINMODE )
        HalLcdWriteString ( pStr, HAL_LCD_LINE_2 );
    }
#endif  // LCD_SUPPORTED
    // show main UI screen 3 seconds after binding
    osal_start_timerEx( zclSampleSw_TaskID, SAMPLESW_MAIN_SCREEN_EVT, 3000 );
  }
  
}

#endif // ZCL_EZMODE

void SendData(int cluID)
{
  //HalLcdWriteString( data, HAL_LCD_LINE_6 );
  //kiem tra do di chuyen cua ZE dua vao truc x & z
  
  
  /*
  if ( AF_DataRequest( &zclSampleSw_DstAddr, &zclSampleSw_epDesc,
  cluID, //=1 tin nhan test
  (byte)osal_strlen( data ) + 1,
  (byte *)&data,
  &zclSampleSw_TransID,
  AF_SKIP_ROUTING, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  
  // Successfully requested to be sent.8
  //HalLcdWriteString( "1234", HAL_LCD_LINE_6 );
}
  else
  {
  // Error occurred in request to send.
  HalLcdWriteString( "Bi loi", HAL_LCD_LINE_6 );
} */
  
}
void SendDataFull(char *dataSend,int cluID, char *info)
{
  if ( AF_DataRequest( &zclSampleSw_DstAddr, &zclSampleSw_epDesc,
  cluID, //=1 tin nhan test
  (byte)osal_strlen( dataSend ) + 1,
  dataSend,
  &zclSampleSw_TransID,
  AF_SKIP_ROUTING, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    // Successfully requested to be sent.8
    HalLcdWriteString( info, HAL_LCD_LINE_6 );
  }
  else
  {
    // Error occurred in request to send.
    HalLcdWriteString( "Bi loi", HAL_LCD_LINE_6 );
  } 
  
}
void ProcessData( afIncomingMSGPacket_t *pkt)
{
  switch ( pkt->clusterId ) 
  { 
  case 0x11: // tin nhan temperature
    {
    SendDataFull(bufTemp,0x21,"Sending Temperature.");
   // HalLcdWriteString( "Nhiet do", HAL_LCD_LINE_6 );
    
    break;
    }
  case 0x12: // bao trom 
    { 
    enableBaotrom = 1;
    SendDataFull("#21*",0x21,"Enable Security");
    break;
    }
  case 0x13: // bao trom
    {
    enableBaotrom = 0;
    SendDataFull("#30*",0x21,"Disable Security");
    break;
    }
  case 0x14: // tin nhan LIGHT
    {
    //HalUARTWrite(SERIAL_APP_PORT,(uint8*)pkt->cmd.Data,14);
    ui16AlsValue=0;
    int ui16AlsValueTB[5];
    ui16AlsValue = alsRead();
    ui16AlsValueTB[0]=alsRead();
    ui16AlsValueTB[1]=alsRead();
    ui16AlsValueTB[2]=alsRead();
    ui16AlsValueTB[3]=alsRead();
    ui16AlsValueTB[4]=alsRead();
    int i;
    for(i=0;i<5;i++)
    {
      ui16AlsValue += ui16AlsValueTB[i];
    }
    ui16AlsValue = ui16AlsValue/5;
    
    char statusLight3Byte[]="#40";
    //while(1)
    {
      if(ui16AlsValue >400)
      {
        statusLight3Byte[2] = '1';
      }
      else
      {
        statusLight3Byte[2] = '0';
      }
      SendDataFull(statusLight3Byte,0x24,"Sending Light.");
      
    }
        
    //HalLcdWriteString( (char*)pkt->cmd.Data, HAL_LCD_LINE_7 );
    //HalUARTWrite( SERIAL_APP_PORT, (uint8*)pkt->cmd.Data, 6 );
    break; 
    }
  case 0x19:
    {
      autoLight=0;
      bspLedSet(BSP_LED_1);
      SendDataFull("#91*",0x21,"Turned On Light 3");
      break; 
    }
  case 0x1a:
    {
      autoLight=0;
      bspLedClear(BSP_LED_1);
      SendDataFull("#a0*",0x21,"Turned Off Light 3");
      break; 
    }
  case 0x1b:
    {
      autoLight=0;
      bspLedSet(BSP_LED_2);
      SendDataFull("#b1*",0x21,"Turned On Light 4");
      break; 
    }
  case 0x1c:
    {
      autoLight=0;
      bspLedClear(BSP_LED_2);
      SendDataFull("#c0*",0x21,"Turned Off Light 4");
      break; 
    }
  case 0x1d:    //enable auto control Light 1 3
    {
      autoLight =1;
      SendDataFull("#d1*",0x21,"Enable Auto Light");
     break;     
    }
  case 0x1e:
    {
     autoLight=0;
     SendDataFull("#e0*",0x21,"Disable Auto Light");
     break;     //disable auto control light 1 3
    }
  case 0x1f:
  {
        char * tmpInfo;
        *(tmpInfo++) = '#';
        *(tmpInfo++) = 'f';
        *(tmpInfo++) = bspLedGet(BSP_LED_1)+48; //led 1 status
        *(tmpInfo++) = bspLedGet(BSP_LED_2)+48; //led 2 status
        *(tmpInfo++) = autoLight+48; //autoLight
        
        int ic;
        for(ic=2;ic<9;ic++)
        {
          *(tmpInfo++) = *(bufTemp+ic);
        }

   SendDataFull("#e",0x21,"Disable Auto Light");
   break;     //disable auto control light 1 3
  }

  }
}

static void initAccelerometer()
{
  accInit();
  //_________________________________________________________________
  ui8RegVal = 0x04;
  accWriteReg(ACC_TAP_INT_SAMP_TH, &ui8RegVal, 1);
  
  //
  // Configure tap detection timing (second tap within 150 ms of the first)
  //
  ui8RegVal = 0x02;
  accWriteReg(ACC_TAP_INT_TIMING, &ui8RegVal, 1);
  
  //
  // Map double tap interrupt to acc's INT1 gpio
  //
  ui8RegVal = ACC_INT_MAP_D_TAP;
  accWriteReg(ACC_INT1_MAP, &ui8RegVal, 1);
  
  //
  // Set acc's INT1 as push-pull, active high
  //
  ui8RegVal = ACC_INT_CFG_INT1_ACTIVE_HI;
  accWriteReg(ACC_INT_PIN_CFG, &ui8RegVal, 1);
  
  //
  // Enable double tap interrupt
  //
  ui8RegVal = ACC_INT_EN0_D_TAP_EN;
  accWriteReg(ACC_INT_EN0, &ui8RegVal, 1);
}
static void sendAccelermeter()
{
  
  accReadData(&i16X, &i16Y, &i16Z);
  if(i16X < -30 | i16X >30 | i16Z < -50 | i16Z >50)
  {
    SendDataFull("#2Co Trom\r\n",0x21,"Co Trom");
    
  }
  else {
    HalLcdWriteString( "Ko Co trom", HAL_LCD_LINE_6 );
  }
  //tinh x
  if(i16X>=0)
  {
    data[0]='+';
    data[1]=i16X/100 + 48;
    data[2]=(i16X%100)/10 + 48;
    data[3]=i16X%10 + 48;
  }
  else{
    i16X *= (-1);
    data[0]='-';
    data[1]=i16X/100 + 48;
    data[2]=(i16X%100)/10 + 48;
    data[3]=i16X%10 + 48;
  }
  //tinh y
  if(i16Y>=0)
  {
    data[4]='+';
    data[5]=i16Y/100 + 48;
    data[6]=(i16Y%100)/10 + 48;
    data[7]=i16Y%10 + 48;
  }
  else{
    i16Y *= (-1);
    data[4]='-';
    data[5]=i16Y/100 + 48;
    data[6]=(i16Y%100)/10 + 48;
    data[7]=i16Y%10 + 48;
  }
  //tinh z
  if(i16Z>=0)
  {
    data[8]='+';
    data[9]=i16Z/100 + 48;
    data[10]=(i16Z%100)/10 + 48;
    data[11]=i16Z%10 + 48;
  }
  else{
    i16Z *= (-1);
    data[8]='-';
    data[9]=i16Z/100 + 48;
    data[10]=(i16Z%100)/10 + 48;
    data[11]=i16Z%10 + 48;
  }
  HalLcdWriteString( data, HAL_LCD_LINE_5 );
  //HalUARTWrite(SERIAL_APP_PORT,(uint8*)data,14);
}
static void sendLight()
{
  ui16AlsValue = alsRead();
  char dataLight[6];  
  dataLight[0]=ui16AlsValue/1000 + 48;
  dataLight[1]=(ui16AlsValue%1000)/100 + 48;
  dataLight[2]=(ui16AlsValue%100)/10 + 48;
  dataLight[3]= ui16AlsValue%10 + 48;
  dataLight[4]='\r';
  dataLight[5]='\n';
  if ( AF_DataRequest( &zclSampleSw_DstAddr, &zclSampleSw_epDesc,
                      2, //=1 tin nhan test
                      (byte)osal_strlen( dataLight ) + 1,
                      (byte *)&dataLight,
                      &zclSampleSw_TransID,
                      AF_SKIP_ROUTING, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    
    // Successfully requested to be sent.8
    // HalLcdWriteString( dataLight, HAL_LCD_LINE_5 );
  }
  else
  {
    // Error occurred in request to send.
    HalLcdWriteString( "Bi loi", HAL_LCD_LINE_5 );
  } 
}
void initTemperature()
{
    SysCtrlClockSet(false, false, SYS_CTRL_SYSDIV_32MHZ);

    //
    // Set IO clock to the same as system clock
    //
    SysCtrlIOClockSet(SYS_CTRL_SYSDIV_32MHZ);
     
    //
    // Enable RF Core (needed to enable temp sensor)
    //
    SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_RFC);
    
    //
    // Set up the serial console to use for displaying messages.  This is
    // just for this example program and is not needed for Systick operation.
    //
    //InitConsole();

    //
    // Display the setup on the console.
    //
    //UARTprintf("ADC temp sens\n");
    
    //
    // Connect temp sensor to ADC
    //
    HWREG(CCTEST_TR0) |= CCTEST_TR0_ADCTM;

    //
    // Enable the temperature sensor 
    //
    HWREG(RFCORE_XREG_ATEST) = 0x01;
    
    //
    // Configure ADC, Internal reference, 512 decimation rate (12bit)
    //
    SOCADCSingleConfigure(SOCADC_12_BIT, SOCADC_REF_INTERNAL);
}
void readTemperature()
{
        //
        // Trigger single conversion on internal temp sensor
        //
        SOCADCSingleStart(SOCADC_TEMP_SENS);
        
        //
        // Wait until conversion is completed
        //
        while(!SOCADCEndOfCOnversionGet())
        {
        }
        //
        // Get data and shift down based on decimation rate
        //
        ui16Dummy = SOCADCDataGet() >> SOCADC_12_BIT_RSHIFT;
        
        //
        // Convert to temperature
        //
        dOutputVoltage = ui16Dummy * CONST1 *4;       
        dOutputVoltage = ((dOutputVoltage - OFFSET_0C) / TEMP_COEFF);
        
        //sprintf(pcTemp, "%.1f", dOutputVoltage);
        
        dOutputVoltage =(dOutputVoltage*10);
        sprintf(pcTemp, "Temp = %d.%d C", ((int)dOutputVoltage/100), ((int)dOutputVoltage%100));
        //sprintf(pcTemp, "%.1f", dOutputVoltage);
        
        //pcTemp[3] = ((int)(dOutputVoltage*10))%10 + 48;
       // pcTemp[2] = '.';
       /// pcTemp[1] = ((int)dOutputVoltage)%10 + 48;
       // pcTemp[0] = (((int)dOutputVoltage)%100)/10 + 48;
        //sprintf(pcTemp, "%f", dOutputVoltage);
        HalLcdWriteString( pcTemp, HAL_LCD_LINE_5);
}

void connect2ZC()
{
  giSwScreenMode = SW_MAINMODE;   // remove help screen if there
    
#ifdef ZCL_EZMODE
    {
      zclEZMode_InvokeData_t ezModeData;
      static uint16 clusterIDs[] = { ZCL_CLUSTER_ID_GEN_ON_OFF };   // only bind on the on/off cluster
      
      // Invoke EZ-Mode
      ezModeData.endpoint = SAMPLESW_ENDPOINT; // endpoint on which to invoke EZ-Mode
      if ( (zclSampleSw_NwkState == DEV_ZB_COORD) ||
          (zclSampleSw_NwkState == DEV_ROUTER)   ||
            (zclSampleSw_NwkState == DEV_END_DEVICE) )
      {
        ezModeData.onNetwork = TRUE;      // node is already on the network
      }
      else
      {
        ezModeData.onNetwork = FALSE;     // node is not yet on the network
      }
      ezModeData.initiator = TRUE;        // OnOffSwitch is an initiator
      ezModeData.numActiveOutClusters = 1;   // active output cluster
      ezModeData.pActiveOutClusterIDs = clusterIDs;
      ezModeData.numActiveInClusters = 0;  // no active input clusters
      ezModeData.pActiveInClusterIDs = NULL;
      zcl_InvokeEZMode( &ezModeData );
      
#ifdef LCD_SUPPORTED
      HalLcdWriteString( "EZMode2222", HAL_LCD_LINE_2 );
#endif
    }
    
#else // NOT ZCL_EZMODE
    // bind to remote light
    zAddrType_t dstAddr;
    HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );
    
    // Initiate an End Device Bind Request, this bind request will
    // only use a cluster list that is important to binding.
    dstAddr.addrMode = afAddr16Bit;
    dstAddr.addr.shortAddr = 0;   // Coordinator makes the match
    ZDP_EndDeviceBindReq( &dstAddr, NLME_GetShortAddr(),
                         SAMPLESW_ENDPOINT,
                         ZCL_HA_PROFILE_ID,
                         0, NULL,   // No incoming clusters to bind
                         ZCLSAMPLESW_BINDINGLIST, bindingOutClusters,
                         TRUE );
    flag =1;
    HalLcdWriteString( "EZMode1111111", HAL_LCD_LINE_6 );
#endif // ZCL_EZMODE
}
static void initUart()
{
     halUARTCfg_t uartConfig; 
    uartConfig.configured = TRUE; // 2430 don't care. 
    uartConfig.baudRate = HAL_UART_BR_9600; 
    uartConfig.flowControl = FALSE; 
    uartConfig.flowControlThreshold = 48; 
    uartConfig.rx.maxBufSize = 128; 
    uartConfig.tx.maxBufSize = 128; 
    uartConfig.idleTimeout = 6; // 2430 don't care. 
    uartConfig.intEnable = TRUE; // 2430 don't care. 
    uartConfig.callBackFunc = rxCB; 
    //PERCFG |= 0x01;
    //HalUARTInit(); 
    HalUARTOpen (SERIAL_APP_PORT, &uartConfig); 
}
static void rxCB( uint8 port, uint8 event )
{
    if ( event != HAL_UART_TX_EMPTY )
  {
    // Read from UART
    HalUARTRead( SERIAL_APP_PORT, bufTemp, 8 );
    //if(pBuf1[0]=='swww')
      //ic=0;
    //els/e if(pBuf1[0]=='fwww')
    //{
      //SendDataFull(tempData, 0x21, "Nhiet Do");
    //}
    //else
    //{
      //tempData[ic++]= pBuf1[0];
    //}
    //SendDataFull(pBuf1, 0x21, "Nhiet Do");
        //SendDataFull("#1",0x11,"Getting Temperature");
    //HalLcdWriteString ( pBuf, HAL_LCD_LINE_6 );
    

}
    
   }