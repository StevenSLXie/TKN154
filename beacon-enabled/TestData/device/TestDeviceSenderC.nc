/* 
 * Copyright (c) 2008, Technische Universitaet Berlin
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * - Neither the name of the Technische Universitaet Berlin nor the names
 *   of its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * - Revision -------------------------------------------------------------
 * $Revision: 1.2 $
 * $Date: 2010-01-05 17:12:56 $
 * @author: Jan Hauer <hauer@tkn.tu-berlin.de>
 * ========================================================================
 */

#include "TKN154.h"
#include "app_profile.h"
#include "TestSerial.h"
#include "printf.h"
//#include "look_up_table.h"

#define MAX(A,B) ((A)>(B))?(A):(B)
#define LIST_LEN 11

module TestDeviceSenderC
{
  uses {
    interface Boot;
	interface SplitControl as Control;
	//interface Timer<TMicro> as TimerSendPac;
	//interface Msp430Timer as TimerSendPac;
	interface Alarm<TMicro, uint16_t> as TimerSendPac;
	interface Timer<TMilli> as TimerChgPrd;
    interface AMSend;
	
    interface MCPS_DATA;
    interface MLME_RESET;
    interface MLME_SET;
    interface MLME_GET;
    interface MLME_SCAN;
    interface MLME_SYNC;
    interface MLME_BEACON_NOTIFY;
    interface MLME_SYNC_LOSS;
    interface IEEE154Frame as Frame;
    interface IEEE154BeaconFrame as BeaconFrame;
    interface Leds;
    interface Packet as MACPacket;
	interface Packet as SerialPacket;
	interface Random;
  }
} implementation {

  message_t m_frame;
  uint8_t m_payloadLen;
  ieee154_PANDescriptor_t m_PANDescriptor;
  bool m_ledCount;
  bool m_wasScanSuccessful;
  uint16_t m_numOfTransmission;
  uint16_t m_numOfSuccess;
  float m_PSR = 0;
  float m_beta;
  float m_delta;
  float m_traffic;
  uint8_t nodeType = 1;
  uint16_t period;
  uint16_t thu;
  uint16_t interval = 0;
  bool locked = FALSE;
  message_t packet;
  
  uint8_t timerFlag = 0;
 
  float betaList[LIST_LEN]={0};
  float PSRList[LIST_LEN] = {0};
  uint8_t j = 0;
  
  bool late = FALSE;
  
  uint16_t trans2 = 0;


  static void startApp();
  task void packetSendTask();
  static uint16_t getRandomNumber();


  event void Boot.booted() {
    char payload[] = "He";
    uint8_t *payloadRegion;
	period = 16000*nodeType+16000;
	thu = 16000*nodeType+16000;

    m_payloadLen = strlen(payload);
    payloadRegion = call MACPacket.getPayload(&m_frame, m_payloadLen);
    if (m_payloadLen <= call MACPacket.maxPayloadLength()){
      memcpy(payloadRegion, payload, m_payloadLen);
      call MLME_RESET.request(TRUE);
    }
	call Control.start();
  }

  event void MLME_RESET.confirm(ieee154_status_t status)
  {
    if (status == IEEE154_SUCCESS)
      startApp();
  }

  static void startApp()
  {
	
  
    ieee154_phyChannelsSupported_t channelMask;
    uint8_t scanDuration = BEACON_ORDER;

    call MLME_SET.phyTransmitPower(TX_POWER);
    call MLME_SET.macShortAddress(TOS_NODE_ID);

    // scan only the channel where we expect the coordinator
    channelMask = ((uint32_t) 1) << RADIO_CHANNEL;

    // we want all received beacons to be signalled 
    // through the MLME_BEACON_NOTIFY interface, i.e.
    // we set the macAutoRequest attribute to FALSE
    call MLME_SET.macAutoRequest(FALSE);
    m_wasScanSuccessful = FALSE;
    call MLME_SCAN.request  (
                           PASSIVE_SCAN,           // ScanType
                           channelMask,            // ScanChannels
                           scanDuration,           // ScanDuration
                           0x00,                   // ChannelPage
                           0,                      // EnergyDetectListNumEntries
                           NULL,                   // EnergyDetectList
                           0,                      // PANDescriptorListNumEntries
                           NULL,                   // PANDescriptorList
                           0                       // security
                        );
	
  }

  event message_t* MLME_BEACON_NOTIFY.indication (message_t* frame)
  {
    // received a beacon frame
    ieee154_phyCurrentPage_t page = call MLME_GET.phyCurrentPage();
    ieee154_macBSN_t beaconSequenceNumber = call BeaconFrame.getBSN(frame);

    if (!m_wasScanSuccessful) {
      // received a beacon during channel scanning
      if (call BeaconFrame.parsePANDescriptor(
            frame, RADIO_CHANNEL, page, &m_PANDescriptor) == SUCCESS) {
        // let's see if the beacon is from our coordinator...
        if (m_PANDescriptor.CoordAddrMode == ADDR_MODE_SHORT_ADDRESS &&
            m_PANDescriptor.CoordPANId == PAN_ID &&
            m_PANDescriptor.CoordAddress.shortAddress == COORDINATOR_ADDRESS){
          // yes! wait until SCAN is finished, then syncronize to the beacons
          m_wasScanSuccessful = TRUE;
        }
      }
    } else { 
      // received a beacon during synchronization, toggle LED2
      if (beaconSequenceNumber & 1)
        call Leds.led2On();
      else
        call Leds.led2Off();   
    }

    return frame;
  }

  event void MLME_SCAN.confirm    (
                          ieee154_status_t status,
                          uint8_t ScanType,
                          uint8_t ChannelPage,
                          uint32_t UnscannedChannels,
                          uint8_t EnergyDetectListNumEntries,
                          int8_t* EnergyDetectList,
                          uint8_t PANDescriptorListNumEntries,
                          ieee154_PANDescriptor_t* PANDescriptorList
                        )
  {
    if (m_wasScanSuccessful) {
      // we received a beacon from the coordinator before
      call MLME_SET.macCoordShortAddress(m_PANDescriptor.CoordAddress.shortAddress);
      call MLME_SET.macPANId(m_PANDescriptor.CoordPANId);
      call MLME_SYNC.request(m_PANDescriptor.LogicalChannel, m_PANDescriptor.ChannelPage, TRUE);
      call Frame.setAddressingFields(
          &m_frame,                
          ADDR_MODE_SHORT_ADDRESS,        // SrcAddrMode,
          ADDR_MODE_SHORT_ADDRESS,        // DstAddrMode,
          m_PANDescriptor.CoordPANId,     // DstPANId,
          &m_PANDescriptor.CoordAddress,  // DstAddr,
          NULL                            // security
          );
      // Initilize the packet transmission timer

	  
	  //call TimerSendPac.startOneShot(period);
	  call TimerSendPac.start(getRandomNumber());
	  // Initialize the transmission rate adjustment algorithm
	  call TimerChgPrd.startOneShot(60000);
	  
    } else
      startApp();
  }
  
  static uint16_t getRandomNumber(){
		uint16_t res = call Random.rand16();
		//uint16_t mask = 0xFFFF;
		//mask <<= bias;
		//mask = ~mask;
		//res &= mask;
		return res;
  }
  
  static float abs(float a){
	 if(a>=0)
		return a;
	 else
		return (-a);
  }
  
  async event void TimerSendPac.fired(){
	 
	call TimerSendPac.start(period);
	post packetSendTask();
	 
  }
  
  static void printfFloat(float toBePrinted) {
     uint32_t fi, f0, f1, f2;
     char c;
     float f = toBePrinted;

     if (f<0){
       c = '-'; f = -f;
     } else {
       c = ' ';
     }

     // integer portion.
     fi = (uint32_t) f;

     // decimal portion...get index for up to 3 decimal places.
     f = f - ((float) fi);
     f0 = f*10;   f0 %= 10;
     f1 = f*100;  f1 %= 10;
     f2 = f*1000; f2 %= 10;
     printf("%c%ld.%d%d%d", c, fi, (uint8_t) f0, (uint8_t) f1,  
(uint8_t) f2);
   }
   
   static float betaTable[] = {0.01, 0.03, 0.06, 0.09, 0.11, 0.14, 0.18, 0.20, 0.22, 0.23, 0.25, 0.27, 0.28, 0.29, 0.30, 0.31, 0.32, 0.33, 0.34, 0.36, 0.37, 0.38, 0.39, 0.40, 0.40, 0.41, 0.42, 0.43, 0.44, 0.44, 0.45, 0.46, 0.47, 0.47, 0.48, 0.48, 0.49, 0.50};
   
   static float quan_beta(float PSR){
   
	uint8_t index = 0;
	index = (uint8_t)((1.0 - PSR) *200);
   
	if(index>37)
		return 0.5;
	else
		return betaTable[index];
	   

}

	static float optBetaTable[] = {0.01, 0.01, 0.01, 0.01, 0.02, 0.04, 0.05, 0.07, 0.08, 0.10, 0.11, 0.13, 0.14, 0.16, 0.18, 0.19, 0.21, 0.23, 0.24, 0.26, 0.28, 0.29, 0.31, 0.33, 0.35, 0.37, 0.39, 0.41, 0.43, 0.46, 0.48, 0.51, 0.55};

	static float quan_opt_beta(float traffic){

		uint8_t index = 0;
		index = (uint8_t)(traffic*400);
		if(index>32)
			return 0.6;
		else
			return optBetaTable[index];

  }
 
  static float fromBetaToTraffic(float beta){
		return beta/(1-beta*beta*beta*beta*beta)/6.0;
  }

  static float findOptimalDelta(float beta, float traffic, float PSR){
		return MAX(beta/(1-beta*beta*beta*beta*beta)/(traffic)/6.0,1.0/PSR);
  }
  
  event void TimerChgPrd.fired(){
    float PSR_b = 0;
	/*
	if(timerFlag < 3){
		timerFlag++;
		call TimerChgPrd.startOneShot(60000);
		return;
	}
	*/
	
	++interval;
	
	PSR_b = (float)m_numOfSuccess/(float)m_numOfTransmission;
	
	
	if(1 == interval){
	
		m_PSR = PSR_b;  // calculate the PSR
		atomic{
			m_beta = quan_beta(m_PSR);
			
			m_traffic = fromBetaToTraffic(m_beta);
		
			if(late){		
				m_beta = quan_opt_beta(m_traffic*m_PSR);
			}
			else{		
				m_beta = quan_opt_beta(m_traffic);
			}
			m_delta = findOptimalDelta(m_beta, m_traffic,m_PSR);		
			period = (float)thu/m_delta;
		}


	}
	
	else if((m_PSR - PSR_b)>=0.02){
		float curTraf = 0;
		m_PSR = PSR_b;  // calculate the PSR
		atomic{
		
			m_beta = quan_beta(m_PSR);
			
			printf("The current beta is:");
			printfFloat(m_beta);
			printf("\n");
			
			curTraf = fromBetaToTraffic(m_beta);
			curTraf -= (m_delta*m_traffic);
			m_traffic += curTraf;
			
			m_beta = quan_opt_beta(m_traffic);
			
			printf("The expected beta is:");
			printfFloat(m_beta);
			printf("\n");
			
			m_delta = findOptimalDelta(m_beta, m_traffic,m_PSR);
		
			period = (float)thu/m_delta;
		}	

	
	}
	else if((PSR_b - m_PSR)>=0.02){
		m_PSR = PSR_b;  // calculate the PSR
		atomic{
			m_beta = quan_beta(m_PSR);
		
			printf("The current beta is:");
			printfFloat(m_beta);
			printf("\n");
			
			m_traffic = fromBetaToTraffic(m_beta);
			m_traffic /= m_delta;
			
			m_beta = quan_opt_beta(m_traffic);
			
			printf("The expected beta is:");
			printfFloat(m_beta);
			printf("\n");
			m_delta = findOptimalDelta(m_beta, m_traffic,m_PSR);
		
			period = (float)thu/m_delta;
		}

		
	}
	else{
		printf("Nothing changes.\nThe current beta and PSR and period are:");
		printfFloat(quan_beta(PSR_b));
		printfFloat(PSR_b);
		printf(" %d",period);
		printf("\n");
	}
	
	if(interval<=LIST_LEN-1){
		betaList[interval] = quan_beta(PSR_b);
		PSRList[interval] = PSR_b;
	}
	
	m_numOfSuccess = 0;
	m_numOfTransmission = 0;
    call TimerChgPrd.startOneShot(60000);

  }
  
  
  
  task void sendToSerial() {
	
	
	printf("the number of transmission is %u.\n",m_numOfTransmission);
	printf("the number of successful transmission is %u.\n",m_numOfSuccess);
	printfFloat(m_beta);
	printfFloat(m_delta);
	printfFloat(m_PSR);
	printfFloat(m_traffic);
	printf("\n");
	
	for(j=0;j<LIST_LEN;j++)
		printfFloat(betaList[j]);
	printf("\n");
	
	for(j=0;j<LIST_LEN;j++)
		printfFloat(PSRList[j]);
	printf("\n");
	printfflush();
	
  }
  
  

  task void packetSendTask()
  {
    if (!m_wasScanSuccessful){
	  call Leds.led0On();
	  call TimerSendPac.stop();
	  startApp();
      return;
	  }
    else if (call MCPS_DATA.request  (
          &m_frame,                         // frame,
          m_payloadLen,                     // payloadLength,
          0,                                // msduHandle,
          TX_OPTIONS_ACK // TxOptions,
          ) != IEEE154_SUCCESS){
	  //trans2++;
      call Leds.led0On();
	  call TimerSendPac.stop();
	
	}
	else{
		call Leds.led0Off();
		++m_numOfTransmission;
		//trans2++;
	}

	
  }

  event void MCPS_DATA.confirm    (
                          message_t *msg,
                          uint8_t msduHandle,
                          ieee154_status_t status,
                          uint32_t timestamp
                        )
  {
    if (status == IEEE154_SUCCESS ) {
	  ++m_numOfSuccess;
	  call Leds.led1Toggle();
	  if(m_numOfTransmission%100 == 0){
		post sendToSerial();
	  }
	  //m_ledCount = 0;
	
    }
	
    
  }

  event void MLME_SYNC_LOSS.indication(
                          ieee154_status_t lossReason,
                          uint16_t PANId,
                          uint8_t LogicalChannel,
                          uint8_t ChannelPage,
                          ieee154_security_t *security)
  {
    m_wasScanSuccessful = FALSE;
    call Leds.led1Off();
    call Leds.led2Off();
	call Leds.led0On();
	call TimerSendPac.stop();
	call TimerChgPrd.stop();
	startApp();
  }

  event message_t* MCPS_DATA.indication (message_t* frame)
  {
    // we don't expect data
    return frame;
  }
  
  event void AMSend.sendDone(message_t* bufPtr, error_t error) {
    if (&packet == bufPtr) {
      locked = FALSE;
    }
  }
  
  event void Control.startDone(error_t err) {}
  event void Control.stopDone(error_t err) {}

}
