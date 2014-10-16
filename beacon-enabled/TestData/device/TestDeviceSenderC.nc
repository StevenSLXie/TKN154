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
//#define ABS(A) ((A)>=0)?((A):(-(A))) 

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
  uint16_t period;
  uint16_t thu = 32000;
  uint16_t interval = 0;
  bool locked = FALSE;
  message_t packet;
  uint8_t nodeType = 1;
  uint8_t timerFlag = 0;
  
  bool late = TRUE;
  
  uint16_t trans2 = 0;


  static void startApp();
  task void packetSendTask();


  event void Boot.booted() {
    char payload[] = "He";
    uint8_t *payloadRegion;

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

	  if (1==nodeType){
		  period = 32000;
	   }else if(2 == nodeType){
		  period = 32000;
	  } else if(3 == nodeType){
		  period = 32000;
	  } else if(4 == nodeType){
		  period = 32000;
	  }
	  //call TimerSendPac.startOneShot(period);
	  call TimerSendPac.start(period);
	  // Initialize the transmission rate adjustment algorithm
	  call TimerChgPrd.startOneShot(60000);
	  
    } else
      startApp();
  }
  
  static uint16_t getRandomNumber(uint8_t bias){
		uint16_t res = call Random.rand16();
		uint16_t mask = 0xFFFF;
		mask <<= bias;
		mask = ~mask;
		res &= mask;
		return res;
  }
  
  static float abs(float a){
	 if(a>=0)
		return a;
	 else
		return (-a);
  }
  
  async event void TimerSendPac.fired(){
	 
	//call TimerSendPac.start(0.9*period+getRandomNumber(0.2*period));
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
   
   static float quan_beta(float PSR){

	if(PSR>0.995)
		return 0.01;
	else if(PSR>0.990)
		return 0.03;
	else if(PSR>0.985)
		return 0.06;
	else if(PSR>0.980)
		return 0.09;
	else if(PSR>0.975)
		return 0.11;
	else if(PSR>0.970)
		return 0.14;
	else if(PSR>0.965)
		return 0.16;
	else if(PSR>0.960)
		return 0.18;
	else if(PSR>0.955)
		return 0.20;
	else if(PSR>0.950)
		return 0.22;
	else if(PSR>0.945)
		return 0.23;
	else if(PSR>0.940)
		return 0.25;
	else if(PSR>0.935)
		return 0.27;
	else if(PSR>0.930)
		return 0.28;
	else if(PSR>0.925)
		return 0.30;
	else if(PSR>0.920)
		return 0.31;
	else if(PSR>0.915)
		return 0.32;
	else if(PSR>0.910)
		return 0.33;
	else if(PSR>0.905)
		return 0.34;
	else if(PSR>0.900)
		return 0.36;
	else if(PSR>0.895)
		return 0.37;
	else if(PSR>0.890)
		return 0.38;
	else if(PSR>0.885)
		return 0.39;
	else if(PSR>0.880)
		return 0.40;
	else if(PSR>0.875)
		return 0.40;
	else if(PSR>0.870)
		return 0.41;
	else if(PSR>0.865)
		return 0.42;
	else if(PSR>0.860)
		return 0.43;
	else if(PSR>0.855)
		return 0.44;
	else if(PSR>0.850)
		return 0.44;
	else if(PSR>0.845)
		return 0.45;
	else if(PSR>0.840)
		return 0.46;
	else if(PSR>0.835)
		return 0.47;
	else if(PSR>0.830)
		return 0.47;
	else if(PSR>0.825)
		return 0.48;
	else if(PSR>0.820)
		return 0.48;
	else if(PSR>0.815)
		return 0.49;
	else if(PSR>0.810)
		return 0.50;
	else if(PSR>0.805)
		return 0.50;
	else if(PSR>0.800)
		return 0.50;
	else
		return 0.60;

}

static float quan_opt_beta(float traffic){
	if(traffic<0.0025)
		return 0.01;
	else if(traffic<0.0050)
		return 0.01;
	else if(traffic<0.0075)
		return 0.01;
	else if(traffic<0.0100)
		return 0.01;
	else if(traffic<0.0125)
		return 0.02;
	else if(traffic<0.0150)
		return 0.04;
	else if(traffic<0.0175)
		return 0.05;
	else if(traffic<0.0200)
		return 0.07;
	else if(traffic<0.0225)
		return 0.08;
	else if(traffic<0.0250)
		return 0.10;
	else if(traffic<0.0275)
		return 0.11;
	else if(traffic<0.0300)
		return 0.13;
	else if(traffic<0.0325)
		return 0.14;
	else if(traffic<0.0350)
		return 0.16;
	else if(traffic<0.0375)
		return 0.18;
	else if(traffic<0.0400)
		return 0.19;
	else if(traffic<0.0425)
		return 0.21;
	else if(traffic<0.0450)
		return 0.23;
	else if(traffic<0.0475)
		return 0.24;
	else if(traffic<0.0500)
		return 0.26;
	else if(traffic<0.0525)
		return 0.28;
	else if(traffic<0.0550)
		return 0.29;
	else if(traffic<0.0575)
		return 0.31;
	else if(traffic<0.0600)
		return 0.33;
	else if(traffic<0.0625)
		return 0.35;
	else if(traffic<0.0650)
		return 0.37;
	else if(traffic<0.0675)
		return 0.39;
	else if(traffic<0.0700)
		return 0.41;
	else if(traffic<0.0725)
		return 0.43;
	else if(traffic<0.0750)
		return 0.46;
	else if(traffic<0.0775)
		return 0.48;
	else if(traffic<0.0800)
		return 0.51;
	else if(traffic<0.0825)
		return 0.55;
	else if(traffic<0.0850)
		return 0.61;
	else if(traffic<0.0875)
		return 0.8;
	else if(traffic<0.0900)
		return 0.8;
	else
		return 0.8;

}
 /* 
  float fromPSRToBeta(float PSR){
	  float betaCan = 0;
	  float PSRCan = 0;
	  uint8_t i;
	  
	  float betaBest = 0;
	  float dif = 100;
	  
	  if(PSR < 0.8)
		  return 0.5;
      else if (PSR <= 0.90){
		  
		  for(i =0;i<= 20;i++){
			   betaCan = 0.30 + 0.01*i;
			   PSRCan = (1.0-betaCan*betaCan*betaCan*betaCan*betaCan)*(1.0-betaCan/(1-betaCan)/6.0);
			   
			   if(abs(PSRCan - PSR)<dif){
					dif = abs(PSRCan - PSR);
					betaBest = betaCan;
			   }
			   
			   if(dif < 0.008){
					return betaBest;	
			   }
		  }
		  return betaBest;
	  }
	  else {
		  for(i =0;i<= 38;i++){
			   betaCan = 0.01*i;
			   PSRCan = (1.0-betaCan*betaCan*betaCan*betaCan*betaCan)*(1.0-betaCan/(1-betaCan)/6.0);
			   
			   if(abs(PSRCan - PSR)<dif){
					dif = abs(PSRCan - PSR);
					betaBest = betaCan;
			   }
			   
			   if(dif < 0.008){
					return betaBest;	
			   }		   
		  }
		  return betaCan;
	  }
	
  }
*/  
  static float fromBetaToTraffic(float beta){
		return beta/(1-beta*beta*beta*beta*beta)/6.0;
  }
/*
  float findOptimalBeta(float traffic){
	  float betaCan = 0;
	  float trafCan;
	  
	  float betaBest = 0;
	  float dif = 100;
	  
	  uint8_t i;
	  
	  if (traffic > 0.085)
		  return 0.7;
      else if (traffic <= 0.05){
		 for(i =0;i<= 30;i++){
			 betaCan = 0.01*i;
			 trafCan = (betaCan-2*betaCan*betaCan-betaCan*betaCan*5.0+5.0*betaCan)/36.0/(1-betaCan);
			 
			 if(abs(trafCan - traffic) < dif){
				dif =abs(trafCan - traffic);
				betaBest = betaCan;
			 }
			 
			 if(dif < 0.002){
					return betaBest;	
			 }			   
		 }
		 return betaBest;
	  }
	  
	  else{
		 for(i =0;i<= 40;i++){
			 betaCan = 0.2+0.01*i;
			 trafCan = (betaCan-2*betaCan*betaCan-betaCan*betaCan*5.0+5.0*betaCan)/36.0/(1-betaCan);
			 
			 if(abs(trafCan - traffic) < dif){
				dif = abs(trafCan - traffic);
				betaBest = betaCan;
			 }
			 
			 if(dif < 0.002){
					return betaBest;	
			 }				   
		 }
		 return betaBest;
	  }
	  
  }
 */ 
  static float findOptimalDelta(float beta, float traffic, float PSR){
		return MAX(beta/(1-beta*beta*beta*beta*beta)/(traffic)/6.0,1.0/PSR);
  }
  
  event void TimerChgPrd.fired(){
    
	/*
	if(timerFlag < 3){
		timerFlag++;
		call TimerChgPrd.startOneShot(60000);
		return;
	}
	*/
	
	
	float PSR_b = 0;
	
	++interval;
	
	PSR_b = (float)m_numOfSuccess/(float)m_numOfTransmission;
	
	
	if(1 == interval){
	
		m_PSR = PSR_b;  // calculate the PSR
		atomic{
			//m_beta = fromPSRToBeta(m_PSR);
			m_beta = quan_beta(m_PSR);
			
			m_traffic = fromBetaToTraffic(m_beta);
		
			if(late){		
				//m_beta = findOptimalBeta(m_traffic*m_PSR);	
				m_beta = quan_opt_beta(m_traffic*m_PSR);
			}
			else{		
				//m_beta = findOptimalBeta(m_traffic);
				m_beta = quan_opt_beta(m_traffic);
			}
			m_delta = findOptimalDelta(m_beta, m_traffic,m_PSR);		
			period = (float)thu/m_delta;
		}


	}
	
	else if((m_PSR - PSR_b)>=0.015){
		float curTraf = 0;
		m_PSR = PSR_b;  // calculate the PSR
		atomic{
		
			//m_beta = fromPSRToBeta(m_PSR);
			m_beta = quan_beta(m_PSR);
			
			printf("The current beta is:");
			printfFloat(m_beta);
			printf("\n");
			
			curTraf = fromBetaToTraffic(m_beta);
			curTraf -= (m_delta*m_traffic);
			m_traffic += curTraf;
			
			//m_beta = findOptimalBeta(m_traffic);
			m_beta = quan_opt_beta(m_traffic);
			
			printf("The expected beta is:");
			printfFloat(m_beta);
			printf("\n");
			
			m_delta = findOptimalDelta(m_beta, m_traffic,m_PSR);
		
			period = (float)thu/m_delta;
		}	

	
	}
	else if((PSR_b - m_PSR)>=0.015){
		m_PSR = PSR_b;  // calculate the PSR
		atomic{
			//m_beta = fromPSRToBeta(m_PSR);
			m_beta = quan_beta(m_PSR);
		
			printf("The current beta is:");
			printfFloat(m_beta);
			printf("\n");
			
			m_traffic = fromBetaToTraffic(m_beta);
			m_traffic /= m_delta;
			
			//m_beta = findOptimalBeta(m_traffic);
			m_beta = quan_opt_beta(m_traffic);
			
			printf("The expected beta is:");
			printfFloat(m_beta);
			printf("\n");
			m_delta = findOptimalDelta(m_beta, m_traffic,m_PSR);
		
			period = (float)thu/m_delta;
		}

		
	}
	else{
		printf("Nothing changes.\nThe current beta is:");
		//printfFloat(fromPSRToBeta(PSR_b));
		printfFloat(quan_beta(PSR_b));
		printf("\n");
	}
	m_numOfSuccess = 0;
	m_numOfTransmission = 0;
    call TimerChgPrd.startOneShot(60000);

  }
  
  
  
  task void sendToSerial() {
   /*
    if (locked) {
      return;
    }
    else {
      test_serial_msg_t* rcm = (test_serial_msg_t*)call SerialPacket.getPayload(&packet, sizeof(test_serial_msg_t));
      if (rcm == NULL) {return;}
      if (call SerialPacket.maxPayloadLength() < sizeof(test_serial_msg_t)) {
	return;
      }
	  
      rcm->numOfTransmission = m_numOfTransmission;
	  rcm->numOfSuccess = m_numOfSuccess;
	  rcm->period = period;
	  
      if (call AMSend.send(AM_BROADCAST_ADDR, &packet, sizeof(test_serial_msg_t)) == SUCCESS) {
	locked = TRUE;
      }
    }
	*/
	
	printf("the number of transmission is %u.\n",m_numOfTransmission);
	printf("the number of successful transmission is %u.\n",m_numOfSuccess);
	printfFloat(m_beta);
	printfFloat(m_delta);
	printfFloat(m_PSR);
	printfFloat(m_traffic);
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
