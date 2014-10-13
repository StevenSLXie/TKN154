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
  uint8_t nodeType = 2;
  uint8_t timerFlag = 0;


  void startApp();
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

  void startApp()
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
  
  uint16_t getRandomNumber(uint8_t bias){
		uint16_t res = call Random.rand16();
		uint16_t mask = 0xFFFF;
		mask <<= bias;
		mask = ~mask;
		res &= mask;
		return res;
  }
  
  async event void TimerSendPac.fired(){
	 
	//call TimerSendPac.start(0.9*period+getRandomNumber(0.2*period));
	call TimerSendPac.start(period);
	post packetSendTask();
	 
  }
  
  float fromPSRToBeta(float PSR){
	  float betaCan = 0;
	  float PSRCan = 0;
	  
	  float betaBest = 0;
	  float dif = 100000;
	  
	  if(PSR < 0.8)
		  return 0.5;
      else if (PSR <= 0.90){
		  for(int i =0;i<= 20;i++){
			   betaCan = 0.30 + 0.01*i;
			   PSRCan = (1-betaCan*betaCan*betaCan*betaCan*betaCan)*(1-betaCan/(1-betaCan)/6.0);
			   
			   if((PSRCan - PSR) < dif || (PSR - PSRCan) < dif){
					dif = ((PSRCan - PSR)>0)? (PSRCan - PSR):(PSR - PSRCan);
					betaBest = betaCan;
			   }
			   
			   if(dif < 0.005){
					return betaBest;	
			   }
		  }
		  return betaBest;
	  }
	  else {
		  for(int i =0;i<= 38;i++){
			   betaCan = 0.01*i;
			   PSRCan = (1-betaCan*betaCan*betaCan*betaCan*betaCan)*(1-betaCan/(1-betaCan)/6.0);
			   
			   if((PSRCan - PSR) < dif || (PSR - PSRCan) < dif){
					dif = ((PSRCan - PSR)>0)? (PSRCan - PSR):(PSR - PSRCan);
					betaBest = betaCan;
			   }
			   
			   if(dif < 0.005){
					return betaBest;	
			   }		   
		  }
		  return betaCan;
	  }
	
  }
  
  float fromBetaToTraffic(float beta){
		return beta/(1-beta*beta*beta*beta*beta)/6.0;
  }
  
  float findOptimalBeta(float traffic){
	  float betaCan;
	  float trafCan;
	  
	  float betaBest;
	  float dif = 10000;
	  
	  if (traffic > 0.085)
		  return 0.7;
      else if (traffic <= 0.05){
		 for(int i =0;i<= 30;i++){
			 betaCan = 0.01*i;
			 trafCan = (betaCan-2*betaCan*betaCan-betaCan*betaCan*5.0+5.0*betaCan)/36.0/(1-betaCan);
			 
			 if((trafCan - traffic) < 0.005 || (PSR - PSRCan) < 0.005){
				dif = ((trafCan - traffic)>0)? (trafCan - traffic):(traffic - trafCan);
				betaBest = betaCan;
			 }
			 
			 if(dif < 0.002){
					return betaBest;	
			 }			   
		 }
		 return betaBest;
	  }
	  
	  else{
		 for(int i =0;i<= 40;i++){
			 betaCan = 0.2+0.01*i;
			 trafCan = (betaCan-2*betaCan*betaCan-betaCan*betaCan*5.0+5.0*betaCan)/36.0/(1-betaCan);
			 
			 if((trafCan - traffic) < 0.005 || (PSR - PSRCan) < 0.005){
				dif = ((trafCan - traffic)>0)? (trafCan - traffic):(traffic - trafCan);
				betaBest = betaCan;
			 }
			 
			 if(dif < 0.002){
					return betaBest;	
			 }				   
		 }
		 return betaBest;
	  }
	  
  }
  
  float findOptimalDelta(float traffic, float beta){
		return beta/(1-beta*beta*beta*beta*beta)/(traffic)/6.0;
  }
  
  event void TimerChgPrd.fired(){
    
	/*
	if(timerFlag < 3){
		timerFlag++;
		call TimerChgPrd.startOneShot(60000);
		return;
	}
	*/
	
	interval++;
	
	float PSR_b = 0;
	
	PSR_b = (float)m_numOfSuccess/(float)m_numOfTransmission;
	
	
	if(1 == interval){
		m_PSR = PSR_b;  // calculate the PSR
		m_beta = fromPSRToBeta(m_PSR);
		m_traffic = fromBetaToTraffic(m_beta);
		m_beta = findOptimalBeta(m_traffic);
		m_delta = findOptimalDelta(m_traffic,m_beta);
		period = (float)thu/m_delta;
	}
	
	if((PSR_b-m_PSR)<0.03 || (m_PSR-PSR_b)<0.03){
	
	}
	else if((PSR_b-m_PSR)>=0.03){
		float curTraf = 0
		m_PSR = PSR_b;  // calculate the PSR
		m_beta = fromPSRToBeta(m_PSR);
		curTraf = fromBetaToTraffic(m_beta);
		curTraf -= (m_delta*m_traffic);
		m_traffic += curTraf;
		m_beta = findOptimalBeta(m_traffic);
		m_delta = findOptimalDelta(m_traffic,m_beta);
		period = (float)thu/m_delta;
	
	}
	else if((m_PSR-PSR_b)>=0.03){
		m_PSR = PSR_b;  // calculate the PSR
		m_beta = fromPSRToBeta(m_PSR);
		m_traffic = fromBetaToTraffic(m_beta);
		m_traffic /= m_delta;
		m_beta = findOptimalBeta(m_traffic);
		m_delta = findOptimalDelta(m_traffic,m_beta);
		period = (float)thu/m_delta;
		
	}
	m_numOfSuccess = 0;
	m_numOfTransmission = 0;
    call TimerChgPrd.startOneShot(60000);

  }
  
  
  
  void sendToSerial() {
    //counter++;
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
      call Leds.led0On();
	
	}
	else{
		call Leds.led0Off();
		m_numOfTransmission++;
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
	  m_numOfSuccess++;
	  call Leds.led1Toggle();
	  if(m_numOfTransmission%100 == 0){
		sendToSerial();
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
