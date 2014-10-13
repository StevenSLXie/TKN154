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
 * $Revision: 1.1 $
 * $Date: 2009-05-18 16:21:55 $
 * @author: Jan Hauer <hauer@tkn.tu-berlin.de>
 * ========================================================================
 */
#include "app_profile.h"
#include "TestSerial.h"
#include "printf.h"

configuration TestDataAppC
{
	//uses{
		//interface Random;
	//}  
} implementation {
  components MainC, LedsC, Ieee802154BeaconEnabledC as MAC;
  //components new TimerMilliC() as TimerSendPac;
  //components new Msp430TimerMicroC() as TimerSendPac;
  components new AlarmMicro16C() as TimerSendPac;
  components new TimerMilliC() as TimerChgPrd;
  components TestDeviceSenderC as App;
  components SerialActiveMessageC as AM;
  components RandomC;
  components PrintfC;
  components SerialStartC;
  
  //TestPrintfC.Boot -> MAC;
  
  
  
  App.Random -> RandomC;
  
  App.Boot -> MainC.Boot;
  App.Control -> AM;
  App.TimerSendPac -> TimerSendPac;
  App.TimerChgPrd -> TimerChgPrd;
  App.AMSend -> AM.AMSend[AM_TEST_SERIAL_MSG];
  App.SerialPacket -> AM;
  
  App.MLME_SCAN -> MAC;
  App.MLME_SYNC -> MAC;
  App.MLME_BEACON_NOTIFY -> MAC;
  App.MLME_SYNC_LOSS -> MAC;
  App.MCPS_DATA -> MAC;
  App.Frame -> MAC;
  App.BeaconFrame -> MAC;
  App.MACPacket -> MAC;

  MainC.Boot <- App;
  App.Leds -> LedsC;
  App.MLME_RESET -> MAC;
  App.MLME_SET -> MAC;
  App.MLME_GET -> MAC;
  
}
