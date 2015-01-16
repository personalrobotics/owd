// Taken from btclient/src/btutil/main.c

/*************************************************************************
 *  Copyright (C) 2003-2008 Barrett Technology, Inc. <support@barrett.com>
 *                          625 Mount Auburn St
 *                          Cambridge, MA 02138, USA
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY BARRETT TECHNOLOGY, INC AND CONTRIBUTORS
 *  ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL BARRETT
 *  TECHNOLOGY, INC OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 *  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 *  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  The views and conclusions contained in the software and documentation
 *  are those of the authors and should not be interpreted as representing
 *  official policies, either expressed or implied, of Barrett Technology.
 *                                                                          *
 * ======================================================================== */

#include <stdio.h>


struct defaultStruct
{
   int *key;
   long val;
};

struct defaultStruct taterDefs[] =
{
   &TIE, 0,
   &ACCEL, 100,
   &AP, 0,
   &CT, 4096,
   &OT, 0,
   &CTS, 4096,
   &DP, 0,
   &MV, 100,
   &MCV, 100,
   &MOV, 100,
   &OT, 0,
   &HOLD, 0,
   &TSTOP, 0,
   &OTEMP, 60,
   &PTEMP, 0,
   &_DS, -256,
   &KP, 2000,
   &KD, 8000,
   &KI, 0,

   NULL, 0
};
long wamDefaultMT[] = {4860, 4860, 4860, 4320, 3900, 3900, 1600};

struct defaultStruct bh8Defs[] =
{
   &TIE, 0,
   &ACCEL, 200,
   &AP, 0,
   &CT, 195e3,
   &OT, 0,
   &CTS, 4096,
   &DP, 45e3,
   &MT, 2200,
   &MV, 200,
   &MCV, 200,
   &MOV, 200,
   &OT, 0,
   &HOLD, 0,
   &TSTOP, 50,
   &OTEMP, 60,
   &PTEMP, 0,
   &POLES, 6,
   &IKI, 204,
   &IKP, 500,
   &IKCOR, 102,
   &IOFF, 0,
   &IVEL, -75,
   &_DS, 25600,
   &KP, 500,
   &KD, 2500,
   &KI, 0,
   &IPNM, 20000,
   &HSG, 0,
   &LSG, 0,
   &GRPA, 0,
   &GRPB, 7,
   &GRPC, 5,

   NULL, 0
};

struct defaultStruct wraptorDefs[] =
{
   &TIE, 0,
   &ACCEL, 100,
   &AP, 0,
   &CT, 750,
   &CTS, 4096,
   &DP, 0,
   &MT, 990,
   &MV, 1500,
   &MCV, 1500,
   &MOV, 1500,
   &DP, 0,
   &OT, 0,
   &CT, 1E6,
   &_DS, 2560,
   &KP, 2000,
   &KD, 8000,
   &KI, 0,
   &IKP, 4096,
   &IKI, 819,
   &IKCOR, 819,
   &GRPA, 0,
   &GRPB, 1,
   &GRPC, 4,
   &POLES, 6,
   &IPNM, 20000,

   NULL, 0
};

struct defaultStruct safetyDefs[] =
{
   &TIE, 0,
   &VOLTL1, 36,
   &VOLTL2, 30,
   &VOLTH1, 54,
   &VOLTH2, 57,
   &GRPA, 1,
   &GRPB, 2,
   &GRPC, 3,

   NULL, 0
};

void paramDefaults(int newID,int targID)
{
   int i;
   long vers, role;

   wakePuck(0,newID);
   getProperty(0, newID, VERS, &vers);
   initPropertyDefs(vers);
   getProperty(0, newID, ROLE, &role);

   switch(role & 0x001F){
      case ROLE_TATER:
      case ROLE_BHAND:
      if(targID >= 1 && targID <= 7) { // WAM pucks
          for(i = 0; taterDefs[i].key; i++){
             setPropertySlow(0, newID, *taterDefs[i].key, 1, taterDefs[i].val);
          }
          setPropertySlow(0, newID, MT, 1, wamDefaultMT[targID-1]);
      }
      //if(role & 0x0100)
      //   setPropertySlow(0, newID, CTS, 0, 4096);

      if(targID <= 4) { //4DOF
         setPropertySlow(0,newID,IKCOR,1,1638);
         setPropertySlow(0,newID,IKP,1,8192);
         setPropertySlow(0,newID,IKI,1,3276);
         setPropertySlow(0,newID,IPNM,1,2700); //2755);
         //setPropertySlow(0,newID,IPNM,0,2562);//2755);
         setPropertySlow(0,newID,POLES,1,12);
         setPropertySlow(0,newID,GRPA,1,0);
         setPropertySlow(0,newID,GRPB,1,1);
         setPropertySlow(0,newID,GRPC,1,4);

	 setPropertySlow(0,newID,JIDX,1,targID);
         setPropertySlow(0,newID,PIDX,1,targID);
         
     } else if(targID <= 7) { //Wrist
         setPropertySlow(0,newID,IKCOR,1,819);
         setPropertySlow(0,newID,IKP,1,4096);
         setPropertySlow(0,newID,IKI,1,819);
         setPropertySlow(0,newID,GRPA,1,0);
         setPropertySlow(0,newID,GRPB,1,2);
         setPropertySlow(0,newID,GRPC,1,4);
	 setPropertySlow(0,newID,JIDX,1,targID);
         if(targID != 7) {
	//	 if(1) { // Temp fix for WK Song
	   setPropertySlow(0,newID,IPNM,1,6500);
	   //setPropertySlow(0,newID,IPNM,0,4961);
	   setPropertySlow(0,newID,POLES,1,8);
         } else {
	   setPropertySlow(0,newID,IPNM,1,17474);
	   setPropertySlow(0,newID,POLES,1,6);
         }
         
         setPropertySlow(0,newID,PIDX,1,targID-4);
      }

      if(targID >= 11 && targID <= 14){ // BH8-280
          for(i = 0; bh8Defs[i].key; i++){
	    setPropertySlow(0, newID, *bh8Defs[i].key, 1, bh8Defs[i].val);
          }
	  setPropertySlow(0,newID,JIDX,0,targID-3);
          if(targID == 14) { // Spread on BH8-280
              setPropertySlow(0,newID,CT,1,35950);
              setPropertySlow(0,newID,DP,1,17975);
              setPropertySlow(0,newID,MV,1,50);
              setPropertySlow(0,newID,HSG,1,0);
              setPropertySlow(0,newID,LSG,1,0);
              setPropertySlow(0,newID,HOLD,1,1);
              setPropertySlow(0,newID,TSTOP,1,150);
              setPropertySlow(0,newID,KP,1,1000);
              setPropertySlow(0,newID,KD,1,10000);
            }
            
	  setPropertySlow(0,newID,PIDX,1,targID-10);
         
         // set IHIT
         if (vers >= 175) {
         	if (targID == 14) {
	         	setPropertySlow(0,newID,108,1, 2200);  // spread
         	} else {
	         	setPropertySlow(0,newID,108,1, 1700);  // fingers
         	}
         }
      }

      break;

      case ROLE_SAFETY:
      for(i = 0; safetyDefs[i].key; i++){
         setPropertySlow(0, newID, *safetyDefs[i].key, 1, safetyDefs[i].val);

      }

      setPropertySlow(0, newID, SAFE, 0, 4);
      setPropertySlow(0, newID, SAFE, 0, 5);
      usleep(1000000); // Wait a sec
      setPropertySlow(0, newID, FIND, 0, VBUS);
      usleep(1000000); // Wait a sec
      setPropertySlow(0, newID, SAFE, 0, 0);

      break;

      case ROLE_WRAPTOR:
      for(i = 0; wraptorDefs[i].key; i++){
         setPropertySlow(0, newID, *wraptorDefs[i].key, 1, wraptorDefs[i].val);

      }

      if(targID < 4){
         // Set inner link parameters

      }else if(targID == 4){
         // Set spread puck parameters: KP=1500 KD=KI=0, ACCEL=10, MV=20, DP=18500, CT=37000, HOLD=1

      }else if(targID > 4){
         // Set outer link parameters

      }
      break;

      default:

      break;

   }

   setPropertySlow(0,newID,SAVE,0,-1); // Save All

   /*

    else if(targID <= 8) {//Gimbals
      setPropertySlow(0,newID,CTS,0,1);
      setPropertySlow(0,newID,OFFSET1,0,-11447);
      setPropertySlow(0,newID,OFFSET2,0,-19834);
      setPropertySlow(0,newID,OFFSET3,0,-12606);
      setPropertySlow(0,newID,GAIN1,0,10981);
      setPropertySlow(0,newID,GAIN2,0,27566);
      setPropertySlow(0,newID,GAIN3,0,26782);
      setPropertySlow(0,newID,GRPA,0,0);
      setPropertySlow(0,newID,GRPB,0,2);
      setPropertySlow(0,newID,GRPC,0,5);
   }
*/

}
