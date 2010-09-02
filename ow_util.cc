/* ======================================================================== *
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

/*
 *  Copyright (C) 2010 Carnegie Mellon University
 *                     Attn: Mike Vande Weghe
 *                     Robotics Institute
 *                     5000 Forbes Ave NSH 4000B
 *                     Pittsburgh, PA 15213
 */


/*==============================*
 * INCLUDES - System Files      *
 *==============================*/
#include <syslog.h>
#include <signal.h>
#include <pthread.h>
#include <errno.h>
#include <curses.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <math.h>
#include <sys/io.h>

#include "CANbus.cc"
#include "Group.cc"
#include "Puck.cc"

/*==============================*
 * PRIVATE Function Prototypes  *
 *==============================*/
void RenderMAIN_SCREEN(void);
void ProcessInput(int c);
void DisplayThread(void);
void clearScreen(void);
void finish_entry(void);
void start_entry(void);
void init_ncurses(void);
void handleMenu(int argc, char **argv);

void enumeratePucks(void *data);
void activePuck(void *data);
void firmwarePuck(void *data); int firmwareDL(int id, char *fn);
void terminalMode(void *data);
void watchProperty(void *data);
void setDefaults(void *data); void paramDefaults(int newID,int targID);
void hallCheck(void *data);
void tensionCable(void *data);
void findOffset(void *data);
void exitProgram(void *data);


/*==============================*
 * Functions                    *
 *==============================*/


void setMofst(CANbus *bus, int newID)
{
  int32_t dat, vers;
   int dummy, i, samples = 1024;
   
   long max, min;
   double sumX, sumX2, mean, stdev;
   int err = 0;
   
   if (bus->get_property_rt(newID,VERS,&vers,10000) != OW_SUCCESS) {
     printf("could not get puck version\n");
     exit(1);
   }
   printf("Puck ID %ld VERS=%ld\n",newID,vers);

   if (bus->get_property_rt(newID,IOFST,&dat,10000) != OW_SUCCESS) {
     printf("Could not get old IOFST\n");
     exit(1);
   }
   printf("The old IOFST was: %ld\n",dat);
   
   // Get a valid IOFST
   #define IOFST_MIN (1950)
   #define IOFST_MAX (2180)
   #define IOFST_STDEV (10.0)
   
   // Collect stats
   sumX = sumX2 = 0;
   max = -2E9;
   min = +2E9;
   for(i = 0; i < samples; i++){
     if (bus->set_property_rt(newID,FIND,IOFST,false) != OW_SUCCESS) {
       printf("Could not set IOFST (iteration %d)\n",i);
       --i;
       continue;
     }
     usleep(100);
     if (bus->get_property_rt(newID,IOFST,&dat,10000) != OW_SUCCESS) {
       printf("Could not get new IOFST (iteration %d)\n",i);
       --i;
       continue;
     }
     if(dat > max) max = dat;
     if(dat < min) min = dat;
     sumX += dat;
     sumX2 += dat * dat;
     usleep(1000);
   }
   mean = 1.0 * sumX / samples;
   stdev = sqrt((1.0 * samples * sumX2 - sumX * sumX) / (samples * samples - samples));
   printf("MIN IOFST = %ld\n", min);
   if(min < IOFST_MIN){
      printf(" -- FAIL");
      ++err;
   } 
   printf("MAX IOFST = %ld\n", max);
   if(max > IOFST_MAX){
      printf(" -- FAIL");
      ++err;
   }
   printf("MEAN IOFST = %.2f\n", mean);
   printf("STDEV IOFST = %.2f\n", stdev);
   if(stdev > IOFST_STDEV){
      printf(" -- FAIL");
      ++err;
   }
   if (bus->set_property_rt(newID, IOFST, (long)mean, false) != OW_SUCCESS) {
     printf("Could not set new IOFST\n");
     exit(1);
   }
   usleep(100);
   printf("The new IOFST is:%d\n",(int)mean);
   
   if(!err){
     if (bus->set_property_rt(newID,MODE,MODE_TORQUE,false) != OW_SUCCESS) {
       printf("Could not switch to MODE_TORQUE\n");
       exit(1);
     }
     usleep(100);
     if (bus->get_property_rt(newID,MOFST,&dat,10000) != OW_SUCCESS) {
       printf("Could not get old MOFST\n");
       exit(1);
     }
     printf("The old MOFST was:%d\n",dat);
   
     if(vers <= 39){
       bus->set_property_rt(newID,ADDR,32971,false);
       usleep(100);
       bus->set_property_rt(newID,VALUE,1,false);
       usleep(100);
     }else{
       if (bus->set_property_rt(newID,FIND,MOFST,false) != OW_SUCCESS) {
	 printf("Could not FIND MOFST\n");
	 exit(1);
       }
     }
     printf("Please wait (10 sec)...\n");
     usleep(10000000); // Sleep for 10 sec
     if(vers <= 39){
       bus->set_property_rt(newID,ADDR,32970,false);
       usleep(100);
       bus->get_property_rt(newID,VALUE,&dat);
      }else{
       if (bus->get_property_rt(newID,MOFST,&dat,10000) != OW_SUCCESS) {
	 printf("Could not get new MOFST\n");
	 exit(1);
       }
      }
      printf("The new MOFST is:%d\n",dat);
      if (bus->set_property_rt(newID,MODE,MODE_IDLE,false) != OW_SUCCESS) {
	printf("Could not switch to MODE_IDLE\n");
	exit(1);
      }
      usleep(100);
      if(vers <= 39){
	bus->set_property_rt(newID,MOFST,dat,false);
	bus->set_property_rt(newID,SAVE,MOFST,false);
      }
   }
   
   printf("\nDone. ");
   printf("\n");
}

int main( int argc, char **argv )
{
  CANbus bus(41,7);
  if (bus.open() == OW_FAILURE) {
    printf("Unable to open CANbus device\n");
    exit(1);
  }

  if (bus.check() == OW_FAILURE) {
    printf("Unable to communicate with the WAM.\n");
    exit(1);
  }
   
  setMofst(&bus,3);
  exit(0);
}
