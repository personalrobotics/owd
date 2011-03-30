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

int firmwareDL(int id, char *fn)
{
   FILE *fp;
   int in_id;
   int in_len;
   unsigned char in_data[8];
   unsigned char out_Data[8];
   int i;
   int cnt = 1;
   char line[100];
   int rcpt;
   int lineTotal, lineCt;

   // Open the file
   if((fp=fopen(fn, "r"))==NULL) {
      return(1);
   }

   // Count the lines
   lineTotal = 0;
   while(fgets(line, 99, fp) != NULL)
      ++lineTotal;

   // Reset the file pointer
   fclose(fp);
   if((fp=fopen(fn, "r"))==NULL) {
      return(1);
   }

   setPropertySlow(0, id, 5, FALSE, 0L); // Reset
   usleep(1000000); // Wait a sec
   setPropertySlow(0, id, 0, FALSE, 0x000000AA);
   // For each line in the file
   //sendData[0] = 0x80 | VERS;
   //sendData[1] = 0x00;
   lineCt = 0;
   if(!curses) printf("\n\n");
   while(fgets(line, 99, fp) != NULL) {
      i = 1;
      ++lineCt;
      //printf("%s", line);
      if(curses){
	mvprintw(entryLine, 1, "Download progress: %d%%", (int)(100.0 * lineCt / lineTotal));

      }else{
         printf("\rDownload progress: %d%%   ", (int)(100.0 * lineCt /
               lineTotal));
         fflush(stdout);
      }
      while(line[i] >= '0') {
         // Wait for n "Get VERS"
         for(rcpt = 0; rcpt < cnt; rcpt++) {
            while(canReadMsg(0, &in_id, &in_len, in_data, 1))
               usleep(100);
         }
         // Send the byte
         out_Data[0] = line[i];
         canSendMsg(0, id, 1, out_Data, 1);
         ++i;
      }
   }
   fclose(fp);
   if(curses)
     mvprintw(entryLine, 1, "Download complete!               ");
   else
      printf("\nDownload complete!   ");

   return(0);
}
