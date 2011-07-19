
void setMofst(int newID)
{
   long dat, vers;
   int dummy, i, samples = 1024;

   long max, min;
   double sumX, sumX2, mean, stdev;
   int err = 0;

   wakePuck(0,newID);
   getProperty(0,newID,VERS,&vers);

   getProperty(0,newID,IOFST,&dat);
   printf("\nThe old IOFST was: %d",dat);

   // Get a valid IOFST
   #define IOFST_MIN (1800)
   #define IOFST_MAX (2230)
   #define IOFST_STDEV (15.0)

   // Collect stats
   sumX = sumX2 = 0;
   max = -2E9;
   min = +2E9;
   for(i = 0; i < samples; i++){
      setPropertySlow(0,newID,FIND,0,IOFST);
      getProperty(0,newID,IOFST,&dat);
      if(dat > max) max = dat;
      if(dat < min) min = dat;
      sumX += dat;
      sumX2 += dat * dat;
      usleep(1000000/samples);
   }
   mean = 1.0 * sumX / samples;
   stdev = sqrt((1.0 * samples * sumX2 - sumX * sumX) / (samples * samples - samples));
   printf("\nMIN IOFST = %ld", min);
   if(min < IOFST_MIN){
      printf(" -- FAIL");
      ++err;
   }
   printf("\nMAX IOFST = %ld", max);
   if(max > IOFST_MAX){
      printf(" -- FAIL");
      ++err;
   }
   printf("\nMEAN IOFST = %.2f", mean);
   printf("\nSTDEV IOFST = %.2f", stdev);
   if(stdev > IOFST_STDEV){
      printf(" -- FAIL");
      ++err;
   }
   setPropertySlow(0, newID, IOFST, 0, (long)mean);
   printf("\nThe new IOFST is:%d\n",(int)mean);

   if(!err){
      setPropertySlow(0,newID,MODE,0,MODE_TORQUE);
      getProperty(0,newID,MOFST,&dat);
      printf("\nThe old MOFST was:%d\n",dat);

      if(vers <= 39){
         setPropertySlow(0,newID,ADDR,0,32971);
         setPropertySlow(0,newID,VALUE,0,1);
      }else{
         setPropertySlow(0,newID,FIND,0,MOFST);
      }
      //printf("\nPress enter when the index pulse is found: ");
      //mygetch();
      //mygetch();
      printf("\nPlease wait (10 sec)...\n");
      usleep(10000000); // Sleep for 10 sec
      if(vers <= 39){
         setPropertySlow(0,newID,ADDR,0,32970);
         getProperty(0,newID,VALUE,&dat);
      }else{
         getProperty(0,newID,MOFST,&dat);
      }
      printf("\nThe new MOFST is:%d\n",dat);
      setPropertySlow(0,newID,MODE,0,MODE_IDLE);
      if(vers <= 39){
         setPropertySlow(0,newID,MOFST,0,dat);
         setPropertySlow(0,newID,SAVE,0,MOFST);
      }
   }

   printf("\nDone. ");
   printf("\n");
}
