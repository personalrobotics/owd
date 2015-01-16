/* changeID for Puck Monitor vers 5+ */
void changeID(int oldID, int newID, int role)
{
   int _LOCK, _SAVE;

   setPropertySlow(0, oldID, 5, 0, 0); /* RESET back to Monitor */
   usleep(2000000); /* Wait 2s */

   if(oldID == 10){ /* Safety puck's LOCK/SAVE commands are different */
      _LOCK = 13; _SAVE = 30;
   }else{
      _LOCK = 8; _SAVE = 9;
   }
   /* Unlock the ID/ROLE for writing */
   setPropertySlow(0, oldID, _LOCK, 0, 18384);
   setPropertySlow(0, oldID, _LOCK, 0, 23);
   setPropertySlow(0, oldID, _LOCK, 0, 3145);
   setPropertySlow(0, oldID, _LOCK, 0, 1024);
   setPropertySlow(0, oldID, _LOCK, 0, 1);

   /* Set the ROLE */
   if(role >= 0){
      setPropertySlow(0, oldID, 1, 0, role);
      setPropertySlow(0, oldID, _SAVE, 0, 1); usleep(2000000);
   }
   setPropertySlow(0, oldID, 3, 0, newID); /* Set the new ID */
   setPropertySlow(0, oldID, _SAVE, 0, 3); usleep(2000000); /* Save the new values to EEPROM */

   /* Reset to load the new ID */
   setPropertySlow(0, oldID, 5, 0, STATUS_RESET);
}

