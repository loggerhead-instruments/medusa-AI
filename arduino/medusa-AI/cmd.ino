#define CMD(a,b) ( a + (b << 8))
#define TRUE 1
#define FALSE 0

int ProcCmd(char *pCmd)
{
  short *pCV;
  short n;
  long lv1, lv2;
  char s[22];
        unsigned int tday;
        unsigned int tmonth;
        unsigned int tyear;
        unsigned int thour;
        unsigned int tmin;
        unsigned int tsec;

  pCV = (short*)pCmd;

  n = strlen(pCmd);
  if(n<2) 
          return TRUE;

  switch(*pCV)
  {                     

  // Accelerometer full scale
    case ('A' + ('G'<<8)):
    {
      sscanf(&pCmd[3],"%d",&lv1);
      accel_scale = lv1;
      break;
    }

  case ('I' + ('M'<<8)):
    {
      sscanf(&pCmd[3],"%d",&lv1);
      imuFlag = lv1;
      break;
    }

  // Hydrophone sensitivity if not default -180
    case ('H' + ('L'<<8)):
    {
      sscanf(&pCmd[3],"%d",&lv1);
      hydroCalLeft = lv1;
      break;
    }

  // Hydrophone sensitivity if not default -180
    case ('H' + ('R'<<8)):
    {
      sscanf(&pCmd[3],"%d",&lv1);
      hydroCalRight = lv1;
      break;
    }
    // Disable LEDs
    case ('L' + ('D'<<8)):
    {
      LEDSON = 0;
      break;
    }
    

    // Set of Real Time Clock
    case ('T' + ('M'<<8)):
    {
         //set time
         sscanf(&pCmd[3],"%d-%d-%d %d:%d:%d",&tyear,&tmonth,&tday,&thour,&tmin,&tsec);
         setTeensyTime(thour, tmin, tsec, tday, tmonth, tyear + 2000);
         Serial.print("Clock Set (now): ");
         Serial.println(now());
         Serial.print("Clock set (getTeensyTime): ");
         Serial.println(getTeensy3Time());
         break;
      }

      case ('N' + ('D'<<8)):
      {
        sscanf(&pCmd[3],"%d",&lv1);
        noDC = lv1;
        break;
      }

    case ('D' + ('I'<<8)):
      {
        printDiags = 1;
        break;
      }
      
      case ('R' + ('D'<<8)):
      {
        sscanf(&pCmd[3],"%d",&lv1);
        rec_dur = lv1;
        analyze_dur = rec_dur;
        break;
      }
      
      case ('R' + ('I'<<8)):
      {
        sscanf(&pCmd[3],"%d",&lv1);
        rec_int = lv1;
        if (rec_int<30) rec_int = 30;
        break;
      } 

      case ('A' + ('D'<<8)):{
        sscanf(&pCmd[3],"%d",&lv1);
        analyze_dur = lv1;
        break;
      }

      case ('M' + ('I'<<8)):
      {
        sscanf(&pCmd[3],"%d",&lv1);
        msg_int = lv1;
        break;
      }

      case ('S' + ('G'<<8)):
      {
        sscanf(&pCmd[3],"%d",&lv1);
        gainSetting = lv1;
        break;
      } 

    // 0 is 8 kHz; 1 is 16 kHz; 2 is 32 kHz; 3 is 44.1 kHz; 4 is 48 kHz; 5 is 96 kHz;
      case ('H' + ('Z'<<8)):
      {
        sscanf(&pCmd[3],"%d",&lv1);
        switch(lv1){
          case(8000):{
            isf=0;
            audio_srate = 8000;
            break;
          }
          case(16000):{
            isf=1;
            audio_srate = 16000;
            break;
          }
          case(32000):{
            isf=2;
            audio_srate = 32000;
            break;
          }
          case(44100):{
            isf=3;
            audio_srate = 44100;
            break;
          }
          case(48000):{
            isf=4;
            audio_srate = 48000;
            break;
          }
          case(96000):{
            isf=5;
            audio_srate = 96000;
            break;
          }
        }
        break;
      }
  } 
  return TRUE;
}

boolean LoadScript()
{
  char s[30];
  char c;
  short i;
  int j = 0;

  File file;
  unsigned long TM_byte;
  int comment_TM = 0;

  // Read card setup.txt file to set date and time, recording interval
  file=sd.open("setup.txt");
 if(file)
 {
   do{
        i = 0;
        s[i] = 0;
        do{
            c = file.read();
            if(c!='\r') s[i++] = c;
            if((c=='T') & (i==1)) 
            {
              TM_byte = file.position() - 1;
              comment_TM = 1;
            }
            if(i>29) break;
          }while(c!='\n');
          s[--i] = 0;
          if(s[0] != '/' && i>1)
          {
            ProcCmd(s);
          }
      }while(file.available());
      file.close();  
      
      // comment out TM line if it exists
      if (comment_TM)
      {
        Serial.print("Comment TM ");
        Serial.println(TM_byte);
        file = sd.open("setup.txt", FILE_WRITE);
        file.seek(TM_byte);
        file.print("//");
        file.close();
      }
      
  }
  else
  {   
    Serial.println("setup.txt not opened");
  }
//    // Read list of bands
//  file = sd.open("bands.txt");
//  Serial.println("Bands file");
//  for(i=0; i<30; i++){
//    s[i]=NULL;
//  }
//  i=0;
//  if(file){
//    do{
//      j = 0;
//      do{ // scan next line
//        c = file.read();
//        
//        if(c!='\r') s[j] = c;
//        j++;
//        if(j>29) break;
//      }while(c!='\n');
//
//      Serial.println("Scan");
//      Serial.println(s);
//      sscanf(s,"%d,%d",&bandLow[i], &bandHigh[i]);
//      Serial.print(i); Serial.print(" ");
//      Serial.print(bandLow[i]);
//      Serial.print(" ");
//      Serial.println(bandHigh[i]);
//      i++;
//      if(i==MAXBANDS) break;
//    }while(file.available());
//    NBANDS = i;
//    Serial.print("NBANDS:");
//    Serial.println(NBANDS);
//    file.close();
//  }
  
 return 1;  
}
