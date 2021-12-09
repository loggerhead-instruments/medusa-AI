float mAmpRec = 50.0;
float mAmpSleep = 5.0;
float mAmpTransmit = 250.0;
int nBatPacks = 8;
float mAhPerBat = 4500.0; // assume 4.5Ah per LiPo; rated at 5 Ah

uint32_t freeMB;
uint32_t filesPerCard;
csd_t m_csd;


void updatePowerDuration(){
// get free space on cards
    uint32_t freeMB;
    cDisplay();
    display.println();
    display.println("Card Free/Total MB");

    sd.card()->readCSD(&m_csd);
    
    //uint32_t volFree = sd.FsVolume.freeClusterCount();
    uint32_t volMB = uint32_t ( 0.000512 * sdCardCapacity(&m_csd));    
    
    Serial.print("Volume (MB): ");
    Serial.println((uint32_t) volMB);

    if (volMB < 200) freeMB = 0;
    else
      freeMB = volMB - 200; // take off 200 MB to be safe
    
    display.print(freeMB);
    display.print("/");
    display.println(volMB);
    display.display();


  uint32_t totalRecSeconds = 0;
  float fileBytes = (2 * rec_dur * audio_srate) + 44;
  float fileMB = (fileBytes + 32768) / 1000 / 1000; // add cluster size so don't underestimate fileMB

  float recDraw = mAmpRec;
  float recFraction = (float) rec_dur / (float) (rec_dur + rec_int);
  float transmitFraction = 40.0 / (float) (rec_dur + rec_int);  //part of sleep time is spent transmitting....assume 40 s on average (reality is about 10s)
  float sleepFraction = 1 - recFraction - transmitFraction;  
  float avgCurrentDraw = (recDraw * recFraction) + (mAmpSleep * sleepFraction) + (mAmpTransmit * transmitFraction);

  uint32_t powerSeconds = uint32_t (3600.0 * nBatPacks * (mAhPerBat / avgCurrentDraw));

  filesPerCard = 0;
  if(freeMB==0) filesPerCard = 0;
  else{
    filesPerCard = (uint32_t) floor(freeMB / fileMB);
  }
  totalRecSeconds += (filesPerCard * rec_dur);
  float totalSecondsMemory = totalRecSeconds / recFraction;

  Serial.print("Rec dur:");
  Serial.println(rec_dur);
  Serial.print("Sleep dur:");
  Serial.println(rec_int);
  Serial.print("Rec fraction:");
  Serial.println(recFraction);
  Serial.print("Transmit fraction:");
  Serial.println(transmitFraction);
  Serial.print("Sleep fraction:");
  Serial.println(sleepFraction);
  Serial.print("Avg power mA:");
  Serial.println(avgCurrentDraw);
  
  if(powerSeconds < totalSecondsMemory){
   // displayClock(getTeensy3Time() + powerSeconds, 45, 0);
    display.setCursor(0, 46);
    display.print("Battery Limit:");
    display.print(powerSeconds / 86400);
    display.print("d");
    recDays = powerSeconds / 86400.0;
    Serial.println("Battery limit: ");
  }
  else{
  //  displayClock(getTeensy3Time() + totalRecSeconds + totalSleepSeconds, 45, 0);
    display.setCursor(0, 46);
    display.print("Memory Limit:");
    display.print(totalSecondsMemory / 86400);
    display.print("d");
    recDays = totalSecondsMemory / 86400.0;
    Serial.println("Memory Limit: ");
  }
  Serial.println(recDays);
}

