// send current values to display or cell
void summarizeSignals(){
  Serial.println("Mean bands");
  for (int i=0; i<4; i++){
    if(meanBand[i]>0.00001){
      float spectrumLevel = 20*log10(meanBand[i] / fftCount) - (10 * log10(binwidth));
      spectrumLevel = spectrumLevel - hydroCal - gainDb;
      Serial.println(spectrumLevel);
    }
     else{
      Serial.println(meanBand[i] / fftCount);
     }
  }
}

// dataPacket should look like this for AWS
//  String data = '1536368460;300;26.4321;-82.3476;w:12;0:75;1:65;2:52;3:48;z:3.2'
//                  UNIX time;duration (s);lat;lon;
void makeDataPacket(){
  float spectrumLevel;
  int iSpectrumLevel;
  time_t packetTime = stopTime - rec_dur;
  //char dateTime[50];
  //sprintf(dateTime,"%04d%02d%02dT%02d%02d%02d", year(packetTime), month(packetTime), day(packetTime), hour(packetTime), minute(packetTime), second(packetTime));
  
  // 31 Bytes:   dateTime,duration,lat,long
  dataPacket = "";
  #ifdef SWARM_MODEM
    dataPacket = "$TD \"";
  #endif
  dataPacket += packetTime;
  dataPacket += ";";
  dataPacket += rec_dur;
  dataPacket += ";";
  dataPacket += String(latitude, 4);
  dataPacket += ";";
  dataPacket += String(longitude, 4);
  
  dataPacket += ";";
  for (int i=0; i<NBANDS; i++){
      if(meanBand[i]>0.00001){
        spectrumLevel = 20*log10(meanBand[i] / fftCount) - (10 * log10(binwidth)); 
      }
      else{
        spectrumLevel = -96 - (10 * log10(binwidth));
       }
      spectrumLevel = spectrumLevel - hydroCal - gainDb;
      iSpectrumLevel = (int) spectrumLevel;
      dataPacket += i;
      dataPacket += ":";
      dataPacket += iSpectrumLevel;
      dataPacket += ";";
   }

  dataPacket += "v:";
  dataPacket += String(voltage, 1);

//  dataPacket += ";t:";
//  dataPacket += String((int) temperature);

  #ifdef PI_PROCESSING
    dataPacket += ";";
    dataPacket += String(piPayload);
    dataPacket.trim(); // remove /n
  #endif
  #ifdef SWARM_MODEM
    dataPacket += "\"";
    uint8_t checksum = nmeaChecksum(&dataPacket[0], dataPacket.length());
    dataPacket += "*";
    if(checksum<17) dataPacket += "0";
    dataPacket += String(checksum, HEX);
    Serial.print("Swarm ");
  #endif
    
   Serial.println(dataPacket);
}


int sendDataPacket(){
  int err = 0;
  //int err = modem.sendSBDText("20180817T140000;26.4321;-82.3476;g:12;nh:[70,10,1,4]");
  if(introPeriod){
    cDisplay();
    display.println("MSG");
    display.println(dataPacket);
    display.display();
  }

  #ifdef SWARM_MODEM
    Serial1.println(dataPacket);
    return 1;
  #endif
    

  #ifdef IRIDIUM_MODEM
    err = modem.sendSBDText(&dataPacket[0]);
    if (err != ISBD_SUCCESS)
    {
      Serial.print("sendSBDText error: ");
      Serial.println(err);
      if(introPeriod){
        cDisplay();
        display.println("");
        display.println("Send fail");
        display.display();
      }
  
      if (err == ISBD_SENDRECEIVE_TIMEOUT){
        Serial.println("Send timeout");
        if(introPeriod){
            display.println("Timeout");
            display.display();
        }
      }
    }
    else{
      if(introPeriod){
        cDisplay();
        display.println("");
        display.println("Msg sent");
        display.display();
      }
  
      Serial.println("Msg Sent");
    }
  #endif
  return err;
}

void resetSignals(){
  for (int i=0; i<NBANDS; i++){
    meanBand[i] = 0;
  }
  fftCount = 0;
}

uint8_t nmeaChecksum(const char *sz, size_t len){
  size_t i = 0;
  uint8_t cs;
  if(sz[0]=='$') i++;
  for(cs = 0; (i<len)&& sz[i]; i++){
    cs ^=((uint8_t) sz[i]);
  }
  return cs;
}
