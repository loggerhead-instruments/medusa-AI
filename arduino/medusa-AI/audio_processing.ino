
// Data packet
uint16_t dataFormat = 2;


typedef struct __attribute__((__packed__)){
  uint16_t dataFormat;
  uint32_t packetStartTime;
  float latitude;
  float longitude;
  uint8_t voltage;
  uint8_t whistles;
  uint8_t band[NBANDS][NCHAN];
  uint8_t peakBand[NPEAKBANDS][NCHAN];
  uint8_t acceleration;
}iridiumStruct;

int makeSendBinaryDataPacket(){ 
   union{
     byte b[sizeof(iridiumStruct)];
     iridiumStruct binaryPacket;
   }u;

   u.binaryPacket.dataFormat = (uint16_t) dataFormat;
//   u.binaryPacket.pointsPerPacket = NCHAN * NBANDS;
   u.binaryPacket.packetStartTime = (uint32_t) packetStartTime;
   u.binaryPacket.latitude = latitude;
   u.binaryPacket.longitude = longitude;

   Serial.print("Struct size: "); Serial.println(sizeof(iridiumStruct));
 //  Serial.print("Data format: "); Serial.println(u.binaryPacket.dataFormat);
   Serial.print("Start time: "); Serial.println(u.binaryPacket.packetStartTime);
   Serial.print("lat: "); Serial.println(u.binaryPacket.latitude);
   Serial.print("lon: "); Serial.println(u.binaryPacket.longitude);
   
   u.binaryPacket.voltage = (uint8_t) (voltage * 10);
   Serial.print("V:"); Serial.println(u.binaryPacket.voltage);
   

   Serial.println("Bands.. Whistles Accel");
   u.binaryPacket.whistles = whistleCount;
   u.binaryPacket.acceleration = (uint8_t) (sdAccelZmg / 10.0);


    // band levels
    float spectrumLevel;
    for (int i=0; i<NBANDS; i++){
      if(meanBand[i][0]>0.000001){
        spectrumLevel = 20*log10(meanBand[i][0] / fftCount) - (10 * log10(binwidth)); 
      }
      else{
        spectrumLevel = -96 - (10 * log10(binwidth));
       }
      spectrumLevel = spectrumLevel - hydroCalLeft - gainDb;
      u.binaryPacket.band[i][0] = (uint8_t) spectrumLevel;
      Serial.print(u.binaryPacket.band[i][0]); Serial.print("  ");
      
      if(NCHAN==2){
       if(meanBand[i][1]>0.000001){
            spectrumLevel = 20*log10(meanBand[i][1] / fftCount) - (10 * log10(binwidth)); 
          }
          else{
            spectrumLevel = -96 - (10 * log10(binwidth));
           }
          spectrumLevel = spectrumLevel - hydroCalRight - gainDb;
          u.binaryPacket.band[i][1] = (uint8_t) spectrumLevel;
          Serial.print(u.binaryPacket.band[i][1]); Serial.print("  ");
       }
   }

   // peak band processing
   Serial.println();
   Serial.println("Peak Bands");
   for (int i=0; i<NPEAKBANDS; i++){
        u.binaryPacket.peakBand[i][0] = (uint8_t) 20*log10(peakBand[i][0]) - (10 * log10(binwidth)) - hydroCalLeft - gainDb; 
        Serial.print(u.binaryPacket.peakBand[i][0]); Serial.print("  ");
        if(NCHAN==2){
          u.binaryPacket.peakBand[i][1] = (uint8_t) 20*log10(peakBand[i][1]) - (10 * log10(binwidth)) - hydroCalRight - gainDb; 
          Serial.print(u.binaryPacket.peakBand[i][1]); Serial.print("  ");
        }
   }

    Serial.println();
    Serial.print("w:");
    Serial.print(u.binaryPacket.whistles); Serial.print("  ");
    Serial.print("a:");
    Serial.println(u.binaryPacket.acceleration);

    Serial.print("Sum Squares:"); Serial.println(sumOfSquares);
    Serial.print("Count:"); Serial.println(sumOfSquaresCount);
    Serial.print("Pos:"); Serial.println(posPeak);
    Serial.print("Neg:"); Serial.println(negPeak);

    
 //   u.binaryPacket.peakPeak = (uint8_t) 20*log10((posPeak - negPeak) / 32768.0) - hydroCalLeft - gainDb;
 //   u.binaryPacket.rms = (uint8_t) 20*log10(sqrt(sumOfSquares / sumOfSquaresCount)) - hydroCalLeft - gainDb;

//    Serial.print("peak-peak:");
//    Serial.println(u.binaryPacket.peakPeak);
//    Serial.print("rms:");
//    Serial.println(u.binaryPacket.rms);

    Serial.print("Data packet size: "); Serial.println(sizeof(u.binaryPacket));
    uint8_t iridiumBuffer[sizeof(u.binaryPacket)];
    for(int i=0; i<sizeof(u.binaryPacket); i++){
      iridiumBuffer[i] = u.b[i];
      if (u.b[i]<0x10) {Serial.print("0");} //print leading 0
      Serial.print(u.b[i], HEX);
    }
    Serial.println();

    size_t rxBufferSize = sizeof(rxBuffer);
    size_t iridiumBufferSize = sizeof(iridiumBuffer);

    if(introPeriod){
      cDisplay();
      display.println();
      display.println("Send Iridium");
      display.println(u.binaryPacket.packetStartTime);
      display.display();
    }
    

   int err;
   if(sendIridium){
    err = modem.sendReceiveSBDBinary(iridiumBuffer, iridiumBufferSize, rxBuffer, rxBufferSize);
    if (err != ISBD_SUCCESS){
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
      if (rxBufferSize >0) parseMessage();
      Serial.println("Msg Sent");
    }
  }
  return err;
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
  dataPacket += packetTime;
  dataPacket += ";";
  dataPacket += rec_dur;
  dataPacket += ";";
  dataPacket += String(latitude, 4);
  dataPacket += ";";
  dataPacket += String(longitude, 4);

  dataPacket += ";";
  for (int i=0; i<NBANDS; i++){
      if(meanBand[i][0]>0.000001){
        spectrumLevel = 20*log10(meanBand[i][0] / fftCount) - (10 * log10(binwidth)); 
      }
      else{
        spectrumLevel = -96 - (10 * log10(binwidth));
       }
      spectrumLevel = spectrumLevel - hydroCalLeft - gainDb;
      iSpectrumLevel = (int) spectrumLevel;
      dataPacket += i;
      dataPacket += ":";
      dataPacket += iSpectrumLevel;
      if(NCHAN==2) dataPacket+= ",";
      else
        dataPacket += ";";
      
      if(NCHAN==2){
       if(meanBand[i][1]>0.000001){
            spectrumLevel = 20*log10(meanBand[i][1] / fftCount) - (10 * log10(binwidth)); 
          }
          else{
            spectrumLevel = -96 - (10 * log10(binwidth));
           }
          spectrumLevel = spectrumLevel - hydroCalRight - gainDb;
          iSpectrumLevel = (int) spectrumLevel;
          dataPacket += iSpectrumLevel;
          dataPacket += ";";
       }
   }

  dataPacket += "v:";
  dataPacket += String((int) (voltage * 10));

  if(imuFlag) {
    dataPacket += ";z:";
    dataPacket += (int) (sdAccelZmg / 10.0);
  }


  dataPacket += ";w:";
  dataPacket += whistleCount;


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
  size_t rxBufferSize = sizeof(rxBuffer);
  err = modem.sendReceiveSBDText(&dataPacket[0], rxBuffer, rxBufferSize);

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
    if (rxBufferSize >0) parseMessage();
    Serial.println("Msg Sent");
  }

  return err;
}

void parseMessage(){
  // process incoming message
  // RM: Go into recovery mode (mode = 3)
  // To Do: R30,S90,M360: Record 30 s, Sleep 90 s, Send Iridium every 360 seconds
  Serial.write((char*) rxBuffer);
  if((rxBuffer[0]=='R') & (rxBuffer[1]=='M')) recoveryMode();  // Go to recovery mode
  
}

void resetSignals(){
  for (int i=0; i<NBANDS; i++){
    meanBand[i][0] = 0;
    peakBand[i][0] = 0;
    if(NCHAN==2) {
      meanBand[i][1] = 0;
      peakBand[i][1] = -1000;
    }
  }
  whistleCount = 0;
  fftCount = 0;
  sumOfSquares = 0;
  posPeak = 0;
  negPeak = 0;
  sumOfSquaresCount = 0;
}
