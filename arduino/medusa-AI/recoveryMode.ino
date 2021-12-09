// Recovery mode sends lat/lon only every 5 minutes
// Expires after 1 day

void recoveryMode(){
  long recoveryTime = getTeensy3Time();
  Serial.println(" Recovery");
  digitalWrite(hydroPowPin, LOW); //hydrophone off          
  audio_power_down();
  if(imuFlag) mpuInit(0);
  displayOff();
  delay(10);

  // 86400 = 1 day
  while(getTeensy3Time() - recoveryTime < 86400){
    alarm.setRtcTimer(0, 5, 0);  // h, m, s
    Snooze.hibernate(config_teensy32);
       
    /// ... Sleeping ....


    digitalWrite(gpsEnable, HIGH); // wake up to get GPS in standby mode
    gpsTimeOutThreshold = 80000; //give 80 seconds to read
    gpsGetTimeLatLon();  
    digitalWrite(gpsEnable, LOW); // turn off once have good GPS
    
    modem.begin();
    modem.adjustSendReceiveTimeout(120);  // timeout in 120 seconds

    // create data packet and send
    dataPacket = "";
    dataPacket += "RM:";
    dataPacket += goodGPS;
    dataPacket += ";";
    dataPacket += getTeensy3Time();
    dataPacket += ";";
    dataPacket += String(latitude, 4);
    dataPacket += ";";
    dataPacket += String(longitude, 4);
    
    sendDataPacket();  //blocking call
    modem.sleep();
  }     
  resetFunc();  //restart
}

