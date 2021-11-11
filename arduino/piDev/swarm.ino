void pollTile(){
//  $DT 20210316170104,V*4c
//  $GN 27.2594,-82.4798,-3,0,2*1f

  while(Serial1.available()){
    byte incomingByte = Serial1.read();
    parseTile(incomingByte);
    Serial.write(incomingByte);
//    if(incomingByte=='$') display.println();
//    else display.print((char) incomingByte);
//    display.display();
  }
}

void parseTile(byte incomingByte){
  char rmcDate[25];
  // check for start of new message
  // if a $, start it at Pos 0, and continue until next $
  if(incomingByte=='$') {
    Serial.print("String position:");
    Serial.println(streamPos);
    Serial.println(gpsStream);
    //process last message
    if(streamPos > 10){
      float rmcLat; //          
      float rmcLon; //           
      float tileAlt;
      float tileCourse;
      float tileSpeed;
      uint8_t rmcChecksum; 

      // Extract Datetime
      // $DT 20210316170059,V*45
      if(gpsStream[1]=='D' & gpsStream[2]=='T'){
       char temp[streamPos + 1];
       const char s[2] = ",";
       char *token;
            
        memcpy(&temp, &gpsStream[4], 14);
        Serial.print("Date string extracted:"); Serial.println(temp);
        sscanf(temp, "%4i%2i%2i%2i%2i%2i", &gpsYear, &gpsMonth, &gpsDay, &gpsHour, &gpsMinute, &gpsSecond);
      }

      // Extract Lat Lon
      if(gpsStream[1]=='G' & gpsStream[2]=='N'){
       char temp[streamPos + 1];
       const char s[2] = ",";
       char *token;
            
        memcpy(&temp, &gpsStream[4], streamPos - 4);
        // 27.2594,-82.4798,-3,0,2*1f
        sscanf(temp, "%f,%f,%f,%f,%f*%2hhx",&rmcLat, &rmcLon, &tileAlt, &tileCourse, &tileSpeed, &rmcChecksum);
        memcpy(&temp, &gpsStream[1], streamPos - 5);
//        Serial.print("Calculated Checksum: ");
//        Serial.println(nmeaChecksum(&temp[0], streamPos-5), HEX);     
        if(nmeaChecksum(&temp[0], streamPos-5) == rmcChecksum){
           latitude = rmcLat;
           longitude = rmcLon;
           goodGPS = 1;
           Serial.println("valid GPS recvd");
        }
      }
      // extract RSSI
      if(gpsStream[1]=='R' & gpsStream[2]=='T'){
       char temp[streamPos + 1];
       char *token;

//       memcpy(&temp, &gpsStream[9], 3);
//       Serial.print("RSSI extracted:"); Serial.println(temp);

//       Serial.print("2 digit");
//       Serial.println(gpsStream[12]);
//       Serial.print("3 digit");
//       Serial.println(gpsStream[13]);
       // two digit values
        if(gpsStream[12]=='*') {
          memcpy(&temp, &gpsStream[9], 3);
          Serial.print("RSSI extracted:"); Serial.println(temp);
          sscanf(temp, "%d", &rssi);
        }
        // three digit values
        if(gpsStream[13]=='*'){
          memcpy(&temp, &gpsStream[9], 4);
          Serial.print("RSSI extracted:"); Serial.println(temp);
          sscanf(temp, "%d", &rssi);
        }
      }
    }
    // start new message here
    streamPos = 0;
  }
  gpsStream[streamPos] = incomingByte;
  streamPos++;
  if(streamPos >= maxChar) streamPos = 0;
}
