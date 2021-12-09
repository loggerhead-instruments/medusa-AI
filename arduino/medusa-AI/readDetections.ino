// detections.txt 
// payload that will be appended to audio stats

boolean readDetections()
{
  #if USE_SDFS==1
    FsFile file;
  #else
    File file;
  #endif

  sd.chdir(); // only to be sure to star from root
  file=sd.open("detections.txt");
  if(file)
  {
    int n = file.fgets(piPayload, sizeof(piPayload));
    if(n<=0){
      file.close();
      return 0;
    }
    file.close();  
    
  }
  else return 0;
  Serial.print("Detections:");
  Serial.println(piPayload);

  // extract whistle count
  char signalLabel[1];
  unsigned int detectionCount;
  sscanf(piPayload, "%c:%d",signalLabel,&detectionCount);
  if(signalLabel[0]=='w') {
    whistleCount = detectionCount;
    Serial.print("scan whistles:");
    Serial.println(whistleCount);
  }
 return 1;  
}
