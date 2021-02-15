// detections.txt 
// payload that will be appended to audio stats

boolean readDetections()
{
  #if USE_SDFS==1
    FsFile file;
  #else
    File file;
  #endif

  // Read card setup.txt file to set date and time, recording interval
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
 return 1;  
}
