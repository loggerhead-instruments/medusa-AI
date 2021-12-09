void noiseBandCalc(){
    // calculate band level noise and band peaks
    fftCount += 1;  // counter to divide meanBand by before sending
    for(int n=0; n<NBANDS; n++){
      for(int i=bandLow[n]; i<bandHigh[n]; i++){
        float value = FFT1.read(i);
        meanBand[n][0] += (value / nBins[n]); // accumulate across band
        if(peakBand[n][0] < value) peakBand[n][0] = value;
        #ifdef TWOCHAN
           value = FFT2.read(i);
           meanBand[n][1] += (value/ nBins[n]); // accumulate across band
           if(peakBand[n][1] < value) peakBand[n][1] = value;
        #endif
      }
    }
}
