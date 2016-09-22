//trying to reduce choppy feedback    
//if(fqNow>fqThen*1.50 || fqNow<fqThen*.5){
//      fqTemp = (fqNow+fqThen)/2;
//      fqNow=fqTemp;
//    } 
//  if(cm=0){
//    tone(pinOut,fqNow);
//    }  
//  else{
//    dist=cm;
//    fqNow=map(dist,1,MAX_DISTANCE,minFq,maxFq);
//    fqNow= (-1)*(fqNow-maxFq);
//    tone(pinOut,fqNow);
//    Serial.print("Frequency is ");
//    Serial.print(fqNow);
//    Serial.println("Hz");
//  }
