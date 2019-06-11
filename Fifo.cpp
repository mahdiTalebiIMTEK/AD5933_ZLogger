#include "Fifo.h"

bool FifoE::putIn(int value){
  fifo[in]=value;
  in = (in+1)%BUFFERSIZE;
}

int FifoE::getOut(){ 
  int tmp=fifo[out];
  out = (out+1)%BUFFERSIZE;
  return tmp;
  
}

int FifoE::count(){
  if(out>in){
    //Serial.println(out-in+(BUFFERSIZE-in));
    return out-in+(BUFFERSIZE-in);
  }
  //Serial.println(in-out);
  return in-out;
}

/*
 int p=1;
  while(1){
   
    p+=10;
    for(int i=0;i<p;i++){
      realData.putIn(i);
    }
    Serial.println("---");
    for(int i=0;i<p;i++){
      Serial.print(realData.count());
      Serial.print(" - ");
      Serial.println(realData.getOut());
      
    }
    delay(1000);
  }
 */
