//writeMicroseconds

class DistanceSensor{

  public:
    int port;

    //buffer to save results
    int* data;
    int dataLength;
    int nextId =0;

    int* sortedArray;
    int medianId;

    int delayTime = 0;

    DistanceSensor(){}

    DistanceSensor(int port,int dataLength) : port(port),dataLength(dataLength){
      data = new int[dataLength];
      sortedArray = new int[dataLength];
      for(int i=0;i<dataLength;i++) data[i] = 0;

      medianId = (int)(dataLength/2); //this is median when length is odd, and the upper median otherwise
    }

    //conversion from raw to distance value need to be implemented
    virtual float rawToDistance(int raw) = 0;

    float getDistance(){
      return rawToDistance(getRawValue());
    }

    //raw value reading same for all
    //read value and store it in buffer
    int getRawValue(){
      int val = analogRead(port);
      data[nextId] = val;
      nextId = (nextId+1) % dataLength;
      return val;       
    }

    int getAllRawValues(){
      nextId = 0;
      getRawValue();
      for (int i=1;i<dataLength;i++) {
        delay(delayTime);
        getRawValue();
      }
      return rawMedian();
      
    }



    //calculate the median of the stored raw values... I assume data buffer is full - otherwise noise
    int rawMedian(){

      for(int i=0;i<dataLength;i++){
        int pos = i;
        int val = data[pos];
        for(int j = pos-1; j >=0 ; j--)
            if(sortedArray[j] > val){
              sortedArray[j+1] = sortedArray[j];
              pos = j;
            }  
        sortedArray[pos] = val;
      }

      return sortedArray[medianId];
      
      
    }

    float getMedianDistance(){
      return rawToDistance(rawMedian());     
    }
  
};



class RawDistanceSensor  : public DistanceSensor {

  public:

    RawDistanceSensor(){}  
  
    RawDistanceSensor(int port,int dataLength): DistanceSensor(port,dataLength) {
        delayTime = 17;
    }

    float rawToDistance(int val){
      return val;
    }

};






