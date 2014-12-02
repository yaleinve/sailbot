//WindvaneTest.ino      Eric Anderson
//Just outputs the voltage being read by the windvane
//NOTE that we need to tune the bearing values- the
//voltages on the data sheet are not particularly
//accurate!!!


#define EPSILON 0.02  //How much error do we tolerate?

void setup()
{
  Serial.begin(115200);
}

int temp;

void loop()
{
  temp = analogRead(0);
  Serial.print("Voltage : ");
  Serial.println((double) temp/1023*5.0,3); 
  Serial.print("Bearing :");
  Serial.println(readBearing(0),1);
  delay(250);
}


//Converts an analog value to an output voltage
//for the sparkfun windvane.  Refer to online data
//sheet for conversion from voltage to bearing.
double readBearing(int windvanePin)
{
  //Need to check arguments here
  //  (code)
  
  
  double val = (double) analogRead(windvanePin)
      /1023.0*5.0;  //Assume a 10 bit analog value!
  Serial.print("In readBearing, voltage is :");
  Serial.println(val,3);
  //Consider doing multiple reads smoothing, although
  //with this shitty resolution it might not make much
  //of a difference
  if(dabs(val-3.84)< EPSILON){
    return 0;  //0 degree heading;
  }else if(dabs(val-1.98)< EPSILON){
    return 22.5;  //22.5 degree heading;
  }else if(dabs(val-2.25)< EPSILON){
    return 45;  //45 degree heading;
  }else if(dabs(val-0.41)< EPSILON){
    return 67.5;  
  }else if(dabs(val-0.45)< EPSILON){
    return 90; 
  }else if(dabs(val-0.32)< EPSILON){
    return 112.5;
  }else if(dabs(val-0.9)< EPSILON){
    return 135;
  }else if(dabs(val-0.62)< EPSILON){
    return 157.5;
  }else if(dabs(val-1.4)< EPSILON){
    return 180;
  }else if(dabs(val-1.19)< EPSILON){
    return 202.5;
  }else if(dabs(val-3.08)< EPSILON){
    return 225;
  }else if(dabs(val-2.93)< EPSILON){
    return 247.5;
  }else if(dabs(val-4.62)< EPSILON){
    return 270;
  }else if(dabs(val-4.04)< EPSILON){
    return 292.5;
  }else if(dabs(val-4.78)< EPSILON){
    return 315;
  }else if(dabs(val-3.43)< EPSILON){
    return 337.5;
  }else{
    //Error here
    Serial.println("Could not identify bearing");
    return -1;
  }
}

//Returns the dabsolute value of a double;
double dabs(double x)
{
  return (x < 0 ? x*(-1):x);
}
