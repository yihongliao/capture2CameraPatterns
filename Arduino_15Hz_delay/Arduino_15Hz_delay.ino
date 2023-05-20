//#define PROJECTOR 10
//#define CAMERA 8

#define PROJECTOR 11
#define CAMERA 13

//#define PROJECTOR 11
//#define CAMERA 9


#define seqA 466 //1470//5 //10 //
#define seqB 66 //3530//12 //24 //

void setup() {
 
  pinMode(PROJECTOR, OUTPUT);
  pinMode(CAMERA, OUTPUT);
  
}

void loop() {
  
  digitalWrite(PROJECTOR, HIGH);
  delayMicroseconds(200);
  digitalWrite(CAMERA, HIGH);

  delayMicroseconds(seqA);
  
  digitalWrite(PROJECTOR, LOW);
  digitalWrite(CAMERA, LOW);

  delay(seqB);
  //cnt++;
}
