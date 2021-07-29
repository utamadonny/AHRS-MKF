float incoming[8];
float ax,ay,az,gx,gy,gz,mx,my,mz;
void setup(){
  Serial.begin(9600);
}

void loop(){
  while(Serial.available() >= 9){
    // fill array
    for (int i = 0; i < 9; i++){
      incoming[i] = Serial.read();
    }
    // use the values
  }
  ax = incoming[0];
  ay = incoming[1];
  az = incoming[2];
  gx = incoming[3];
  gy = incoming[4];
  gz = incoming[5];
  mx = incoming[6];
  my = incoming[7];
  mz = incoming[8];
  Serial.println(ax);
  delay(1000);
}