#include <InverseK.h>
#include <Braccio.h>

/*
 * TODO:
 * Rotation of gripper
 * Commands close and open to close and open gripper
 * Automation-push button to confirm- keep in txt file
 */

Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_ver;
Servo wrist_rot;
Servo gripper;

int potentiometer = A0;

float x = 200;
float y = 0;
float z = 200;
float rotation = 0;
float g = 10;

void setup() {
  pinMode(potentiometer, INPUT);
  Braccio.begin();
  Serial.begin(9600);
  Serial.println("Begin Setup");
  // Setup the lengths and rotation limits for each link
  Link base, upperarm, forearm, hand;
  
  base.init(0, b2a(0.0), b2a(180.0));
  upperarm.init(200, b2a(15.0), b2a(165.0));
  forearm.init(200, b2a(0.0), b2a(180.0));
  hand.init(270, b2a(0.0), b2a(180.0));

  // Attach the links to the inverse kinematic model
  InverseK.attach(base, upperarm, forearm, hand);
  moveToPos(x,y,z, rotation, g);
}

void loop() {
  String msg = "";
  if(Serial.available()>0){
    while(Serial.available()>0){
      msg += char(Serial.read());
      delay(250);
    }
  }
  if (msg == "close"){
    
  }
  //use regex to get xyz coordinates from msg of form x,y,z
  int comma1;
  int comma2;
  int comma3;
  for (int i = 0; i < msg.length(); i++){
    if (msg[i] == ','){
      comma1 = i;
      break;
    }
  }
  for (int i = comma1+1; i < msg.length(); i++){
    if (msg[i] == ','){
      comma2 = i;
      break;
    }
  }
  for (int i = comma2+1; i < msg.length(); i++){
    if (msg[i] == ','){
      comma3 = i;
      break;
    }
  }
  String xstring;
  String ystring;
  String zstring;
  String closestring;
  for (int i = 0; i<comma1; i++){
    xstring += msg[i];
  }
  for (int i = comma1+1; i<comma2; i++){
    ystring += msg[i];
  }
  for (int i = comma2+1; i<comma3; i++){
    zstring += msg[i];
  }
  for (int i = comma3+1; i<msg.length(); i++){
    closestring += msg[i];
  }
  x += xstring.toFloat();
  y += ystring.toFloat();
  z += zstring.toFloat();
  float prevg = g;
  if (closestring.toFloat()!=0){
    g = closestring.toFloat();
  }

  int potentiometerValue = analogRead(potentiometer);
  if ((xstring.toFloat()!=0) or (ystring.toFloat()!=0) or (zstring.toFloat()!=0) or (g!= prevg)){
    Serial.println(potentiometerValue);
    if (potentiometerValue > 500){
      Serial.println("Pot");
      moveToPosAndOrn(x,y,z, rotation, g,-1.5);
    }
    else{
      moveToPos(x,y,z,rotation, g);
    }
    
  }
  
}


void moveToPosAndOrn(float x, float y, float z, float rotation, float g, float r){
  float a0, a1, a2, a3;
  if(InverseK.solve(x, -1*y, z, a0, a1, a2, a3, r)) {
      Serial.println("success");
      Serial.print(x); Serial.print(',');
      Serial.print(y); Serial.print(',');
      Serial.print(z); Serial.print(',');
      Serial.print(rotation); Serial.print(',');
      Serial.println(g);
      Braccio.ServoMovement(20, a2b(a0), a2b(a1), a2b(a2), a2b(a3), rotation, g);
      
    } else {
      Serial.println("failed");
      Serial.print(x); Serial.print(',');
      Serial.print(y); Serial.print(',');
      Serial.print(z); Serial.print(',');
      Serial.print(rotation); Serial.print(',');
      Serial.println(g);
    }  
}

void moveToPos(float x, float y, float z,float rotation, float g){
  float a0, a1, a2, a3;
  if(InverseK.solve(x, -1*y, z, a0, a1, a2, a3)) {
      Serial.println("success");
      Serial.print(x); Serial.print(',');
      Serial.print(y); Serial.print(',');
      Serial.print(z); Serial.print(',');
      Serial.print(rotation); Serial.print(',');
      Serial.println(g);
      Braccio.ServoMovement(20, a2b(a0), a2b(a1), a2b(a2), a2b(a3),rotation, g);
      
    } else {
      Serial.println("failed");
      Serial.print(x); Serial.print(',');
      Serial.print(y); Serial.print(',');
      Serial.print(z); Serial.print(',');
      Serial.print(rotation); Serial.print(',');
      Serial.println(g);
    }  
}

// Quick conversion from the Braccio angle system to radians 
float b2a(float b){ 
  return b / 180.0 * PI - HALF_PI;
}

// Quick conversion from radians to the Braccio angle system
float a2b(float a) { 
  return (a + HALF_PI) * 180 / PI;
}
