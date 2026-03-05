# include <Wire.h>


int leftPot = A1;
int rightPot = A2;

long rAngle = 0;
long lAngle = 0;
long  prevtime = 0;
long deltat = 0;

//robot length constants     b is final arm, a is driven arm, c is shaft to center of robot
float a = 470, b= 508, c= 51;
//robot variables
long hr = 0;
float angleIr = 0, angleAr = 0;
long hl = 0; 
float angleIl = 0, angleAl = 0;

//communication with python
int pyDat = 1;


//coefficients for pid controller
const int kp = 22;    
const int kd = 22;    
const int ki = 0;


long pidTimerL = 0, pidTimerR = 0;//time passed in pid loop in milliseconds (for the integral part)
int errorL = 0, errorR = 0; // error for left and right motor positions
int errorSumL = 0, errorSumR = 0; // summed error, for integral part of pid
int prevErrorL = 0, prevErrorR = 0;


//acceleration controll
int prevPWMvalL = 0, prevPWMvalR = 0;
const int maxAcceleration = 25;
int maxSpeed = 200;

//variables for the location of the robot
int Xmm = 0 + 10, Ymm = 0 + 365; //second part is to adjust for distance from table top
long goalAngleL = 45, goalAngleR= 45;  // where do we want the motors to go

//motor pwm variables
int leftPWM1 = 0, leftPWM2 = 0;
int rightPWM1 = 0, rightPWM2 = 0;



//motor pins
#define IN1L 3
#define IN2L 6
#define IN1R 11
#define IN2R 10

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();

  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);

  // Set motor & enable connections as outputs
  pinMode(IN1L, OUTPUT);
  pinMode(IN2L, OUTPUT);
  pinMode(IN1R, OUTPUT);
  pinMode(IN2R, OUTPUT);

  // Stop motors
  analogWrite(IN1L, 0);
  analogWrite(IN2L, 0);
  analogWrite(IN1R, 0);
  analogWrite(IN2R, 0);

  //pinMode(52,OUTPUT); pinMode(53,OUTPUT);

  delay(1000);
}

// main loop of code
void loop() {

  //read in serial data from python, to know where to go
  while(Serial.available()>0){
      pyDat = Serial.parseInt();
      Serial.read();   
  }
  //if data is positive, it is Y coordinate, if negative, it's X coord, and must be adjusted 
  if(pyDat>=0) {Ymm = 365 + pyDat; constrain (Ymm,325,850);}
  if(pyDat<0) {Xmm = 0 + 5000 + pyDat; constrain (Xmm,-300,300);}

  //find current angle of robot arms
  getAngle();  
  // code to find out what the goal position is goes here
  getGoalAngle(Xmm,Ymm);
  
  //finding the error 
  errorL = goalAngleL - lAngle, errorR = goalAngleR - rAngle;

  //compute pid controller for left and right  
  pidControl(errorL,leftPWM1,leftPWM2,0);
  pidControl(errorR,rightPWM1, rightPWM2,1);

  //make the left motor move
  analogWrite(IN1L,leftPWM1);  analogWrite(IN2L,leftPWM2);
  //make right motor move
  analogWrite(IN1R,rightPWM1);  analogWrite(IN2R,rightPWM2);

  /*
  Serial.print("GL: ");
  Serial.print(goalAngleL);
  Serial.print("  GR: ");
  Serial.print(goalAngleR);
  Serial.print("    TL: ");
  Serial.print(lAngle);
  Serial.print("  TR: ");
  Serial.println(rAngle);
  */   
  

  delay(5);

}

void getAngle(){
  //angles are measured from x axis to arm (they go opposite directions)

  int rawLeftPotVal = analogRead(leftPot);
  lAngle = map(rawLeftPotVal,760,420,0,90);

  int rawRightPotVal = analogRead(rightPot);
  rAngle = map(rawRightPotVal,447,785,90,0);

  //for initializing pot angles
  
  /*
  Serial.print("L: ");
  Serial.print(rawLeftPotVal);
  Serial.print("      R: ");
  Serial.println(rawRightPotVal);
  */

}

void getGoalAngle(long goalX, long goalY){

  //find the hypotenouses of the arms
  hr = sqrt((goalY*goalY)+((c-goalX)*(c-goalX)));
  hl = sqrt((goalY*goalY)+((c+goalX)*(c+goalX)));
  //now we know the lengths of all sides of the triangle (a,b, and h(l/r))
 
  // find the angle between a and the hypotenous  (units in degrees)
  angleAr = 57.3*acos(((hr*hr)+(a*a)-(b*b))/(2*a*hr));
  angleAl = 57.3*acos(((hl*hl)+(a*a)-(b*b))/(2*a*hl));

  //find angle between hypotenouse and robot datum
  angleIr = 57.3*acos(((4*c*c)+(hr*hr)-(hl*hl))/(4*c*hr));
  angleIl = 57.3*acos(((4*c*c)+(hl*hl)-(hr*hr))/(4*c*hl));

  goalAngleL = 180 -(angleAl+angleIl);
  goalAngleR = 180 - (angleAr+angleIr);

  constrain(goalAngleL,-10,130); constrain(goalAngleR,-10,130); //constrain angles to not go past the edge
  
}


// pid controller to determine speed.
void pidControl(int error, int & pin1Output, int & pin2Output, int LorR){ // LorR = 0= left, =1, right

  //delta t between each loop is always 5 milliseconds

  int errorDerriv = 0;
  int rawOut = 0;
  int kdMult = 1;
  if(abs(error) <5) {kdMult = 2;}

  if (LorR == 0){
    errorSumL = errorSumL+(error);  // find integral error for left
    errorDerriv = (error - prevErrorL); // find derrivative of error
    rawOut = kp*error + kdMult*kd* errorDerriv; // find the raw output value 
    //if(abs(error)<4){rawOut = rawOut+(ki*errorSumL)/10;}
    prevErrorL = error;    // update previous error
  }
  else{   // same thing for right side
    errorSumR = errorSumR +(error);
    errorDerriv = (error - prevErrorR);
    rawOut = kp*error + kdMult*kd* errorDerriv;
    prevErrorR = error;
  }

 // cap  output to 242, make positive
  int pwmVal = abs(rawOut);
  pwmVal = constrain(pwmVal,0,maxSpeed);

  //stop motors when very close, reduce noise/current
  if(abs(error)<=2){pwmVal = 0;}
  //pwmVal = pwmVal/2; // just for testing

  //make sure doesnt exceed max acceleration
  if (LorR ==0){
    if (pwmVal>prevPWMvalL+maxAcceleration || pwmVal < prevPWMvalL-maxAcceleration){
      pwmVal = constrain(pwmVal,prevPWMvalL-maxAcceleration,prevPWMvalL+maxAcceleration);}
    prevPWMvalL = pwmVal;
  }
  else{
    if (pwmVal>prevPWMvalR+maxAcceleration || pwmVal < prevPWMvalR-maxAcceleration){
      pwmVal = constrain(pwmVal,prevPWMvalR-maxAcceleration,prevPWMvalR+maxAcceleration);}
    prevPWMvalR = pwmVal;
  }

  // if negative or positive, set the direction save
  if (rawOut>0){
    pin1Output = 0; pin2Output = pwmVal;
  }
  else{pin1Output = pwmVal; pin2Output = 0; }

  //Serial.println(pwmVal);
}
