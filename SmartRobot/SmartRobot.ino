#include <IRremote.h>
#include <Servo.h>  
//------- IR REMOTE CODES ---------//
#define FORWARD 16736925
#define BACK    16754775
#define LEFT    16720605
#define RIGHT   16761405
#define STOP    16712445

#define RECV_PIN  12

#define ENB 5   // Left  wheel speed
#define IN1 7   // Left  wheel forward
#define IN2 8   // Left  wheel reverse
#define IN3 9   // Right wheel reverse
#define IN4 11  // Right wheel forward
#define ENA 6   // Right wheel speed
#define carSpeed 200  // initial speed of car >=0 to <=255


IRrecv irrecv(RECV_PIN);
decode_results results;
unsigned long val;
unsigned long preMillis;

//servo and ultrasonic components
Servo myservo; 
int Echo = A4; 
int Trig = A5; 
int rightDistance = 0, leftDistance = 0, middleDistance = 0;

//variables controlling wheel speeds
int backRightWheel = 0; 
int backLeftWheel = 0; 
int frontRightWheel = 0;
int frontLeftWheel = 0;
void setup() {
  // put your setup code here, to run once:
  servosetup(); 
  Serial.begin(9600);
  enableMotorDriver(); 
  stop();
  irsetup();
}

void ultrasonicsetup(){
  pinMode(Echo, INPUT);
  pinMode(Trig, OUTPUT);
}

void servosetup(){
    myservo.attach(3);  // attach servo on pin 3 to servo object
    myservo.write(90);  //setservo position according to scaled value
    
}

void stop(){
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  Serial.println("STOP!");
}

void irsetup(){
  irrecv.enableIRIn();  
}
void enableMotorDriver(){
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
}

//Ultrasonic distance measurement Sub function
int getDistance() {
    digitalWrite(Trig, LOW);
    delayMicroseconds(2);
    digitalWrite(Trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(Trig, LOW);
    return (int)pulseIn(Echo, HIGH) / 58;
}



void forward() {
  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("go forward!");
}

void back(){
  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("go back!");
}

void left(){
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH); 
  Serial.println("go left!");
}

void right(){
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("go right!");
}

void moveCar(){
  
}

void recvIR(IRrecv *recvd){
  if (irrecv.decode(&results)){ 
    preMillis = millis();
    val = results.value;
    Serial.println(val);
    irrecv.resume();
    switch(val){
      case FORWARD: forward();  break;
      case BACK:    back();     break;
      case LEFT:    left();     break;
      case RIGHT:   right();    break;
      case STOP:    stop();     break;
      default:                  break;
    }
  }
  else{
    if(millis() - preMillis > 500){
      stop();
      preMillis = millis();
    }
  }
}

void ultraServoScan(){ //rotate servo and scan for nearby objects
  delay(500);
    middleDistance = getDistance();

    if(middleDistance <= 20) {
      stop();
      delay(500);
      myservo.write(10);
      delay(1000);
      rightDistance = getDistance();

      delay(500);
      myservo.write(90);
      delay(1000);
      myservo.write(180);
      delay(1000);
      leftDistance = getDistance();

      delay(500);
      myservo.write(90);
      delay(1000);
      if(rightDistance > leftDistance) {
        right();
        delay(360);
      }
      else if(rightDistance < leftDistance) {
        left();
        delay(360);
      }
      else if((rightDistance <= 20) || (leftDistance <= 20)) {
        back();
        delay(180);
      }
      else {
        forward();
      }
    }
    else {
        forward();
    }
}

void loop() {
  // put your main code here, to run repeatedly:

}
