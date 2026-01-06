// Connect 4 Robot - Arduino Controller
// Handles motor control and sensor input
// Development version, subject to change

#include <Encoder.h>
#include <Servo.h>

class Motor{
  private: 

    int pinA;
    int pinB;
    int pinPWM;

  public:

    //pinA - cw
    Motor(int A, int B, int PWM){
      pinA = A;
      pinB = B;
      pinPWM = PWM;

      pinMode(pinA, OUTPUT);
      pinMode(pinB, OUTPUT);
      pinMode(pinPWM, OUTPUT);
    }

    void setSpeed(int pwm){
      
      if(pwm == 0){
        stop();
        return;
      }

      if(pwm > 0){
        digitalWrite(pinA, HIGH);
        digitalWrite(pinB, LOW);
      }else{
        digitalWrite(pinB, HIGH);
        digitalWrite(pinA, LOW);
      }
      analogWrite(pinPWM, abs(pwm));
    }

    void stop(){
      digitalWrite(pinA, LOW);
      digitalWrite(pinB, LOW);
      digitalWrite(pinPWM, LOW);
    }

};

//encoder vars
int encY = 3;
int encB = 2;
Encoder myEnc(encY, encB);
long encCount = 0;

//PID vars
int dt_min = 30000UL;
double Kp = 3.0;
double Ki = 0;
double Kd = 0.15;
int targetCount = 0;
int err = 0;
int pos_tol = 3;
double v_tol = 0.0; 
long unsigned lastTick = 0;
double integral;

//servo vars
int cartServoPin = 7;
Servo cartServo;

//DC motor vars
int mtrA = 12;
int mtrB = 11;
int mtrPWM = 5;
Motor cartMtr(mtrA, mtrB, mtrPWM);

//control state
enum State {
  IDLE,
  MOVE_CART,
  DROP_PIECE,
  USER_MOVE,
  COMPUTER_MOVE,
};
State currentState = IDLE;


void setup() {
  Serial.begin(9600);
  Serial.println("STARTED");
  cartServo.attach(cartServoPin);
  cartServo.write(10);

  pinMode(A1, INPUT);
  pinMode(9, INPUT);
  pinMode(8, INPUT);

  targetCount = 136 * -2;
  err = targetCount - myEnc.read();
  currentState = MOVE_CART; 
  currentState = IDLE;
  
}


void loop() {
  // put your main code here, to run repeatedly:
  
  // noInterrupts();
  // long newCount = myEnc.read();
  // interrupts();
  // if(newCount != encCount){
  //   encCount = newCount;
  //   double pos = encCount * 1.25 / 4.0;
  //   Serial.println(encCount);
  // }
  //Serial.println(analogRead(A6));
  // if(digitalRead(8)){
  //   Serial.println(1);
  // }
  //Serial.println(digitalRead(8));
  Serial.print(digitalRead(8));
  Serial.print("  ");
  Serial.println(digitalRead(9));

  switch(currentState){
    case IDLE:
      //Serial.print("IDLE");
      break; 

    case DROP_PIECE:
      Serial.println("DROPPING PIECE");
      cartServo.write(110);
      delay(750);
      cartServo.write(10);
      currentState = IDLE;
      break;

    case USER_MOVE:
      break;

    case COMPUTER_MOVE:
      break;

    case MOVE_CART:
      unsigned long now = micros();
      if(now - lastTick >= dt_min){

        noInterrupts();
        long newCount = myEnc.read();
        interrupts(); 

        int newErr = targetCount - newCount;
        double dt = (now - lastTick) / 1e6;

        integral += newErr * dt;
        double derivative = (double)(newErr - err) / dt;
        err = newErr;

        int pwm = (int)(Kp*err + Ki*integral + Kd*derivative);
        Serial.println(pwm);

        // Serial.print("POS: ");
        // Serial.println(newCount);
        Serial.print("ERR: ");
        Serial.println(err);
        // Serial.print("INT: ");
        // Serial.println(integral);
        // Serial.print("DER: ");
        // Serial.println(derivative);
        

        if(pwm < -255){
          pwm = -255;
        }else if(pwm > 255){
          pwm = 255;
        }else if(pwm != 0 && derivative == 0.0){
          int s = pwm / abs(pwm);
          pwm = s * 255;
        }

        Serial.print("PWM: ");
        Serial.println(pwm);
        Serial.println();
        cartMtr.setSpeed(pwm);

        if(abs(err) <= pos_tol && abs(derivative) <= v_tol){
          integral = 0.0;

          cartMtr.stop();

          currentState = DROP_PIECE;
          Serial.print("TAR: ");
          Serial.println(targetCount);
          Serial.print("POS: ");
          Serial.println(myEnc.read());
          Serial.println("PID DONE");
        }
      }
      break;
    
    
  }

}
