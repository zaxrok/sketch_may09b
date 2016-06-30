/*
 * 
 * 0.2 대구 시연 6/7 버튼 클릭시 덴싱 모드/ 라인트레이스/ Obstacle avoidance/ wall follower
 * 0.1 교사연구회 시연 5/14
 */
// for codestar mobile
#include <IRremote.h>
#include <Thread.h>
#include <SoftwareSerial.h>
#include <ThreadController.h>

#ifdef __AVR__
  #include <avr/power.h>
#endif

// protocol
#define GET 1
#define RUN 2
#define RESET 4
#define START 5
//#define NEO
// mobile
#define FORWARD   0
#define REVERSE   1
#define MOTOR_L   0
#define MOTOR_R   1

const int pinLT1 = 7;  // IR 센서 1번(IN1) -- LEFT
const int pinLT2 = 8;  // IR 센서 2번(IN2) -- RIGHT
const int pinButton = 12; // 푸시버튼 연결 핀 번호

const int pinRGB_Red = 9;    // RGB LED의 Red 연결 핀 번호
const int pinRGB_Green = 10; // RGB LED의 Green 연결 핀 번호
const int pinRGB_Blue = 11;  // RGB LED의 Blue 연결 핀 번호
const int pinWhite = 3;    

#ifdef NEO
  #include <Adafruit_NeoPixel.h>
  const int pinLedBar = 7;  // RGB LED Strip Bar, WS2812
  Adafruit_NeoPixel strip = Adafruit_NeoPixel(8, pinLedBar, NEO_GRB + NEO_KHZ800);
#endif

const int pinMic = A2;  // 마이크 연결 핀 번호

Thread neoThread = Thread();
// create bluetooth instnace
SoftwareSerial bt(1, 0);

// packet
double  currentTime = 0.0; // on request TIMER
double  lastTime = 0.0;    // on request TIMER
boolean isAvailable = false;  // serial
boolean isStart = false;  // fine package
String  mVersion = "1.0.0";
byte    dataLen = 0;      // package data length;
char    serialRead;       // real data
unsigned char prevc = 0;  // real data
byte    index = 0;        // buffer index
char    buffer[52];       // real buffer

// mobile
// MOTOR_LEFT
int pinDirL = 2;
int pinSpeedL = 5;

// MOTOR_RIGHT
int pinSpeedR = 6;
int pinDirR = 3;

// sonar
const int pinEcho = A1;  // 초음파 센서 Echo 단자 연결 핀 번호
const int pinTrig = 13;  // 초음파 센서 Trig 단자 연결 핀 번호
int preDistance = 0;      // 초음파 -1 때문에

// left ir or wall following
const int pinLeftIR = A7;  

// Healing mode
bool bFadeColor = false;
// start
const int pinBuzzer = A3;  // 부저(스피커) 연결 핀 번호
int tones[] = { 261, 294, 330, 349, 392, 440, 494, 523 };

// Variables will change:
int ledState = HIGH;         // the current state of the output pin
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin

// the following variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 50;    // the debounce time; increase if the output flickers
int modeCount = 1; 
/////////////  라인트레이스 /////////////

// remote
#define LINE_TRACER     0
#define REMOTE_CONTROL  1
#define AVOIDENCE       2
#define WALLFOLLOW      3

const int pinRemocon = 4;
IRrecv remocon(pinRemocon);
decode_results results;
int DriveMode = REMOTE_CONTROL;

int LeftTurn = 0;
int RightTurn = 0; 
#define MAX_TURN_STEP1    8  // 연속 좌/우 회전 감지 카운터, 1/8 회전씩 감축용
#define MAX_TURN_STEP2    15  //
#define MAX_TURN_STEP3    21  //
#define MAX_TURN_STEP4    26  //
#define MAX_TURN_STEP5    30  //
#define MAX_TURN_STEP6    33  //
#define MAX_TURN_STEP7    35  // 정지 회전, (만약 초과시 원래 역회전)
int Power = 200;
float Power1Ratio = 1.0;
float Power2Ratio = 1.0;

/// obstacle avoid
unsigned long msLastTick;
boolean onlyOne   = false;
boolean isAvoided = false;
int     wheelSwitch = 0;
boolean wheelTick = false;
ThreadController controll = ThreadController();
Thread worker1 = Thread();
Thread worker2 = Thread(); 

/*
ff 55 len x  GET  sensor  port  slot  data a
0  1  2   3   4      5      6     7     8
*/
void runModule(int device){
  // for 1 device
  int ldir = 0;
  int rdir = 0;
  int lvel = 0;
  int rvel = 0;

  // for 3 device
  int lpos = 0;
  int ledr = 0;
  int ledg = 0;
  int ledb = 0;
  switch(device){
    case 1: // move
      ldir = readBuffer(6); // left direction
      lvel = readBuffer(7); // left velocity
      rdir = readBuffer(8); // right direction
      rvel = readBuffer(9); // right velocity
      move(MOTOR_L, ldir, lvel*28);
      move(MOTOR_R, rdir, rvel*28);
      break;
    case 2: // stop
      stop();
      break;
    case 3: // led
      lpos = readBuffer(6);  // led position(don't care)
      ledr = readBuffer(7);  // led red color
      ledg = readBuffer(8);  // led green color
      ledb = readBuffer(9);  // led blue color
      color(ledr, ledg, ledb);
      break;
    case 4: // led off
      ledoff();
      break;
    case 5: // healing mode
      bFadeColor = true;
      break;
    case 6: // healing mode off
      bFadeColor = false;
      ledoff();
    default:
    break;
  }
}

void readSerial(){
  isAvailable = false;
  if(bt.available() > 0){
    isAvailable = true;
    serialRead = bt.read();
  }
}

void writeBuffer(int idx, unsigned char c){
  buffer[idx] = c;
}
unsigned char readBuffer(int idx){
  return buffer[idx];
}
void writeHead(){
  writeSerial(0xff);
  writeSerial(0x55);
}
void writeEnd(){
  bt.println();
}
void writeSerial(unsigned char c){
  bt.write(c);
}
void callOK(){
  writeHead();
  writeEnd();
}

void  FadeColor(int pinStart, int pinLast, int pinOff){
  int  brightness;  // pinStart 빛의 밝기
  
  analogWrite(pinOff, 0);  // 꺼 둘 색상
  
  for(brightness=255; brightness>=0; brightness--) // 255에서 0까지 
  {
    analogWrite(pinStart, brightness);    // 처음 색상
    analogWrite(pinLast, 255-brightness); // 최종 색상
    
    delay(20);  // 20 밀리초 기다리기 (합성한 빛을 보여주는 시간)
  }
}

void color(int r, int g, int b){
  digitalWrite(pinRGB_Red, r*28);
  digitalWrite(pinRGB_Green, g*28);
  digitalWrite(pinRGB_Blue, b*28);

}
void ledoff(){
  digitalWrite(pinRGB_Red, LOW);   // 빨간색 켜기
  digitalWrite(pinRGB_Green, LOW); // 초록색 켜기
  digitalWrite(pinRGB_Blue, LOW);   // 파란색 끄기
}
void stop()  // 정지
{
  analogWrite(pinSpeedL, 0);
  analogWrite(pinSpeedR, 0);
}
void move(int motor, int direction, int speed)
{
  boolean inPin1, inPin2;

  if(direction == FORWARD)
    inPin1 = HIGH;
  else // REVERSE
    inPin1 = LOW;

  if(motor == MOTOR_L)
  {
    digitalWrite(pinDirL, inPin1);
    analogWrite(pinSpeedL, speed);
  } else { // MOTOR_R
    digitalWrite(pinDirR, inPin1);
    analogWrite(pinSpeedR, speed);
  }
}

/*
ff 55 len idx action device port  slot  data a
0  1  2   3   4      5      6     7     8
*/
void parseData(){
  int idx = readBuffer(3);
  int action = readBuffer(4);
  int device = readBuffer(5);

  switch(action){
    case GET:
     
    break;
    case RUN:
      runModule(device);
      //callOK(); // ack signal
    break;
    case RESET:
    
    break;
    case START:
    
    break;
  }
}

#ifdef NEO
void neoCallback(){
  int value = analogRead( pinMic );  // 소리 세기 읽기
  int lowLimit = 20, highLimit = 200;
  int valLimit = constrain( value, lowLimit, highLimit );
  //Serial.println(value);
  int brightness = map( valLimit, lowLimit, highLimit, 0, 255 );
  analogWrite(pinWhite, brightness);
  for(int i=0;i<8;i++){
    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    strip.setPixelColor(i, strip.Color(random(0, 155),brightness,random(0, 155))); // Moderately bright green color.
  }
  strip.show(); // This sends the updated pixel color to the hardware.
}
#endif

void callback1(){
  if(DriveMode == AVOIDENCE && !isAvoided){
    if(wheelTick){
      if(++wheelSwitch % 2){
        move(MOTOR_L, REVERSE, 100);
        move(MOTOR_R, FORWARD, 100); 
      }else{
        move(MOTOR_L, FORWARD, 100);
        move(MOTOR_R, REVERSE, 100); 
      }
    }else{
      move(MOTOR_L, FORWARD, 100);
      move(MOTOR_R, FORWARD, 100); 
    }
  }  
}
// led on/off
void callback2(){ // for test
  wheelTick = !wheelTick;
}
void setup() {

  // 3 color led
  pinMode(pinRGB_Red, OUTPUT);   // RGB LED의 Red 핀을 출력용 핀으로 설정
  pinMode(pinRGB_Green, OUTPUT); // RGB LED의 Green 핀을 출력용 핀으로 설정
  pinMode(pinRGB_Blue, OUTPUT);  // RGB LED의 Blue 핀을 출력용 핀으로 설정
  pinMode(pinWhite, OUTPUT); 

  // mobile
  pinMode(pinSpeedL, OUTPUT);
  pinMode(pinDirL, OUTPUT);

  pinMode(pinSpeedR, OUTPUT);
  pinMode(pinDirR, OUTPUT);

  pinMode(pinLT1, INPUT);
  pinMode(pinLT2, INPUT);
  
  pinMode(pinButton, INPUT); // 푸시버튼 핀을 입력용 핀으로 설정
  pinMode(pinBuzzer, OUTPUT); // 부저(스피커) 핀을 출력용 핀으로 설정

  pinMode(pinLeftIR, INPUT); // for wall following
  // put your setup code here, to run once:
  bt.begin( 9600 );  // 블루투스 통신 초기화 (속도= 9600 bps)
  //Serial.begin(9600);
  // Red : 빨간색 불빛 켜기
  digitalWrite(pinRGB_Red, HIGH);   // 빨간색 켜기
  digitalWrite(pinRGB_Green, LOW);  // 초록색 끄기
  digitalWrite(pinRGB_Blue, LOW);   // 파란색 끄기
  move(MOTOR_L, REVERSE, 5*28);
  move(MOTOR_R, FORWARD, 5*28);
  delay(500);

  // Yellow : 노란색(빨간색+초록색) 불빛 켜기
  digitalWrite(pinRGB_Red, HIGH);   // 빨간색 켜기
  digitalWrite(pinRGB_Green, HIGH); // 초록색 켜기
  digitalWrite(pinRGB_Blue, LOW);   // 파란색 끄기
  move(MOTOR_L, FORWARD, 5*28);
  move(MOTOR_R, REVERSE, 5*28); // 0~255
  delay(500);

  // Green : 초록색 불빛 켜기
  digitalWrite(pinRGB_Red, LOW);    // 빨간색 끄기
  digitalWrite(pinRGB_Green, HIGH); // 초록색 켜기
  digitalWrite(pinRGB_Blue, LOW);   // 파란색 끄기
  delay(500); 
  
  noTone(pinBuzzer);  

  ledoff();
  stop();

  pinMode(pinMic, INPUT); // 마이크 핀을 입력용 핀으로 설정  
#ifdef NEO
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  
  neoThread.onRun(neoCallback); 
  neoThread.setInterval(20);
#endif

// sonar
  pinMode(pinTrig, OUTPUT); // 출력용 핀으로 설정
  pinMode(pinEcho, INPUT);  // 입력용 핀으로 설정

  worker1.onRun(callback1);
  worker1.setInterval(500);

  worker2.onRun(callback2);
  worker2.setInterval(2000);

  controll.add(&worker1);
  controll.add(&worker2);

  msLastTick = millis();  
}
// test
long interval = 1000;
int motorState = LOW;
long previousMillis = 0;

void timerTest(){
  unsigned long currentMillis = millis();

  if(currentMillis - previousMillis > interval){
    previousMillis = currentMillis;

    if(motorState == LOW){/*
      move(MOTOR_L, REVERSE, 5*28);
      move(MOTOR_R, FORWARD, 5*28);
      digitalWrite(pinRGB_Red, HIGH);   // 빨간색 켜기
      digitalWrite(pinRGB_Green, HIGH); // 초록색 켜기
      digitalWrite(pinRGB_Blue, LOW);   // 파란색 끄기*/
      motorState = HIGH;
    }
    else{/*
      move(MOTOR_L, FORWARD, 5*28);
      move(MOTOR_R, REVERSE, 5*28); // 0~255
      digitalWrite(pinRGB_Red, LOW);    // 빨간색 끄기
      digitalWrite(pinRGB_Green, HIGH); // 초록색 켜기
      digitalWrite(pinRGB_Blue, LOW);   // 파란색 끄기*/
      motorState = LOW;
    }
  }
}
void  Forward( int power )  // 전진
{
  move(MOTOR_L, FORWARD, power);
  move(MOTOR_R, FORWARD, power);
}
void Forward2( int power1, int power2 )
{
  move(MOTOR_L, FORWARD, power1);
  move(MOTOR_R, FORWARD, power2);
}

void  Backward2( int power1, int power2 )
{
  move(MOTOR_L, REVERSE, power1);
  move(MOTOR_R, REVERSE, power2);
}

void loop() {

#ifdef NEO
  if(neoThread.shouldRun())
    neoThread.run();
#endif
  controll.run();
  // put your main code here, to run repeatedly:
  currentTime = millis()/1000.0-lastTime;
  readSerial();
  if(isAvailable){
    unsigned char c = serialRead & 0xff;
    if(c == 0x55 && isStart == false){
      if(prevc == 0xff){
        index = 1;
        isStart = true;
      }
    }else{  // isn't 0x55
      prevc = c;
      if(isStart){  // finded header code
        if(index == 2){ // is index length?
          dataLen = c;
        }else if(index > 2){
          dataLen--;
        }
        writeBuffer(index, c);    // real value
      }
    }

    index++;
    if(index > 51){ // checking max buffer
      index = 0;
      isStart = false;
    }

    if(isStart && dataLen == 0 && index > 3){
      isStart = false;    // reset
      parseData();
      index = 0;
    }
    
#ifdef FADE
    if(bFadeColor){
      // Red -> Green
      FadeColor( pinRGB_Red, pinRGB_Green, pinRGB_Blue );
      
      // Green -> Blue
      FadeColor( pinRGB_Green, pinRGB_Blue, pinRGB_Red );
    
      // Blue -> Red 불빛 변화
      FadeColor( pinRGB_Blue, pinRGB_Red, pinRGB_Green );
    }   
#endif 
  }
  ///////////////////////////////by js /////////////////////////
  if( remocon.decode(&results) ){  // 받은 신호가 있나? (받고나면 수신 차단됨)
    remocon.resume(); // 다음 리모콘 신호를 수신하는 상태로
  }

  if( DriveMode == LINE_TRACER ){
    int  value1 = digitalRead( pinLT1 ); // 첫 번째 LEFT IR 센서 값 읽기
    int  value2 = digitalRead( pinLT2 ); // 두 번째 RIGHT IR 센서 값 읽기
    if( value1 && value2 ){  // 2개의 모두 흰색인 경우 전진
      Forward( Power );
      if( LeftTurn > MAX_TURN_STEP6 )
        LeftTurn = MAX_TURN_STEP5;
      else if( LeftTurn > MAX_TURN_STEP4 )
        LeftTurn = MAX_TURN_STEP3;
      else if( LeftTurn > MAX_TURN_STEP2 )
        LeftTurn = MAX_TURN_STEP1;
      else   
        LeftTurn = 0;
        
      if( RightTurn > MAX_TURN_STEP6 )
        RightTurn = MAX_TURN_STEP5;
      else if( RightTurn > MAX_TURN_STEP4 )
        RightTurn = MAX_TURN_STEP3;
      else if( RightTurn > MAX_TURN_STEP2 )
        RightTurn = MAX_TURN_STEP1;
      else   
        RightTurn = 0;
    }
    else if( (value1 == 0) && value2 ){  // 왼쪽에 검은 라인이 감지되면 좌회전
      if( LeftTurn < MAX_TURN_STEP1 )
        Forward2( Power*7/8, Power );
      else if( LeftTurn < MAX_TURN_STEP2 )
        Forward2( Power*6/8, Power );
      else if( LeftTurn < MAX_TURN_STEP3 )
        Forward2( Power*5/8, Power );
      else if( LeftTurn < MAX_TURN_STEP4 )
        Forward2( Power*4/8, Power );
      else if( LeftTurn < MAX_TURN_STEP5 )
        Forward2( Power*3/8, Power );
      else if( LeftTurn < MAX_TURN_STEP6 )
        Forward2( Power*2/8, Power );
      else if( LeftTurn < MAX_TURN_STEP7 )
        Forward2( Power*1/8, Power );
      else
        Forward2( 0, Power );

     LeftTurn++;
     RightTurn = 0;
   }
   else if( value1 && (value2 == 0) ){  // 오른쪽에 검은 라인이 감지되면 우회전
      if( RightTurn < MAX_TURN_STEP1 )
      Forward2( Power, Power*7/8 );
      else if( RightTurn < MAX_TURN_STEP2 )
      Forward2( Power, Power*6/8 );
      else if( RightTurn < MAX_TURN_STEP3 )
      Forward2( Power, Power*5/8 );
      else if( RightTurn < MAX_TURN_STEP4 )
      Forward2( Power, Power*4/8 );
      else if( RightTurn < MAX_TURN_STEP5 )
      Forward2( Power, Power*3/8 );
      else if( RightTurn < MAX_TURN_STEP6 )
      Forward2( Power, Power*2/8 );
      else if( RightTurn < MAX_TURN_STEP7 )
      Forward2( Power, Power*1/8 );
      else
      Forward2( Power, 0 );
      
      RightTurn++;
      LeftTurn = 0;
    }
    else{  // 두 값이 모두 ??색인 경우        
      Forward( Power );
      
      if( LeftTurn > MAX_TURN_STEP6 )
        LeftTurn = MAX_TURN_STEP5;
      else if( LeftTurn > MAX_TURN_STEP4 )
        LeftTurn = MAX_TURN_STEP3;
      else if( LeftTurn > MAX_TURN_STEP2 )
        LeftTurn = MAX_TURN_STEP1;
      else   
        LeftTurn = 0;
        
      if( RightTurn > MAX_TURN_STEP6 )
        RightTurn = MAX_TURN_STEP5;
      else if( RightTurn > MAX_TURN_STEP4 )
        RightTurn = MAX_TURN_STEP3;
      else if( RightTurn > MAX_TURN_STEP2 )
        RightTurn = MAX_TURN_STEP1;
      else   
        RightTurn = 0;
    }
  }
  else if(DriveMode == AVOIDENCE){
    int distance = GetDistance();
    if(distance < 7){
      isAvoided = true;
      move(MOTOR_L, REVERSE, 100);
      move(MOTOR_R, REVERSE, 100);
      unsigned long msTick = millis() - msLastTick;  
      if(!onlyOne && msTick >= 2000){
        msLastTick = millis();
        onlyOne = true;
        writeHead();
        writeSerial(0xde);
        writeSerial(2);
        writeSerial(6);
        writeEnd();
      }
    }
    else {
      isAvoided = false;
      onlyOne = false;
    }
  }
  else if(DriveMode == WALLFOLLOW){
    int value = analogRead(pinLeftIR);  // 세기 읽기
    if(value > 13){
      move(MOTOR_L, FORWARD, 70);
      move(MOTOR_R, FORWARD, 100);
    }else{
      move(MOTOR_L, FORWARD, 100);
      move(MOTOR_R, FORWARD, 70);
    }
  }
  // read the state of the switch into a local variable:
  int reading = digitalRead(pinButton);

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH),  and you've waited
  // long enough since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;

      // only toggle the LED if the new button state is HIGH
      if (buttonState == HIGH) {
        ledState = !ledState;
      }else{
        if(modeCount == 1){
          DriveMode = LINE_TRACER;
        }else if(modeCount == 3){
          DriveMode = AVOIDENCE;    
          stop();      
        }else if(modeCount == 4){
          DriveMode = WALLFOLLOW;
          stop();
        }else{
          DriveMode = REMOTE_CONTROL;
          stop();
        }
        writeHead();
        writeSerial(0xde);
        writeSerial(2);
        writeSerial(modeCount++);
        writeEnd();
        
        if(modeCount > 5) modeCount = 1;
      }
    }
  }
  // save the reading.  Next time through the loop,
  // it'll be the lastButtonState:
  lastButtonState = reading;  
  
}

//////////////////////////////////// sonar function ////////////////////////////////////
int  GetDistance(){
  
  digitalWrite( pinTrig, LOW ); 
  delayMicroseconds( 2 );   
  digitalWrite( pinTrig, HIGH );
  delayMicroseconds( 10 );  
  digitalWrite( pinTrig, LOW ); 
  int  duration = pulseIn( pinEcho, HIGH );
  int  distance = (duration / 2) / 29.1;   
  if(distance < 0 ) distance = preDistance;
  preDistance = distance;
  return  distance;
}
