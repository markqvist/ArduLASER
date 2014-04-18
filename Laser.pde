#include <AFMotor.h>

boolean debug = true;

int sensorPin = 14;
int laserPin = 13;
int buttonPin = 19;
int switchPin = 17;
int rLEDPin = 2;
int gLEDPin = 9;
int bLEDPin = 10;

AF_Stepper x(24, 2);
AF_Stepper y(24, 1);
static int METHOD = INTERLEAVE;

boolean laserPower;
boolean xPower;
boolean yPower;
boolean standby = true;
boolean calibrated;

int xres = 27;
int xmres = 27;
int yres = 10;
int ymres = 10;

int xpos = 0;
int ypos = 0;
int xsize = 0;
int ysize = 0;
int xmsize = 0;
int ymsize = 0;
int pattern = 0;
int shape = 0;
boolean automode = false;

int sDot = 0;
int sHPlane1 = 1;
int sHPlane2 = 2;
int sHPlane3 = 3;
int sVPlane1 = 4;
int sVPlane2 = 5;
int sVPlane3 = 6;
int sHPlaneL = 7;
int sVPlaneL = 8;
int sSquare1 = 9;
int sSquare2 = 10;
int sSquare3 = 11;
int sSquare4 = 12;
int sSquare5 = 13;
int sSquareR = 14;
int sSmiley = 15;
int sPath = 16;
int sMan = 17;
int sHSweep = 18;
int sVSweep = 19;

int sVarPX = 0;
int sVarPY = 0;
int sVarSquareSize = 1;

int pStill = 0;
int pRandom = 1;
int pSweepX = 2;
int pSweepY = 3;

int tick = 0;
int pSpeed = 20;

int defaultSpeed = 500;
int delayTime = 0;

void setup() {
  calibrated = false;
  
  // Start serial connection
  Serial.begin(9600);
  Serial.println("Starting...");
  
  // Pin setup
  
  pinMode(rLEDPin, OUTPUT);
  pinMode(gLEDPin, OUTPUT);
  pinMode(bLEDPin, OUTPUT);
  
  int blinkI = 30;
  
  while(blinkI--) {
  rOn();
  delay(50);
  rOff();
  gOn();
  delay(50);
  gOff();
  bOn();
  delay(50);
  bOff();
  }
  rOn();

  pinMode(laserPin, OUTPUT); // Laser
  pinMode(sensorPin, INPUT); // Calibration sensor
  pinMode(buttonPin, INPUT);
  pinMode(switchPin, INPUT);
  
  // Initialise random
  randomSeed(analogRead(18));
  
  // Set up motors
  x.setSpeed(defaultSpeed);
  y.setSpeed(defaultSpeed);
  
  // Calibrate mirrors
  if (!switchRead() && calibrate()) {
    calibrated = true;
    Serial.println("Mirrors calibrated");
    rOff();
  } else {
    rOn();
    Serial.println("Could not calibrate mirrors");
    calibrated = false;
  }
  
  if (debug) {
    Serial.println("Entered debug mode");
    calibrated = true;
  }
  
  Serial.println("Ready");
}

boolean calibrate() {
  if (calibrated) {
    Serial.println("Already calibrated, not redoing!");
    return true;
  }
  
  Serial.println("Starting mirror calibration...");
  
  for (int xstep = 0; xstep < 133; xstep++) {
    moveRight(1);
    laserOn();
    if (checkSensor()) return true;
  }
  moveLeft(133);
  
  moveUp(21);
  if (checkSensor()) return true;
  
  for (int ystep = 0; ystep < 42; ystep++) {
    moveDown(1);
    for (int xstep = 0; xstep < 133; xstep++) {
      if (checkSensor()) return true;
      moveRight(1);
    }
    moveLeft(133);
  }
  
  return false;
}

boolean checkSensor() {
  laserOn();
  delay(15);
  if (readSensor() >= 113) {
    laserOff();
    moveLeft(106);
    xpos = 0;
    ypos = 0;
    int blinkI = 30;
    while (blinkI--) {
      laserOn();
      delay(50);
      laserOff();
      delay(50);
    }
    calibrated = true;
    rOff();
    return true;
  } else {
    laserOff();
    return false;
  }
}

void loop() {
  tick++;
  if (tick > 500) tick = 0;
  
  if (!standby) {
    if (!automode) serialHandler();
    if (automode) autoLoop();
    patternLoop();
    shapeLoop();
  }
  
  if (tick % 10 == 0) buttonLoop();
  //Serial.println(tick);
}

void buttonLoop() {
  if (calibrated) rOff();
  if (!standby && buttonRead()) {
    automode = !automode;
    if (automode) {
      bOn();
    }
    if (!automode) {
      bOff();
      pattern = 0;
      shape = 0;
      resetPosition();
    }
    delay(500);
  }
  if (switchRead()) {
    automode = false;
    bOff();
    standby = true;
    pattern = 0;
    shape = 0;
    resetPosition();
    laserOff();
    gOff();
    if (calibrated) rOff();
  } else {
    if (standby) laserOn();
    standby = false;
    if (calibrated == true) {
      gOn();
    }
    if (calibrated == false) calibrate();
  }
}

boolean buttonRead() {
  if (digitalRead(buttonPin) == HIGH) {
    return true;
  } else {
    return false;
  }
}

boolean switchRead() {
    if (digitalRead(switchPin) == HIGH) {
    return true;
  } else {
    return false;
  }
}


void releaseCoils() {
  x.release();
  y.release();
  xpos = 0;
  ypos = 0;
}

void rOn() {
  digitalWrite(rLEDPin, HIGH);
}

void gOn() {
  digitalWrite(gLEDPin, HIGH);
}

void bOn() {
  digitalWrite(bLEDPin, HIGH);
}

void rOff() {
  digitalWrite(rLEDPin, LOW);
}

void gOff() {
  digitalWrite(gLEDPin, LOW);
}

void bOff() {
  digitalWrite(bLEDPin, LOW);
}

void laserOn() {
  digitalWrite(laserPin, HIGH);
  laserPower = true;
}

void laserOff() {
  digitalWrite(laserPin, LOW);
  laserPower = false;
}

void laserToggle() {
  if (laserPower) {
    laserOff();
    Serial.println("Turning laser off");
  } else {
    laserOn();
    Serial.println("Turning laser on");
  }
}

int readSensor() {
  int sensorVal = analogRead(sensorPin);
  Serial.println(sensorVal);
  return sensorVal;
}

void stepX(int delta) {
  xpos += delta;
  if (delta > 0) {
    x.step(delta, BACKWARD, METHOD);
  }
  if (delta < 0) {
    x.step(0-delta, FORWARD, METHOD);
  }
}

void stepY(int delta) {
  ypos += delta;
  if (delta > 0) {
    y.step(delta, BACKWARD, METHOD);
  }
  if (delta < 0) {
    y.step(0-delta, FORWARD, METHOD);
  }
}

void moveLeft(int delta) {
  stepX(0-delta);
}

void moveRight(int delta) {
  stepX(delta);
}

void moveUp(int delta) {
  stepY(delta);
}

void moveDown(int delta) {
  stepY(0-delta);
}

void moveMulti(int deltax, int deltay) {
  moveX(deltax);
  moveY(deltay);
  /*while (deltax != 0 || deltay != 0) {
    if (deltax < 0) {
      stepX(-1);
      deltax++;
    }
    if (deltax > 0) {
      stepX(1);
      deltax--;
    }
    if (deltay < 0) {
      stepY(-1);
      deltay++;
    }
    if (deltay > 0) {
      stepY(1);
      deltay--;
    }
  }*/
  if (delayTime > 0) delay(delayTime);
}

void moveTo(int x, int y) {
  moveMulti(x-xpos, y-ypos);
}

void moveX(int delta) {
  stepX(delta);
}

void moveY(int delta) {
  stepY(delta);
}

void resetPosition() {
  moveTo(0, 0);
}

void autoLoop() {
  //Serial.println("Automode");
  if (tick == 0) {
    pattern = random(1, 3);
    shape = random(0, 20);
    if (shape == 17) pattern = 0;
    pSpeed = random(5, 20);
    if (shape == 7 || shape == 8) tick = 450;
    if (shape == 18 || shape == 19) tick = 492;
    if (shape == 15) tick = 350;
    if (shape == 17) tick = 300;
    Serial.print("Shape is now ");
    Serial.println(shape);
    Serial.print("Pattern is now ");
    Serial.println(pattern);
  }
}

void shapeLoop() {
  if (shape == sDot) {
    delay(10);
  }
  
  if (shape == sSquare1) {
    shapeSquare(1);
  }
  if (shape == sSquare2) {
    shapeSquare(2);
  }
  if (shape == sSquare3) {
    shapeSquare(3);
  }
  if (shape == sSquare4) {
    shapeSquare(4);
  }
  if (shape == sSquare5) {
    shapeSquare(5);
  }
  if (shape == sSquareR) {
    if (tick % pSpeed == 0) sVarSquareSize = random(1, 8);
    shapeSquare(sVarSquareSize);
  }
  
  if (shape == sHPlane1) {
    shapeHPlane(1);
  }
  if (shape == sHPlane2) {
    shapeHPlane(2);
  }
  if (shape == sHPlane3) {
    shapeHPlane(3);
  }
  
  if (shape == sVPlane1) {
    shapeVPlane(1);
  }
  if (shape == sVPlane2) {
    shapeVPlane(2);
  }
  if (shape == sVPlane3) {
    shapeVPlane(3);
  }
  
  if (shape == sHPlaneL) {
    shapeHPlaneL();
  }
  if (shape == sVPlaneL) {
    shapeVPlaneL();
  }
  
  if (shape == sSmiley) {
    shapeSmiley();
  }
  
  if (shape == sPath) {
    shapePath();
  }
  
  if (shape == sMan) {
    shapeMan();
  }
  
  if (shape == sHSweep) {
    shapeHSweep();
  }
  
  if (shape == sVSweep) {
    shapeVSweep();
  }
  
  
}

void patternLoop() {
  if (pattern == pStill) {
    
  }
  
  if (pattern == pRandom) {
    if (tick%pSpeed == 0) {
      int xmax = xres-xsize;
      int ymax = yres-ysize;
      int xmin = 0-xmres+xmsize;
      int ymin = 0-ymres+ymsize;
      int newX = random(xmin, xmax);
      int newY = random(ymin, ymax);
      laserOff();
      moveTo(newX, newY);
      laserOn();
    }
  }
  
  if (pattern == pSweepX) {
    if (tick%pSpeed == 0) {
      if (xpos+xsize >= xres) {
        laserOff();
        moveTo(0-xmres+xmsize, ypos);
        laserOn();
      } else {
        laserOff();
        moveTo(xpos+1, ypos);
        laserOn();
      }
    }
  }
  
  if (pattern == pSweepY) {
    if (tick%pSpeed == 0) {
      if (ypos+ysize >= yres) {
        laserOff();
        moveTo(xpos, 0-ymres+ymsize);
        laserOn();
      } else {
        laserOff();
        moveTo(xpos, ypos+1);
        laserOn();
      }
    }
  }
}

void serialHandler() {
  while (Serial.available() > 0) {
		// read the incoming byte:
		int incomingByte = Serial.read();

                if (incomingByte == 108) {
                  laserToggle();
                }
                
                if (incomingByte == 114) {
                  releaseCoils();
                }
                
                // Key up
                if (incomingByte == 65) {
                  moveUp(1);
                }
                
                // Key down
                if (incomingByte == 66) {
                  moveDown(1);
                }
                
                // Key right
                if (incomingByte == 67) {
                  moveRight(1);
                }
                
                // Key left
                if (incomingByte == 68) {
                  moveLeft(1);
                }
                
                if (incomingByte == 99) {
                  resetPosition();
                }
                
                if (incomingByte == 43) {
                  defaultSpeed += 10;
                  Serial.print("Speed is now: ");
                  Serial.println(defaultSpeed);
                  x.setSpeed(defaultSpeed);
                  y.setSpeed(defaultSpeed);
                }
                
                if (incomingByte == 45) {
                  defaultSpeed -= 10;
                  Serial.print("Speed is now: ");
                  Serial.println(defaultSpeed);
                  x.setSpeed(defaultSpeed);
                  y.setSpeed(defaultSpeed);
                }
                
                if (incomingByte == 166) {
                  delayTime++;
                  Serial.print("Delay is now: ");
                  Serial.println(delayTime);
                }
                
                if (incomingByte == 184) {
                  delayTime--;
                  Serial.print("Delay is now: ");
                  Serial.println(delayTime);
                }
                
                if (incomingByte == 103) {
                  xpos = 0;
                  ypos = 0;
                }
                
                if (incomingByte == 112) {
                  pattern++;
                  if (pattern > 3) pattern = 0;
                  Serial.print("Cycling pattern to ");
                  Serial.println(pattern);
                  //resetPosition();
                }
                
                if (incomingByte == 115) {
                  shape++;
                  if (shape > 19) shape = 0;
                  Serial.print("Cycling shape to ");
                  Serial.println(shape);
                  resetPosition();
                }
                
                if (incomingByte == 97) {
                  automode = true;
                }
                
                if (incomingByte == 106) {
                  readSensor();
                }
                
                //Serial.print("Position: x=");
                //Serial.print(xpos);
                //Serial.print(" y=");
                //Serial.println(ypos);
		Serial.println(incomingByte, DEC);
	}
}

// Shapes

void shapeHPlane(int size) {
  xsize = size;
  ysize = 0;
  xmsize = 0;
  ymsize = 0;
  moveTo(xpos+size, ypos);
  moveTo(xpos-size, ypos);
}

void shapeVPlane(int size) {
  xsize = 0;
  ysize = size;
  xmsize = 0;
  ymsize = 0;
  moveTo(xpos, ypos+size);
  moveTo(xpos, ypos-size);
}

void shapeHPlaneL() {
  xsize = 27;
  ysize = 0;
  xmsize = -27;
  ymsize = 0;
  moveTo(-27, ypos);
  moveTo(27, ypos);
}

void shapeVPlaneL() {
  xsize = 0;
  ysize = 8;
  xmsize = 0;
  ymsize = -12;
  moveTo(xpos, -12);
  moveTo(xpos, 8);
}

void shapeSquare(int size) {
  xsize = size;
  ysize = size;
  xmsize = size;
  ymsize = size;
  moveX(size);
  moveY(0-size);
  moveX(0-size);
  moveY(size);
}

void shapeSmiley() {
  moveX(5);
  moveY(-5);
  moveX(-5);
  moveY(5);
  
  laserOff();
  moveTo(xpos+1, ypos-1);
  laserOn();
  
  moveX(1);
  moveY(-1);
  moveX(-1);
  moveY(1);
  
  laserOff();
  moveTo(xpos+2, ypos);
  laserOn();
  
  moveX(1);
  moveY(-1);
  moveX(-1);
  moveY(1);

  laserOff();
  moveTo(xpos-2, ypos-2);
  laserOn();
  
  moveX(3);
  moveY(-1);
  moveX(-3);
  moveY(1);
  
  laserOff();
  moveTo(xpos-1, ypos+3);
  laserOn();
}

void shapePath() {
  if (tick % 3 == 0) {
    int dir = random(0,4);
    if (dir == 0) {
      sVarPX = 1;
      sVarPY = 0;
    }
    if (dir == 1) {
      sVarPX = -1;
      sVarPY = 0;
    }
    if (dir == 2) {
      sVarPX = 0;
      sVarPY = 1;
    }
    if (dir == 3) {
      sVarPX = 0;
      sVarPY = -1;
    }
  }
  
  moveX(sVarPX);
  moveY(sVarPY);
  
  if (xpos > xres || xpos < -xmres || ypos > yres || ypos < -ymres) { 
    moveTo(0,0);
  }
  //moveTo(xpos+sVarPX, ypos+sVarPY);
}

void shapeMan() {
  delayTime = 2;
  laserOn();
  moveX(2);
  moveY(2);
  moveX(-2);
  moveY(-2);
  laserOff();
  moveX(1);
  laserOn();
  moveY(-3);  
  laserOff();
  moveX(-1);
  moveY(-1);
  laserOn();
  moveY(1);
  moveX(2);
  moveY(-1);
  
  laserOff();
  
  if (tick % 6 > 2) {
    moveY(2);
    laserOn();
    moveY(1);
    moveX(-3);
  } else {
    moveX(1);
    moveY(3);
    laserOn();
    moveX(-3);
    moveY(-1);
  }
  
  laserOff();
  
  moveTo(0, 0);
  delayTime = 0;
}

void shapeHSweep() {
  laserOff();
  moveTo(-xmres, -ymres);
  laserOn();
  for (int i = 0; i <= yres; i++) {
    moveX(xres*2);
    moveY(1);
    moveX(-xres*2);
    moveY(1);
  }
  laserOff();
}

void shapeVSweep() {
  laserOff();
  moveTo(-xmres, -ymres);
  laserOn();
  for (int i = 0; i <= xres; i++) {
    moveY(yres*2);
    moveX(1);
    moveY(-yres*2);
    moveX(1);
  }
  laserOff();
}

void rotationTest() {
  while (1) {
    moveLeft(1);
    moveUp(1);
  }
}

