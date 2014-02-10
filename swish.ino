#include <spline.h>

const byte ARROW_LEFT = 0x50;
const byte ARROW_RIGHT = 0x4F;
const byte ARROW_UP = 0x10;
const byte ARROW_DOWN = 0x20;

// Consumer keys
const byte PLAY_PAUSE = 0x80;
const byte VOL_UP = 0x10;
const byte VOL_DOWN = 0x20;

const byte ZERO_HEX  = 0x00;
const byte NEXT_HIGH = 0x02;
const byte PREV_HIGH = 0x04;

int potentiometerPin = A0;
byte loopDelay = 100;
//arrays fixed size so we need to determine the maximum size a gesture can be: 10*100=1000ms=1s long gesture

const int maxGestSize = 40;
double* accel_X = new double[maxGestSize];
double* accel_Y = new double[maxGestSize];
double* accel_Z = new double[maxGestSize];

const byte capTouchSensor = 8;

int index = 0;

const byte trainedDataSize = 10;
const int minGestSize = 10;

const byte numTrainingSamples = 2;

int gestToKeyCodes[numTrainingSamples] = {ARROW_LEFT, ARROW_RIGHT};

// THIS HOLDS RAW TRAINING DATA

const byte maxRawTrainingSize = 30;

const double leftSwipeX[trainedDataSize] = {0};
const double leftSwipeY[trainedDataSize] = {0};
const double leftSwipeZ[trainedDataSize] = {0.26, 1.00, 1.00, 0.19, -0.04, -0.97, -0.97, -0.41, -0.05, -0.11};

const double rightSwipeX[trainedDataSize] = {0};
const double rightSwipeY[trainedDataSize] = {0};
const double rightSwipeZ[trainedDataSize] = {-0.12, -1.00, -0.58, -0.27, 0.29, 0.75, 0.76, 0.20, 0.07, 0.05};


// THIS HOLDS PROCESSED TRAINING DATA
const double* trainingX[numTrainingSamples] = {leftSwipeX, rightSwipeX};
const double* trainingY[numTrainingSamples] = {leftSwipeY, rightSwipeY};
const double* trainingZ[numTrainingSamples] = {leftSwipeZ, rightSwipeZ};

void setup() {
  Serial.begin(9600);
  Serial1.begin(115200);
  Serial.println("Starting setup!");
  
  //BPMod.begin(BP_MODE_HID,BP_HID_KEYBOARD);   //Begin HID Mode with HID KEYBOARD AS TYPE
  
  delay(5000);
  setupMPU();
  //processRawData();
  Serial.println("All setup!");
  
  ////////
  
  
  /*
    //double rawRightSwipeZ[23] = {-0.232361, -0.936279, -2.0, -2.0, -2.0, -1.183228, -0.907532, -0.719299, -0.441345, -0.093933, 0.389099, 1.211365, 1.746399, 1.428345, 1.501282, 1.567505, 1.442688, 0.856995, 0.340515, 0.137573, 0.131714, 0.156494, 0.090088};
    //double rawLeftSwipeZ[20] = {0.512085, 1.697144, 1.999939, 1.999939, 1.999939, 1.999939, 0.217651, 0.463806, 0.215942, -0.123169, -1.258423, -1.929321, -2.0, -1.987976, -1.871948, -1.215698, -0.520203, -0.122131, -0.104431, -0.213379};
    //double rawUpSwipeX[25] = {0.20636, 1.084106, 1.616028, 1.664612, 1.918274, 1.999939, 1.81842, 1.136291, 0.565369, 0.057312, -0.560425, -1.312805, -2.0, -2.0, -2.0, -0.957214, -0.547607, -0.600159, -0.637573, -0.222534, 0.186157, 0.252991, 0.064392, -0.130432, 0.0354};
    double rawDownSwipeX[21] = {-0.139465, -0.73999, -2.0, -2.0, -2.0, -2.0, -2.0, -0.951233, 0.566895, 1.75061, 1.999939, 1.999939, 1.999939, 1.999939, 1.786499, 0.801636, 0.236023, 0.185242, -0.039185, -0.127441, -0.072815};
  
    //double zeros1[23] = {0};
    //double zeros12[23] = {0};
    //double zeros2[20] = {0};  
    //double zeros3[25] = {0};
    double zeros4[21] = {0};    
    
    //normalizeHeight(rawRightSwipeZ, zeros1, zeros12, 23);
    //normalizeHeight(rawLeftSwipeZ,  zeros2, zeros2, 20);
    //normalizeHeight(rawUpSwipeX,    zeros3, zeros3, 25);
    normalizeHeight(rawDownSwipeX,  zeros4, zeros4, 21);
    
    //printArray(rawRightSwipeZ, 23);
   
    //double* new1 = normalizeLength(rawRightSwipeZ, trainedDataSize, 23);
    //double* new2 = normalizeLength(rawLeftSwipeZ, trainedDataSize, 20);
    //double* new3 = normalizeLength(rawUpSwipeX, trainedDataSize, 25);
    double* new4 = normalizeLength(rawDownSwipeX, trainedDataSize, 21);
    
    //printArray(new1, 10);
    //printArray(new2, 10);
    //printArray(new3, 10);
    printArray(new4, 10);
    */
  ////////
  
  pinMode(capTouchSensor, INPUT);
  Serial.println(freeRam());
}


boolean thisSeqHasBeenClassified = false;


int freeRam() {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}


void loop() {
  
  unsigned long startTime = millis();
  byte tickMilliDelay = 25;
  //Serial.println(accelX());
  // 1000/tickMilliDelay times per second, run this code
  // Make sure the code can run in less than 1000/tickMilliDelay seconds.
  
  pollMPU(); 
  //Serial.println("AFTER");
  int maxGestureSize = 1000 / tickMilliDelay;
  
  if (digitalRead(capTouchSensor) == LOW) {
    index = 0;
    return;
  }
  
  if (gestTakingPlace(maxGestureSize))
  {
    thisSeqHasBeenClassified = false;
    if (index == maxGestSize)
    {
      index = 0;
      return;
    }
    
    accel_X[index] = accelX();
    accel_Y[index] = accelY();
    accel_Z[index] = accelZ();

    
    Serial.print(accel_X[index], 6);
    Serial.print(" ");
    Serial.print(accel_Y[index], 6);
    Serial.print(" ");
    Serial.print(accel_Z[index], 6);
    Serial.println();
    
    
    index++;

    delay(tickMilliDelay-(millis()-startTime));
  } else if (!thisSeqHasBeenClassified) {
    
    thisSeqHasBeenClassified = true;
    
    if (index < minGestSize){
      Serial.println();
      index = 0;
      return;
    }
    
    normalizeHeight(accel_X, accel_Y, accel_Z, index);
    double* newX = normalizeLength(accel_X, trainedDataSize, index);
    double* newY = normalizeLength(accel_Y, trainedDataSize, index);
    double* newZ = normalizeLength(accel_Z, trainedDataSize, index);
    index = 0;
    
    // Classification is an int that's >= 0
    int classification = classify(newX, newY, newZ);
    
    /*
    
    if (classification == 0) {
      BPMod.keyboardPress(BP_KEY_LEFT_ARROW, BP_MOD_NOMOD);
    }
    else if (classification == 1) {
      BPMod.keyboardPress(BP_KEY_RIGHT_ARROW, BP_MOD_NOMOD);
    }
    BPMod.keyboardReleaseAll();
    index*/
    
    delete[] newX;
    delete[] newY;
    delete[] newZ;
    
    //printArray(posX, index);
    Serial.print("CLASS: ");

    Serial.println(classification);
    Serial.println();
    
    if (classification == 3){
      sendConsumerKey(ZERO_HEX, NEXT_HIGH);
     // sendConsumerKey(VOL_UP);
      //sendConsumerKey(VOL_UP);
      return;
    } else if (classification == 2) {
      sendConsumerKey(ZERO_HEX, PREV_HIGH);
      //sendConsumerKey(VOL_DOWN);
      //sendConsumerKey(VOL_DOWN);
      return;
    }
    
    sendViaBluetooth(gestToKeyCodes[classification]);
    
    Serial.println(freeRam());
  }
}

// returns 0 to 1023 mapped to 0-5v
int pollTouchPad(){
  return analogRead(potentiometerPin);
}


double* normalizeLength(double seq[], int desiredLength, int initialLength)
{
  //Serial.println("normLength");
  
  float* x = new float[initialLength];
  float* y = new float[initialLength];

   
  for (int i = 0; i < initialLength; i++)  {
    x[i] = i;
    y[i] = float(seq[i]);
  }
  
  Spline linearSpline(x,y,initialLength,1);
  
  delete[] x;
  delete[] y;
  double* normalized = new double[desiredLength];

  //normalized[desiredLength];
  double slope = double(initialLength)/(desiredLength-1);
  //Serial.println(initialLength);
  //Serial.println(desiredLength);

  //Serial.println("find the slope");
  for (int i=0; i < desiredLength; i++)
  {
    double xVal = i*slope;
    normalized[i] = linearSpline.value(xVal);
  }
  //printArray(normalized, desiredLength);
  //printArray(seq, initialLength);
  //Serial.println("returning from norm length");
  //printArray(normalized, desiredLength);
  return normalized;
}

void normalizeHeight(double* x, double* y, double* z, int seqSize){
  //printArray(x, seqSize);
  
  int maxI = 0;
  double maxVal = 0.0;
  
  for (int d = 0; d < 3; d++){
    for (int i = 0; i < seqSize; i++){
      double val;
      if (d == 0){
        val = x[i];
      } else if (d == 1){
        val = y[i];
      } else {
        val = z[i];
      }
      
      if (abs(val) > maxVal){
        maxI = i;
        maxVal = abs(val);
      }
    }
  }
  
  //Serial.println("MAX:"); Serial.println(maxVal);
  
  for (int i = 0; i < seqSize; i++){
    //if (maxVal != 0.0) {
      x[i] = x[i] / maxVal;
      y[i] = y[i] / maxVal;
      z[i] = z[i] / maxVal;
    //}
  }
}

int classify(double* gestureX, double* gestureY, double* gestureZ){
  //printArray(gestureZ, trainedDataSize);
  double minLoss = 999999;
  int bestGesture = 0;
  
  for (int i = 0; i < numTrainingSamples; i++){
    double loss = squaredLossDifference(gestureX, trainingX[i], trainedDataSize);
    loss += squaredLossDifference(gestureY, trainingY[i], trainedDataSize);
    loss += squaredLossDifference(gestureZ, trainingZ[i], trainedDataSize);
    //Serial.print("Class: "); Serial.print(i); Serial.print(" - loss: "); Serial.println(loss);
    if (loss < minLoss){
      minLoss = loss;
      bestGesture = i;
    }
  }
  
  return bestGesture;
}

double squaredLossDifference(double a[], const double b[], int dataSize){
  double sum = 0;
  
  for (int i = 0; i < dataSize; i++){
    if (a[i] == 0.0 || b[i] == 0.0) continue;
    //Serial.println(a[i]);
    sum += (a[i] - b[i]) * (a[i] - b[i]);
  }
  
  //Serial.println(sum);
  return sum;
}


void printArray(double* a, int length){
  for (int i = 0; i < length; i++){
    Serial.print(a[i]);
    Serial.print(", ");
  }
  Serial.println();
}


// A simple queue with a pointer to the front of it
const byte qSize = 4;
double* lastMags = new double[qSize];
int front = 0;

boolean gestTakingPlace(int maxGestureSize){
  double thresh = 0.1;
  
  // current mag
  double magnitude = sqrt(accelX() * accelX()) + (accelY() * accelY()) + (accelZ() * accelZ());
  
  // push it on the to queue, and pop and releast the front
  //Serial.println(front);
  lastMags[front] = magnitude;
  front++;
  if (front == qSize) front = 0;
  
  double sum = 0.0;
  for (int i = 0; i < qSize; i++){
    sum += lastMags[i];
  }
  
  double avgMag = sum / qSize;
  //printArray(lastMags, qSize);
  //if (avgMag > thresh) Serial.println(avgMag);
  return avgMag > thresh;
}

/*
boolean isCapTouchHigh() {
  // Variables used to translate from Arduino to AVR pin naming
  volatile uint8_t* port;
  volatile uint8_t* ddr;
  volatile uint8_t* pin;
  // Here we translate the input pin number from
  //  Arduino pin number to the AVR PORT, PIN, DDR,
  //  and which bit of those registers we care about.
  byte bitmask;
  port = portOutputRegister(digitalPinToPort(capTouchSensor));
  ddr = portModeRegister(digitalPinToPort(capTouchSensor));
  bitmask = digitalPinToBitMask(capTouchSensor);
  pin = portInputRegister(digitalPinToPort(capTouchSensor));
  // Discharge the pin first by setting it low and output
  *port &= ~(bitmask);
  *ddr  |= bitmask;
  delay(1);
  // Prevent the timer IRQ from disturbing our measurement
  noInterrupts();
  // Make the pin an input with the internal pull-up on
  *ddr &= ~(bitmask);
  *port |= bitmask;

  // Now see how long the pin to get pulled up. This manual unrolling of the loop
  // decreases the number of hardware cycles between each read of the pin,
  // thus increasing sensitivity.
  uint8_t cycles = 17;
       if (*pin & bitmask) { cycles =  0;}
  else if (*pin & bitmask) { cycles =  1;}
  else if (*pin & bitmask) { cycles =  2;}
  else if (*pin & bitmask) { cycles =  3;}
  else if (*pin & bitmask) { cycles =  4;}
  else if (*pin & bitmask) { cycles =  5;}
  else if (*pin & bitmask) { cycles =  6;}
  else if (*pin & bitmask) { cycles =  7;}
  else if (*pin & bitmask) { cycles =  8;}
  else if (*pin & bitmask) { cycles =  9;}
  else if (*pin & bitmask) { cycles = 10;}
  else if (*pin & bitmask) { cycles = 11;}
  else if (*pin & bitmask) { cycles = 12;}
  else if (*pin & bitmask) { cycles = 13;}
  else if (*pin & bitmask) { cycles = 14;}
  else if (*pin & bitmask) { cycles = 15;}
  else if (*pin & bitmask) { cycles = 16;}

  // End of timing-critical section
  interrupts();

  // Discharge the pin again by setting it low and output
  //  It's important to leave the pins low if you want to 
  //  be able to touch more than 1 sensor at a time - if
  //  the sensor is left pulled high, when you touch
  //  two sensors, your body will transfer the charge between
  //  sensors.
  *port &= ~(bitmask);
  *ddr  |= bitmask;
  Serial.println(cycles);
  return cycles >= 2;
}
*/

void sendViaBluetooth(byte b){
  sendViaBluetoothRaw(b);
  releaseKeys();
}

void sendViaBluetoothRaw(int b){
  Serial1.write((byte)0xFD); //Start HID Report
  Serial1.write((byte)0x9); //Length byte
  Serial1.write((byte)0x1); //Descriptor byte
  Serial1.write((byte)0x00); //Modifier byte
  Serial1.write((byte)0x00); //-
  
  Serial1.write(b); //Send KEY
  
  for(byte i = 0 ; i < 5;i++){ //Send five zero bytes
    Serial1.write((byte)0x00);
  } 
}

void sendConsumerKey(byte low, byte high){
  Serial1.write((byte)0xFD);
  Serial1.write((byte)0x03);
  Serial1.write((byte)0x03);
  
  Serial1.write(low);
  Serial1.write(high);

  Serial1.write((byte)0xFD);
  Serial1.write((byte)0x03);
  Serial1.write((byte)0x03);
  
  Serial1.write((byte)0x00);
  Serial1.write((byte)0x00);
    
}



void releaseKeys(){
  sendViaBluetoothRaw(0x00);
}



