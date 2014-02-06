#include <spline.h>

const byte ARROW_LEFT = 0x50;
const byte ARROW_RIGHT = 0x4F;
const byte ARROW_UP = 0x51;
const byte ARROW_DOWN = 0x52;

int potentiometerPin = A0;
byte loopDelay = 100;
//arrays fixed size so we need to determine the maximum size a gesture can be: 10*100=1000ms=1s long gesture

const int maxGestSize = 40;
double* accel_X = new double[maxGestSize];
double* accel_Y = new double[maxGestSize];
double* accel_Z = new double[maxGestSize];

//int capTouchSensor = 10;

int index = 0;

const byte trainedDataSize = 10;
const int minGestSize = 10;

const byte numTrainingSamples = 4;

byte gestToKeyCodes[numTrainingSamples] = {ARROW_LEFT, ARROW_RIGHT, ARROW_DOWN, ARROW_UP};

// THIS HOLDS RAW TRAINING DATA

const byte maxRawTrainingSize = 30;

const double leftSwipeX[trainedDataSize] = {0};
const double leftSwipeY[trainedDataSize] = {0};
const double leftSwipeZ[trainedDataSize] = {-0.1393167, -0.1629588, 0.7660555, 0.5086825, -0.0870388, -0.3063347, -0.2092654, -0.1607817, -0.1000600, -0.0581378};

const double rightSwipeX[trainedDataSize] = {0};
const double rightSwipeY[trainedDataSize] = {0};
const double rightSwipeZ[trainedDataSize] = {0.1276222, -0.3994384, -0.9805164, -0.5485724, 0.0697532, 0.4876562, 0.8735553, 0.3338853, 0.0898926, -0.0041152};

const double downSwipeX[trainedDataSize] = {0.05, 0.50, 0.99, 0.01, -0.28, -0.78, -0.60, -0.40, 0.01, 0.03};
const double downSwipeY[trainedDataSize] = {0};
const double downSwipeZ[trainedDataSize] = {0};

const double upSwipeX[trainedDataSize] = {-0.42, -0.94, -0.58, -0.07, 0.55, 0.62, 0.38, 0.10, 0.04, 0.02};
const double upSwipeY[trainedDataSize] = {0};
const double upSwipeZ[trainedDataSize] = {0};

// THIS HOLDS PROCESSED TRAINING DATA
const double* trainingX[numTrainingSamples] = {leftSwipeX, rightSwipeX, downSwipeX, upSwipeX};
const double* trainingY[numTrainingSamples] = {leftSwipeY, rightSwipeY, downSwipeY, upSwipeY};
const double* trainingZ[numTrainingSamples] = {leftSwipeZ, rightSwipeZ, downSwipeZ, upSwipeZ};

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
  
  /*double rawUpSwipeX[23] = {-0.850708, -1.253418, -1.720947, -2.0, -2.0, -1.209351, -0.729797, -0.401001, -0.02124, 0.487305, 0.99231, 1.455078, 1.435425, 1.188171, 0.944031, 0.832397, 0.638916, 0.364441, 0.171753, 0.201416, 0.125183, 0.042297, 0.039368};


    double rawDownSwipeX[28] = {0.211182, 0.534546, 1.451355, 1.999939, 1.999939, 1.999939, 1.999939, 1.925964, 1.108398, 0.145447, -0.226379, -0.076111, -0.421082, -0.711487, -1.337769, -1.551697, -1.566528, -1.665649, -1.486389, -1.043396, -0.509644, -0.730835, -0.812744, -0.355896, -0.046814, 0.024414, 0.056946, 0.066223};
    double zeros1[23] = {0};
    double zeros2[23] = {0};  
    double zeros3[28] = {0};
    double zeros4[28] = {0};    
    //normalizeHeight(rawUpSwipeX, zeros1, zeros2, 23);
    normalizeHeight(rawDownSwipeX, zeros1, zeros2, 28);
    //printArray(rawTrainingX[i], trainingSizes[i]);
   
    //double* newX = normalizeLength(rawUpSwipeX, trainedDataSize, 23);
    double* newX2 = normalizeLength(rawDownSwipeX, trainedDataSize, 28);
    //printArray(newX, 10);
    printArray(newX2, 10);
    */
  ////////
  
  
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
  
  // 1000/tickMilliDelay times per second, run this code
  // Make sure the code can run in less than 1000/tickMilliDelay seconds.

  pollMPU(); 
  
  int maxGestureSize = 1000 / tickMilliDelay;
  
  
  /*if (!isCapTouchHigh()) {
    return;
  }*/
  
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
    */
    
    delete[] newX;
    delete[] newY;
    delete[] newZ;
    
    //printArray(posX, index);
    Serial.print("CLASS: ");

    Serial.println(classification);
    sendViaBluetooth(gestToKeyCodes[classification]);
    Serial.println();
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
  //Serial.println(desiredLength);
  
  float* x = new float[initialLength];
  float* y = new float[initialLength];
  //Serial.println("norms");

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
    Serial.print(" ");
  }
  Serial.println();
}


// A simple queue with a pointer to the front of it
const byte qSize = 5;
double* lastMags = new double[qSize];
int front = 0;

boolean gestTakingPlace(int maxGestureSize){
  double thresh = 0.1;
  
  // current mag
  double magnitude = sqrt(accelX() * accelX()) + (accelY() * accelY()) + (accelZ() * accelZ());
  
  // push it on the to queue, and pop and releast the front
  int back = front - 1 % qSize;

  lastMags[back] = magnitude;
  front++;
  if (front == qSize) front = 0;
  
  double sum = 0.0;
  for (int i = 0; i < qSize; i++){
    sum += lastMags[i];
  }
  
  double avgMag = sum / qSize;
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

  return cycles >= 3;
}
*/

void sendViaBluetooth(byte b){
  sendViaBluetoothRaw(b);
  releaseKeys();
}

void sendViaBluetoothRaw(byte b){
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

void releaseKeys(){
  sendViaBluetoothRaw(0x00);
}



