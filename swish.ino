#include <spline.h>

int potentiometerPin = A0;
int loopDelay = 100;
//arrays fixed size so we need to determine the maximum size a gesture can be: 10*100=1000ms=1s long gesture
double accel_X[40];
double accel_Y[40];
double accel_Z[40];

int capTouchSensor = 10;

int index = 0;

const int trainedDataSize = 10;

const int numTrainingSamples = 2;

// THIS HOLDS PROCESSED TRAINING DATA
double* trainingX[numTrainingSamples] = {};
double* trainingY[numTrainingSamples] = {};
double* trainingZ[numTrainingSamples] = {};

// THIS HOLDS RAW TRAINING DATA

const int maxRawTrainingSize = 30;

double rawLeftSwipeX[24] = {0};
double rawLeftSwipeY[24] = {0};
double rawLeftSwipeZ[24] = {-0.278625, -0.504822, -0.482117, -0.247803, 0.314636, 1.309998, 1.976196, 1.999939, 1.017334, 0.080139, -0.066895, -0.227661, -0.603394, -0.64856, -0.540832, -0.432861, -0.418518, -0.378052, -0.327087, -0.318787, -0.308838, -0.222107, -0.156128, -0.116272};

double rawRightSwipeX[26] = {0};
double rawRightSwipeY[26] = {0};
double rawRightSwipeZ[26] = {0.174133, 0.184204, -0.157043, -0.593506, -0.968872, -1.244812, -1.364441, -1.285583, -1.054077, -0.595703, -0.22821, -0.007568, 0.177368, 0.274597, 0.542358, 0.819153, 0.980835, 1.131287, 1.313171, 0.868042, 0.499268, 0.302612, 0.237244, 0.13446, 0.028198, -0.005615};

double rawUpSwipeX[23] = {-0.850708, -1.253418, -1.720947, -2.0, -2.0, -1.209351, -0.729797, -0.401001, -0.02124, 0.487305, 0.99231, 1.455078, 1.435425, 1.188171, 0.944031, 0.832397, 0.638916, 0.364441, 0.171753, 0.201416, 0.125183, 0.042297, 0.039368};
double rawUpSwipeY[23] = {0};
double rawUpSwipeZ[23] = {0};

double rawDownSwipeX[28] = {0.211182, 0.534546, 1.451355, 1.999939, 1.999939, 1.999939, 1.999939, 1.925964, 1.108398, 0.145447, -0.226379, -0.076111, -0.421082, -0.711487, -1.337769, -1.551697, -1.566528, -1.665649, -1.486389, -1.043396, -0.509644, -0.730835, -0.812744, -0.355896, -0.046814, 0.024414, 0.056946, 0.066223};
double rawDownSwipeY[28] = {0};
double rawDownSwipeZ[28] = {0};

const int trainingSizes[numTrainingSamples] = {24, 26};

// THIS HOLDS PROCESSED TRAINING DATA
double* rawTrainingX[numTrainingSamples] = {rawLeftSwipeX, rawRightSwipeX};
double* rawTrainingY[numTrainingSamples] = {rawLeftSwipeY, rawRightSwipeY};
double* rawTrainingZ[numTrainingSamples] = {rawLeftSwipeZ, rawRightSwipeZ};

void setup() {
  Serial.begin(9600);

  Serial.println("Starting setup!");
  delay(5000);
  setupMPU();
  processRawData();
  Serial.println("All setup!");
}

void processRawData(){
  Serial.println("processing");
  for (int i = 0; i < numTrainingSamples; i++) {
    Serial.print("RAW TRAINING DATA [gesture="); Serial.print(i); Serial.println("]:");
    //Serial.println("WORKING");
    //printArray(*rawTrainingX, trainingSizes[i]);
    //printArray(*rawTrainingY, trainingSizes[i]);
    //printArray(*rawTrainingZ, trainingSizes[i]);
    int gestSize = trainingSizes[i];
    normalizeHeight(rawTrainingX[i], rawTrainingY[i], rawTrainingZ[i], gestSize);
    //printArray(rawTrainingX[i], trainingSizes[i]);
   
    double* newX = normalizeLength(rawTrainingX[i], trainedDataSize, gestSize);
  
 
    double* newY = normalizeLength(rawTrainingY[i], trainedDataSize, gestSize);
   
    double* newZ = normalizeLength(rawTrainingZ[i], trainedDataSize, gestSize);
    
    //printArray(newX, trainingSizes[i]);
    
    Serial.print("PROCESSESD TRAINING DATA: [gesture="); Serial.print(i); Serial.println("]:");
    //printArray(newX, trainedDataSize);
    //printArray(newY, trainedDataSize);
    //printArray(newZ, trainedDataSize);
    trainingX[i] = newX;
    trainingY[i] = newY;
    trainingZ[i] = newZ;
  }  
}

boolean thisSeqHasBeenClassified = false;

/*
int freeRam() {
extern int __heap_start, *__brkval;
int v;
return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}*/

void loop() {
  //Serial.println(freeRam());
  unsigned long startTime = millis();
  int tickMilliDelay = 25;
  
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
    if (index == maxGestureSize)
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
    
    normalizeHeight(accel_X, accel_Y, accel_Z, index);
    double* newX = normalizeLength(accel_X, trainedDataSize, index);
    double* newY = normalizeLength(accel_Y, trainedDataSize, index);
    double* newZ = normalizeLength(accel_Z, trainedDataSize, index);
    index = 0;
    
    int classification = classify(newX, newY, newZ);
    delete[] newX;
    delete[] newY;
    delete[] newZ;
    
    //printArray(posX, index);
    Serial.print("CLASS: ");

    Serial.println(classification);
    Serial.println();
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
  
  float x[initialLength];
  float y[initialLength];
  //Serial.println("norms");

  for (int i = 0; i < initialLength; i++)  {
    x[i] = i;
    y[i] = float(seq[i]);
  }
  

  Spline linearSpline(x,y,initialLength,1);

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
  
  printArray(gestureZ, trainedDataSize);
  double minLoss = 999999;
  int bestGesture = 0;
  
  for (int i = 0; i < numTrainingSamples; i++){
    double loss = squaredLossDifference(gestureX, trainingX[i], trainedDataSize);
    loss += squaredLossDifference(gestureY, trainingY[i], trainedDataSize);
    loss += squaredLossDifference(gestureZ, trainingZ[i], trainedDataSize);
    Serial.print("Class: "); Serial.print(i); Serial.print(" - loss: "); Serial.println(loss);
    if (loss < minLoss){
      minLoss = loss;
      bestGesture = i;
    }
  }
  
  return bestGesture;
}

double squaredLossDifference(double a[], double b[], int dataSize){
  double sum = 0;
  
  for (int i = 0; i < dataSize; i++){
    if (a[i] == 0.0 || b[i] == 0.0) continue;
    //Serial.println(a[i]);
    sum += (a[i] - b[i]) * (a[i] - b[i]);
  }
  
  //printArray(b, dataSize);
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
const int qSize = 5;
double lastMags[qSize];
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

