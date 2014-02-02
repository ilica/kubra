#include <spline.h>

int potentiometerPin = A0;
int loopDelay = 100;
//arrays fixed size so we need to determine the maximum size a gesture can be: 10*100=1000ms=1s long gesture
double accel_X[40];
double accel_Y[40];
double accel_Z[40];

/*
double posX[40] = {};
double posY[40] = {};
double posZ[40] = {};
*/

int index = 0;

const int trainedDataSize = 10;

double rightSwipeX[trainedDataSize] = {};
double rightSwipeY[trainedDataSize] = {};
double rightSwipeZ[trainedDataSize] = {};

double leftSwipeX[trainedDataSize]  = {};
double leftSwipeY[trainedDataSize]  = {};
double leftSwipeZ[trainedDataSize]  = {};

const int numTrainingSamples = 2;

// THIS HOLDS PROCESSED TRAINING DATA
double* trainingX[numTrainingSamples] = {leftSwipeX, rightSwipeX};
double* trainingY[numTrainingSamples] = {leftSwipeY, rightSwipeY};
double* trainingZ[numTrainingSamples] = {leftSwipeZ, rightSwipeZ};

//need to add gyro later

// THIS HOLDS RAW TRAINING DATA

const int maxRawTrainingSize = 30;

double rawLeftSwipeX[27] = {0.026306, 0.021729, 0.078491, 0.181458, 0.315002, 0.276672, 0.425842, 0.463074, 0.029358, 0.070129, 0.190247, 0.018372, -0.079651, -0.091248, -0.134338, -0.185364, -0.171997, -0.191223, -0.198486, -0.167542, -0.143982, -0.082214, -0.144775, -0.097351, -0.027283, 0.003296, 0.03772};
double rawLeftSwipeY[27] = {-0.277588, -0.323975, -0.146606, 0.500488, 1.063477, 1.466003, 1.472534, 1.589111, 1.554871, 1.124023, 0.640564, 0.202209, -0.15033, -0.34613, -0.473328, -0.667725, -0.726562, -0.720764, -0.524658, -0.364868, -0.433655, -0.466614, -0.3255, -0.216797, -0.212769, -0.220642, -0.178528};
double rawLeftSwipeZ[27] = {-0.189026, -0.123291, -0.128052, 0.021729, 0.481812, 0.860291, 1.168884, 1.211731, 1.086487, 0.853027, 0.377502, 0.039612, -0.007813, -0.190308, -0.412964, -0.40863, -0.392578, -0.413391, -0.3526, -0.344543, -0.329712, -0.265381, -0.189087, -0.200684, -0.200439, -0.141235, -0.112549};

double rawRightSwipeX[33] = {0.112061, 0.054749, 0.05957, 0.070435, 0.023071, -0.071472, -0.137207, -0.548096, -1.014343, -0.688538, -0.500183, -0.534546, -0.407898, -0.219604, 0.055847, 0.165161, 0.19928, 0.189819, 0.191772, 0.329285, 0.307861, 0.232483, 0.268982, 0.305908, 0.188782, 0.151489, 0.143188, 0.209351, 0.152893, 0.045959, 0.043213, 0.094727, 0.056824};
double rawRightSwipeY[33] = {0.151611, 0.23407, 0.326294, 0.488342, 0.21991, -0.584534, -2.0, -2.0, -2.0, -1.743713, -1.382202, -1.125427, -0.790771, -0.403137, 0.465454, 1.017639, 1.035217, 0.983337, 0.726807, 0.750732, 0.722351, 0.606384, 0.600952, 0.490967, 0.331421, 0.27124, 0.295471, 0.381958, 0.391602, 0.201843, 0.139709, 0.105957, 0.025085};
double rawRightSwipeZ[33] = {-0.007324, 0.045898, 0.188354, 0.276917, 0.130127, -0.82605, -1.424133, -1.804382, -1.33197, -0.960999, -0.727417, -0.470154, -0.351318, -0.030823, 0.187012, 0.387634, 0.640442, 0.633423, 0.548035, 0.492004, 0.451782, 0.457825, 0.428345, 0.378723, 0.329041, 0.345337, 0.305786, 0.231567, 0.25, 0.290039, 0.190125, 0.136108, 0.166138};

int trainingSizes[] = {27, 33};

// THIS HOLDS PROCESSED TRAINING DATA
double* rawTrainingX[numTrainingSamples] = {rawLeftSwipeX, rawRightSwipeX};
double* rawTrainingY[numTrainingSamples] = {rawLeftSwipeY, rawRightSwipeY};
double* rawTrainingZ[numTrainingSamples] = {rawLeftSwipeZ, rawRightSwipeZ};

void setup() {
  //Serial.begin(9600);

  Serial.println("Starting setup!");
  delay(5000);
  setupMPU();
  processRawData();
  Serial.println("All setup!");
}

void processRawData(){
  for (int i = 0; i < numTrainingSamples; i++) {
    Serial.print("RAW TRAINING DATA [gesture="); Serial.print(i); Serial.println("]:");
    Serial.println("WORKING");
    //printArray(*rawTrainingX, trainingSizes[i]);
    //printArray(*rawTrainingY, trainingSizes[i]);
    //printArray(*rawTrainingZ, trainingSizes[i]);
    normalizeHeight(rawTrainingX[i], rawTrainingY[i], rawTrainingZ[i], trainingSizes[i]);
    //printArray(rawTrainingX[i], trainingSizes[i]);
    double* newX = normalizeLength(rawTrainingX[i], trainedDataSize, trainingSizes[i]);
    double* newY = normalizeLength(rawTrainingY[i], trainedDataSize, trainingSizes[i]);
    double* newZ = normalizeLength(rawTrainingZ[i], trainedDataSize, trainingSizes[i]);
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

void loop() {
  Serial.println("LOOPING");
  unsigned long startTime = millis();
  int tickMilliDelay = 25;
    
  // 1000/tickMilliDelay times per second, run this code
  // Make sure the code can run in less than 1000/tickMilliDelay seconds.

  pollMPU(); 
  int maxGestureSize = 1000 / tickMilliDelay;
  
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
    
   /* 
    posX[index] += accelX();
    posY[index] += accelY();
    posZ[index] += accelZ();
    
    if (index != 0){
      posX[index] += posX[index-1];
      posY[index] += posY[index-1];
      posZ[index] += posZ[index-1];
    }*/

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
    index = 0;
    
    normalizeHeight(accel_X, accel_Y, accel_Z, index);
    
    double* newX = normalizeLength(accel_X, trainedDataSize, index);
    double* newY = normalizeLength(accel_Y, trainedDataSize, index);
    double* newZ = normalizeLength(accel_Z, trainedDataSize, index);
    
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
  Serial.println("normLength");
  Serial.println(desiredLength);
  Serial.println(initialLength);
  
  float x[initialLength];
  float y[initialLength];
  Serial.println("norms");

  for (int i = 0; i < initialLength; i++)  {
    x[i] = i;
    y[i] = float(seq[i]);
  }
  
  
  Spline linearSpline(x,y,initialLength,1);
  double* normalized = new double[desiredLength];

  //normalized[desiredLength];
  double slope = double(initialLength)/(desiredLength-1);
  Serial.println("find the slope");
  for (int i=0; i < desiredLength; i++)
  {
    double xVal = i*slope;
    normalized[i] = linearSpline.value(xVal);
  }
  //printArray(normalized, desiredLength);
  //printArray(seq, initialLength);
  Serial.println("returning from norm length");
  return normalized;
}

void normalizeHeight(double* x, double* y, double* z, int seqSize){
  printArray(x, seqSize);
  
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

int classify(double gestureX[], double gestureY[], double gestureZ[]){
  double minLoss = 999999;
  int bestGesture = 0;
  
  for (int i = 0; i < numTrainingSamples; i++){
    double loss = squaredLossDifference(gestureX, trainingX[i], trainedDataSize);
    loss += squaredLossDifference(gestureY, trainingY[i], trainedDataSize);
    loss += squaredLossDifference(gestureZ, trainingZ[i], trainedDataSize);
    
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
  
  printArray(b, dataSize);
  //Serial.println(sum);
  return sum;
}

void printArray(double a[], int length){
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

