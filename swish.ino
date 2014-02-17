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

const byte tickMilliDelay = 25;
double dt = ((double)tickMilliDelay) / 1000.0;


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

const byte numTrainingSamples = 4;

int gestToKeyCodes[numTrainingSamples] = {ARROW_LEFT, ARROW_RIGHT, ARROW_DOWN, ARROW_UP};

// THIS HOLDS RAW TRAINING DATA

const byte maxRawTrainingSize = 30;



// THIS HOLDS PROCESSED TRAINING DATA
const double* trainingX[numTrainingSamples] = {};
const double* trainingY[numTrainingSamples] = {};
const double* trainingZ[numTrainingSamples] = {};

void setup() {
  Serial.begin(9600);
  Serial1.begin(115200);
  
  //BPMod.begin(BP_MODE_HID,BP_HID_KEYBOARD);   //Begin HID Mode with HID KEYBOARD AS TYPE
  
  delay(5000);
  setupMPU();
  Serial.println("All setup!");

  //double gestY[trainedDataSize] = {0.10, 0.83, 0.95, 0.20, -0.69, -1.00, -0.29, -0.02, 0.01, 0.02};

  double rawSSwipeZ[45] = {18.483999, 25.336, 47.768001, 66.419998, 51.172, 25.555999, -0.768, -8.420001, -8.82, -17.299999, -35.04, -41.736, -34.448001, -35.944, -41.84, -33.495998, -28.239999, -30.559999, -29.576, -36.027999, -37.291999, -51.368, -64.484001, -42.527999, -47.063999, -43.872001, -55.655998, -33.659999, -28.992, -31.808, -40.776, -26.267999, -14.936, 7.832, 25.587999, 32.195999, 41.335998, 43.212001, 47.875999, 56.144001, 40.763999, 34.835998, 19.107999, 16.767999, 7.532};
  //double rawSSwipeZ[45] = {1.7, 4.192, 20.455999, 42.155998, 48.720001, 38.448001, 29.36, 18.752, 18.931999, 19.856, 15.484001, 6.168, 4.808, -4.012, -22.827999, -37.287998, -41.591999, -43.268001, -33.063999, -34.555999, -36.551998, -36.956001, -30.395999, -13.592, -9.384, -1.636, 4.564, 11.812001, 16.403999, 17.04, 22.076, 32.291999, 42.551998, 45.372001, 38.423999, 27.591999, 20.219999, 24.423999, 29.643999, 37.363998, 25.908, 7.936, -6.74, -5.56, -0.9};
 
  for (int i = 0; i < 45; i++){
    // rawSSwipeX[i] = rawSSwipeX[i] * dt;
     rawSSwipeZ[i] = rawSSwipeZ[i] * dt;
  }  
  
  for (int i = 1; i < 45; i++){
    //rawSSwipeX[i] = rawSSwipeX[i-1] + (rawSSwipeX[i]); 
    rawSSwipeZ[i] = rawSSwipeZ[i-1] + (rawSSwipeZ[i]);
  }

  //anglesToPos(rawSSwipeX, 45);
  anglesToPos(rawSSwipeZ, 45);
  
  //normalizeHeight(rawSSwipeX, 45);
  normalizeHeight(rawSSwipeZ, 45);
  
  //double* newY = normalizeLength(rawSSwipeX, trainedDataSize, 45);
  double* newZ = normalizeLength(rawSSwipeZ, trainedDataSize, 45);
  
  //printArray(newY, 10);
  printArray(newZ, 10);
  
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
  //Serial.println(gyroX());
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
    
    if (index == 0){
      accel_Y[index] = gyroY() * dt;
      accel_Z[index] = gyroZ() * dt;
    } else {
      accel_Y[index] = accel_Y[index - 1] + (gyroY() * dt);
      accel_Z[index] = accel_Z[index - 1] + (gyroZ() * dt);
    }
    
    Serial.print(accelX(), 6);
    Serial.print(" ");
    Serial.print(accelY(), 6);
    Serial.print(" ");
    Serial.print(accelZ(), 6);
    Serial.print(" ");
    Serial.print(gyroX(), 6);
    Serial.print(" ");
    Serial.print(gyroY(), 6);
    Serial.print(" ");
    Serial.print(gyroZ(), 6);
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
    
    anglesToPos(accel_Y, index);
    anglesToPos(accel_Z, index);
    normalizeHeight(accel_Y, index);
    normalizeHeight(accel_Z, index);
    double* newY = normalizeLength(accel_Y, trainedDataSize, index);
    double* newZ = normalizeLength(accel_Z, trainedDataSize, index);
    index = 0;
    
    // Classification is an int that's >= 0
    int classification = classify(newY, newZ);
    
    /*
    if (classification == 0) {
      BPMod.keyboardPress(BP_KEY_LEFT_ARROW, BP_MOD_NOMOD);
    }
    else if (classification == 1) {
      BPMod.keyboardPress(BP_KEY_RIGHT_ARROW, BP_MOD_NOMOD);
    }
    BPMod.keyboardReleaseAll();
    index*/
    
    delete[] newY;
    delete[] newZ;
    
    //printArray(posX, index);
    Serial.print("CLASS: ");

    Serial.println(classification);
    Serial.println();
    
    if (classification == 3){
//      sendConsumerKey(ZERO_HEX, NEXT_HIGH);

      return;
    } else if (classification == 2) {
 //     sendConsumerKey(ZERO_HEX, PREV_HIGH);

      return;
    }
    
 //   sendViaBluetooth(gestToKeyCodes[classification]);
    
    Serial.println(freeRam());
  }
}

// returns 0 to 1023 mapped to 0-5v
int pollTouchPad(){
  return analogRead(potentiometerPin);
}


double* normalizeLength(double seq[], int desiredLength, int initialLength)
{ 
  
  Serial.println("1");
  
  Serial.println(freeRam());
  
  float* x = new float[initialLength];
  float* y = new float[initialLength];
    Serial.println(initialLength);
   
  for (int i = 0; i < initialLength; i++)  {
    x[i] = i;
    y[i] = float(seq[i]);
  }
    Serial.println("2");

  Spline linearSpline(x,y,initialLength,1);
    Serial.println("3");

  
  double* normalized = new double[desiredLength];
  Serial.println("4");

  //normalized[desiredLength];
  double slope = double(initialLength)/(desiredLength-1);
 
  for (int i=0; i < desiredLength; i++)
  {
    double xVal = i * slope;
    Serial.println(linearSpline.value(xVal));
    normalized[i] = linearSpline.value(xVal);
  }
    Serial.println("5");
delete[] x;
  delete[] y;

  return normalized;
}

void normalizeHeight(double* seq, int seqSize){
  //printArray(seq, seqSize);
  
  double maxVal = 0.0;
  double minVal = 9999.9;
  
  for (int i = 0; i < seqSize; i++){
    double val = seq[i];
    if (val > maxVal){
      maxVal = val;
    }
    if (val < minVal){
      minVal = val;
    }
  }
  
  //Serial.println("MAX:"); Serial.println(maxVal);
  
  for (int i = 0; i < seqSize; i++) {
     seq[i]  = (seq[i] - minVal) / (maxVal - minVal);
  }
}




void anglesToPos(double* seq, int seqSize){
  
  for (int i = 0; i < seqSize; i++) {
    seq[i] = sin(seq[i] * (PI / 180.0)) * 10.0;
  }
  printArray(seq, seqSize);
}

int classify(double* gestureY, double* gestureZ){
  //printArray(gestureZ, trainedDataSize);
  double minLoss = 999999;
  int bestGesture = 0;
  
  for (int i = 0; i < numTrainingSamples; i++){
    double loss = 0.0;
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
    Serial.print(a[i], 4);
    Serial.print(", ");
  }
  Serial.println();
}


// A simple queue with a pointer to the front of it
const byte qSize = 4;
double* lastMags = new double[qSize];
int front = 0;

boolean gestTakingPlace(int maxGestureSize){
  double thresh = 50.0;
  
  // current mag
  double magnitude = sqrt(gyroX() * gyroX()) + (gyroY() * gyroY()) + (gyroZ() * gyroZ());
  
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
*/
