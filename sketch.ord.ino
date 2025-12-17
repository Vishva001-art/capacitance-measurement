#define analogPin      A0
#define chargePin      13
#define dischargePin   3
#define buzzerPin      4
#define greenLED       5
#define redLED         7
#define resistorValue  10000.0F

// Timing optimization
unsigned long startTime;
unsigned long elapsedTime;
float microFarads = 0.0;
int currentStep = -1;
int lastStep = -1;
bool stepValidated[6] = {false, false, false, false, false, false};
bool step0Reached = false;  // Track if Step 0 has been reached at startup

// Debouncing variables
int stepChangeCounter = 0;
const int STEP_CONFIRMATION_THRESHOLD = 3; // Confirm step after 3 consistent readings
int pendingStep = -1;

// Calibration factor (adjust if needed)
float calibrationFactor = 1.0;

void setup() {
  pinMode(chargePin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(redLED, OUTPUT);
  
  digitalWrite(chargePin, LOW);
  digitalWrite(buzzerPin, LOW);
  digitalWrite(greenLED, LOW);
  digitalWrite(redLED, LOW);
  
  Serial.begin(115200); // Faster serial communication
  
  // Warm-up sequence - CRITICAL for accuracy
  Serial.println("Warming up sensor...");
  for (int i = 0; i < 10; i++) {
    measureCapacitance();
    delay(50);
  }
  
  delay(500);
  Serial.println("========================================");
  Serial.println("Fan Regulator Capacitance Tester Ready");
  Serial.println("========================================");

  // Initial reading with multiple samples for accuracy
  microFarads = fastStableRead();
  currentStep = getStep(microFarads);
  lastStep = currentStep;

  Serial.println("\n--- Initial Check ---");
  if (currentStep != 0) {
    Serial.print("⚠  Not at Step 0! Capacitance: ");
    Serial.print(microFarads, 3);
    Serial.println(" uF");
    Serial.println("⚠  ALERT: Please set regulator to Step 0");
    Serial.println("========================================\n");
    // DO NOT set step0Reached to true - keep it false to continue alerting
  } else {
    Serial.print("✓ Step 0 Detected! Capacitance: ");
    Serial.print(microFarads, 3);
    Serial.println(" uF");
    validateAndIndicate(currentStep, microFarads);
    step0Reached = true;  // Mark that Step 0 has been reached
    Serial.println("========================================\n");
  }
}

void loop() {
  // If Step 0 hasn't been reached yet, continuously alert
  if (!step0Reached) {
    microFarads = fastStableRead();
    currentStep = getStep(microFarads);
    
    if (currentStep != 0) {
      // Keep red LED on and buzz continuously
      digitalWrite(redLED, HIGH);
      digitalWrite(greenLED, LOW);
      
      Serial.print("⚠  WAITING FOR STEP 0 | Current Step: ");
      Serial.print(currentStep);
      Serial.print(" | Capacitance: ");
      Serial.print(microFarads, 3);
      Serial.println(" uF");
      
      // Continuous buzzer pattern - short beep
      digitalWrite(buzzerPin, HIGH);
      delay(150);
      digitalWrite(buzzerPin, LOW);
      delay(1000); // Total 2 second cycle
      
    } else {
      // Step 0 finally reached!
      step0Reached = true;
      digitalWrite(redLED, LOW);
      digitalWrite(buzzerPin, LOW);
      
      Serial.println("\n========================================");
      Serial.println("✓✓✓ STEP 0 REACHED! ✓✓✓");
      //Serial.print("Capacitance: ");
      //Serial.print(microFarads, 3);
      //Serial.println(" uF");
      Serial.println("========================================\n");
      
      // Success indication
      validateAndIndicate(currentStep, microFarads);
      lastStep = currentStep;
    }
    return;  // Skip normal operation until Step 0 is reached
  }

  // Normal operation after Step 0 has been reached
  // Fast reading - only 2 samples for speed
  microFarads = fastStableRead();
  int detectedStep = getStep(microFarads);

  // Debouncing logic - confirm step change
  if (detectedStep != currentStep) {
    if (detectedStep == pendingStep) {
      stepChangeCounter++;
      
      // Confirm step change after consistent readings
      if (stepChangeCounter >= STEP_CONFIRMATION_THRESHOLD) {
        currentStep = detectedStep;
        
        Serial.println("\n--- Step Change Detected ---");
        
        // Show direction
        if (currentStep > lastStep) {
          Serial.println("Direction: Forward →");
        } else if (currentStep < lastStep) {
          Serial.println("Direction: Reverse ←");
        }

        // Display step and capacitance
        Serial.print("Step: ");
        Serial.print(currentStep);
        Serial.print(" | Capacitance: ");
        Serial.print(microFarads, 3);
        Serial.println(" uF");

        // Validate and indicate
        validateAndIndicate(currentStep, microFarads);

        lastStep = currentStep;
        stepChangeCounter = 0;
        pendingStep = -1;
        Serial.println("----------------------------\n");
      }
    } else {
      // New pending step detected
      pendingStep = detectedStep;
      stepChangeCounter = 1;
    }
  } else {
    // Reset if back to current step
    stepChangeCounter = 0;
    pendingStep = -1;
  }

  delay(50); // Minimal delay - fast response
}

// ---------- Fast Stable Read (2 samples only) ----------
float fastStableRead() {
  float reading1 = measureCapacitance();
  delay(30);
  float reading2 = measureCapacitance();
  
  // If readings are close, return average
  if (abs(reading1 - reading2) < 0.3) {
    return (reading1 + reading2) / 2.0;
  }
  
  // Otherwise take one more sample
  delay(30);
  float reading3 = measureCapacitance();
  return (reading1 + reading2 + reading3) / 3.0;
}

// ---------- Optimized Measure Capacitance ----------
float measureCapacitance() {
  // Ensure complete discharge first
  digitalWrite(chargePin, LOW);
  pinMode(dischargePin, OUTPUT);
  digitalWrite(dischargePin, LOW);
  delayMicroseconds(5000); // 5ms discharge
  
  // Wait for complete discharge
  int dischargeTimeout = 0;
  while (analogRead(analogPin) > 2 && dischargeTimeout < 100) {
    delayMicroseconds(100);
    dischargeTimeout++;
  }
  
  pinMode(dischargePin, INPUT);
  delayMicroseconds(100); // Settle time
  
  // Start charging
  digitalWrite(chargePin, HIGH);
  startTime = micros();

  // Wait for 63.2% charge (648 out of 1023)
  int chargeTimeout = 0;
  while (analogRead(analogPin) < 648 && chargeTimeout < 50000) {
    chargeTimeout++;
  }

  elapsedTime = micros() - startTime;
  
  // Apply calibration factor
  float cap = ((float)elapsedTime / resistorValue) * calibrationFactor;
  
  // Discharge immediately
  digitalWrite(chargePin, LOW);
  
  return cap;
}

// ---------- Improved Step Detection with Hysteresis ----------
int getStep(float c) {
  static int lastStableStep = 0;
  int detectedStep;

  // Adjusted thresholds with hysteresis zones
  if (c < 0.20) {
    detectedStep = 0;
  } else if (c < 3.15) {
    detectedStep = 1;
  } else if (c < 5.15) {
    detectedStep = 2;
  } else if (c < 6.80) {
    detectedStep = 3;
  } else if (c < 7.65) {
    detectedStep = 4;
  } else {
    detectedStep = 5;
  }

  return detectedStep;
}

// ---------- Validate and Indicate (Non-blocking) ----------
void validateAndIndicate(int step, float cap) {
  bool isValid = validateCapacitance(step, cap);
  
  if (isValid) {
    digitalWrite(greenLED, HIGH);
    digitalWrite(redLED, LOW);
    stepValidated[step] = true;
    
    Serial.println("✅ Status: VALID - Capacitance within range");
    Serial.print("Expected Range: ");
    printExpectedRange(step);
    
    // Quick visual feedback
    delay(300);
    digitalWrite(greenLED, LOW);
    delay(80);
    digitalWrite(greenLED, HIGH);
    delay(2000); // Reduced from 5000ms
    digitalWrite(greenLED, LOW);
    
  } else {
    digitalWrite(redLED, HIGH);
    digitalWrite(greenLED, LOW);
    stepValidated[step] = false;
    
    Serial.println("❌ Status: ERROR - Capacitance OUT OF RANGE!");
    Serial.print("Expected Range: ");
    printExpectedRange(step);
    
    if (step == 0) {
      Serial.println("⚠  Please set regulator to Step 0");
    }
    
    // Quick error indication
    for (int i = 0; i < 2; i++) {
      digitalWrite(redLED, LOW);
      delay(100);
      digitalWrite(redLED, HIGH);
      delay(2000); // Reduced from 5000ms
    }
    digitalWrite(redLED, LOW);
  }
}

// ---------- Validate Capacitance (Adjusted Ranges) ----------
bool validateCapacitance(int step, float cap) {
  switch (step) {
    case 0:
      return (cap <= 0.20);
    case 1:
      return (cap >= 2.20 && cap <= 2.70); // Wider tolerance
    case 2:
      return (cap >= 3.70 && cap <= 4.20);
    case 3:
      return (cap >= 5.20 && cap <= 5.80);
    case 4:
      return (cap >= 6.90 && cap <= 7.40);
    case 5:
      return (cap > 7.30);
    default:
      return false;
  }
}

// ---------- Print Expected Range ----------
void printExpectedRange(int step) {
  switch (step) {
    case 0:
      Serial.println("≤ 0.20 uF");
      break;
    case 1:
      Serial.println("2.20 - 2.70 uF");
      break;
    case 2:
      Serial.println("3.70 - 4.20 uF");
      break;
    case 3:
      Serial.println("5.20 - 5.80 uF");
      break;
    case 4:
      Serial.println("6.90 - 7.40 uF");
      break;
    case 5:
      Serial.println("> 7.30 uF");
      break;
  }
}

// ---------- Quick Buzzer (Non-blocking) ----------
void quickBuzz(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(buzzerPin, HIGH);
    delay(80);
    digitalWrite(buzzerPin, LOW);
    delay(80);
  }
}