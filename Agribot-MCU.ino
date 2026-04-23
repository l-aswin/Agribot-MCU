// Map ESP8266 NodeMCU pins
const int pinD0 = D0;
const int pinD1 = D1;
const int pinD2 = D2;
const int pinD3 = D3;

// Variables for the UART state machine
int serialState = 0;
uint8_t commandPayload = 0;

void setup() {
  // Initialize serial communication. 
  // Ensure the Jetson Nano is set to the same baud rate!
  Serial.begin(115200);

  // Configure motor control pins as outputs
  pinMode(pinD0, OUTPUT);
  pinMode(pinD1, OUTPUT);
  pinMode(pinD2, OUTPUT);
  pinMode(pinD3, OUTPUT);

  // Ensure all pins start LOW for safety
  stopMotors();
}

void loop() {
  // Process incoming serial data
  while (Serial.available() > 0) {
    uint8_t incomingByte = Serial.read();
    Serial.printf("incomingByte:%c\n", incomingByte);  
      switch (serialState) {
      case 0: // 1. Waiting for Start Flag (254)
        if (incomingByte == 254) {
          serialState = 1;
        }
        break;

      case 1: // 2. Reading Command Payload
        commandPayload = incomingByte;
        serialState = 2;
        break;

      case 2: // 3. Waiting for End Flag (255)
        if (incomingByte == 255) {
          executeCommand(commandPayload);
        }
        // Reset state to wait for the next frame
        serialState = 0; 
        break;
    }
  }
}

// Helper function to stop all movement
void stopMotors() {
  digitalWrite(pinD0, LOW);
  digitalWrite(pinD1, LOW);
  digitalWrite(pinD2, LOW);
  digitalWrite(pinD3, LOW);
}

// Function to trigger pins based on the command received
void executeCommand(uint8_t cmd) {
  // It is best practice to turn all pins off before applying a new movement direction
  // to prevent electrical shorts or conflicting motor states.
  stopMotors();

  switch (cmd) {
    case 0: // Move Forward Command
      digitalWrite(pinD0, HIGH);
      digitalWrite(pinD1, HIGH);
      break;
      
    case 1: // Move Backward Command
      digitalWrite(pinD2, HIGH);
      digitalWrite(pinD3, HIGH);
      break;
      
    case 2: // Turn Right Command
      digitalWrite(pinD0, HIGH);
      break;
      
    case 3: // Turn Left Command
      digitalWrite(pinD3, HIGH);
      break;
      
    case 4: // Stop Command
      // Already handled by stopMotors() at the top of the function
      break;
      
    default:
      // Unknown command received; safely ignore it
      break;
  }
}