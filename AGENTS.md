# AGENTS.md - Robotics Arduino Project Guidelines

## Overview
This is a robotics class project using Arduino for compilation and deployment. Agents should follow Arduino-specific coding conventions and best practices for embedded systems development.

## Build Commands

### Arduino IDE Compilation
```bash
# Compile and upload to Arduino board (requires Arduino IDE or CLI)
arduino-cli compile --fqbn arduino:avr:uno .
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno .

# Alternative: Use Arduino IDE's "Verify" and "Upload" buttons
```

### PlatformIO (Recommended for Advanced Projects)
```bash
# If using PlatformIO for better dependency management
pio run -t upload
pio run -t uploadfs  # Upload filesystem if using ESP boards
```

## Test Commands

### Unit Testing (Limited in Arduino)
```bash
# For Arduino projects with unit testing frameworks
# Install ArduinoUnit: https://github.com/mmurdoch/arduinounit
# Run tests via Serial Monitor or custom test runner

# PlatformIO testing
pio test
pio test -e native  # Run on host machine
```

### Integration Testing
- Manual testing: Upload code and observe robot behavior
- Serial output monitoring for debugging
- Sensor/actuator validation through physical testing

## Lint Commands

### Code Quality Tools
```bash
# Arduino Lint (if available)
arduino-lint

# C/C++ linting with clang-tidy
clang-tidy *.ino -- -std=c++11

# PlatformIO linting
pio check
```

## Code Style Guidelines

### File Organization
- Use `.ino` files for Arduino sketches
- Separate complex logic into `.h`/`.cpp` files for larger projects
- Keep main sketch file simple and focused on setup()/loop()

### Naming Conventions
- **Functions**: camelCase, descriptive names (e.g., `readSensorData()`, `controlMotorSpeed()`)
- **Variables**: camelCase, descriptive names (e.g., `sensorValue`, `motorSpeed`)
- **Constants**: UPPER_SNAKE_CASE (e.g., `MAX_MOTOR_SPEED`, `SENSOR_PIN`)
- **Classes**: PascalCase (e.g., `MotorController`, `SensorReader`)
- **Pin definitions**: Use `#define` with descriptive names (e.g., `#define MOTOR_LEFT_PIN 9`)

### Code Structure
```cpp
// Include libraries at top
#include <Servo.h>
#include <Wire.h>

// Pin definitions
#define MOTOR_LEFT_PIN 9
#define MOTOR_RIGHT_PIN 10
#define SENSOR_TRIGGER_PIN 7
#define SENSOR_ECHO_PIN 8

// Global variables
int motorSpeed = 100;
volatile long pulseDuration;

// Function prototypes (for complex sketches)
void setupMotors();
void readUltrasonicSensor();

// Setup function
void setup() {
  // Initialize pins and peripherals
  pinMode(MOTOR_LEFT_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_PIN, OUTPUT);
  pinMode(SENSOR_TRIGGER_PIN, OUTPUT);
  pinMode(SENSOR_ECHO_PIN, INPUT);

  Serial.begin(9600);

  setupMotors();
}

// Main loop
void loop() {
  // Read sensors
  long distance = readUltrasonicSensor();

  // Control logic
  if (distance > 20) {
    moveForward();
  } else {
    stopMotors();
  }

  delay(100);
}

// Helper functions
void setupMotors() {
  analogWrite(MOTOR_LEFT_PIN, 0);
  analogWrite(MOTOR_RIGHT_PIN, 0);
}

long readUltrasonicSensor() {
  digitalWrite(SENSOR_TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(SENSOR_TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(SENSOR_TRIGGER_PIN, LOW);

  pulseDuration = pulseIn(SENSOR_ECHO_PIN, HIGH);
  return pulseDuration * 0.034 / 2;
}
```

### Best Practices

#### Memory Management
- Be conscious of limited RAM (2KB on Arduino Uno)
- Use `const` for read-only data
- Avoid dynamic memory allocation in loop()
- Use PROGMEM for large constant arrays

#### Timing and Delays
- Use `millis()` instead of `delay()` for non-blocking code
- Implement proper state machines for complex behaviors
- Avoid long delays that block sensor reading

#### Error Handling
- Check sensor readings for validity
- Use Serial output for debugging
- Implement fail-safe behaviors (e.g., stop motors on error)
- Validate pin connections before use

#### Power Management
- Consider sleep modes for battery-powered projects
- Minimize unnecessary computations
- Use efficient algorithms for real-time operations

### Imports and Dependencies
- Include only necessary libraries
- Prefer standard Arduino libraries when possible
- Document third-party libraries in comments
- Keep library versions consistent across team members

### Documentation
- Use inline comments for complex logic
- Document pin connections and hardware setup
- Include function descriptions for non-obvious operations
- Add TODO comments for incomplete features

### Arduino-Specific Patterns

#### Pin Management
```cpp
// Good: Descriptive pin names
#define LEFT_MOTOR_PIN 9
#define RIGHT_MOTOR_PIN 10

// Avoid: Magic numbers
// digitalWrite(9, HIGH); // What is pin 9?
```

#### Sensor Reading
```cpp
// Good: Validate sensor readings
int sensorValue = analogRead(A0);
if (sensorValue >= 0 && sensorValue <= 1023) {
  // Process valid reading
} else {
  Serial.println("Invalid sensor reading");
}
```

#### Motor Control
```cpp
// Good: Use PWM for speed control
analogWrite(MOTOR_PIN, speed); // 0-255 range

// Avoid: Digital on/off only when speed control needed
// digitalWrite(MOTOR_PIN, HIGH);
```

### Testing Guidelines

#### Hardware Testing
- Test components individually before integration
- Verify power supplies and connections
- Use multimeter for circuit debugging
- Document expected vs. actual behavior

#### Software Testing
- Test edge cases (min/max values, boundary conditions)
- Verify timing-critical operations
- Test interrupt handlers
- Validate communication protocols

### Debugging Tips
- Use Serial.println() liberally during development
- Implement heartbeat LEDs to verify program execution
- Use oscilloscope for timing analysis
- Keep backup versions of working code

### Version Control
- Commit frequently with descriptive messages
- Include hardware changes in commit messages
- Tag releases with working hardware configurations
- Document breaking changes

### Collaboration
- Comment out unused code rather than deleting
- Use consistent coding style across team
- Review code for hardware implications
- Test on target hardware before merging

## Common Pitfalls to Avoid
- Blocking delays in main loop
- Floating pin states
- Missing pull-up/down resistors
- Inadequate power supply for motors/servos
- Race conditions in interrupt handlers
- Buffer overflows in serial communication