/**
 * Arduino PWM Test - Cycling PWM Values
 *
 * This code cycles PWM output from 0 to 255 and back down continuously
 * for testing PWM functionality on Arduino boards.
 *
 * Hardware: Arduino Uno/Nano/Mega (or similar)
 * PWM Pin: Arduino pin 9 (OC1A, Timer1)
 *
 * Author: azzar budiyanto
 * Date: November 2025
 */

int PWM_Pin = 9;   // Arduino pin 9 (OC1A) for PWM output - Timer1
int pwmValue = 0;  // Current PWM value (0-255)
int direction = 1; // 1 = increasing, -1 = decreasing
int delayMs = 10; // Delay between PWM changes (ms)

void setup()
{
  pinMode(PWM_Pin, OUTPUT);  // Declare pin as output
  // Arduino default PWM frequency is ~490Hz for pins 5,6 (Timer0)
  // Pin 9 uses Timer1 at ~490Hz by default - good for testing
}

void loop()
{
  // Set PWM value
  analogWrite(PWM_Pin, pwmValue);

  // Update PWM value for next cycle
  pwmValue += direction;

  // Reverse direction at boundaries
  if (pwmValue >= 255) {
    pwmValue = 255;
    direction = -1;  // Start decreasing
  }
  else if (pwmValue <= 0) {
    pwmValue = 0;
    direction = 1;   // Start increasing
  }

  // Small delay for smooth transitions
  delay(delayMs);
}
