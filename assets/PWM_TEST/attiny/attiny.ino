/**
 * ATtiny PWM Test - Cycling PWM Values
 *
 * This code cycles PWM output from 0 to 255 and back down continuously
 * for testing PWM functionality on ATtiny microcontrollers.
 *
 * Hardware: ATtiny85/ATtiny45/ATtiny25
 * PWM Pin: Arduino pin 0 (PB0, physical pin 5)
 *
 * Author: azzar budiyanto
 * Date: November 2025
 */

int PWM_Pin = 0;  // Arduino pin 0 (PB0) for PWM output
int pwmValue = 0; // Current PWM value (0-255)
int direction = 1; // 1 = increasing, -1 = decreasing
int delayMs = 10; // Delay between PWM changes (ms)

void setup()
{
  pinMode(PWM_Pin, OUTPUT);  // Declare pin as output
  TCCR0B = TCCR0B & 0b11111000 | 0b001;  // Set ~30.64Hz PWM frequency
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
