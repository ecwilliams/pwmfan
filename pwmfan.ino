/*
 * Fan speed controller for UDOO Bolt computer.  
 * <wd6cmu@comcast.net> 
 * 7/13/2020
 * 
 * Fan is Noctua A6x25 5V PWM on CN28 Grove connector:
 *    Pin 1: GND
 *    Pin 2: +5V
 *    Pin 3: Tachometer in
 *    Pin 4: PWM out
 * 
 * Heatsink temperature sensor is LM34 on CN26 Grove connector:
 *    Pin 1: GND
 *    Pin 2: +5V
 *    Pin 3: Analog temp in
 *    Pin 4: n/c
 * 
 * Note: Fan PWM frequency is supposed to be ~25kHz, but the Arduino Leonardo
 * puts out about 490Hz instead.  Fortunately, the Noctua doesn't seem to be
 * picky about the frequency but other fans might be.  Altering the TC0 clock
 * divider is possible, but Arduino also uses it for millis() timing, so using
 * a different TC and wiring to CN24 would probably be a better way to go.
 */
 
int pwmPin = 3;
int tachPin = 2;
int ledPin = 13;
int tempPin = 1;  // LM34 sensor: 10mV per degree F

/*
 * We're using a simple temperature regulation profile here to increase fan speed
 * lineraly above the setpoint and stop the fan below it.  We have to stop the fan
 * at some point because we have no way of knowing if the CPU has been shut down
 * other than that the temperature has dropped.  Keeping the heatsink temperature
 * high also has the advantage of increasing its efficiency: It takes less air to
 * remove a watt of power from a hot heatsink than from a warm one.  This also 
 * helps keep the fan noise low.
 */

#define F_PER_BIT (2.56/1023.0/0.01)  // Degrees F per ADC bit
#define SETPT int(115.0/F_PER_BIT)    // Setpoint in ADC bits
#define MIN_PWM 15                    // Experimentally determined minimum PWM to get fan to spin
#define PWM_GAIN int(10/F_PER_BIT)    // Scale PWM increase linearly over 10 degrees above SETPT

/*
 * Tachometer readings are being calculated and reported in the main loop, but no 
 * action is being taken on them at this time.  They could be read from the Arduino 
 * serial port by the AMD CPU and used to shut down the computer, or they could be 
 * used by the Arduino to activate an alarm buzzer (maybe on A0?) but I haven't 
 * done either yet.
 */

void setup() {
  // put your setup code here, to run once:
  pinMode(pwmPin, OUTPUT);
  pinMode(tachPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  analogReference(INTERNAL);  // 2.56v
  Serial.begin(115200);
}

// LED blink pattern.  Times in milliseconds, terminated by 0
// Must be even number of non-zero milliseconds, first entry is OFF
int blinks[] = {1930,10,50,10,0};

void loop() {
  int i;
  int pwm = 0;          // PWM output to fan
  int temp, last_temp;
  uint8_t analog_count; // counts analog temp samples for averaging
  unsigned long sec_t;  // millis fence-post for 1-second intervals
  unsigned long bl_t;   // blink time fence-post
  uint8_t bl_s, bl_i;   // blink state and index
  uint8_t tach_s, tach_os;  // tach pin state and old state
  int tach_count;

  sec_t = bl_t = millis();
  temp = 0;
  analog_count = 0;
  bl_s = bl_i = 0;
  tach_count = 0;
  tach_s = tach_os = digitalRead(tachPin);
  while(1) {

    // Sample temperature and set fan PWM
    if (analog_count++ < 32)
      temp += analogRead(tempPin);  // sum 32 10-bit samples
    else {
      temp >>= 5;   // divide by 32 to get average
      if (temp > SETPT) {
        pwm = MIN_PWM+(temp-SETPT)*255/PWM_GAIN;
        if (pwm >255) pwm = 255;
      } else
        pwm = 0;
      analogWrite(pwmPin, pwm);
      analog_count = 0;
      last_temp = temp;   // save for periodic report
      temp = 0;
    }

    // Sample tach
    tach_s = digitalRead(tachPin);
    if (tach_s != tach_os) {
      if (tach_s) tach_count++;   // count rising edges only
      tach_os = tach_s;
    }

    // Every second, print report and evaluate tach_count
    if (millis() - sec_t >= 1000) {
      i = tach_count; tach_count = 0;   // In case Serial.print is slow
      Serial.print(last_temp);
      Serial.print("->");
      Serial.print(int(last_temp*F_PER_BIT));
      Serial.print("F PWM=");
      Serial.print(pwm);
      Serial.print(" tach=");
      Serial.println(i);

      sec_t += 1000;
    }

    if (millis() - bl_t >= blinks[bl_i]) {
      bl_s = (bl_s == LOW) ? HIGH : LOW;
      digitalWrite(ledPin, bl_s);
      bl_t += blinks[bl_i];
      ++bl_i;
      if (blinks[bl_i] == 0) bl_i = 0;
    }
  }

#ifdef OLD_CODE

  // Determine minimum PWM to turn fan
  for (pwm = 0; pwm < 256; pwm++) {
    Serial.print(pwm); Serial.print(": ");
    analogWrite(pwmPin, pwm);
    sec_t = millis();
    while (millis() - sec_t < 1000) 
      ; // delay for fan speed to settle
    tach_s = tach_os = digitalRead(tachPin);
    tach_count = 0;
    sec_t = millis();
    while (millis() - sec_t < 1000) {
      tach_s = digitalRead(tachPin);
      if (tach_s != tach_os) {
        if (tach_s) tach_count++;
        tach_os = tach_s;
      }
    }
    Serial.println(tach_count);
  }

  // First test: Run fan speed up and down while monitoring temperature
  while (1) {
    analogWrite(pwmPin, pwm);
    analogWrite(ledPin, pwm);
    pwm += direction;
    if (pwm > 255) {
      pwm = 255;
      direction = -1;
    }
    if (pwm < 0) {
      pwm = 0;
      direction = +1;
      temp = analogRead(tempPin);
      Serial.println(temp*F_PER_BIT,1);
    }
    delay(10);
  }
#endif //OLD_CODE
}
