


int PWM_FREQUENCY = 1000; // Hz

// # -----------------------------
// # Speed settings
// # -----------------------------
DEFAULT_MAX_PWM = 40;  // used if you just press Enter at the prompt
PWM_CEILING = 100;  // J can never raise maximum PWM above this
PWM_STEP = 2;  // J / L increment
ADJUST_INTERVAL = 0.10;  // s, minimum time between J / L steps while held

// Ramp rates, in PWM % per second
ACCEL_RATE = 60.0;  // while W / S is held
COAST_RATE = 30.0;  // after letting go of W / S
BRAKE_RATE = 100.0;  // while K is held (must be faster than COAST_RATE)
