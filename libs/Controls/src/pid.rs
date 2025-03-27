use core::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Sub, SubAssign};

/// A simple PID controller implementation.
///
/// Maintains the time between updates to calculate the derivative term.
/// In the future we may want to allow the user to specify the time directly to avoid
/// floating point division.
///
/// In an RTOS you would want to use a timer to update the PID controller at a fixed rate.
/// But RTOS's timer might not be precise enough for some applications.
///
pub struct PID<T> {
    kp: T,
    ki: T,
    kd: T,
    integral: T,
    last_error: T,
    last_time: T,
}

pub struct Config<T> {
    pub integral_windup_guard: T,
    pub integral_windup_max: T,
    pub num_samples: T,
    pub enable_integral_anti_windup: bool,
}

impl<
        T: Default
            + Copy
            + Mul<Output = T>
            + MulAssign
            + Div<Output = T>
            + DivAssign
            + Sub<Output = T>
            + SubAssign
            + Add<Output = T>
            + AddAssign,
    > PID<T>
{
    pub fn new(kp: T, ki: T, kd: T) -> Self {
        PID {
            kp,
            ki,
            kd,
            integral: T::default(),
            last_error: T::default(),
            last_time: T::default(),
        }
    }

    pub fn update(&mut self, error: T, time: T) -> T {
        let dt = time - self.last_time;
        let derivative = (error - self.last_error) / dt;
        self.integral += error * dt;
        self.last_error = error;
        self.last_time = time;
        self.kp * error + self.ki * self.integral + self.kd * derivative
    }

    pub fn reset(&mut self) {
        self.integral = T::default();
        self.last_error = T::default();
        self.last_time = T::default();
    }

    pub fn set_pid_values(&mut self, kp: Option<T>, ki: Option<T>, kd: Option<T>) {
        if let Some(kp_value) = kp {
            self.kp = kp_value;
        }
        if let Some(ki_value) = ki {
            self.ki = ki_value;
        }
        if let Some(kd_value) = kd {
            self.kd = kd_value;
        }
    }

    #[allow(dead_code)]
    fn set_integral(&mut self, integral: T) {
        self.integral = integral;
    }

    #[allow(dead_code)]
    fn set_last_error(&mut self, last_error: T) {
        self.last_error = last_error;
    }

    #[allow(dead_code)]
    fn set_last_time(&mut self, last_time: T) {
        self.last_time = last_time;
    }

    pub fn get_kp(&self) -> T {
        self.kp
    }

    pub fn get_ki(&self) -> T {
        self.ki
    }

    pub fn get_kd(&self) -> T {
        self.kd
    }

    pub fn get_integral(&self) -> T {
        self.integral
    }

    pub fn get_last_error(&self) -> T {
        self.last_error
    }

    pub fn get_last_time(&self) -> T {
        self.last_time
    }

    pub fn get_error(&self) -> T {
        self.kp * self.last_error
            + self.ki * self.integral
            + self.kd * (self.last_error - self.integral)
    }
}
