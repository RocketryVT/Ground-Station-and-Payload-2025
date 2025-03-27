// use faer::*;
use nalgebra::{Quaternion, SVector};

/// Based on PX4 EKF2
#[derive(Debug)]
pub struct Ekf<T> {
    pub quat_nominal: Quaternion<T>,
    pub velocity: nalgebra::Vector3<T>,
    pub position: nalgebra::Vector3<T>,
    pub gyro_bias: nalgebra::Vector3<T>,
    pub accel_bias: nalgebra::Vector3<T>,
    pub mag_i: nalgebra::Vector3<T>,
    pub mag_b: nalgebra::Vector3<T>,
    pub wind_vel: nalgebra::Vector2<T>,
    pub terrain: T,
}

impl<T: nalgebra::Scalar + Copy + num_traits::Zero> Ekf<T> {
    /// Converts the state into a single vector of size 25
    pub fn to_state_vector(&self) -> SVector<T, 25> {
        let mut state = SVector::<T, 25>::zeros();
        state.fixed_rows_mut::<4>(0).copy_from(&self.quat_nominal.coords);
        state.fixed_rows_mut::<3>(4).copy_from(&self.velocity);
        state.fixed_rows_mut::<3>(7).copy_from(&self.position);
        state.fixed_rows_mut::<3>(10).copy_from(&self.gyro_bias);
        state.fixed_rows_mut::<3>(13).copy_from(&self.accel_bias);
        state.fixed_rows_mut::<3>(16).copy_from(&self.mag_i);
        state.fixed_rows_mut::<3>(19).copy_from(&self.mag_b);
        state.fixed_rows_mut::<2>(22).copy_from(&self.wind_vel);
        state[24] = self.terrain;
        state
    }
}

pub struct IdxDof {
    
}