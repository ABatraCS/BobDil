//! Global position, dead-reckoned from body-frame velocity.
//!
//! `VehicleFMI` reports body-frame velocities and a yaw rate but no global
//! position, and adding one would mean editing BobLib -- which BobDil reads and
//! never writes. So the kernel integrates it, and it is a module rather than
//! five lines inside the loop because two things need the identical arithmetic:
//! the live loop, which feeds the view, and replay, which feeds A/B. If those
//! two ever disagreed, a paired comparison would show a trajectory difference
//! that came from the integrator rather than from the setup -- the exact
//! failure A/B exists to rule out.
//!
//! It is first-order on purpose. The pose is a *presentation* quantity: the
//! view draws it and the A/B diff compares it, but nothing in the physics reads
//! it back, so its error cannot accumulate into the plant.

use crate::generated::frames::VehicleState;

/// Where the car is, in the world frame it started in.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Pose {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
}

impl Pose {
    /// Advance by one step from this frame's body-frame velocities.
    ///
    /// Yaw is advanced first and the rotation is taken at the new heading, which
    /// is the ordering the live loop has always used. The choice matters less
    /// than the fact that there is only one copy of it.
    pub fn advance(&mut self, state: &VehicleState, dt: f64) {
        self.yaw += state.yaw_vel * dt;
        let (sin_yaw, cos_yaw) = self.yaw.sin_cos();
        self.x += (state.vel_x * cos_yaw - state.vel_y * sin_yaw) * dt;
        self.y += (state.vel_x * sin_yaw + state.vel_y * cos_yaw) * dt;
    }

    /// Copy the pose into the frame that will be published or recorded.
    pub fn apply(&self, state: &mut VehicleState) {
        state.pos_x = self.x;
        state.pos_y = self.y;
        state.yaw = self.yaw;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(vel_x: f64, vel_y: f64, yaw_vel: f64) -> VehicleState {
        VehicleState {
            vel_x,
            vel_y,
            yaw_vel,
            ..Default::default()
        }
    }

    #[test]
    fn straight_ahead_advances_x_only() {
        let mut pose = Pose::default();
        for _ in 0..1000 {
            pose.advance(&frame(10.0, 0.0, 0.0), 1e-3);
        }
        assert!((pose.x - 10.0).abs() < 1e-9);
        assert!(pose.y.abs() < 1e-12);
        assert_eq!(pose.yaw, 0.0);
    }

    #[test]
    fn a_constant_yaw_rate_closes_a_circle() {
        // 10 m/s at 1 rad/s is a 10 m radius circle; one full turn takes 2*pi s.
        let mut pose = Pose::default();
        // dt derived from the step count, not the other way round: a truncated
        // count leaves the circle a fraction of a step short, and the test then
        // measures the truncation rather than the integrator.
        let steps = 62_832;
        let dt = std::f64::consts::TAU / steps as f64;
        for _ in 0..steps {
            pose.advance(&frame(10.0, 0.0, 1.0), dt);
        }
        // First-order integration of a circle leaves a small radial error; it
        // must be small, and it must not be a drift in the heading.
        assert!(pose.x.hypot(pose.y) < 0.05, "closed to {:?}", pose);
        assert!((pose.yaw - std::f64::consts::TAU).abs() < 1e-6);
    }

    #[test]
    fn lateral_velocity_moves_the_car_sideways_in_the_world_frame() {
        let mut pose = Pose {
            yaw: std::f64::consts::FRAC_PI_2,
            ..Default::default()
        };
        pose.advance(&frame(0.0, 1.0, 0.0), 1.0);
        // Heading is +90 degrees, so body +y points at world -x.
        assert!((pose.x + 1.0).abs() < 1e-12);
        assert!(pose.y.abs() < 1e-12);
    }

    #[test]
    fn apply_writes_every_pose_field_into_the_frame() {
        let pose = Pose {
            x: 1.0,
            y: 2.0,
            yaw: 3.0,
        };
        let mut state = VehicleState::default();
        pose.apply(&mut state);
        assert_eq!((state.pos_x, state.pos_y, state.yaw), (1.0, 2.0, 3.0));
    }
}
