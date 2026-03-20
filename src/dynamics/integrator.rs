use crate::dynamics::body::Body;
use crate::math::vec2::Vec2;

pub fn integrate(body: &mut Body, gravity: Vec2, dt: f32) {
    if body.is_static() || dt <= 0.0 {
        return;
    }

    integrate_forces(body, gravity, dt);
    integrate_velocity(body, dt);
    integrate_forces(body, gravity, dt);
    body.clear_accumulators();
}

fn integrate_forces(body: &mut Body, gravity: Vec2, dt: f32) {
    let curr_linear_accel: Vec2 = body.linear_acceleration() + gravity;
    let curr_angular_accel: f32 = body.angular_acceleration();

    let half_dt = 0.5 * dt;
    body.add_linear_velocity_delta(curr_linear_accel * half_dt);
    body.add_angular_velocity_delta(curr_angular_accel * half_dt);
}

fn integrate_velocity(body: &mut Body, dt: f32) {
    body.translate(body.linear_velocity() * dt);
    body.rotate(body.angular_velocity() * dt);
}

#[cfg(test)]
mod tests {
    use super::integrate;
    use crate::dynamics::body::Body;
    use crate::math::vec2::Vec2;

    fn assert_approx_eq(actual: f32, expected: f32) {
        assert!(
            (actual - expected).abs() <= 1e-6,
            "expected {expected}, got {actual}"
        );
    }

    fn assert_vec2_approx_eq(actual: Vec2, expected: Vec2) {
        assert_approx_eq(actual.x, expected.x);
        assert_approx_eq(actual.y, expected.y);
    }

    #[test]
    fn static_bodies_do_not_move() {
        let initial_position = Vec2::new(2.0, 3.0);
        let mut body = Body::new_static_circle(1, initial_position, 0.25, 1.0, 0.4, 0.6);

        body.add_force(Vec2::new(10.0, -5.0));
        body.add_torque(7.5);

        integrate(&mut body, Vec2::new(0.0, -9.81), 0.5);

        assert_vec2_approx_eq(body.position(), initial_position);
        assert_approx_eq(body.rotation(), 0.25);
        assert_vec2_approx_eq(body.linear_velocity(), Vec2::ZERO);
        assert_approx_eq(body.angular_velocity(), 0.0);
        assert_vec2_approx_eq(body.linear_acceleration(), Vec2::ZERO);
        assert_approx_eq(body.angular_acceleration(), 0.0);
    }

    #[test]
    fn integrates_gravity_and_forces() {
        let mut body = Body::new_dynamic_circle(1, Vec2::ZERO, 0.0, 1.0, 2.0, 0.3, 0.5);
        body.add_force(Vec2::new(6.0, 2.0));

        integrate(&mut body, Vec2::new(0.0, -10.0), 0.5);

        assert_vec2_approx_eq(body.linear_velocity(), Vec2::new(1.5, -4.5));
        assert_vec2_approx_eq(body.position(), Vec2::new(0.375, -1.125));
        assert_vec2_approx_eq(body.linear_acceleration(), Vec2::ZERO);
    }

    #[test]
    fn integrates_existing_linear_and_angular_velocity() {
        let mut body = Body::new_dynamic_box(
            1,
            Vec2::new(1.0, -2.0),
            0.5,
            Vec2::new(2.0, 1.0),
            3.0,
            0.2,
            0.8,
        );

        body.apply_linear_impulse(Vec2::new(9.0, -6.0));
        body.apply_angular_impulse(5.0);
        body.add_torque(3.0);

        integrate(&mut body, Vec2::ZERO, 0.5);

        assert_vec2_approx_eq(body.position(), Vec2::new(2.5, -3.0));
        assert_approx_eq(body.rotation(), 1.075);
        assert_vec2_approx_eq(body.linear_velocity(), Vec2::new(3.0, -2.0));
        assert_approx_eq(body.angular_velocity(), 1.3);
        assert_approx_eq(body.angular_acceleration(), 0.0);
    }
}
