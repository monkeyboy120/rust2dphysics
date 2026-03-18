use crate::math::vec2::Vec2;

#[derive(Debug, PartialEq)]
pub enum Shape {
    Circle { radius: f32 },
    Box { half_extents: Vec2 },
}

#[derive(Debug, PartialEq)]
pub enum BodyType {
    Static,  // no movement inf mass
    Dynamic, // moves, responds to force + collisions
}

#[derive(Debug, PartialEq)]
pub struct Body {
    id: u16, // identifier for body
    body_type: BodyType,
    position: Vec2,        // location in world space
    rotation: f32,         // orientation of body in rad
    linear_velocity: Vec2, // lin velocity in x y
    angular_velocity: f32, // positive = counter clockwise, negative = clockwise
    force: Vec2,           // sum of forces
    torque: f32,           // rotational force
    inv_mass: f32,         // store inv for faster physics calcs
    inv_inertia: f32,
    restitution: f32, // bounciness ( 0 -> inelastic 1 -> elastic )
    friction: f32,
    shape: Shape,
}

impl Body {
    // constructors
    pub fn new_dynamic_circle(
        id: u16,
        position: Vec2,
        rotation: f32,
        radius: f32,
        mass: f32,
        restitution: f32,
        friction: f32,
    ) -> Self {
        let inv_mass = if mass > 0.0 { 1.0 / mass } else { 0.0 };

        // moment of inertia for solid circle = 1/2 * m * r^2
        let inertia = 0.5 * mass * radius * radius;
        let inv_inertia = if inertia > 0.0 { 1.0 / inertia } else { 0.0 };

        Self {
            id,
            body_type: BodyType::Dynamic,
            position,
            rotation,
            linear_velocity: Vec2 { x: 0.0, y: 0.0 },
            angular_velocity: 0.0,
            force: Vec2 { x: 0.0, y: 0.0 },
            torque: 0.0,
            inv_mass,
            inv_inertia,
            restitution,
            friction,
            shape: Shape::Circle { radius },
        }
    }

    pub fn new_static_circle(
        id: u16,
        position: Vec2,
        rotation: f32,
        radius: f32,
        restitution: f32,
        friction: f32,
    ) -> Self {
        Self {
            id,
            body_type: BodyType::Static,
            position,
            rotation,
            linear_velocity: Vec2 { x: 0.0, y: 0.0 },
            angular_velocity: 0.0,
            force: Vec2 { x: 0.0, y: 0.0 },
            torque: 0.0,
            inv_mass: 0.0,
            inv_inertia: 0.0,
            restitution,
            friction,
            shape: Shape::Circle { radius },
        }
    }
    pub fn new_dynamic_box(
        id: u16,
        position: Vec2,
        rotation: f32,
        half_extents: Vec2,
        mass: f32,
        restitution: f32,
        friction: f32,
    ) -> Self {
        let inv_mass = if mass > 0.0 { 1.0 / mass } else { 0.0 };

        // moment of inertia for box = 1/12 * m*(w^2 + h^2)
        // becauase half extents 1/12 * 2 * 2 = 1/3
        let inertia =
            1.0 / 3.0 * mass * (half_extents.x * half_extents.x + half_extents.y * half_extents.y);
        let inv_inertia = if inertia > 0.0 { 1.0 / inertia } else { 0.0 };

        Self {
            id,
            body_type: BodyType::Dynamic,
            position,
            rotation,
            linear_velocity: Vec2 { x: 0.0, y: 0.0 },
            angular_velocity: 0.0,
            force: Vec2 { x: 0.0, y: 0.0 },
            torque: 0.0,
            inv_mass,
            inv_inertia,
            restitution,
            friction,
            shape: Shape::Box { half_extents },
        }
    }

    pub fn new_static_box(
        id: u16,
        position: Vec2,
        rotation: f32,
        half_extents: Vec2,
        restitution: f32,
        friction: f32,
    ) -> Self {
        Self {
            id,
            body_type: BodyType::Static,
            position,
            rotation,
            linear_velocity: Vec2 { x: 0.0, y: 0.0 },
            angular_velocity: 0.0,
            force: Vec2 { x: 0.0, y: 0.0 },
            torque: 0.0,
            inv_mass: 0.0,
            inv_inertia: 0.0,
            restitution,
            friction,
            shape: Shape::Box { half_extents },
        }
    }

    // helpers
    pub fn is_static(&self) -> bool {
        return self.body_type == BodyType::Static;
    }

    pub fn is_dynamic(&self) -> bool {
        return !self.is_static();
    }

    pub fn add_force(&mut self, force: Vec2) {
        self.force += force;
    }

    pub fn add_torque(&mut self, torque: f32) {
        self.torque += torque;
    }

    pub fn clear_forces(&mut self) {
        self.force = Vec2::ZERO;
    }

    pub fn clear_torque(&mut self) {
        self.torque = 0.0;
    }

    pub fn clear_accumulators(&mut self) {
        self.force = Vec2::ZERO;
        self.torque = 0.0;
    }

    pub fn inv_mass(&self) -> f32 {
        self.inv_mass
    }

    pub fn inv_inertia(&self) -> f32 {
        self.inv_inertia
    }

    pub fn is_circle(&self) -> bool {
        // use matches to compare
        matches!(&self.shape, Shape::Circle { .. })
    }

    pub fn is_box(&self) -> bool {
        !self.is_circle()
    }

    pub fn radius(&self) -> Option<f32> {
        match &self.shape {
            Shape::Circle { radius } => Some(*radius),
            _ => None,
        }
    }

    pub fn half_extents(&self) -> Option<Vec2> {
        match &self.shape {
            Shape::Box { half_extents } => Some(*half_extents),
            _ => None,
        }
    }

    pub fn apply_linear_impulse(&mut self, impulse: Vec2) {
        self.linear_velocity += impulse * self.inv_mass;
    }

    pub fn apply_angular_impulse(&mut self, impulse: f32) {
        self.angular_velocity += impulse * self.inv_inertia;
    }

    pub fn linear_acceleration(&self) -> Vec2 {
        self.force * self.inv_mass
    }

    pub fn angular_acceleration(&self) -> f32 {
        self.torque * self.inv_inertia
    }
}
