use std::ops::{Add, Div, Mul, Sub};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vec2 {
    pub x: f32,
    pub y: f32,
}

impl Vec2 {
    pub const ZERO: Self = Self { x: 0.0, y: 0.0 };

    fn dot(self, other: Self) -> f32 {
        return self.x * other.x + self.y * other.y;
    }

    fn cross(self, other: Self) -> f32 {
        return self.x * other.y - other.x * self.y;
    }

    fn length(self) -> f32 {
        return f32::sqrt(self.x.powi(2) + self.y.powi(2));
    }

    fn normalize(self) -> Self {
        let len = self.length();
        Self {
            x: self.x / len,
            y: self.y / len,
        }
    }

    fn perp(self) -> Self {
        Self {
            x: -self.x,
            y: self.y,
        }
    }

    fn new(x: f32, y: f32) -> Self {
        Self { x: x, y: y }
    }
}

impl Add for Vec2 {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}

impl Sub for Vec2 {
    type Output = Self;

    fn sub(self, other: Self) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }
}
