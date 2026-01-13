pub fn limit(mag: f32, low: f32, high: f32) -> f32 {
    if mag > high {
        high
    } else if mag < low {
        low
    } else {
        mag
    }
}