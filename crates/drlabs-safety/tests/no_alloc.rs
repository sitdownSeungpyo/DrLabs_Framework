//! `Guard::evaluate` allocates nothing, warnings or no warnings.

use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::Instant;

use drlabs_core::bus::{AxisFault, AxisReply, AxisStatus};
use drlabs_core::units::{Amps, Celsius, Rev, RevPerSec, Volts};
use drlabs_safety::Guard;

struct Counting;

static ALLOCATIONS: AtomicUsize = AtomicUsize::new(0);

unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        ALLOCATIONS.fetch_add(1, Ordering::Relaxed);
        System.alloc(layout)
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        System.dealloc(ptr, layout)
    }
}

#[global_allocator]
static GLOBAL: Counting = Counting;

#[test]
fn evaluating_an_axis_does_not_allocate() {
    // Every warning at once, and a limiting fault: the paths that cost
    // `Monitor` a string each.
    let noisy = AxisReply {
        status: AxisStatus::Alert,
        mode: Some(10),
        fault: Some(AxisFault {
            code: 102,
            hard: false,
        }),
        position: Some(Rev(0.52)),
        velocity: Some(RevPerSec(0.0)),
        torque: None,
        current: Some(Amps(50.0)),
        voltage: Some(Volts(41.0)),
        board_temperature: Some(Celsius(70.0)),
        motor_temperature: Some(Celsius(90.0)),
    };
    let mut guards = vec![Guard::default(); 26];
    let now = Instant::now();

    let before = ALLOCATIONS.load(Ordering::Relaxed);
    for tick in 0..500u32 {
        let at = now + std::time::Duration::from_millis(2 * tick as u64);
        for guard in guards.iter_mut() {
            let _ = guard.evaluate(&noisy, RevPerSec(1.0), true, at);
            let _ = guard.preflight(&noisy);
        }
    }
    let after = ALLOCATIONS.load(Ordering::Relaxed);
    assert_eq!(
        after - before,
        0,
        "evaluate allocated {} times",
        after - before
    );
}
