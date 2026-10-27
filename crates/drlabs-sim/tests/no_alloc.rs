//! The bus tick allocates nothing.
//!
//! A real-time thread that touches the allocator is a real-time thread that
//! can block on another thread's `free`. The simulator is the one bus that
//! runs in every test, so it is where the rule is enforced; the hardware
//! buses are held to the same rule by review and by the same counter.

use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicUsize, Ordering};

use drlabs_core::bus::{AxisCmd, AxisReply, Bus};
use drlabs_core::command::{Command, Field, Mode, QueryProfile};
use drlabs_core::units::Rev;
use drlabs_sim::SimBus;

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
fn a_bus_cycle_does_not_allocate() {
    let mut bus = SimBus::new(26);
    bus.open().unwrap();
    let cmds: Vec<AxisCmd> = (0..26)
        .map(|i| AxisCmd {
            command: Command {
                position: Field::Set(Rev(i as f64 * 0.01)),
                ..Command::new(Mode::Position)
            },
            profile: if i % 10 == 0 {
                QueryProfile::Full
            } else {
                QueryProfile::Fast
            },
        })
        .collect();
    let mut out = vec![AxisReply::SILENT; 26];

    // Warm up: anything lazily initialised happens here.
    for _ in 0..10 {
        bus.cycle(&cmds, &mut out).unwrap();
    }

    let before = ALLOCATIONS.load(Ordering::Relaxed);
    for _ in 0..500 {
        bus.cycle(&cmds, &mut out).unwrap();
    }
    let after = ALLOCATIONS.load(Ordering::Relaxed);
    assert_eq!(
        after - before,
        0,
        "the tick allocated {} times",
        after - before
    );
}
