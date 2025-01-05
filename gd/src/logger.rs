//! Colored logs to godot output
//!

use std::{
    collections::VecDeque,
    sync::{
        LazyLock, Mutex,
    },
};

use godot::global::godot_print_rich;

static LOG_QUEUE: LazyLock<Mutex<VecDeque<String>>> = LazyLock::new(|| Mutex::new(VecDeque::new()));

static LOGGER: GodotLogger = GodotLogger {};

pub fn init() {
    let _ = log::set_logger(&LOGGER);
    log::set_max_level(log::LevelFilter::Trace);
}

struct GodotLogger {}

impl log::Log for GodotLogger {
    fn enabled(&self, _metadata: &log::Metadata) -> bool {
        true
    }

    fn log(&self, record: &log::Record) {
        let color = match record.level() {
            log::Level::Error => "red",
            log::Level::Warn => "yellow",
            log::Level::Info => "cyan",
            log::Level::Debug => "magenta",
            log::Level::Trace => "pink",
        };
        LOG_QUEUE.lock().unwrap().push_back(format!(
            "[color={color}]{}[/color] - {}",
            record.level(),
            record.args()
        ));
    }

    fn flush(&self) {}
}

use godot::prelude::*;

#[derive(GodotClass)]
#[class(base=Node)]
struct Logger {}

#[godot_api]
impl INode for Logger {
    fn init(_base: Base<Node>) -> Self {
        Self {}
    }

    fn process(&mut self, _delta: f64) {
        let mut queue = LOG_QUEUE.lock().unwrap();
        while let Some(msg) = queue.pop_front() {
            godot_print_rich!("{}", msg);
        }
    }
}


