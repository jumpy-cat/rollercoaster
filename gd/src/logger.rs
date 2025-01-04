//! Colored logs to godot output
//!

use godot::global::godot_print_rich;

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
        //godot_print_rich!()
        let color = match record.level() {
            log::Level::Error => "red",
            log::Level::Warn => "yellow",
            log::Level::Info => "cyan",
            log::Level::Debug => "magenta",
            log::Level::Trace => "pink",
        };
        godot_print_rich!(
            "[color={color}]{}[/color] - {}",
            record.level(),
            record.args()
        );
    }

    fn flush(&self) {}
}
