
use godot::prelude::*;

mod gd;

struct MyExtension;


static LOGGER: GodotLogger = GodotLogger {};

#[gdextension]
unsafe impl ExtensionLibrary for MyExtension {
    fn on_level_init(level: InitLevel) {
        if level == InitLevel::Scene {
            log::set_logger(&LOGGER).unwrap();
            log::set_max_level(log::LevelFilter::Trace);
            log::error!("Hi!");
            log::warn!("Hi!");
            log::info!("Hi!");
            log::debug!("Hi!");
            log::trace!("Hi!");
            godot_print!(">!!!!");
        }
    }
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
        godot_print_rich!("[color={color}]{}[/color] - {}", record.level(), record.args());
    }

    fn flush(&self) {
        
    }
}

