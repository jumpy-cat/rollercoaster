
use godot::prelude::*;

mod gd;

struct MyExtension;


static LOGGER: GodotLogger = GodotLogger {};

#[gdextension]
unsafe impl ExtensionLibrary for MyExtension {
    fn on_level_init(level: InitLevel) {
        match level {
            InitLevel::Scene => {
                log::set_logger(&LOGGER).unwrap();
                log::set_max_level(log::LevelFilter::Trace);
                log::error!("Hello Godot!");
                godot_print!(">!!!!");
            }
            _ => {}
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
        godot_print!("{} - {}", record.level(), record.args())
    }

    fn flush(&self) {
        
    }
}

