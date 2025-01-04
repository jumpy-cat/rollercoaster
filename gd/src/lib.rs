use godot::prelude::*;

mod gd;
mod logger;
mod persist;

struct MyExtension;

#[gdextension]
unsafe impl ExtensionLibrary for MyExtension {
    fn on_level_init(level: InitLevel) {
        //if level == InitLevel::Scene {
            logger::init();
            log::info!("Hi");
        //}
    }
}
