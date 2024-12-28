#![feature(let_chains)]

use godot::prelude::*;

// this adds all the godot exports
mod gd;

struct MyExtension;

#[gdextension]
unsafe impl ExtensionLibrary for MyExtension {}
