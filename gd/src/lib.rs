#![feature(let_chains)]

use godot::prelude::*;

mod gd;

struct MyExtension;

#[gdextension]
unsafe impl ExtensionLibrary for MyExtension {}
