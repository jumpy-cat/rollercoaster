#![feature(let_chains, array_windows)]
//#![recursion_limit = "2048"]

extern crate nalgebra as na;

use godot::prelude::*;

mod hermite;
mod optimizer;
mod physics;
mod point;

// this adds all the godot exports
mod gd;

struct MyExtension;

#[gdextension]
unsafe impl ExtensionLibrary for MyExtension {}


