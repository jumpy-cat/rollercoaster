@tool

class_name ParamsManager extends Control

signal params_changed();

# default parameter values
var learning_rate: float = 1.0
var mass: float = 1.00
var gravity: float = -0.01
var friction: float = 0.05
var anim_step_size: float = 0.05
var com_offset_mag: float = 1.0

# parameter editing
@export var lr_edit: FloatEdit
@export var mass_edit: FloatEdit
@export var gravity_edit: FloatEdit
@export var friction_edit: FloatEdit
@export var anim_step_edit: FloatEdit
@export var com_offset_edit: FloatEdit

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	# ui setup
	lr_edit.set_value(learning_rate)
	mass_edit.set_value(mass)
	gravity_edit.set_value(gravity)
	friction_edit.set_value(friction)
	anim_step_edit.set_value(anim_step_size)
	com_offset_edit.set_value(com_offset_mag)


func apply_to_optimizer(optimizer: Optimizer) -> void:
	optimizer.set_lr(learning_rate)
	optimizer.set_mass(mass)
	optimizer.set_gravity(gravity)
	optimizer.set_mu(friction)
	optimizer.set_com_offset_mag(com_offset_mag)


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	pass


func _on_lr_edit_value_changed(value: float) -> void:
	print(value)
	learning_rate = value
	params_changed.emit()
	#optimizer.set_lr(learning_rate)


func _on_mass_edit_value_changed(value: float) -> void:
	mass = value
	params_changed.emit()
	#optimizer.set_mass(mass)


func _on_gravity_edit_value_changed(value: float) -> void:
	gravity = value
	params_changed.emit()
	#optimizer.set_gravity(gravity)


func _on_friction_edit_value_changed(value: float) -> void:
	friction = value
	params_changed.emit()
	#optimizer.set_mu(friction)


func _on_anim_step_edit_value_changed(value: float) -> void:
	anim_step_size = value
	params_changed.emit()


func _on_com_offset_edit_value_changed(value: float) -> void:
	com_offset_mag = value
	params_changed.emit()
