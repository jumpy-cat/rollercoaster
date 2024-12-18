extends Node3D

# Used to show control points
@export var control_point_scene: PackedScene

# Relevant child nodes
@onready var camera: Camera3D = $PanOrbitCamera;
@onready var basic_lines: MeshInstance3D = $BasicLines;
@onready var optimizer: Optimizer = $Optimizer;
@onready var anim: Node3D = $Anim;
@onready var label: Label = $VBoxContainer/MainStats;

const utils = preload("res://utils.gd")

var optimize: bool = false
var pos: Array[Vector3] = []
var curve: CoasterCurve
var physics: CoasterPhysics

# default parameter values
var learning_rate: float = 1.0
var mass: float = 1.00
var gravity: float = -0.01
var friction: float = 0.05

# paste in points from outside source here
var initial_pos = [[30, 24, 1], [21, 6, 1], [19, 12, 1], [21, 18, 3],
[25, 16, 2], [23, 9, 4], [20, 13, 4], [16, 2, 6], [15, 7, 4],
[17, 7, 1], [14, 3, 2], [12, 5, 0], [7, 6, 6], [6, 8, 12],
[11, 9, 3], [8, 13, 3], [2, 5, 3], [0, 0, 0], [47, 0, 1], [43, 0, 1]]

func _ready() -> void:
	# create list of position vectors, add points to scene, calculate center
	var avg_pos = Vector3.ZERO
	for p in initial_pos:
		var v = Vector3(p[0], p[1], p[2])
		pos.push_back(v)
		avg_pos += v
		var control_point = control_point_scene.instantiate();
		control_point.initialize(v)
		add_child(control_point)
		
	avg_pos /= len(initial_pos)
	
	# position the camera so it can see all points
	camera.op = avg_pos
	var points_not_in_frame = true
	while points_not_in_frame:
		camera.recalculate_transform()
		camera.od += 1
		points_not_in_frame = false
		for p in pos:
			if not camera.is_position_in_frustum(p):
				points_not_in_frame = true
				break
	# prepare the optimizer
	optimizer.set_points(pos)
	optimizer.set_mass(mass)
	optimizer.set_gravity(gravity)
	optimizer.set_mu(friction)
	optimizer.set_lr(learning_rate)


func _process(_delta: float) -> void:
	# handle key input
	if Input.is_action_just_pressed("reset_curve"):
		optimizer.set_points(pos)
	if Input.is_action_just_pressed("run_simulation"):
		curve = optimizer.get_curve()
		physics = CoasterPhysics.create(mass, gravity, friction)
			
	if Input.is_action_just_pressed("toggle_optimizer"):
		optimize = !optimize
		if optimize:
			optimizer.enable_optimizer()
		else:
			optimizer.disable_optimizer()
	
	# update physics simulation
	if curve != null:
		anim.visible = true
		physics.step(curve)
		var anim_pos = physics.pos(curve)
		if anim_pos != null:
			anim.position = anim_pos
		else:
			anim.visible = false

	# generate mesh for curves between control points
	var curve_points = optimizer.as_segment_points();
	if len(curve_points) > 1:
		var m = basic_lines.mesh;
		m.clear_surfaces();
		m.surface_begin(Mesh.PRIMITIVE_TRIANGLES);

		utils.cylinder_line(m, optimizer.as_segment_points(), 0.2)
				
		m.surface_end();
	
	# update ui
	var format_values;
	if physics != null:
		format_values = [
			optimizer.cost(),
			physics.speed(),
			physics.accel(),
			physics.g_force(),
			physics.max_g_force(),
			physics.cost()
		]
	else:
		format_values = [
			optimizer.cost(),
			0.0,
			0.0,
			0.0,
			0.0,
			0.0,
		]
	label.text = "Cost: %.3f\n\nSpeed: %.3f\nAccel: %.3f\nGs: %.3f\nMax Gs: %.3f\nCost: %.3f" % format_values
			


func _on_lr_edit_value_changed(value: float) -> void:
	learning_rate = value
	optimizer.set_lr(learning_rate)


func _on_mass_edit_value_changed(value: float) -> void:
	mass = value
	optimizer.set_mass(mass)


func _on_gravity_edit_value_changed(value: float) -> void:
	gravity = value
	optimizer.set_gravity(gravity)


func _on_friction_edit_value_changed(value: float) -> void:
	friction = value
	optimizer.set_mu(friction)


func _on_check_button_toggled(toggled_on: bool) -> void:
	if toggled_on:
		optimizer.enable_optimizer()
	else:
		optimizer.disable_optimizer()
