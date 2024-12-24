extends Node3D

# Used to show control points
@export var control_point_scene: PackedScene

# Relevant child nodes
@onready var camera: Camera3D = $PanOrbitCamera;
@onready var basic_lines: MeshInstance3D = $BasicLines;
@onready var optimizer: Optimizer = $Optimizer;
@onready var anim: Node3D = $Anim;
@onready var label: Label = $VBoxContainer/MainStats;
@onready var optimizer_speed_label: Label = $VBoxContainer/OptimizerSpdLabel
@onready var optimizer_checkbox: CheckButton = $VBoxContainer/CheckButton
@onready var load_file_dialog: FileDialog = $LoadDialogue
@onready var save_file_dialog: FileDialog = $SaveDialogue

# Point editing
@onready var x_edit: FloatEdit = $VBoxContainer/XEdit
@onready var y_edit: FloatEdit = $VBoxContainer/YEdit
@onready var z_edit: FloatEdit = $VBoxContainer/ZEdit
var selected_index;
var selected_point;

var optimize: bool = false
var control_points: Array[ControlPoint]

var curve: CoasterCurve
var physics: CoasterPhysicsV2

# default parameter values
var learning_rate: float = 1.0
var mass: float = 1.00
var gravity: float = -0.01
var friction: float = 0.05
var animation_step_size: float = 0.01
var com_offset_mag: float = 1.0

## Input is an array of Vector3
func set_points(points: Variant) -> void:
	# remove old control points
	for cp in control_points:
		cp.queue_free()
	control_points = []
	# create list of position vectors, calculate center
	var avg_pos = Vector3.ZERO
	for i in range(len(points)):
		var p = points[i]
		var v = Vector3(p[0], p[1], p[2])
		avg_pos += v
		var control_point: ControlPoint = control_point_scene.instantiate();
		control_point.initialize(v, i)
		control_point.connect("clicked", Callable(self, "_on_control_point_clicked"))
		control_points.push_back(control_point)
		
	avg_pos /= len(points)
	
	# position the camera so it can see all points
	camera.op = avg_pos
	var points_not_in_frame = true
	while points_not_in_frame:
		camera.recalculate_transform()
		camera.od += 1
		points_not_in_frame = false
		for cp in control_points:
			var p = cp.position
			if not camera.is_position_in_frustum(p):
				points_not_in_frame = true
				break

	# add points to scene
	for cp in control_points:
		add_child(cp)

	var positions: Array[Vector3] = []
	for cp in control_points:
		positions.push_back(cp.position)
	
	# update optimizer
	optimizer.set_points(positions)


## This function is called when the node is added to the scene.
## Initializes control points, positions the camera, and prepares the optimizer.
func _ready() -> void:
	load_file_dialog.use_native_dialog = true
	load_file_dialog.popup()

	# prepare the optimizer
	optimizer.set_mass(mass)
	optimizer.set_gravity(gravity)
	optimizer.set_mu(friction)
	optimizer.set_lr(learning_rate)
	optimizer.set_com_offset_mag(com_offset_mag)

	# ui setup
	x_edit.theme.set_color("font_color", "Label", Color.RED)
	x_edit.theme.set_color("font_color", "LineEdit", Color.RED)
	y_edit.theme.set_color("font_color", "Label", Color.GREEN)
	y_edit.theme.set_color("font_color", "LineEdit", Color.GREEN)
	z_edit.theme.set_color("font_color", "Label", Color.SKY_BLUE)
	z_edit.theme.set_color("font_color", "LineEdit", Color.SKY_BLUE)


func _process(_delta: float) -> void:
	# handle key input
	if Input.is_action_just_pressed("reset_curve"):
		var positions: Array[Vector3] = []
		for cp in control_points:
			positions.push_back(cp.position)
		optimizer.set_points(positions)
	if Input.is_action_just_pressed("run_simulation"):
		curve = optimizer.get_curve()
		physics = CoasterPhysicsV2.create(mass, gravity, com_offset_mag)		
	
	# update physics simulation
	if curve != null:
		anim.visible = true
		physics.step(curve, animation_step_size)
		var anim_pos = physics.pos(curve)
		var anim_vel = physics.velocity()
		print(anim_vel)
		var anim_up = physics.hl_normal()
		if anim_pos != null:
			anim.look_at_from_position(anim_pos, anim_pos + anim_vel, anim_up)
			#anim.position = anim_pos
			#anim. = Quaternion(anim_up, Vector3.UP).get_euler()
		else:
			anim.visible = false

	# generate mesh for curves between control points
	var curve_points = optimizer.as_segment_points();
	if len(curve_points) > 1:
		var m = basic_lines.mesh;
		m.clear_surfaces();
		m.surface_begin(Mesh.PRIMITIVE_TRIANGLES);

		Utils.cylinder_line(m, optimizer.as_segment_points(), 0.2)
				
		m.surface_end();
	
	# update ui
	var format_values;
	if physics != null:
		var v = physics.velocity()
		var n = physics.hl_normal()
		format_values = [
			optimizer.cost(),
			physics.speed(),
			physics.accel(),
			physics.g_force(),
			physics.max_g_force(),
			physics.cost(),
			v,
			n,
			90 - rad_to_deg(v.signed_angle_to(n, v.cross(n)))
		]
	else:
		format_values = [
			optimizer.cost(),
			0.0,
			0.0,
			0.0,
			0.0,
			0.0,
			Vector3.ZERO,
			Vector3.ZERO,
			0.0
		]
	label.text = """Cost: %.3f

		Speed: %.3f
		Accel: %.3f
		Gs: %.3f
		Max Gs: %.3f
		Cost: %.3f
		Velocity: %.3v
		Normal Force: %.3v
		Angle Error: %.3f""" % format_values
	var ips = optimizer.iters_per_second()
	if ips == null:
		optimizer_speed_label.text = "-- iter/s"
	else:
		optimizer_speed_label.text = "%.1f iter/s" % optimizer.iters_per_second()


func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventMouseButton\
		and event.button_index == MOUSE_BUTTON_LEFT\
		and event.is_pressed()\
	:
		selected_index = null
		selected_point = null
		for cp in control_points:
			cp.selected = false


func _unhandled_key_input(event: InputEvent) -> void:
	if event.is_action_pressed("toggle_optimizer"):
		optimize = !optimize
		if optimize:
			optimizer.enable_optimizer()
		else:
			optimizer.disable_optimizer()
	optimizer_checkbox.button_pressed = optimize


func _on_lr_edit_value_changed(value: float) -> void:
	print(value)
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


func _on_control_point_clicked(index: int) -> void:
	selected_index = index
	selected_point = optimizer.get_point(index)
	x_edit.set_value(selected_point.get_x())
	y_edit.set_value(selected_point.get_y())
	z_edit.set_value(selected_point.get_z())

	# update selected point
	for i in range(control_points.size()):
		control_points[i].selected = (i == index)


func _on_x_edit_value_changed(value: float) -> void:
	selected_point.set_x(value)
	control_points[selected_index].position.x = value
	optimizer.set_point(selected_index, selected_point)


func _on_y_edit_value_changed(value: float) -> void:
	selected_point.set_y(value)
	control_points[selected_index].position.y = value
	optimizer.set_point(selected_index, selected_point)


func _on_z_edit_value_changed(value: float) -> void:
	selected_point.set_z(value)
	control_points[selected_index].position.z = value
	optimizer.set_point(selected_index, selected_point)


func _on_load_button_pressed() -> void:
	load_file_dialog.popup()


func _on_save_button_pressed() -> void:
	save_file_dialog.popup()


func _on_save_dialogue_file_selected(path: String) -> void:
	print(path)

	var file = FileAccess.open(path, FileAccess.WRITE)
	print(FileAccess.get_open_error())

	var diag = AcceptDialog.new()
	diag.content_scale_factor = 2
	diag.dialog_text = "Failed to write file"

	if file == null:
		add_child(diag)
		diag.popup_centered_ratio()
		return
	
	file.store_string(
		JSON.stringify(
			control_points\
				.map(func(cp): return [cp.position.x, cp.position.y, cp.position.z]),
			"\t"
		)
	)
	print(file.get_error())
	file.close()
	print(file.get_error())


func _on_load_dialogue_file_selected(path: String) -> void:
	var file = FileAccess.open(path, FileAccess.READ)

	var diag = AcceptDialog.new()
	diag.content_scale_factor = 2
	diag.dialog_text = "Failed to open file"

	if file == null:
		add_child(diag)
		diag.popup()
		return

	var json = JSON.parse_string(file.get_as_text())
	if json is not Array:
		add_child(diag)
		diag.popup_centered_ratio()

		return
	
	for item in json:
		if item is not Array\
			or len(item) != 3\
			or item[0] is not float\
			or item[1] is not float\
			or item[2] is not float\
		:
			add_child(diag)
			diag.popup_centered_ratio()
			return

	var pts = json.map(func(i): return Vector3(i[0], i[1], i[2]))
	set_points(pts)
	file.close()


func _on_save_dialogue_confirmed() -> void:
	print("confirmed")


func _on_anim_step_edit_value_changed(value: float) -> void:
	animation_step_size = value
