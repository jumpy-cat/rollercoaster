extends Node3D

#const __ = preload("res://components/PointEditComponent.tscn")
#const ___ = preload("res://components/ParamsManager.tscn")

# Used to show control points
@export var control_point_scene: PackedScene
@export var point_edit_component: PointEditComponent
@export var params_manager: ParamsManager
@export var save_file_dialog: FileDialog
@export var ke_chart: Chart

# Relevant child nodes
@onready var camera: Camera3D = $PanOrbitCamera;
@onready var basic_lines: MeshInstance3D = $BasicLines;

@onready var anim: Node3D = $Anim;
@onready var label: Label = $VBoxContainer/MainStats;
@onready var optimizer_speed_label: Label = $VBoxContainer/OptimizerSpdLabel
@onready var optimizer_checkbox: CheckButton = $VBoxContainer/CheckButton

var optimizer: Optimizer;
var camera_follow_anim: bool = false
var manual_physics = false
var optimize: bool = false

var selected_index;
var selected_point;
var control_points: Array[ControlPoint]

var curve: CoasterCurve
var physics: CoasterPhysicsV3

var f1: Function


## This function is called when the node is added to the scene.
## Initializes control points, positions the camera, and prepares the optimizer.
func _ready() -> void:
	point_edit_component.request_points();

	# prepare the optimizer
	optimizer = Optimizer.create();
	params_manager.apply_to_optimizer(optimizer)
	
	var conf = DebugDraw2D.get_config()
	conf.set_text_default_size(30)
	DebugDraw2D.set_config(conf)

	# Let's create our @x values
	var x: Array = [0,0]
	
	# And our y values. It can be an n-size array of arrays.
	# NOTE: `x.size() == y.size()` or `x.size() == y[n].size()`
	var y: Array = ["Translation", "Rotation"]
	
	# Let's customize the chart properties, which specify how the chart
	# should look, plus some additional elements like labels, the scale, etc...
	var cp: ChartProperties = ChartProperties.new()
	cp.colors.frame = Color("#161a1d")
	cp.colors.background = Color.TRANSPARENT
	cp.colors.grid = Color("#283442")
	cp.colors.ticks = Color("#283442")
	cp.colors.text = Color.WHITE_SMOKE
	cp.show_tick_labels = false
	cp.draw_bounding_box = false
	cp.title = "Kinetic Energy"
	cp.draw_grid_box = false
	cp.show_legend = true
	cp.interactive = true # false by default, it allows the chart to create a tooltip to show point values
	# and intercept clicks on the plot
	
	var gradient: Gradient = Gradient.new()
	gradient.set_color(0, Color.BLUE)
	gradient.set_color(1, Color.DEEP_PINK)
	
	# Let's add values to our functions
	f1 = Function.new(
		x, y, "Language", # This will create a function with x and y values taken by the Arrays 
						# we have created previously. This function will also be named "Pressure"
						# as it contains 'pressure' values.
						# If set, the name of a function will be used both in the Legend
						# (if enabled thourgh ChartProperties) and on the Tooltip (if enabled).
		{
			gradient = gradient,
			type = Function.Type.PIE
		}
	)
	
	# Now let's plot our data
	ke_chart.plot([f1], cp)


var hist_pos = []
var last_pos = Vector3.ZERO

const COM_OFFSET = 1;


func _process(_delta: float) -> void:
	optimizer.update()
	DebugDraw2D.set_text("FPS", Engine.get_frames_per_second())
	
	# update physics simulation
	if curve != null:
		anim.visible = true
		"""&& physics.found_exact_solution()"""
		var physics_did_step = ((!manual_physics) # && physics.found_exact_solution())
			|| Input.is_action_just_pressed("step_physics"))
		if physics_did_step:
			physics.step(curve, params_manager.anim_step_size)
		var anim_pos = physics.pos()
		var anim_vel = physics.vel()
		var anim_up = physics.hl_normal()

		if camera_follow_anim:
			camera.op = anim_pos
		
		for i in range(len(hist_pos) - 1):
			DebugDraw3D.draw_line(
				hist_pos[i],
				hist_pos[i + 1],
				Color.PURPLE
			)
		const HIST_LINE_UPDATE_DIST = 0.1
		if (anim_pos - last_pos).length() > HIST_LINE_UPDATE_DIST:
			hist_pos.push_back(anim_pos)
			last_pos = anim_pos

		const MULT = 1000;

		DebugDraw3D.draw_line(anim_pos, anim_pos + MULT * anim_vel, Color.BLUE)
		#DebugDraw3D.draw_line(anim_pos, anim_pos + MULT * anim_up, Color.GREEN)
		var cp = curve.pos_at(physics.u());
		var r = 1 / curve.kappa_at(physics.u()) + COM_OFFSET
		var cpag = cp + MULT * physics.vel().length() ** 2 / r * curve.normal_at(physics.u())
		DebugDraw3D.draw_line(cp, cpag, Color.RED)
		DebugDraw3D.draw_line(cpag, cpag + MULT * Vector3(0, 0.01, 0), Color.ORCHID)
		DebugDraw3D.draw_line(cp, cpag + MULT * Vector3(0, 0.01, 0), Color.WHITE)

		DebugDraw3D.draw_line(cp, cp + MULT * curve.normal_at(physics.u()), Color.YELLOW)

		if !physics.jitter_detected():
			DebugDraw3D.draw_sphere(curve.pos_at(physics.u()), 0.4, Color.YELLOW)
		else:
			DebugDraw3D.draw_sphere(curve.pos_at(physics.u()), 0.6, Color.RED)
		DebugDraw3D.draw_sphere(physics.null_tgt_pos(), 0.2, Color.PURPLE)
		DebugDraw3D.draw_sphere(physics.tgt_pos(), 0.2, Color.PINK)

		f1.set_point(0, physics.t_kinetic_energy(), 0)
		f1.set_point(1, physics.r_kinetic_energy(), 0)
		ke_chart.queue_redraw()

		if anim_pos != null:
			if anim_pos != anim_pos + anim_vel:
				anim.look_at_from_position(anim_pos, anim_pos + anim_vel, anim_up)
		else:
			anim.visible = false

	# generate mesh for curves between control points
	var curve_points = optimizer.as_segment_points();
	if len(curve_points) > 1:
		var m = basic_lines.mesh;
		m.clear_surfaces();
		m.surface_begin(Mesh.PRIMITIVE_LINES);

		Utils.cylinder_line(m, optimizer.as_segment_points(), 0.2)
				
		m.surface_end();

	if physics == null:
		label.text = "physics not initialized"
	else:
		label.text = physics.description()
	var ips = optimizer.iters_per_second()
	if ips == null:
		optimizer_speed_label.text = "-- iter/s"
	else:
		optimizer_speed_label.text = "%.1f iter/s" % optimizer.iters_per_second()


## Input is an array of Vector3
func set_points(points: Array[Vector3]) -> void:
	# remove old control points
	for cp in control_points:
		cp.queue_free()
	control_points = []

	# create list of position vectors, calculate center, and create control points
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


func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventMouseButton \
		and event.button_index == MOUSE_BUTTON_LEFT \
		and event.is_pressed() \
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
	if event.is_action_pressed("reset_curve"):
		var positions: Array[Vector3] = []
		for cp in control_points:
			positions.push_back(cp.position)
		optimizer.set_points(positions)
	if event.is_action_pressed("toggle_follow_anim"):
		camera_follow_anim = !camera_follow_anim
	if event.is_action_pressed("run_simulation"):
		curve = optimizer.get_curve()

		physics = CoasterPhysicsV3.create(params_manager.mass, params_manager.gravity, curve, COM_OFFSET)


func _on_check_button_toggled(toggled_on: bool) -> void:
	if toggled_on:
		optimizer.enable_optimizer()
	else:
		optimizer.disable_optimizer()


func _on_control_point_clicked(index: int) -> void:
	selected_index = index
	selected_point = optimizer.get_point(index)
	point_edit_component.set_point_pos(selected_point)

	# update selected point
	for i in range(control_points.size()):
		control_points[i].selected = (i == index)


func _on_check_box_toggled(toggled_on: bool) -> void:
	manual_physics = toggled_on


func _on_save_button_pressed() -> void:
	save_file_dialog.popup()


func _on_save_dialogue_file_selected(path: String) -> void:
	var p: Array[Vector3] = [];
	for cp in control_points:
		p.push_back(cp.posi)
	
	var saver = Saver.to_path(path, p)
	if saver.success():
		return

	var diag = AcceptDialog.new()
	diag.content_scale_factor = 2
	diag.dialog_text = "Failed to write file"
	add_child(diag)
	diag.popup_centered_ratio()


func _on_point_edit_component_points_loaded(pts: Array) -> void:
	set_points(pts)


func _on_point_edit_component_pos_changed(pos: Vector3) -> void:
	if selected_point:
		selected_point.set_x(pos.x)
		selected_point.set_y(pos.y)
		selected_point.set_z(pos.z)
		control_points[selected_index].position = pos
		optimizer.set_point(selected_index, selected_point)


func _on_point_edit_component_points_failed_to_load() -> void:
	var diag = AcceptDialog.new()
	diag.content_scale_factor = 2

	diag.dialog_text = "Failed to open file"

	add_child(diag)
	diag.popup_centered_ratio()


func _on_params_manager_params_changed() -> void:
	params_manager.apply_to_optimizer(optimizer)
