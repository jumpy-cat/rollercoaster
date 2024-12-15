extends Node3D

@export var control_point_scene: PackedScene
@onready var camera: Camera3D = $PanOrbitCamera;
@onready var basic_lines: MeshInstance3D = $BasicLines;
@onready var optimizer: Optimizer = $Optimizer;

var optimize = false
var pos: Array[Vector3] = []
var initial_pos = [[30, 24, 1], [21, 6, 1], [19, 12, 1], [21, 18, 3],
[25, 16, 2], [23, 9, 4], [20, 13, 4], [16, 2, 6], [15, 7, 4],
[17, 7, 1], [14, 3, 2], [12, 5, 0], [7, 6, 6], [6, 8, 12],
[11, 9, 3], [8, 13, 3], [2, 5, 3], [0, 0, 0], [47, 0, 1], [43, 0, 1]]

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	var avg_pos = Vector3.ZERO
	for p in initial_pos:
		var v = Vector3(p[0], p[1], p[2])
		pos.push_back(v)
		avg_pos += v
		var control_point = control_point_scene.instantiate();
		control_point.initialize(v)
		add_child(control_point)
		
	avg_pos /= len(initial_pos)
	
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
	optimizer.set_points(pos)


func _process(_delta: float) -> void:
	if Input.is_action_just_pressed("toggle_optimizer"):
		optimize = !optimize
		if optimize:
			optimizer.enable_optimizer()
		else:
			optimizer.disable_optimizer()

	var m = basic_lines.mesh;
	m.clear_surfaces();
	m.surface_begin(Mesh.PRIMITIVE_TRIANGLES);
	
	#cylinder_line(m, pos, 0.01)

	#print(optimizer.as_segment_points())

	cylinder_line(m, optimizer.as_segment_points(), 0.2)
			
	m.surface_end();

func cylinder_line(m, points, radius):
	if len(points) > 1:
		for i in range(1, len(points)):
			# create cylinder
			const NUM_QUADS = 10
			var p = points[i-1]
			var q = points[i]
			var pq: Vector3 = q - p
			var rot = Quaternion(pq.normalized(), Vector3.UP)
			
			for j in range(NUM_QUADS):
				var theta0 = 2*PI * j / NUM_QUADS
				var theta1 = 2*PI * (j+1) / NUM_QUADS
				var offset0 = (Vector3(1, 0, 0) * Quaternion(Vector3.UP, theta0)) * rot
				var offset1 = (Vector3(1, 0, 0) * Quaternion(Vector3.UP, theta1)) * rot
				var quad = [
					p + radius * (offset1),
					q + radius * (offset1),
					q + radius * (offset0),
					p + radius * (offset0),
					
					offset1,
					offset1,
					offset0,
					offset0,
				]
				add_quad(
					m,
					quad[0], quad[1], quad[2], quad[3],
					quad[4], quad[5], quad[6], quad[7]
				)


func add_quad(m, p1, p2, p3, p4, n1, n2, n3, n4):
	add_tri(m, p1, p2, p3, n1, n2, n3)
	add_tri(m, p3, p4, p1, n3, n4, n1)

func add_tri(m, p1, p2, p3, n1, n2, n3):
	m.surface_set_normal(n1)
	m.surface_add_vertex(p1)
	m.surface_set_normal(n2)
	m.surface_add_vertex(p2)
	m.surface_set_normal(n3)
	m.surface_add_vertex(p3)

	
