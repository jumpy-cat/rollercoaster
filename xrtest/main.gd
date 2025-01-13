extends Node3D

var webxr_interface
var vr_supported = false
var active = false
var t = 0

var pos = []
var up = []
var fwd = []

@onready var mesh = $MeshInstance3D2

func arr_to_vec(x):
	return Vector3(x[0], x[1], x[2])

func _ready():
	# coaster setup
	#var rail1 = JSON.parse_string(
	#	FileAccess.open("/tmp/rail1.json", FileAccess.READ).get_as_text()
	#)
	var data = Data.new()
	var rail1 = data.rail1
	
	var rail1pts = rail1.map(arr_to_vec)
	var rail2pts = data.rail2.map(arr_to_vec)
	pos = data.pos.map(arr_to_vec)
	up = data.up.map(arr_to_vec)
	fwd = data.fwd.map(arr_to_vec)

	var m = mesh.mesh
	m.surface_begin(Mesh.PRIMITIVE_TRIANGLES)
	cylinder_line(m, rail1pts, 0.1)
	cylinder_line(m, rail2pts, 0.1)

	# cross
	var cross = data.cross
	for i in range(cross.size()):
		if cross[i]:
			cylinder_line(m, [rail1pts[i], rail2pts[i]], 0.1)
	
	m.surface_end()

	# We assume this node has a button as a child.
	# This button is for the user to consent to entering immersive VR mode.
	$Button.pressed.connect(self._on_button_pressed)
	$Button2.pressed.connect(self._on_normal_button_pressed)
	$Button3.pressed.connect(self._on_reset_button_pressed)

	webxr_interface = XRServer.find_interface("WebXR")
	if webxr_interface:
		# WebXR uses a lot of asynchronous callbacks, so we connect to various
		# signals in order to receive them.
		webxr_interface.session_supported.connect(self._webxr_session_supported)
		webxr_interface.session_started.connect(self._webxr_session_started)
		webxr_interface.session_ended.connect(self._webxr_session_ended)
		webxr_interface.session_failed.connect(self._webxr_session_failed)

		# This returns immediately - our _webxr_session_supported() method
		# (which we connected to the "session_supported" signal above) will
		# be called sometime later to let us know if it's supported or not.
		webxr_interface.is_session_supported("immersive-vr")

func _webxr_session_supported(session_mode, supported):
	if session_mode == 'immersive-vr':
		vr_supported = supported

func _on_normal_button_pressed():
	active = not active

func _on_reset_button_pressed():
	t = 0

func _on_button_pressed():
	if not vr_supported:
		OS.alert("Your browser doesn't support VR")
		return
	active = true

	# We want an immersive VR session, as opposed to AR ('immersive-ar') or a
	# simple 3DoF viewer ('viewer').
	webxr_interface.session_mode = 'immersive-vr'
	# 'bounded-floor' is room scale, 'local-floor' is a standing or sitting
	# experience (it puts you 1.6m above the ground if you have 3DoF headset),
	# whereas as 'local' puts you down at the XROrigin.
	# This list means it'll first try to request 'bounded-floor', then
	# fallback on 'local-floor' and ultimately 'local', if nothing else is
	# supported.
	webxr_interface.requested_reference_space_types = 'bounded-floor, local-floor, local'
	# In order to use 'local-floor' or 'bounded-floor' we must also
	# mark the features as required or optional. By including 'hand-tracking'
	# as an optional feature, it will be enabled if supported.
	webxr_interface.required_features = 'local-floor'
	webxr_interface.optional_features = 'bounded-floor, hand-tracking'

	# This will return false if we're unable to even request the session,
	# however, it can still fail asynchronously later in the process, so we
	# only know if it's really succeeded or failed when our
	# _webxr_session_started() or _webxr_session_failed() methods are called.
	if not webxr_interface.initialize():
		OS.alert("Failed to initialize")
		return

func _webxr_session_started():
	$Button.visible = false
	# This tells Godot to start rendering to the headset.
	get_viewport().use_xr = true
	# This will be the reference space type you ultimately got, out of the
	# types that you requested above. This is useful if you want the game to
	# work a little differently in 'bounded-floor' versus 'local-floor'.
	print("Reference space type: ", webxr_interface.reference_space_type)
	# This will be the list of features that were successfully enabled
	# (except on browsers that don't support this property).
	print("Enabled features: ", webxr_interface.enabled_features)

func _webxr_session_ended():
	$Button.visible = true
	# If the user exits immersive mode, then we tell Godot to render to the web
	# page again.
	get_viewport().use_xr = false

func _webxr_session_failed(message):
	OS.alert("Failed to initialize: " + message)

func _process(_delta):
	if t < len(pos):
		$XROrigin3D.look_at_from_position(pos[t], pos[t] + fwd[t], up[t])
	if active:
		t += 1
		#$XROrigin3D.global_position.z -= _delta


## Adds 3d lines between [points] with the given [radius]
static func cylinder_line(m, points, radius):
	if len(points) > 1:
		for i in range(1, len(points)):
			# create cylinder
			const NUM_QUADS = 10
			var p = points[i-1]
			var q = points[i]
			var pq: Vector3 = q - p
			var rot = Quaternion(pq.normalized(), Vector3.UP).normalized()
			
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


## Adds a quad to ImmediateMesh
## CCW winding order
static func add_quad(m, p1, p2, p3, p4, n1, n2, n3, n4):
	add_tri(m, p1, p2, p3, n1, n2, n3)
	add_tri(m, p3, p4, p1, n3, n4, n1)

## Adds a triangle to ImmediateMesh
## CCW winding order
static func add_tri(m, p1, p2, p3, n1, n2, n3):
	m.surface_set_normal(n1)
	m.surface_add_vertex(p1)
	m.surface_set_normal(n2)
	m.surface_add_vertex(p2)
	m.surface_set_normal(n3)
	m.surface_add_vertex(p3)
