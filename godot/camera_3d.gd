extends Camera3D

@export var op: Vector3 = Vector3.ZERO;
@export var od: float = 1.0;
const MIN_D: float = 1.0;
# angle ccw from x axis on the xz plane
# looking in the -y direction
var ob: float = 0.0;
var oe: float = 0.0;
const OE_LIM: float = 60 * PI / 180;



func _ready() -> void:
	pass # Replace with function body.


func _process(delta: float) -> void:
	const PAN_SPD: float = 0.8;
	const ORBIT_SPD: float = 1.0;
	
	const ZOOM_SPD: float = 1.0;
	const ZOOM_SPD_MULT: float = 1.0;
	
	var pan_mult = PAN_SPD * delta * od;
	if Input.is_action_pressed("camera_forward"):
		op += pan_mult * Vector3(-cos(ob), 0, -sin(ob)) 
	if Input.is_action_pressed("camera_back"):
		op += pan_mult * Vector3(cos(ob), 0, sin(ob)) 
	if Input.is_action_pressed("camera_left"):
		op += pan_mult * Vector3(-sin(ob), 0, cos(ob))
	if Input.is_action_pressed("camera_right"):
		op += pan_mult * Vector3(sin(ob), 0, -cos(ob))
	if Input.is_action_pressed("camera_up"):
		op += pan_mult * Vector3.UP;
	if Input.is_action_pressed("camera_down"):
		op += pan_mult * Vector3.DOWN;
	
	if not Input.is_key_pressed(KEY_SHIFT):
		if Input.is_action_pressed("zoom_in"):
			od = max(od - ZOOM_SPD * delta, MIN_D);
			od /= 1.0 + ZOOM_SPD_MULT * delta;
		if Input.is_action_pressed("zoom_out"):
			od += ZOOM_SPD * delta;
			od *= 1.0 + ZOOM_SPD_MULT * delta;
		if Input.is_action_pressed("orbit_left"):
			ob += ORBIT_SPD * delta;
		if Input.is_action_pressed("orbit_right"):
			ob += -ORBIT_SPD * delta;
		if Input.is_action_pressed("orbit_up"):
			oe += -ORBIT_SPD * delta;
		if Input.is_action_pressed("orbit_down"):
			oe += ORBIT_SPD * delta;
		oe = clamp(oe, -OE_LIM, OE_LIM)
		
	recalculate_transform();


func recalculate_transform():
	var optc = Vector3(1.0, 0.0, 0.0);
	var bquat = Quaternion(Vector3.UP, ob);
	optc *= bquat;
	optc *= Quaternion((Vector3(0, 0, 1) * bquat), oe);
	look_at_from_position(op + optc * od, op)

	
