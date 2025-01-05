extends Camera3D

@export var op: Vector3 = Vector3.ZERO;
@export var od: float = 1.0;
const MIN_D: float = 1.0;
# angle ccw from x axis on the xz plane
# looking in the -y direction
var ob: float = 0.0;
var oe: float = 0.0;
const OE_LIM: float = 60 * PI / 180;

# input state
var forward_input: bool = false
var back_input: bool = false
var left_input: bool = false
var right_input: bool = false
var up_input: bool = false
var down_input: bool = false


func _ready() -> void:
	pass # Replace with function body.


func _process(delta: float) -> void:
	const PAN_SPD: float = 0.8;
	const ORBIT_SPD: float = 1.0;
	
	const ZOOM_SPD: float = 1.0;
	const ZOOM_SPD_MULT: float = 1.0;
	var pan_mult = PAN_SPD * delta * od;

	if Input.is_key_pressed(KEY_SHIFT):
		if forward_input:
			op += pan_mult * Vector3(-cos(ob), 0, -sin(ob)) 
		if back_input:
			op += pan_mult * Vector3(cos(ob), 0, sin(ob)) 
		if left_input:
			op += pan_mult * Vector3(-sin(ob), 0, cos(ob))
		if right_input:
			op += pan_mult * Vector3(sin(ob), 0, -cos(ob))
		if up_input:
			op += pan_mult * Vector3.UP;
		if down_input:
			op += pan_mult * Vector3.DOWN;
	else:
		if forward_input:
			od = max(od - ZOOM_SPD * delta, MIN_D);
			od /= 1.0 + ZOOM_SPD_MULT * delta;
		if back_input:
			od += ZOOM_SPD * delta;
			od *= 1.0 + ZOOM_SPD_MULT * delta;
		if left_input:
			ob += ORBIT_SPD * delta;
		if right_input:
			ob += -ORBIT_SPD * delta;
		if up_input:
			oe += -ORBIT_SPD * delta;
		if down_input:
			oe += ORBIT_SPD * delta;
		oe = clamp(oe, -OE_LIM, OE_LIM)
		
	recalculate_transform();


func recalculate_transform():
	var optc = Vector3(1.0, 0.0, 0.0);
	var bquat = Quaternion(Vector3.UP, ob);
	optc *= bquat;
	optc *= Quaternion((Vector3(0, 0, 1) * bquat), oe);
	look_at_from_position(op + optc * od, op)


func _unhandled_key_input(event: InputEvent) -> void:
	if event.is_action_pressed("zoom_out"):
		back_input = true
	if event.is_action_released("zoom_out"):
		back_input = false
	if event.is_action_pressed("zoom_in"):
		forward_input = true
	if event.is_action_released("zoom_in"):
		forward_input = false
	if event.is_action_pressed("orbit_left"):
		left_input = true
	if event.is_action_released("orbit_left"):
		left_input = false
	if event.is_action_pressed("orbit_right"):
		right_input = true
	if event.is_action_released("orbit_right"):
		right_input = false
	if event.is_action_pressed("orbit_up"):
		up_input = true
	if event.is_action_released("orbit_up"):
		up_input = false
	if event.is_action_pressed("orbit_down"):
		down_input = true
	if event.is_action_released("orbit_down"):
		down_input = false
	

	
