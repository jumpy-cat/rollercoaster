extends Node3D
class_name ControlPoint


# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	pass # Replace with function body.


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	pass


func initialize(pos):
	translate(pos)


func _on_static_body_3d_mouse_entered() -> void:
	scale = Vector3(2.0, 2.0, 2.0)


func _on_static_body_3d_mouse_exited() -> void:
	scale = Vector3.ONE


func _on_static_body_3d_input_event(_camera: Node, event: InputEvent, _event_position: Vector3, _normal: Vector3, _shape_idx: int) -> void:
	if event is InputEventMouseButton and event.button_index == MOUSE_BUTTON_LEFT and event.is_pressed():
		pass
