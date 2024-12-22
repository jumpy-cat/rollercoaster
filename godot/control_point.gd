extends Node3D
class_name ControlPoint

@onready var axes: MeshInstance3D = $Axes

var index: int
var selected: bool = false
var hovered: bool = false

signal clicked(index)

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
    pass

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(_delta: float) -> void:
    if selected or hovered:
        scale = Vector3(2.0, 2.0, 2.0)
    else:
        scale = Vector3.ONE

    var m: ImmediateMesh = axes.mesh
    m.clear_surfaces()
    if selected:
        m.surface_begin(Mesh.PRIMITIVE_TRIANGLES)
        Utils.cylinder_line(m, [Vector3.ZERO, Vector3(0, 100, 0)], 0.2)
        Utils.cylinder_line(m, [Vector3.ZERO, Vector3(0, 0, 100)], 0.2)
        Utils.cylinder_line(m, [Vector3.ZERO, Vector3(100, 0, 0)], 0.2)
        m.surface_end()


func initialize(pos: Vector3, index_: int) -> void:
    translate(pos)
    index = index_


## Increase size when mouse hovering over control point
func _on_static_body_3d_mouse_entered() -> void:
    hovered = true


## Reset size when mouse leaves control point
func _on_static_body_3d_mouse_exited() -> void:
    hovered = false


## Handle mouse click on control point
func _on_static_body_3d_input_event(
    _camera: Node,
    event: InputEvent,
    _event_position: Vector3,
    _normal: Vector3,
    _shape_idx: int
) -> void:
    if event is InputEventMouseButton and event.button_index == MOUSE_BUTTON_LEFT and event.is_pressed():
        clicked.emit(index)
