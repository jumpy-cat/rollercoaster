@tool

extends Control
class_name PointEditComponent

@export var x_edit: FloatEdit
@export var y_edit: FloatEdit
@export var z_edit: FloatEdit
@export var load_file_dialog: FileDialog

signal points_loaded(pts: Array)
signal points_failed_to_load();
signal pos_changed(pos: Vector3)

var point_pos: Vector3

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	x_edit.theme.set_color("font_color", "Label", Color.RED)
	x_edit.theme.set_color("font_color", "LineEdit", Color.RED)
	y_edit.theme.set_color("font_color", "Label", Color.GREEN)
	y_edit.theme.set_color("font_color", "LineEdit", Color.GREEN)
	z_edit.theme.set_color("font_color", "Label", Color.SKY_BLUE)
	z_edit.theme.set_color("font_color", "LineEdit", Color.SKY_BLUE)


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	pass


func request_points() -> void:
	load_file_dialog.use_native_dialog = true
	load_file_dialog.popup()


func set_point_pos(p: CoasterPoint) -> void:
	x_edit.set_value(p.get_x())
	y_edit.set_value(p.get_y())
	z_edit.set_value(p.get_x())


func _on_load_button_pressed() -> void:
	load_file_dialog.popup()


func _on_x_edit_value_changed(value: float) -> void:
	point_pos.x = value
	pos_changed.emit(point_pos)


func _on_y_edit_value_changed(value: float) -> void:
	point_pos.y = value
	pos_changed.emit(point_pos)


func _on_z_edit_value_changed(value: float) -> void:
	point_pos.z = value
	pos_changed.emit(point_pos)


func _on_load_dialogue_file_selected(path: String) -> void:
	var loader = Loader.from_path(path)
	if not loader.success():
		points_failed_to_load.emit()
		return
	
	var pts = loader.get_points();
	points_loaded.emit(pts)


func _on_save_dialogue_confirmed() -> void:
	print("confirmed")
