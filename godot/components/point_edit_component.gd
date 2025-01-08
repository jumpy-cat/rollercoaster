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
signal d1_changed(pos: Vector3)
signal d2_changed(pos: Vector3)
signal d3_changed(pos: Vector3)

signal should_show(deriv: int)

var editing = 0
var edited_pos: Vector3

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	x_edit.theme.set_color("font_color", "Label", Color.RED)
	x_edit.theme.set_color("font_color", "LineEdit", Color.RED)
	y_edit.theme.set_color("font_color", "Label", Color.GREEN)
	y_edit.theme.set_color("font_color", "LineEdit", Color.GREEN)
	z_edit.theme.set_color("font_color", "Label", Color.SKY_BLUE)
	z_edit.theme.set_color("font_color", "LineEdit", Color.SKY_BLUE)


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(_delta: float) -> void:
	pass


func request_points() -> void:
	load_file_dialog.use_native_dialog = true
	load_file_dialog.popup()


func set_point(p: CoasterPoint) -> void:
	match editing:
		0:
			x_edit.set_value(p.get_x())
			y_edit.set_value(p.get_y())
			z_edit.set_value(p.get_z())
		1: 
			x_edit.set_value(p.get_xp())
			y_edit.set_value(p.get_yp())
			z_edit.set_value(p.get_zp())
		2: 
			x_edit.set_value(p.get_xpp())
			y_edit.set_value(p.get_ypp())
			z_edit.set_value(p.get_zpp())
		3: 
			x_edit.set_value(p.get_xppp())
			y_edit.set_value(p.get_yppp())
			z_edit.set_value(p.get_zppp())
		_:
			push_error("Unknown editing mode: " + str(editing))
			
	edited_pos = Vector3(x_edit.value, y_edit.value, z_edit.value)


func _on_load_button_pressed() -> void:
	load_file_dialog.popup()


func emit_pos(pos: Vector3) -> void:
	match editing:
		0: pos_changed.emit(pos)
		1: d1_changed.emit(pos)
		2: d2_changed.emit(pos)
		3: d3_changed.emit(pos)
		_: push_error("Unknown editing mode: " + str(editing))


func _on_x_edit_value_changed(value: float) -> void:
	edited_pos.x = value
	emit_pos(edited_pos)


func _on_y_edit_value_changed(value: float) -> void:
	edited_pos.y = value
	emit_pos(edited_pos)


func _on_z_edit_value_changed(value: float) -> void:
	edited_pos.z = value
	emit_pos(edited_pos)


func _on_load_dialogue_file_selected(path: String) -> void:
	print("Loading points...")
	var loader = Loader.from_path(path)
	if not loader.success():
		print("Loading failed")
		points_failed_to_load.emit()
		return
	
	var pts = loader.get_points();
	print(pts)
	points_loaded.emit(pts)


func _on_save_dialogue_confirmed() -> void:
	print("confirmed")


func set_editing(i: int) -> void:
	assert(i >= 0 and i <= 3)
	$VBoxContainer/OptionButton.select(i)
	_on_option_button_item_selected(i)


func _on_option_button_item_selected(index: int) -> void:
	print("index: " + str(index))
	editing = index
	should_show.emit(index)
