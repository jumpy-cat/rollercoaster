@tool
extends HBoxContainer
class_name FloatEdit

signal value_changed(value: float)

@export var _name: String
@export var value: float
var revert_to: String

@onready var label: Label = $Label
@onready var line_edit: LineEdit = $LineEdit


func _ready() -> void:
	label.text = _name
	line_edit.text = String.num(value)


func _process(_delta: float) -> void:
	# update label and value if in editor
	if Engine.is_editor_hint():
		label.text = _name
		line_edit.text = String.num(value)


func set_value(v: float) -> void:
	line_edit.text = String.num(v)
	revert_to = line_edit.text
	value = line_edit.text.to_float()


func _on_line_edit_text_submitted(_new_text: String) -> void:
	line_edit.release_focus()


func _on_line_edit_focus_exited() -> void:
	var s = line_edit.text
	if s.is_valid_float():
		value = s.to_float()
		revert_to = String.num(value)
		line_edit.text = revert_to
		value_changed.emit(value)
	else:
		line_edit.text = revert_to
	
