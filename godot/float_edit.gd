@tool
extends HBoxContainer

signal value_changed(value: float)

@export var _name: String
@export var value: float
var revert_to: String

@onready var label: Label = $Label
@onready var line_edit: LineEdit = $LineEdit


func _ready() -> void:
	label.text = _name
	line_edit.text = String.num(value)


func _on_line_edit_text_submitted(_new_text: String) -> void:
	line_edit.release_focus()


func _on_line_edit_focus_exited() -> void:
	var s = line_edit.text
	if s.is_valid_float():
		value = s.to_float()
		revert_to = s
		value_changed.emit(value)
	else:
		line_edit.text = revert_to
	
