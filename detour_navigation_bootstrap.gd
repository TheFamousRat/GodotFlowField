extends Spatial
tool
onready var to_load_script = preload("res://addons/GodotFlowField/bin/detour_navigation.gdns")

func _ready():
	set_script(to_load_script)


