extends Node3D
class_name GD_Boid

@export var boid_oop : BoidOOP = BoidOOP.new()
@onready var mesh : MeshInstance3D = $MeshInstance3D

func _ready() -> void:
	boid_oop.position = global_position
	BoidSystem.register_boid(boid_oop)


func _physics_process(_delta: float) -> void:
	global_position = boid_oop.position
	look_at(global_position + boid_oop.velocity)

func _exit_tree() -> void:
	BoidSystem.unregister_boid(boid_oop)
