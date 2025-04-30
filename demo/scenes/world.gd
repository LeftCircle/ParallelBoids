extends Node3D

@export var spawn_region : AABB
@export var steps_per_sim : int = 100
@export var boid_range : Vector2i = Vector2i(100, 1000)

var boids_to_avg_time : Array[SimData] = []
var n_timesteps : int = 0

func _ready() -> void:
	for i in range(100):
		var new_boid : GD_Boid = load("res://scenes/Boid.tscn").instantiate()
		var boid_oop = BoidOOP.new()
		#boid_oop.separation_weight = 50
		new_boid.boid_oop = boid_oop
		new_boid.position.x = randf_range(spawn_region.position.x, spawn_region.end.x)
		new_boid.position.y = randf_range(spawn_region.position.y, spawn_region.end.y)
		new_boid.position.z = randf_range(spawn_region.position.z, spawn_region.end.z)

		boid_oop.velocity = new_boid.position.direction_to(Vector3.ZERO) * boid_oop.max_speed
		add_child(new_boid)

func _physics_process(delta: float) -> void:
	BoidSystem.update_boids_cpu(delta)
	#BoidSystem.update_boids_cuda(delta)
