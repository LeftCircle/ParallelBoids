extends Node3D

enum SIMS{CPU, CUDA}

#@export var spawn_region : AABB
@export var steps_per_sim : int = 100
@export var boids_to_increase_by : int = 500
@export var boid_range : Vector2i = Vector2i(100, 1000)
@export var prof_cpu : bool = false
@export var to_profile : bool = false

var boid_count : int = 0
var boids_to_avg_time : Array[SimData] = []
var current_timestep : int = 0
var sim_to_run = SIMS.CPU

var profiler = ProfileTimer.new()
var data : SimData = SimData.new()
var spawn_region : AABB

func _ready() -> void:
	var mesh : Mesh = $MeshInstance3D.mesh
	spawn_region = mesh.get_aabb()
	$MeshInstance3D.queue_free()
	BoidSystem.set_bounds(spawn_region.position, spawn_region.end)
	_add_boids(boid_range.x)

func _physics_process(delta: float) -> void:
	if to_profile:
		_profile(delta)
	else:
		BoidSystem.update_boids_cuda(delta)

func _add_boids(n_boids : int = boids_to_increase_by) -> void:
	for i in range(n_boids):
		var new_boid : GD_Boid = load("res://scenes/Boid.tscn").instantiate()
		var boid_oop = BoidOOP.new()
		#boid_oop.separation_weight = 50
		new_boid.boid_oop = boid_oop
		new_boid.position.x = randf_range(spawn_region.position.x, spawn_region.end.x)
		new_boid.position.y = randf_range(spawn_region.position.y, spawn_region.end.y)
		new_boid.position.z = randf_range(spawn_region.position.z, spawn_region.end.z)

		boid_oop.velocity = new_boid.position.direction_to(Vector3.ZERO) * boid_oop.max_speed
		add_child(new_boid)
	boid_count += n_boids

func _profile(delta : float) -> void:
	if boid_count > boid_range.y:
		print("FINISHED")
		set_physics_process(false)
	if current_timestep >= steps_per_sim:
		if sim_to_run == SIMS.CPU:
			# Collect data and switch to cuda
			data.n_boids = boid_count
			data.cpu_avg_time = profiler.get_avg()
			sim_to_run = SIMS.CUDA
			pass
		elif sim_to_run == SIMS.CUDA:
			# Collect data, add biods, and swith to cpu
			data.cuda_avg_time = profiler.get_avg()
			data.print_results()
			boids_to_avg_time.append(data.duplicate(true))
			data = SimData.new()
			_add_boids()
			sim_to_run = SIMS.CPU
			pass

	if sim_to_run == SIMS.CPU and prof_cpu:
		profiler.start()
		BoidSystem.update_boids_cpu(delta)
		profiler.stop()
	else:
		profiler.start()
		BoidSystem.update_boids_cuda(delta)
		profiler.stop()
	current_timestep += 1
