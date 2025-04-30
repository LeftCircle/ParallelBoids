extends Resource
class_name SimData


@export var n_boids : int
@export var cpu_avg_time : float
@export var cuda_avg_time : float


func print_results() -> void:
	print("----------------------")
	print("N boids   = ", n_boids)
	print("CPU time  = ", cpu_avg_time)
	print("CUDA time = ", cuda_avg_time)
	print("------------------------")
