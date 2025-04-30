extends RefCounted
class_name ProfileTimer


var accumulated_times : int = 0
var iterations : int = 0
var start_time : int
var end_time : int

func start() -> void:
	start_time = Time.get_ticks_usec()

func stop() -> void:
	end_time = Time.get_ticks_usec()
	accumulated_times += end_time - start_time
	iterations += 1

func get_avg() -> float:
	var avg : float = float(accumulated_times) / float(iterations)
	accumulated_times = 0
	iterations = 0
	return avg
