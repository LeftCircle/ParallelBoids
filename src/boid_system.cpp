#include "boid_system.h"
#include "boid_oop.h"
#include "boid_cuda.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/time.hpp>

#include <vector>

using namespace godot;

void BoidSystem::_bind_methods() {
	ClassDB::bind_method(D_METHOD("update_boids_cpu", "delta"), &BoidSystem::update_boids_cpu);
	ClassDB::bind_method(D_METHOD("update_boids_cuda", "delta"), &BoidSystem::update_boids_cuda);
	ClassDB::bind_method(D_METHOD("register_boid", "boid"), &BoidSystem::register_boid);
	ClassDB::bind_method(D_METHOD("unregister_boid", "boid"), &BoidSystem::unregister_boid);
	ClassDB::bind_method(D_METHOD("get_boids"), &BoidSystem::get_boids);
	ClassDB::bind_method(D_METHOD("set_bounds", "min", "max"), &BoidSystem::set_bounds);
	
}

BoidSystem::BoidSystem() {}
BoidSystem::~BoidSystem() {
	boid_oops.clear();
}

void BoidSystem::_ready() {
	boid_oops.clear();
}

void BoidSystem::register_boid(const Ref<BoidOOP> &boid) {
	boid_oops.push_back(boid);
}

void BoidSystem::unregister_boid(const Ref<BoidOOP> &boid) {
	if (boid.is_null()) {
		return;
	}

	for (int i = 0; i < boid_oops.size(); ++i) {
		Ref<BoidOOP> existing_boid_ref = boid_oops[i];
		if (existing_boid_ref == boid) {
			boid_oops.remove_at(i);
			break;
		}
	}
}

void BoidSystem::update_boids_cpu(double delta) {
	for (int i = 0; i < boid_oops.size(); ++i) {
		Ref<BoidOOP> current_boid_ref = boid_oops[i];
		if (current_boid_ref.is_valid()) {
			BoidOOP* current_boid = current_boid_ref.ptr();
			TypedArray<BoidOOP> neighbors = current_boid->find_neighbors(boid_oops);
			current_boid->update(delta, neighbors);
			Vector3 current_pos = current_boid->get_position();
			current_pos = wrap_position(current_pos);
			current_boid->set_position(current_pos);
		}
	}
}

void BoidSystem::update_boids_cuda(double delta) {
	int num_boids = boid_oops.size();
	if (num_boids == 0) {
		return;
	}

	float neighbor_dist = 5.0f;
	float separation_w = 1.5f;
	float alignment_w = 1.0f;
	float cohesion_w = 1.0f;
	float max_speed = 5.0f;
	float min_speed = 0.5f; 
	float max_force = 30.0f;

	Ref<BoidOOP> first_boid_ref = boid_oops[0];
	if (first_boid_ref.is_valid()) {
		BoidOOP* first_boid = first_boid_ref.ptr();
		neighbor_dist = first_boid->get_neighbor_distance();
		separation_w = first_boid->get_separation_weight();
		alignment_w = first_boid->get_alignment_weight();
		cohesion_w = first_boid->get_cohesion_weight();
		max_speed = first_boid->get_max_speed();
		min_speed = first_boid->get_min_speed();
		max_force = first_boid->get_max_force();
	}

	godot::Vector<godot::Vector3> new_velocities;
	try {
		new_velocities = _calculate_boid_update_cuda_internal(
			delta,
			neighbor_dist,
			separation_w,
			alignment_w,
			cohesion_w,
			max_speed,
			min_speed,
			max_force
		);
	} catch (const std::exception& e) {
		UtilityFunctions::printerr("CUDA boid update failed: ", e.what());
		return;
	}

	// --- Apply results ---
	if (new_velocities.size() != num_boids) {
		// Error should have been caught inside _calculate_boid_update_cuda_internal
		// or the size mismatch is unexpected.
		UtilityFunctions::printerr("CUDA returned incorrect number of velocities. Expected: ", num_boids, ", Got: ", new_velocities.size());
		return;
	}

	for (int i = 0; i < num_boids; ++i) {
		Ref<BoidOOP> current_boid_ref = boid_oops[i];
		if (current_boid_ref.is_valid()) {
			BoidOOP* current_boid = current_boid_ref.ptr();

			// Get the new velocity calculated by CUDA
			Vector3 new_vel = new_velocities[i];

			// Update the boid's velocity
			current_boid->set_velocity(new_vel);

			// Update the boid's position using the new velocity
			Vector3 current_pos = current_boid->get_position();
			Vector3 next_pos = current_pos + new_vel * (float)delta;
			next_pos = wrap_position(next_pos);
			current_boid->set_position(next_pos);
		}
	}
}

// --- Private Helper for CUDA Calculation using Member Vectors ---
godot::Vector<godot::Vector3> BoidSystem::_calculate_boid_update_cuda_internal(
	double delta_time,
	float neighbor_distance,
	float separation_weight,
	float alignment_weight,
	float cohesion_weight,
	float max_speed,
	float min_speed, // <-- Add min_speed parameter
	float max_force
) {
	int num_boids = boid_oops.size();
	godot::Vector<godot::Vector3> host_new_velocities;
	host_new_velocities.resize(num_boids);

	if (num_boids == 0) {
		return host_new_velocities;
	}

	if (cuda_positions.size() != num_boids) {
		cuda_positions.resize(num_boids);
		cuda_current_velocities.resize(num_boids);
		cuda_new_velocities.resize(num_boids);
	}

	for (int i = 0; i < num_boids; ++i) {
		Variant v = boid_oops[i];
		BoidOOP* boid_ptr = Object::cast_to<BoidOOP>(v.operator Object*());
		if (boid_ptr) {
			Vector3 pos = boid_ptr->get_position();
			Vector3 vel = boid_ptr->get_velocity();
			cuda_positions[i] = {pos.x, pos.y, pos.z};
			cuda_current_velocities[i] = {vel.x, vel.y, vel.z};
		} else {
			cuda_positions[i] = {0.0f, 0.0f, 0.0f};
			cuda_current_velocities[i] = {0.0f, 0.0f, 0.0f};
			UtilityFunctions::printerr("Non-BoidOOP object found in array at index ", i, " during CUDA update prep.");
		}
	}

	int cuda_result = calculate_boid_update_cuda_c_interface(
		cuda_positions.data(),
		cuda_current_velocities.data(),
		cuda_new_velocities.data(),
		num_boids,
		(float)delta_time,
		neighbor_distance,
		separation_weight,
		alignment_weight,
		cohesion_weight,
		max_speed,
		min_speed,
		max_force
	);

	if (cuda_result != 0) {
		UtilityFunctions::printerr("CUDA calculation failed with error code: ", cuda_result);
		// Return empty vector to signal failure to the caller
		return godot::Vector<godot::Vector3>();
	}

	// 5. Convert Results back to Godot Types
	for(int i = 0; i < num_boids; ++i) {
		host_new_velocities.set(i, Vector3(cuda_new_velocities[i].x, cuda_new_velocities[i].y, cuda_new_velocities[i].z));
	}

	return host_new_velocities;
}

Vector3 BoidSystem::wrap_position(Vector3 pos) const {
    Vector3 size = max_bound - min_bound;

    if (size.x <= 0.0f) {
        pos.x = min_bound.x;
    } else {
        if (pos.x < min_bound.x) {
            pos.x = max_bound.x - fmod(min_bound.x - pos.x, size.x);
        } else if (pos.x > max_bound.x) {
            pos.x = min_bound.x + fmod(pos.x - max_bound.x, size.x);
        }
         if (pos.x == max_bound.x) pos.x = min_bound.x;
    }

    if (size.y <= 0.0f) {
        pos.y = min_bound.y;
    } else {
        if (pos.y < min_bound.y) {
            pos.y = max_bound.y - fmod(min_bound.y - pos.y, size.y);
        } else if (pos.y > max_bound.y) {
            pos.y = min_bound.y + fmod(pos.y - max_bound.y, size.y);
        }
         if (pos.y == max_bound.y) pos.y = min_bound.y;
    }

    if (size.z <= 0.0f) {
        pos.z = min_bound.z;
    } else {
        if (pos.z < min_bound.z) {
            pos.z = max_bound.z - fmod(min_bound.z - pos.z, size.z);
        } else if (pos.z > max_bound.z) {
            pos.z = min_bound.z + fmod(pos.z - max_bound.z, size.z);
        }
         if (pos.z == max_bound.z) pos.z = min_bound.z;
    }

    return pos;
}

// --- C++ Wrapper for CUDA Availability Check ---
namespace godot {

bool is_cuda_available() {
	// Call the C interface function
	return is_cuda_available_c_interface();
}

} // namespace godot