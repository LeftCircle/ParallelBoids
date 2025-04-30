#include "boid_system.h"
#include "boid_oop.h"
#include "boid_cuda.h" // Include the C interface header

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/time.hpp>

#include <vector> // For std::vector

using namespace godot;

void BoidSystem::_bind_methods() {
	ClassDB::bind_method(D_METHOD("update_boids_cpu", "delta"), &BoidSystem::update_boids_cpu);
	ClassDB::bind_method(D_METHOD("update_boids_cuda", "delta"), &BoidSystem::update_boids_cuda);
	ClassDB::bind_method(D_METHOD("register_boid", "boid"), &BoidSystem::register_boid);
	ClassDB::bind_method(D_METHOD("unregister_boid", "boid"), &BoidSystem::unregister_boid);
	ClassDB::bind_method(D_METHOD("get_boids"), &BoidSystem::get_boids);
	
}

BoidSystem::BoidSystem() {}
BoidSystem::~BoidSystem() {
	boid_oops.clear();
}

void BoidSystem::_ready() {
	boid_oops.clear();
}

void BoidSystem::register_boid(const Ref<BoidOOP> &boid) {
	boid_oops.push_back(boid); // Add the boid to the array
	// No need to resize CUDA vectors here, resize happens in the update function
}

void BoidSystem::unregister_boid(const Ref<BoidOOP> &boid) {
	if (boid.is_null()) {
		return; // Cannot unregister null
	}

	for (int i = 0; i < boid_oops.size(); ++i) {
		Ref<BoidOOP> existing_boid_ref = boid_oops[i];
		if (existing_boid_ref == boid) {
			boid_oops.remove_at(i);
			// No need to resize CUDA vectors here, resize happens in the update function
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
		}
	}
}

void BoidSystem::update_boids_cuda(double delta) {
	int num_boids = boid_oops.size();
	if (num_boids == 0) {
		return;
	}

	// --- Get Parameters (Example: Use first boid's parameters) ---
	float neighbor_dist = 5.0f;
	float separation_w = 1.5f;
	float alignment_w = 1.0f;
	float cohesion_w = 1.0f;
	float max_speed = 5.0f;
	float min_speed = 0.5f; 
	float max_force = 1000.0f; // Example max force

	Ref<BoidOOP> first_boid_ref = boid_oops[0];
	if (first_boid_ref.is_valid()) {
		BoidOOP* first_boid = first_boid_ref.ptr();
		neighbor_dist = first_boid->get_neighbor_distance();
		separation_w = first_boid->get_separation_weight();
		alignment_w = first_boid->get_alignment_weight();
		cohesion_w = first_boid->get_cohesion_weight();
		max_speed = first_boid->get_max_speed();
		min_speed = first_boid->get_min_speed(); // <-- Get min_speed from boid
		// max_force = first_boid->get_max_force(); // Assuming max_force is also a property
	}

	// --- Call internal CUDA function --- 
	godot::Vector<godot::Vector3> new_velocities;
	try {
		new_velocities = _calculate_boid_update_cuda_internal(
			delta,
			neighbor_dist,
			separation_w,
			alignment_w,
			cohesion_w,
			max_speed,
			min_speed, // <-- Pass min_speed
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
			current_boid->set_position(current_pos + new_vel * (float)delta);
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
	godot::Vector<godot::Vector3> host_new_velocities; // Godot Vector for return
	host_new_velocities.resize(num_boids);

	if (num_boids == 0) {
		return host_new_velocities;
	}

	// 1. Ensure member vectors have the correct size
	if (cuda_positions.size() != num_boids) {
		cuda_positions.resize(num_boids);
		cuda_current_velocities.resize(num_boids);
		cuda_new_velocities.resize(num_boids);
	}

	// 2. Prepare Host Data (Populate member vectors)
	for (int i = 0; i < num_boids; ++i) {
		Variant v = boid_oops[i];
		BoidOOP* boid_ptr = Object::cast_to<BoidOOP>(v.operator Object*());
		if (boid_ptr) { // Check if cast is successful
			Vector3 pos = boid_ptr->get_position();
			Vector3 vel = boid_ptr->get_velocity();
			cuda_positions[i] = {pos.x, pos.y, pos.z};
			cuda_current_velocities[i] = {vel.x, vel.y, vel.z};
		} else {
			// Handle error or default values if a non-boid object is in the array
			cuda_positions[i] = {0.0f, 0.0f, 0.0f};
			cuda_current_velocities[i] = {0.0f, 0.0f, 0.0f};
			UtilityFunctions::printerr("Non-BoidOOP object found in array at index ", i, " during CUDA update prep.");
		}
	}

	// 3. Call the C Interface Function using member vector data
	int cuda_result = calculate_boid_update_cuda_c_interface(
		cuda_positions.data(),
		cuda_current_velocities.data(),
		cuda_new_velocities.data(), // Pass pointer to output buffer
		num_boids,
		(float)delta_time,
		neighbor_distance,
		separation_weight,
		alignment_weight,
		cohesion_weight,
		max_speed,
		min_speed, // <-- Pass min_speed to C interface
		max_force
	);

	// 4. Handle Potential Errors from CUDA call
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

// --- CPU Boid Logic ---
void BoidSystem::_process_cpu(double delta) {
	TypedArray<BoidOOP> current_boids = boid_oops; // Work on a copy? Or directly? Be careful if modifying array during iteration.
	//Vector<Vector3> forces;
	//forces.resize(current_boids.size()); // Pre-allocate forces vector
	TypedArray<Vector3> forces;
	forces.resize(current_boids.size()); // Pre-allocate forces vector

	for (int i = 0; i < current_boids.size(); ++i) {
		Variant v_i = current_boids[i];
		BoidOOP *boid_i = Object::cast_to<BoidOOP>(v_i.operator Object*());
		if (!boid_i) continue;

		Vector3 separation_force;
		Vector3 alignment_sum;
		Vector3 cohesion_center;
		int neighbor_count = 0;

		for (int j = 0; j < current_boids.size(); ++j) {
			if (i == j) continue;

			Variant v_j = current_boids[j];
			BoidOOP *boid_j = Object::cast_to<BoidOOP>(v_j.operator Object*());
			if (!boid_j) continue;

			float dist_sq = boid_i->get_position().distance_squared_to(boid_j->get_position());

			if (dist_sq > 0 && dist_sq < 5.0f * 5.0f) {
				neighbor_count++;
				// Separation
				Vector3 diff = boid_i->get_position() - boid_j->get_position();
				// Normalize and weight by 1/distance (stronger when closer)
				separation_force += diff.normalized() / sqrt(dist_sq); // Using sqrt here for 1/dist weighting

				// Alignment
				alignment_sum += boid_j->get_velocity();

				// Cohesion
				cohesion_center += boid_j->get_position();
			}
		}

		Vector3 total_force;
		if (neighbor_count > 0) {
			// Finalize Separation
			separation_force = (separation_force / neighbor_count).normalized() * 5.0f;
			separation_force = (separation_force - boid_i->get_velocity()) * 1.5f; // Steering force

			// Finalize Alignment
			alignment_sum = (alignment_sum / neighbor_count).normalized() * 5.0f;
			alignment_sum = (alignment_sum - boid_i->get_velocity()) * 1.0f; // Steering force

			// Finalize Cohesion
			cohesion_center /= neighbor_count;
			Vector3 desired_cohesion = (cohesion_center - boid_i->get_position()).normalized() * 5.0f;
			desired_cohesion = (desired_cohesion - boid_i->get_velocity()) * 1.0f; // Steering force

			total_force = separation_force + alignment_sum + desired_cohesion;

			// Limit Force
			if (total_force.length_squared() > 10.0f * 10.0f) {
				total_force = total_force.normalized() * 10.0f;
			}
		}
		forces[i] = total_force; // Store calculated force
	}

	// Apply forces and update positions
	for (int i = 0; i < current_boids.size(); ++i) {
		Variant v = current_boids[i];
		BoidOOP *boid = Object::cast_to<BoidOOP>(v.operator Object*());
		if (boid) {
			Vector3 velocity = boid->get_velocity();
			// Acceleration = Force / Mass (assume mass = 1)
			Vector3 acceleration = forces[i];
			velocity += acceleration * delta;

			// Limit Speed
			if (velocity.length_squared() > 5.0f * 5.0f) {
				velocity = velocity.normalized() * 5.0f;
			}

			boid->set_velocity(velocity);
			boid->set_position(boid->get_position() + velocity * delta);
		}
	}
}

// --- C++ Wrapper for CUDA Availability Check ---
namespace godot {

bool is_cuda_available() {
	// Call the C interface function
	return is_cuda_available_c_interface();
}

} // namespace godot