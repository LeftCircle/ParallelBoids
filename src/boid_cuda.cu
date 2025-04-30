#include "boid_cuda.h" // Include the C interface header

#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <stdio.h> // For basic error printing
#include <vector> // Keep for temporary host storage if needed inside C interface

__device__ float3_simple operator+(const float3_simple& a, const float3_simple& b) {
	return {a.x + b.x, a.y + b.y, a.z + b.z};
}

__device__ float3_simple operator-(const float3_simple& a, const float3_simple& b) {
	return {a.x - b.x, a.y - b.y, a.z - b.z};
}

__device__ float3_simple operator*(const float3_simple& a, const float b) {
	return {a.x * b, a.y * b, a.z * b};
}

__device__ float3_simple operator/(const float3_simple& a, const float b) {
	if (b == 0.0f) return {0.0f, 0.0f, 0.0f};
	return {a.x / b, a.y / b, a.z / b};
}

__device__ float3_simple normalize(const float3_simple& v) {
	float length_sq = v.x * v.x + v.y * v.y + v.z * v.z;
	if (length_sq > 0) { // Avoid sqrt(0) and division by zero
		float length = sqrtf(length_sq);
		return {v.x / length, v.y / length, v.z / length};
	}
	return {0.0f, 0.0f, 0.0f};
}

__device__ float length_squared(const float3_simple& v) {
	return v.x * v.x + v.y * v.y + v.z * v.z;
}

__device__ float length(const float3_simple& v) {
	return sqrtf(length_squared(v));
}


__global__ void calculate_boid_forces_kernel(
	const float3_simple* current_positions, // Input
	const float3_simple* current_velocities, // Input
	float3_simple* new_velocities,          // Output
	int num_boids,
	float delta_time,
	float neighbor_distance_sq,
	float separation_weight,
	float alignment_weight,
	float cohesion_weight,
	float max_speed,
	float min_speed, // <-- Add min_speed parameter
	float max_force
) {
	int idx = blockIdx.x * blockDim.x + threadIdx.x;

	if (idx < num_boids) {
		float3_simple pos = current_positions[idx];
		float3_simple vel = current_velocities[idx];

		float3_simple separation_force = {0.0f, 0.0f, 0.0f};
		float3_simple alignment_sum = {0.0f, 0.0f, 0.0f};
		float3_simple cohesion_center = {0.0f, 0.0f, 0.0f}; // Center of mass for cohesion
		int neighbor_count = 0;

		// Loop through other boids
		for (int i = 0; i < num_boids; ++i) {
			if (idx == i) continue;

			float3_simple other_pos = current_positions[i];
			float3_simple other_vel = current_velocities[i];

			// Calculate distance squared
			float3_simple diff = pos - other_pos;
			float dist_sq = length_squared(diff);

			// Check if neighbor (use squared distance)
			if (dist_sq > 0 && dist_sq < neighbor_distance_sq) {
				neighbor_count++;

				// 1. Separation: Steer away from neighbors
				separation_force = separation_force + (diff / dist_sq); 
				// 2. Alignment: Steer towards average neighbor velocity
				alignment_sum = alignment_sum + other_vel;

				// 3. Cohesion: Steer towards average neighbor position (center of mass)
				cohesion_center = cohesion_center + other_pos;
			}
		}

		float3_simple total_force = {0.0f, 0.0f, 0.0f};

		if (neighbor_count > 0) {
			// Average separation force (optional, can be strong)
			separation_force = separation_force / neighbor_count;
			separation_force = separation_force - vel; // Steering force
			separation_force = separation_force * separation_weight;

			alignment_sum = alignment_sum / neighbor_count; // Average velocity
			alignment_sum = alignment_sum - vel; // Steering force
			alignment_sum = alignment_sum * alignment_weight;

			cohesion_center = cohesion_center / neighbor_count; // Center of mass
			float3_simple desired_cohesion = cohesion_center - pos; // Vector towards center
			desired_cohesion = desired_cohesion - vel; // Steering force
			desired_cohesion = desired_cohesion * cohesion_weight;

			// --- Combine Forces ---
			total_force = separation_force + alignment_sum + desired_cohesion;

			// --- Limit Force ---
			float force_mag_sq = length_squared(total_force);
			if (force_mag_sq > max_force * max_force) {
				total_force = normalize(total_force) * max_force;
			}
		}

		// --- Apply Force & Update Velocity ---
		// Acceleration = Force / Mass (assume mass = 1 for simplicity)
		float3_simple acceleration = total_force; // If mass is 1
		vel = vel + acceleration * delta_time;

		// --- Limit Speed (Max and Min) ---
		float speed_sq = length_squared(vel);
		if (speed_sq > max_speed * max_speed) {
			vel = normalize(vel) * max_speed;
		} else if (speed_sq < min_speed * min_speed) {
			// Only apply min speed if the boid is actually moving (speed_sq > epsilon)
			// to avoid giving stationary boids a random direction.
			// A very small epsilon prevents issues with floating point inaccuracies near zero.
			float epsilon_sq = 1e-9f; // Square of a small epsilon
			if (speed_sq > epsilon_sq) {
				vel = normalize(vel) * min_speed;
			} else {
				// If velocity is essentially zero, leave it zero.
				vel = {0.0f, 0.0f, 0.0f};
			}
		}

		// Write the final calculated velocity for this boid
		new_velocities[idx] = vel;
	}
}


// --- Helper function to check CUDA errors (standard C version) ---
static int HandleCudaError(cudaError_t err, const char *file, int line) {
	if (err != cudaSuccess) {
		fprintf(stderr, "CUDA Error: %s in %s at line %d\n", cudaGetErrorString(err), file, line);
		return 1; // Return non-zero on error
	}
	return 0; // Return 0 on success
}
#define CUDA_CHECK(err) do { if (HandleCudaError(err, __FILE__, __LINE__)) return 1; } while(0) // Return error code from calling function


// --- C-style interface function implementation ---
int calculate_boid_update_cuda_c_interface(
	const float3_simple* host_positions,
	const float3_simple* host_current_velocities,
	float3_simple* host_new_velocities, // Output parameter
	int num_boids,
	float delta_time,
	float neighbor_distance,
	float separation_weight,
	float alignment_weight,
	float cohesion_weight,
	float max_speed,
	float min_speed, // <-- Add min_speed parameter
	float max_force
) {
	if (num_boids == 0) {
		return 0; // No work to do, success
	}

	// --- 1. Host data is already prepared and passed as pointers ---

	// --- 2. Allocate GPU Memory ---
	float3_simple* device_positions = nullptr;
	float3_simple* device_current_velocities = nullptr;
	float3_simple* device_new_velocities = nullptr;
	size_t positions_size = num_boids * sizeof(float3_simple);
	size_t velocities_size = num_boids * sizeof(float3_simple);

	// Use CUDA_CHECK macro for error handling
	CUDA_CHECK(cudaMalloc(&device_positions, positions_size));
	CUDA_CHECK(cudaMalloc(&device_current_velocities, velocities_size));
	CUDA_CHECK(cudaMalloc(&device_new_velocities, velocities_size));

	// --- 3. Copy Data Host -> Device ---
	CUDA_CHECK(cudaMemcpy(device_positions, host_positions, positions_size, cudaMemcpyHostToDevice));
	CUDA_CHECK(cudaMemcpy(device_current_velocities, host_current_velocities, velocities_size, cudaMemcpyHostToDevice));

	// --- 4. Launch Kernel ---
	int threads_per_block = 256;
	int blocks_per_grid = (num_boids + threads_per_block - 1) / threads_per_block;
	float neighbor_distance_sq = neighbor_distance * neighbor_distance;

	calculate_boid_forces_kernel<<<blocks_per_grid, threads_per_block>>>(
		device_positions,
		device_current_velocities,
		device_new_velocities,
		num_boids,
		delta_time, // Already float
		neighbor_distance_sq,
		separation_weight,
		alignment_weight,
		cohesion_weight,
		max_speed,
		min_speed, // <-- Pass min_speed to kernel
		max_force
	);

	// Check for kernel launch errors
	CUDA_CHECK(cudaGetLastError());
	// Synchronize device to ensure kernel completion before copying back
	CUDA_CHECK(cudaDeviceSynchronize());


	// --- 5. Copy Data Device -> Host ---
	// Copy directly into the output pointer provided by the caller
	CUDA_CHECK(cudaMemcpy(host_new_velocities, device_new_velocities, velocities_size, cudaMemcpyDeviceToHost));

	// --- 6. Free GPU Memory ---
	cudaError_t free_err; // Check errors individually during cleanup
	free_err = cudaFree(device_positions);
	if (free_err != cudaSuccess) fprintf(stderr, "CUDA Error freeing device_positions: %s\n", cudaGetErrorString(free_err));
	free_err = cudaFree(device_current_velocities);
	if (free_err != cudaSuccess) fprintf(stderr, "CUDA Error freeing device_current_velocities: %s\n", cudaGetErrorString(free_err));
	free_err = cudaFree(device_new_velocities);
	if (free_err != cudaSuccess) fprintf(stderr, "CUDA Error freeing device_new_velocities: %s\n", cudaGetErrorString(free_err));

	// Even if freeing fails, we might have successfully computed the result.
	// The primary return value indicates success/failure of the computation itself.
	return 0; // Success
}


// --- Implementation for is_cuda_available_c_interface ---
bool is_cuda_available_c_interface() {
	int device_count = 0;
	cudaError_t err = cudaGetDeviceCount(&device_count);
	if (err != cudaSuccess) {
		// Don't print error here, let the caller decide based on the return value
		// fprintf(stderr, "CUDA Error getting device count: %s\n", cudaGetErrorString(err));
		return false;
	}
	if (device_count == 0) {
		return false;
	}
	// Could add more checks here (e.g., compute capability) if needed
	return true;
}