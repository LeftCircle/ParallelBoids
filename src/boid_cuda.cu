#include "boid_cuda.h"

#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <stdio.h> 
#include <vector> 

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
	if (length_sq > 0) {
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
	float min_speed,
	float max_force
) {
	int idx = blockIdx.x * blockDim.x + threadIdx.x;

	if (idx < num_boids) {
		float3_simple pos = current_positions[idx];
		float3_simple vel = current_velocities[idx];

		float3_simple separation_force = {0.0f, 0.0f, 0.0f};
		float3_simple alignment_sum = {0.0f, 0.0f, 0.0f};
		float3_simple cohesion_center = {0.0f, 0.0f, 0.0f};
		int neighbor_count = 0;

		// Loop through other boids
		for (int i = 0; i < num_boids; ++i) {
			if (idx == i) continue;

			float3_simple other_pos = current_positions[i];
			float3_simple other_vel = current_velocities[i];

			float3_simple diff = pos - other_pos;
			float dist_sq = length_squared(diff);

			if (dist_sq > 0 && dist_sq < neighbor_distance_sq) {
				neighbor_count++;

				separation_force = separation_force + (diff / dist_sq); 
				alignment_sum = alignment_sum + other_vel;
				cohesion_center = cohesion_center + other_pos;
			}
		}

		float3_simple total_force = {0.0f, 0.0f, 0.0f};

		if (neighbor_count > 0) {
			// Average separation force (optional, can be strong)
			separation_force = separation_force / neighbor_count;
			separation_force = separation_force - vel;
			separation_force = separation_force * separation_weight;

			alignment_sum = alignment_sum / neighbor_count; 
			alignment_sum = alignment_sum - vel;
			alignment_sum = alignment_sum * alignment_weight;

			cohesion_center = cohesion_center / neighbor_count; 
			float3_simple desired_cohesion = cohesion_center - pos; 
			desired_cohesion = desired_cohesion - vel;
			desired_cohesion = desired_cohesion * cohesion_weight;

			total_force = separation_force + alignment_sum + desired_cohesion;

			float force_mag_sq = length_squared(total_force);
			if (force_mag_sq > max_force * max_force) {
				total_force = normalize(total_force) * max_force;
			}
		}

		// --- Apply Force & Update Velocity ---
		float3_simple acceleration = total_force; // If mass is 1
		vel = vel + acceleration * delta_time;

		float speed_sq = length_squared(vel);
		if (speed_sq > max_speed * max_speed) {
			vel = normalize(vel) * max_speed;
		} else if (speed_sq < min_speed * min_speed) {
			vel = normalize(vel) * min_speed;
		}
		new_velocities[idx] = vel;
	}
}


// --- Helper function to check CUDA errors (standard C version) ---
static int HandleCudaError(cudaError_t err, const char *file, int line) {
	if (err != cudaSuccess) {
		fprintf(stderr, "CUDA Error: %s in %s at line %d\n", cudaGetErrorString(err), file, line);
		return 1;
	}
	return 0;
}
#define CUDA_CHECK(err) do { if (HandleCudaError(err, __FILE__, __LINE__)) return 1; } while(0)


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
	float min_speed,
	float max_force
) {
	if (num_boids == 0) {
		return 0; // No work to do, success
	}
	// Allocate GPU Memory
	float3_simple* device_positions = nullptr;
	float3_simple* device_current_velocities = nullptr;
	float3_simple* device_new_velocities = nullptr;
	size_t positions_size = num_boids * sizeof(float3_simple);
	size_t velocities_size = num_boids * sizeof(float3_simple);

	// Use CUDA_CHECK macro for error handling
	CUDA_CHECK(cudaMalloc(&device_positions, positions_size));
	CUDA_CHECK(cudaMalloc(&device_current_velocities, velocities_size));
	CUDA_CHECK(cudaMalloc(&device_new_velocities, velocities_size));

	// Copy Data Host
	CUDA_CHECK(cudaMemcpy(device_positions, host_positions, positions_size, cudaMemcpyHostToDevice));
	CUDA_CHECK(cudaMemcpy(device_current_velocities, host_current_velocities, velocities_size, cudaMemcpyHostToDevice));

	// Launch Kernel
	int threads_per_block = 256;
	int blocks_per_grid = (num_boids + threads_per_block - 1) / threads_per_block;
	float neighbor_distance_sq = neighbor_distance * neighbor_distance;

	calculate_boid_forces_kernel<<<blocks_per_grid, threads_per_block>>>(
		device_positions,
		device_current_velocities,
		device_new_velocities,
		num_boids,
		delta_time,
		neighbor_distance_sq,
		separation_weight,
		alignment_weight,
		cohesion_weight,
		max_speed,
		min_speed,
		max_force
	);

	CUDA_CHECK(cudaGetLastError());
	CUDA_CHECK(cudaDeviceSynchronize());

	// Copy directly into the output pointer provided by the caller
	CUDA_CHECK(cudaMemcpy(host_new_velocities, device_new_velocities, velocities_size, cudaMemcpyDeviceToHost));

	//  Free GPU Memory
	cudaError_t free_err; // Check errors individually during cleanup
	free_err = cudaFree(device_positions);
	if (free_err != cudaSuccess) fprintf(stderr, "CUDA Error freeing device_positions: %s\n", cudaGetErrorString(free_err));
	free_err = cudaFree(device_current_velocities);
	if (free_err != cudaSuccess) fprintf(stderr, "CUDA Error freeing device_current_velocities: %s\n", cudaGetErrorString(free_err));
	free_err = cudaFree(device_new_velocities);
	if (free_err != cudaSuccess) fprintf(stderr, "CUDA Error freeing device_new_velocities: %s\n", cudaGetErrorString(free_err));
	return 0;
}


// --- Implementation for is_cuda_available_c_interface ---
bool is_cuda_available_c_interface() {
	int device_count = 0;
	cudaError_t err = cudaGetDeviceCount(&device_count);
	if (err != cudaSuccess) {
		return false;
	}
	if (device_count == 0) {
		return false;
	}
	return true;
}