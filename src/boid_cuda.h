#ifndef BOID_CUDA_H
#define BOID_CUDA_H

typedef struct {
    float x;
    float y;
    float z;
} float3_simple;


#ifdef __cplusplus
extern "C" {
#endif

// C-style interface function
// Returns an error code (0 for success, non-zero for failure)
int calculate_boid_update_cuda_c_interface(
    const float3_simple* host_positions,         // Input
    const float3_simple* host_current_velocities, // Input
    float3_simple* host_new_velocities,          // Output
    int num_boids,
    float delta_time,
    float neighbor_distance,
    float separation_weight,
    float alignment_weight,
    float cohesion_weight,
    float max_speed,
    float min_speed, 
    float max_force
);

bool is_cuda_available_c_interface();


#ifdef __cplusplus
}
#endif

#endif // BOID_CUDA_H

