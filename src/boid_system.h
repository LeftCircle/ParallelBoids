#pragma once

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/core/binder_common.hpp>
#include <godot_cpp/variant/typed_array.hpp>
#include "boid_cuda.h"
#include <vector>

namespace godot {

class BoidOOP; // Forward declaration of BoidOOP class

class BoidSystem : public Node {
    GDCLASS(BoidSystem, Node);

protected:
    static void _bind_methods();

private:
	TypedArray<BoidOOP> boid_oops;
	Vector3 min_bound = Vector3(-100, -100, -100);
	Vector3 max_bound = Vector3(100, 100, 100);

    // Member vectors for CUDA data transfer to avoid re-allocation
    std::vector<float3_simple> cuda_positions;
    std::vector<float3_simple> cuda_current_velocities;
    std::vector<float3_simple> cuda_new_velocities;

    godot::Vector<godot::Vector3> _calculate_boid_update_cuda_internal(
        double delta_time,
        float neighbor_distance,
        float separation_weight,
        float alignment_weight,
        float cohesion_weight,
        float max_speed,
        float min_speed,
        float max_force
    );

	Vector3 wrap_position(Vector3 pos) const;

public:
    BoidSystem();
    ~BoidSystem();

    void _ready() override;

    void register_boid(const Ref<BoidOOP> &boid); 
    void unregister_boid(const Ref<BoidOOP> &boid);

	void update_boids_cpu(double delta);
    void update_boids_cuda(double delta);
	void set_bounds(const Vector3 &min, const Vector3 &max) { min_bound = min; max_bound = max; }
	TypedArray<BoidOOP> get_boids() { return boid_oops; }

};

// Declare the C++ wrapper functions (implementation in boid_system.cpp)
// These are the functions that Godot/C++ code will call.
godot::Vector<godot::Vector3> calculate_boid_update_cuda(
    const godot::TypedArray<godot::BoidOOP>& boids,
    double delta_time,
    float neighbor_distance,
    float separation_weight,
    float alignment_weight,
    float cohesion_weight,
    float max_speed,
    float max_force
);

bool is_cuda_available();

} // namespace godot