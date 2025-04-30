#pragma once

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/core/binder_common.hpp>
#include <godot_cpp/variant/typed_array.hpp> 
#include "boid_cuda.h" // Include CUDA interface

namespace godot {

class BoidOOP; // Forward declaration

class BoidSystem : public Node {
    GDCLASS(BoidSystem, Node);

protected:
    static void _bind_methods();

private:
	TypedArray<BoidOOP> boid_oops; 

    // Add the CPU processing function declaration
    void _process_cpu(double delta);

public:
    BoidSystem();
    ~BoidSystem();

    void _ready() override;
    //void _process(double delta) override;

    void register_boid(const Ref<BoidOOP> &boid); 
    void unregister_boid(const Ref<BoidOOP> &boid);

    void update_boids_cuda(double delta); // Add new CUDA update function
	TypedArray<BoidOOP> get_boids() { return boid_oops; } // Getter for boids array

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