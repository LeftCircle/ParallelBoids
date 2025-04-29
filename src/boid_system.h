#pragma once

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/core/binder_common.hpp>
#include <godot_cpp/variant/typed_array.hpp> 


namespace godot {

class BoidOOP; // Forward declaration

class BoidSystem : public Node {
    GDCLASS(BoidSystem, Node);

protected:
    static void _bind_methods();

private:
	TypedArray<BoidOOP> boid_oops; 

public:
    BoidSystem();
    ~BoidSystem();

    void _ready() override;
    void _process(double delta) override;

    void register_boid(const Ref<BoidOOP> &boid); 
    void unregister_boid(const Ref<BoidOOP> &boid);

    void update_boids_oop(double delta);
	TypedArray<BoidOOP> get_boids() { return boid_oops; } // Getter for boids array

};
} // namespace godot