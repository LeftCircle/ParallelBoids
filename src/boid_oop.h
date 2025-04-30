#pragma once

#include <godot_cpp/classes/resource.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/templates/vector.hpp> 
#include <godot_cpp/variant/typed_array.hpp>


#include <random>
#include <vector>



namespace godot {


class BoidOOP : public Resource {
	GDCLASS(BoidOOP, Resource);

protected:
	static void _bind_methods();

private:
	Vector3 position;
	Vector3 velocity;
	float max_speed = 5.0f;
	float min_speed = 0.5f; // <-- Add min_speed property
	float neighbor_distance = 5.0f;
	float separation_weight = 3.0f;
	float alignment_weight = 1.0f;
	float cohesion_weight = 2.0f;
	float max_force = 10.0f;

public:
	BoidOOP();
	~BoidOOP();

	void set_position(const Vector3 &p_position);
	Vector3 get_position() const;
	void set_velocity(const Vector3 &p_velocity);
	Vector3 get_velocity() const;
	void set_max_speed(float p_max_speed);
	float get_max_speed() const;
	void set_min_speed(float p_speed); // <-- Add setter
	float get_min_speed() const;      // <-- Add getter
	void set_neighbor_distance(float p_distance);
	float get_neighbor_distance() const;
	void set_separation_weight(float p_weight);
	float get_separation_weight() const;
	void set_alignment_weight(float p_weight);
	float get_alignment_weight() const;
	void set_cohesion_weight(float p_weight);
	float get_cohesion_weight() const;
	void set_max_force(float p_force) { max_force = p_force; };
	float get_max_force()  const { return max_force; };


	TypedArray<BoidOOP> find_neighbors(const TypedArray<BoidOOP> &boids) const;
	void update(double delta, const TypedArray<BoidOOP> &neighbors);
};
} // namespace godot