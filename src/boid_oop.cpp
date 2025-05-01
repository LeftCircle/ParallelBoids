#include "boid_oop.h"

using namespace godot;

void BoidOOP::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_position", "position"), &BoidOOP::set_position);
	ClassDB::bind_method(D_METHOD("get_position"), &BoidOOP::get_position);
	ClassDB::bind_method(D_METHOD("set_velocity", "velocity"), &BoidOOP::set_velocity);
	ClassDB::bind_method(D_METHOD("get_velocity"), &BoidOOP::get_velocity);
	ClassDB::bind_method(D_METHOD("set_max_speed", "max_speed"), &BoidOOP::set_max_speed);
	ClassDB::bind_method(D_METHOD("get_max_speed"), &BoidOOP::get_max_speed);
	ClassDB::bind_method(D_METHOD("set_min_speed", "min_speed"), &BoidOOP::set_min_speed);
	ClassDB::bind_method(D_METHOD("get_min_speed"), &BoidOOP::get_min_speed);
	ClassDB::bind_method(D_METHOD("set_neighbor_distance", "distance"), &BoidOOP::set_neighbor_distance);
	ClassDB::bind_method(D_METHOD("get_neighbor_distance"), &BoidOOP::get_neighbor_distance);
	ClassDB::bind_method(D_METHOD("set_separation_weight", "weight"), &BoidOOP::set_separation_weight);
	ClassDB::bind_method(D_METHOD("get_separation_weight"), &BoidOOP::get_separation_weight);
	ClassDB::bind_method(D_METHOD("set_alignment_weight", "weight"), &BoidOOP::set_alignment_weight);
	ClassDB::bind_method(D_METHOD("get_alignment_weight"), &BoidOOP::get_alignment_weight);
	ClassDB::bind_method(D_METHOD("set_cohesion_weight", "weight"), &BoidOOP::set_cohesion_weight);
	ClassDB::bind_method(D_METHOD("get_cohesion_weight"), &BoidOOP::get_cohesion_weight);
	ClassDB::bind_method(D_METHOD("set_max_force", "max_force"), &BoidOOP::set_max_force);
	ClassDB::bind_method(D_METHOD("get_max_force"), &BoidOOP::get_max_force);
	ClassDB::bind_method(D_METHOD("update", "delta", "neighbors"), &BoidOOP::update);
	ClassDB::bind_method(D_METHOD("find_neighbors", "boids"), &BoidOOP::find_neighbors);
	

	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "position"), "set_position", "get_position");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "velocity"), "set_velocity", "get_velocity");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_speed"), "set_max_speed", "get_max_speed");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "min_speed"), "set_min_speed", "get_min_speed");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "neighbor_distance"), "set_neighbor_distance", "get_neighbor_distance");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "separation_weight"), "set_separation_weight", "get_separation_weight");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "alignment_weight"), "set_alignment_weight", "get_alignment_weight");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "cohesion_weight"), "set_cohesion_weight", "get_cohesion_weight");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_force"), "set_max_force", "get_max_force");
}

BoidOOP::BoidOOP() {
	position = Vector3(0, 0, 0);
    velocity = Vector3(UtilityFunctions::randf() * 2.0 - 1.0, UtilityFunctions::randf() * 2.0 - 1.0, UtilityFunctions::randf() * 2.0 - 1.0).normalized() * ((max_speed + min_speed) / 2.0);
}
BoidOOP::~BoidOOP() {}


TypedArray<BoidOOP> BoidOOP::find_neighbors(const TypedArray<BoidOOP>& boids) const {
	TypedArray<BoidOOP> neighbors;
	int n_boids = boids.size();
	for (int i = 0; i < n_boids; ++i) {
		BoidOOP* other = Object::cast_to<BoidOOP>(boids[i].operator Object*());
		if (!other || other == this) continue;
		float dist = position.distance_to(other->get_position());
		if (dist < neighbor_distance) {
			neighbors.push_back(boids[i]);
		}
	}
	return neighbors;
}

void BoidOOP::update(double delta, const TypedArray<BoidOOP>& neighbors){

	Vector3 separation_force = Vector3(0, 0, 0);
	Vector3 alignment_sum = Vector3(0, 0, 0);
	Vector3 cohesion_center = Vector3(0, 0, 0);;
	int n_neighbors = neighbors.size();

	for (int i = 0; i < n_neighbors; ++i) {
		Variant v = neighbors[i];
		BoidOOP* other = Object::cast_to<BoidOOP>(v.operator Object*());

		Vector3 diff = position - other->position;
		float dist_sq = diff.length_squared();
		
		separation_force = separation_force + (diff / dist_sq); 
		alignment_sum = alignment_sum + other->velocity;
		cohesion_center = cohesion_center + other->position;
	}

	Vector3 total_force = Vector3(0, 0, 0);

	if (n_neighbors > 0) {
		separation_force = separation_force / n_neighbors;
		separation_force = separation_force - velocity;
		separation_force = separation_force * separation_weight;

		alignment_sum = alignment_sum / n_neighbors;
		alignment_sum = alignment_sum - velocity;
		alignment_sum = alignment_sum * alignment_weight;

		cohesion_center = cohesion_center / n_neighbors;
		Vector3 desired_cohesion = cohesion_center - position;
		desired_cohesion = desired_cohesion - velocity;
		desired_cohesion = desired_cohesion * cohesion_weight;

		total_force = separation_force + alignment_sum + desired_cohesion;
		
		float force_mag_sq = total_force.length_squared();
		if (force_mag_sq > max_force * max_force) {
			total_force = total_force.normalized() * max_force;
		}
	}

	// Acceleration = Force / Mass (assume mass = 1 for simplicity)
	Vector3 acceleration = total_force; // If mass is 1
	velocity += acceleration * delta;

	float speed_sq = velocity.length_squared();
	if (speed_sq > max_speed * max_speed) {
		velocity = velocity.normalized() * max_speed;
	} else if (speed_sq < min_speed * min_speed) {
		velocity = velocity.normalized() * min_speed;
	}
}

void BoidOOP::set_position(const Vector3 &p_position) {
	position = p_position;
}

Vector3 BoidOOP::get_position() const {
	return position;
}

void BoidOOP::set_velocity(const Vector3 &p_velocity) {
	velocity = p_velocity;
}

Vector3 BoidOOP::get_velocity() const {
	return velocity;
}

void BoidOOP::set_max_speed(float p_max_speed) {
	max_speed = p_max_speed;
}

float BoidOOP::get_max_speed() const {
	return max_speed;
}

void BoidOOP::set_min_speed(float p_speed) {
	min_speed = p_speed;
}

float BoidOOP::get_min_speed() const {
	return min_speed;
}

void BoidOOP::set_neighbor_distance(float p_distance) {
	neighbor_distance = p_distance;
}

float BoidOOP::get_neighbor_distance() const {
	return neighbor_distance;
}

void BoidOOP::set_separation_weight(float p_weight) {
	separation_weight = p_weight;
}

float BoidOOP::get_separation_weight() const {
	return separation_weight;
}

void BoidOOP::set_alignment_weight(float p_weight) {
	alignment_weight = p_weight;
}

float BoidOOP::get_alignment_weight() const {
	return alignment_weight;
}

void BoidOOP::set_cohesion_weight(float p_weight) {
	cohesion_weight = p_weight;
}

float BoidOOP::get_cohesion_weight() const {
	return cohesion_weight;
}

