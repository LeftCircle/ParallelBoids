#include "boid_system.h"
#include "boid_oop.h" 
#include <godot_cpp/variant/utility_functions.hpp>
#include <algorithm> 

using namespace godot;

void BoidSystem::_bind_methods() {
    ClassDB::bind_method(D_METHOD("update_boids_oop", "delta"), &BoidSystem::update_boids_oop);
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

void BoidSystem::_process(double delta) {
    
}


void BoidSystem::register_boid(const Ref<BoidOOP> &boid) {
	bool found = false;
	int n_boids = boid_oops.size();
	// Check if the boid is already registered
	for (int i = 0; i < n_boids; ++i) {
		Ref<BoidOOP> existing_boid_ref = boid_oops[i];
		if (existing_boid_ref == boid) {
			found = true;
			break;
		}
	}
	if (!found) {
			boid_oops.push_back(boid);
	}
}

void BoidSystem::unregister_boid(const Ref<BoidOOP> &boid) {
	if (boid.is_null()) {
		return; // Cannot unregister null
	}

	// Find the Ref in the array and remove it
	for (int i = 0; i < boid_oops.size(); ++i) {
		Ref<BoidOOP> existing_boid_ref = boid_oops[i];
		if (existing_boid_ref == boid) {
			boid_oops.remove_at(i);
			break; // Assuming no duplicates, exit after finding
		}
	}
}

void BoidSystem::update_boids_oop(double delta) {


	for (int i = 0; i < boid_oops.size(); ++i) {
		Ref<BoidOOP> current_boid_ref = boid_oops[i];
		if (current_boid_ref.is_valid()) {
			BoidOOP* current_boid = current_boid_ref.ptr();

			TypedArray<BoidOOP> neighbors = current_boid->find_neighbors(boid_oops);
			current_boid->update(delta, neighbors);
		}
	}
}