#pragma once
#include "FlockBehaviour.h"

class SteeredCohesion : public FlockBehaviour
{
public:
	SteeredCohesion();
	~SteeredCohesion();

	// Takes the boid in concern, other bodies such as boids or obstactles as a list, and a given flock
	virtual btVector3 CalculateMovement(Boid &boid, btAlignedObjectArray<btRigidBody*> &others, Flock &flock) const;
};

