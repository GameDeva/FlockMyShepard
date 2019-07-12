#pragma once
#include "FlockBehaviour.h"

class Alignment : public FlockBehaviour
{
public:
	Alignment();
	~Alignment();

	// Takes the boid in concern, other bodies such as boids or obstactles as a list, and a given flock
	virtual btVector3 CalculateMovement(Boid &boid, btAlignedObjectArray<btRigidBody*> &others, Flock &flock) const;

};

