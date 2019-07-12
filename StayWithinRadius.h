#pragma once
#include "FlockBehaviour.h"

class StayWithinRadius : public FlockBehaviour
{
public:
	btVector3 _centre;
	btScalar _radius;

	StayWithinRadius(btVector3 centre, btScalar radius);
	~StayWithinRadius();

	// Takes the boid in concern, other bodies such as boids or obstactles as a list, and a given flock
	virtual btVector3 CalculateMovement(Boid &boid, btAlignedObjectArray<btRigidBody*> &others, Flock &flock) const;
};

