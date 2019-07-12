#pragma once
#include "Boid.h"
#include "Flock.h"
#include "btBulletDynamicsCommon.h"
#include <vector>

// Pure Virtual Class
class FlockBehaviour
{
public:
	FlockBehaviour();
	~FlockBehaviour();

	// Takes the boid in concern, other bodies such as boids or obstactles as a list, and a given flock
	virtual btVector3 CalculateMovement(Boid &boid, btAlignedObjectArray<btRigidBody*> &others, Flock &flock) const = 0;

};

