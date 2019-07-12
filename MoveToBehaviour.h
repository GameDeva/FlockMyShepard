#pragma once
#include "FlockBehaviour.h"

class MoveToBehaviour : public FlockBehaviour
{
public:
	MoveToBehaviour(btVector3* position);
	~MoveToBehaviour();

	// Takes the boid in concern, other bodies such as boids or obstactles as a list, and a given flock
	virtual btVector3 CalculateMovement(Boid &boid, btAlignedObjectArray<btRigidBody*> &others, Flock &flock) const;

private:
	btVector3* _position;

};

