#pragma once
#include "FlockBehaviour.h"
#include "Cohesion.h"
#include "Separation.h"
#include "Alignment.h"
#include <map>

class CombinedBehaviour : public FlockBehaviour
{
public:
	CombinedBehaviour(std::map<FlockBehaviour*, float> &behavioursToWeighting);
	~CombinedBehaviour();

	// Map of behaviours and their respective weightings
	std::map<FlockBehaviour*, float> *_behavioursToWeighting;

	// Takes the boid in concern, other bodies such as boids or obstactles as a list, and a given flock
	virtual btVector3 CalculateMovement(Boid &boid, btAlignedObjectArray<btRigidBody*> &others, Flock &flock) const;

};

