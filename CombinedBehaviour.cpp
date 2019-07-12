#include "CombinedBehaviour.h"



CombinedBehaviour::CombinedBehaviour(std::map<FlockBehaviour*, float> &behavioursToWeighting) : _behavioursToWeighting(&behavioursToWeighting)
{
}


CombinedBehaviour::~CombinedBehaviour()
{
}

// Calculates total movement for each of the behaviours, taking into account weighting
btVector3 CombinedBehaviour::CalculateMovement(Boid & boid, btAlignedObjectArray<btRigidBody*>& others, Flock & flock) const
{
	btVector3 movement = btVector3(0, 0, 0);

	// Iterate over behaviours
	for (auto it = _behavioursToWeighting->begin(); it != _behavioursToWeighting->end(); ++it)
	{
		float weighting = (*it).second;
		btVector3 partialMovement = btVector3(0, 0, 0);

		// Apply calculation and multiply by the associated weighting
		partialMovement = (*it).first->CalculateMovement(boid, others, flock) * weighting;

		if (partialMovement != btVector3(0, 0, 0))
		{
			// Make sure magnitude is not greater than weighting
			if (partialMovement.length2() > weighting * weighting)
			{
				partialMovement.normalize();
				partialMovement *= weighting;
			}

			movement += partialMovement;
		}
	}

	return movement;
}
