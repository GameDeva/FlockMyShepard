#include "SteeredCohesion.h"



SteeredCohesion::SteeredCohesion()
{
}


SteeredCohesion::~SteeredCohesion()
{
}

// Not used, attempt at having smoother motion
btVector3 SteeredCohesion::CalculateMovement(Boid & boid, btAlignedObjectArray<btRigidBody*>& others, Flock & flock) const
{
	// If no others, return zero vector
	if (others.size() == 0)
		return btVector3(0, 0, 0);

	// Get average position between others
	btVector3 avgPosition = btVector3(0, 0, 0);
	int arrSize = others.size();
	for (int i = 0; i < arrSize; ++i)
	{
		avgPosition += others[i]->getWorldTransform().getOrigin();
	}
	avgPosition /= arrSize;

	// Return direction from boid position to avgOthersPosition
	// return btVector3(0,0,0); 
	btVector3 out =  avgPosition - boid.getBody().getWorldTransform().getOrigin();
	out = lerp(boid.GetLocalForward(), out, 0.9f);
	return out;
}
