#include "Cohesion.h"



Cohesion::Cohesion()
{
}


Cohesion::~Cohesion()
{
}

btVector3 Cohesion::CalculateMovement(Boid & boid, btAlignedObjectArray<btRigidBody*>& others, Flock & flock) const
{
	// If no others, return zero vector
	if (others.size() == 0)
		return btVector3(0,0,0);

	// Get average position between others
	btVector3 avgPosition = btVector3(0,0,0);
	int arrSize = others.size();
	for (int i = 0; i < arrSize; ++i)
	{
		avgPosition += others[i]->getWorldTransform().getOrigin(); 
	}
	avgPosition /= arrSize;
	
	// Return direction from boid position to avgOthersPosition
	// return btVector3(0,0,0); 
	return avgPosition - boid.getBody().getWorldTransform().getOrigin();
}
