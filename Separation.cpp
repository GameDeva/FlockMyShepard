#include "Separation.h"



Separation::Separation()
{
}


Separation::~Separation()
{
}

btVector3 Separation::CalculateMovement(Boid & boid, btAlignedObjectArray<btRigidBody*>& others, Flock & flock) const
{
	// If no others, return zero vector
	if (others.size() == 0)
		return btVector3(0, 0, 0);

	// Get average position between others
	btVector3 avgPosition = btVector3(0, 0, 0);
	int separateFromNum = 0;
	int arrSize = others.size();
	for (int i = 0; i < arrSize; ++i)
	{
		// Check if distance between boid and other object is less than the separation radius
		if (btDistance2(boid.getBody().getWorldTransform().getOrigin(), others[i]->getWorldTransform().getOrigin()) < flock.getSquareSeparationRadius())
		{
			// If so, increment separateFromNum and add to running avg
			++separateFromNum;
			btVector3 ToAgent = (boid.getBody().getWorldTransform().getOrigin() - others[i]->getWorldTransform().getOrigin());
			avgPosition += ToAgent;// .normalized() / ToAgent.length();
		}
	}
	// If there were any to separate from, get the average
	if (separateFromNum > 0)
		avgPosition /= separateFromNum;

	return avgPosition;
}