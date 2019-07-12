#include "Alignment.h"



Alignment::Alignment()
{
}


Alignment::~Alignment()
{
}


btVector3 Alignment::CalculateMovement(Boid & boid, btAlignedObjectArray<btRigidBody*>& others, Flock & flock) const
{
	// If no others, keep same alignment
	if (others.size() == 0)
	{
		// Returns rigibody's forward vector
		return btVector3(0, 0, 0); // boid.GetLocalForward();
	}

	// Get average alignment, based on others' forward vectors
	btVector3 avgHeading = btVector3(0, 0, 0);
	btQuaternion avgRotation = btQuaternion(others[0]->getWorldTransform().getRotation());
	int arrSize = others.size();
	for (int i = 1; i < arrSize; ++i)
	{
		// Get world trasnform
		btTransform transform = others[i]->getWorldTransform();
		// Add each objects' forward vector
		avgHeading += (transform * WORLDFORWARD - transform.getOrigin()) - boid.GetLocalForward();
		avgRotation += others[i]->getWorldTransform().getRotation();
		avgRotation = avgRotation.slerp(others[i]->getWorldTransform().getRotation(), 0.5);

	}
	
	boid.getBody().getWorldTransform().setRotation(avgRotation.slerp(boid.getBody().getWorldTransform().getRotation(), 0.1));

	avgHeading /= arrSize;
	// avgHeading -= boid.GetLocalForward();


	return btVector3(0,0,0);
}