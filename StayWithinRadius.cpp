#include "StayWithinRadius.h"



StayWithinRadius::StayWithinRadius(btVector3 centre, btScalar radius) : _centre(centre), _radius(radius)
{
}


StayWithinRadius::~StayWithinRadius()
{
}

// Ensures boids stay within volume of given radius
btVector3 StayWithinRadius::CalculateMovement(Boid & boid, btAlignedObjectArray<btRigidBody*>& others, Flock & flock) const
{
	// Vector from boid position to centre
	btVector3 centreOffset = _centre - boid.getBody().getCenterOfMassPosition();
	// TODO: Change this to length2 [square magnitude]
	// taio of distance to radius
	btScalar t = centreOffset.length() / _radius;

	// If within radius do nothing
	if (t < 0.8)
		return btVector3(0, 0, 0);
	
	// Otherwise return a quadratic of the ratio in direction to centre
	return centreOffset * t * t;
}
