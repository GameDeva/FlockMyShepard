#include "MoveToBehaviour.h"



MoveToBehaviour::MoveToBehaviour(btVector3* position) : _position(position)
{
}


MoveToBehaviour::~MoveToBehaviour()
{
}


btVector3 MoveToBehaviour::CalculateMovement(Boid & boid, btAlignedObjectArray<btRigidBody*>& others, Flock & flock) const
{
	// Return direction to position
	return (*_position - boid.getBody().getWorldTransform().getOrigin()).normalized();
}