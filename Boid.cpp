#include "Boid.h"


// Sets up boids
Boid::Boid(btRigidBody &body) : _body(&body)
{
	steeringForce = btVector3(0, 0, 0);


	// _body->setDamping(0.1, 0.9);
	// btVector3 randomAxis = btVector3(rand(), rand(), rand()).normalized();
	// _body->getWorldTransform().setRotation(btQuaternion(randomAxis, rand()));

	// _body->applyCentralForce(GetLocalForward().normalized() * 100.f);
	// _body->setLinearVelocity(GetLocalForward().normalized() * 100.f);
}


Boid::~Boid()
{
}

void Boid::Update(btVector3 dir)
{
	if (!dir.isZero())
		dir = dir.normalized() * boidSpeed;

	steeringForce = getLimitedSteeringForce(dir);

	// Add the steering force
	_body->applyCentralForce(steeringForce);
	
	// Attempted torque calculation did not work
	// btVector3 tor = GetLocalForward().cross(acceleration);
	//_body->applyTorque(tor);
	//btVector3 t = _body->getTotalTorque();
	
	// Set back to zero
	steeringForce.setZero();
}

// Gets the steering force, which is desired velocity - current velocity
// Ensures that it is not above maxForce
btVector3 Boid::getLimitedSteeringForce(btVector3 desiredVelocity)
{
	btVector3 force = desiredVelocity - _body->getLinearVelocity();
	// If magnitude of force greater than max then limit it
	if (force.length2() > maxForceSquared)
		return force.normalized() * maxForceSquared;
	return force;
}

//// Not used anymore
//void Boid::ApplyMovement(btVector3 velocity)
//{
//	btScalar angle  = velocity.angle(GetLocalForward());
//	
//	// _body->getWorldTransform().setRotation();
//	_body->applyCentralForce(velocity);
//	// _body->applyTorque(btVector3(100, 100, 100));
//	btVector3 rotation = GetLocalForward().cross(velocity);
//	_body->applyTorque(velocity);
//}


btVector3 Boid::GetLocalForward()
{
	// Get rigbod's transform
	btTransform transform = _body->getWorldTransform();
	// Get local forward vector
	return transform * WORLDFORWARD - transform.getOrigin();
}

btVector3 Boid::GetLocalUp()
{
	// Get rigbod's transform
	btTransform transform = _body->getWorldTransform();
	// Get local up vector
	return transform * WORLDUP - transform.getOrigin();
}

