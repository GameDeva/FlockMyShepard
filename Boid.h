#pragma once
#include "btBulletDynamicsCommon.h"
#include <vector>

const btVector3 WORLDFORWARD = btVector3(0, 0, 1);
const btVector3 WORLDUP = btVector3(0, 1, 0);

class Boid
{
public:
	Boid(btRigidBody & body);
	~Boid();

	btScalar boidSpeed = 10;

	btVector3 targetToMoveTo;
	btScalar maxForceSquared = 50;
	btVector3 steeringForce;
	void Update(btVector3 dir);
	btVector3 getLimitedSteeringForce(btVector3 desiredVelocity);



	btRigidBody & getBody() { return *_body; }
	btRigidBody * getBodyPointer() { return _body; }

	// void ApplyMovement(btVector3 velocity);
	btVector3 GetLocalForward();
	btVector3 GetLocalUp();

private:

	btRigidBody *_body;


};

