/*
Bullet Continuous Collision Detection and Physics Library
RagdollDemo
Copyright (c) 2007 Starbreeze Studios

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

Written by: Marten Svanfeldt
*/

#ifndef RAGDOLLDEMO_H
#define RAGDOLLDEMO_H

#include "GlutDemoApplication.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "Flock.h"
#include "CombinedBehaviour.h"
#include "SteeredCohesion.h"
#include "Cohesion.h"
#include "Separation.h"
#include "Alignment.h"
#include "MoveToBehaviour.h"
#include "StayWithinRadius.h"
#include "BulletCollision/CollisionDispatch/btGhostObject.h"
class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;

class FlockingDemo : public GlutDemoApplication
{
	Flock* flock1;
	// Flock 1 data
	const btVector3 startPosition = btVector3(0, 10, 0);
	const btVector3 startNumber = btVector3(3, 3, 3); // Number of boids on each axis

	const btScalar cohesionWeighting = 2;
	const btScalar stayWithinRadiusWeighting = 3;
	// const btScalar steeredCohesionWeighting = 2;
	const btScalar alignmentWeighting = 1;
	const btScalar separationWeighting = 2;
	const btScalar moveToWeighting = 5;

	const btVector3 stayWithinMidPoint = btVector3(-50, 150, 0);
	const btScalar stayWithinRadius = 200;
	int stayMinX = stayWithinMidPoint.getX() - stayWithinRadius;
	int stayMaxX = stayWithinMidPoint.getX() + stayWithinRadius;
	int stayMinY = stayWithinMidPoint.getY() - stayWithinRadius ;
	int stayMaxY = stayWithinMidPoint.getY() + stayWithinRadius;
	int stayMinZ = stayWithinMidPoint.getZ() - stayWithinRadius;
	int stayMaxZ = stayWithinMidPoint.getZ() +stayWithinRadius;

	btVector3* moveToPoint;
	const float timeToResetMoveToPoint = 3.f;
	float currentMoveToResetTimer = 0.f;

	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;

	btBroadphaseInterface*	m_broadphase;

	btCollisionDispatcher*	m_dispatcher;

	btConstraintSolver*	m_solver;

	btDefaultCollisionConfiguration* m_collisionConfiguration;

	Boid *boid;


public:
	void initPhysics();

	void exitPhysics();

	virtual ~FlockingDemo()
	{
		exitPhysics();
	}

	
	virtual void clientMoveAndDisplay();

	virtual void displayCallback();

	virtual void keyboardCallback(unsigned char key, int x, int y);

	static DemoApplication* Create()
	{
		FlockingDemo* demo = new FlockingDemo();
		demo->myinit();
		demo->initPhysics();
		return demo;
	}
	
};


#endif
