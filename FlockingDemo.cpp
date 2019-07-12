/*
Bullet Continuous Collision Detection and Physics Library
Ragdoll Demo
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

#define CONSTRAINT_DEBUG_SIZE 0.2f


#include "btBulletDynamicsCommon.h"
#include "GlutStuff.h"
#include "GL_ShapeDrawer.h"

#include "LinearMath/btIDebugDraw.h"

#include "GLDebugDrawer.h"
GLDebugDrawer debugDrawer;
#include "FlockingDemo.h"


// Enrico: Shouldn't these three variables be real constants and not defines?

#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2     1.57079632679489661923
#endif

#ifndef M_PI_4
#define M_PI_4     0.785398163397448309616
#endif

void FlockingDemo::initPhysics()
{
	// Setup the basic world
	debugDrawer = GLDebugDrawer();
	setTexturing(true);
	setShadows(true);

	setCameraDistance(btScalar(20.));

	m_collisionConfiguration = new btDefaultCollisionConfiguration();

	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

	btVector3 worldAabbMin(-10000,-10000,-10000);
	btVector3 worldAabbMax(10000,10000,10000);
	m_broadphase = new btAxisSweep3 (worldAabbMin, worldAabbMax);

	m_solver = new btSequentialImpulseConstraintSolver;

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
	//m_dynamicsWorld->getDispatchInfo().m_useConvexConservativeDistanceUtil = true;
	//m_dynamicsWorld->getDispatchInfo().m_convexConservativeDistanceThreshold = 0.01f;



	// Setup a big ground box
	{
		btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(200.),btScalar(10.),btScalar(200.)));
		m_collisionShapes.push_back(groundShape);
		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0,-10,0));

#define CREATE_GROUND_COLLISION_OBJECT 1
#ifdef CREATE_GROUND_COLLISION_OBJECT
		btCollisionObject* fixedGround = new btCollisionObject();
		fixedGround->setCollisionShape(groundShape);
		fixedGround->setWorldTransform(groundTransform);
		m_dynamicsWorld->addCollisionObject(fixedGround);
#else
		localCreateRigidBody(btScalar(0.),groundTransform,groundShape);
#endif //CREATE_GROUND_COLLISION_OBJECT

	}

	m_dynamicsWorld->setGravity(btVector3(0, 0, 0));

	// Create boid shape
	btConvexHullShape* boidShape = new btConvexHullShape();
	btVector3 boidShapeVertices[6];
	boidShapeVertices[0] = btVector3(2, 0, 0);
	boidShapeVertices[1] = btVector3(0, 0.5f, 0);
	boidShapeVertices[2] = btVector3(0, -0.5f, 0);
	boidShapeVertices[3] = btVector3(0, 0, 1);
	boidShapeVertices[4] = btVector3(0, 0, -1);
	boidShapeVertices[5] = btVector3(-1, 0, 0);
	for (int i = 0; i < 6; i++) {
		boidShape->addPoint(boidShapeVertices[i]);
	}

	// Get random move to point
	moveToPoint = new btVector3(10, 10, -100);

	// Create behaviours weightings for combinedbehaviour
	std::map<FlockBehaviour*, float> * behavioursToWeighting = new  std::map<FlockBehaviour*, float>();
	behavioursToWeighting->insert(std::pair<FlockBehaviour*, float>(new Cohesion(), cohesionWeighting));
	behavioursToWeighting->insert(std::pair<FlockBehaviour*, float>(new StayWithinRadius(stayWithinMidPoint, stayWithinRadius), stayWithinRadiusWeighting));
	// behavioursToWeighting->insert(std::pair<FlockBehaviour*, float>(new SteeredCohesion(), steeredCohesionWeighting));
	behavioursToWeighting->insert(std::pair<FlockBehaviour*, float>(new Alignment(), alignmentWeighting));
	behavioursToWeighting->insert(std::pair<FlockBehaviour*, float>(new Separation(), separationWeighting));
	behavioursToWeighting->insert(std::pair<FlockBehaviour*, float>(new MoveToBehaviour(moveToPoint), moveToWeighting));

	// Create the flock, with anew combined behaviour
	flock1 = new Flock(*m_dynamicsWorld, *(new CombinedBehaviour(*behavioursToWeighting)), startPosition, startNumber, boidShape);
	
	//// Create behaviours weightings for combinedbehaviour
	//std::map<FlockBehaviour*, float> * behavioursToWeighting2 = new  std::map<FlockBehaviour*, float>();
	//behavioursToWeighting2->insert(std::pair<FlockBehaviour*, float>(new Cohesion(), 2));
	//behavioursToWeighting2->insert(std::pair<FlockBehaviour*, float>(new StayWithinRadius(btVector3(0, 20, 0), 40.f), 3));
	//// behavioursToWeighting2->insert(std::pair<FlockBehaviour*, float>(new SteeredCohesion(), 2));
	//behavioursToWeighting2->insert(std::pair<FlockBehaviour*, float>(new Alignment(), 4));
	//behavioursToWeighting2->insert(std::pair<FlockBehaviour*, float>(new Separation(), 3));
	//// behavioursToWeighting2->insert(std::pair<FlockBehaviour*, float>(new MoveToBehaviour(btVector3(10, 10, 50)), 1));
	//
	//
	//// flock2 = new Flock(*m_dynamicsWorld, *(new CombinedBehaviour(*behavioursToWeighting2)), btVector3(40, 10, 0), btVector3(3, 3, 3));


	//
	//
	////
	// Steering test
	//btTransform startTransform;
	//startTransform.setIdentity();
	//btScalar mass(1.f);
	//// btVector3 localInertia(0, 0, 0);
	//// boidShape->calculateLocalInertia(mass, localInertia); // Assuming all boids are dynamic 
	//startTransform.setOrigin(btVector3(0, 5, 0));
	//// btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	//// btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, boidShape);
	//btRigidBody* body = localCreateRigidBody(mass, startTransform, boidShape);

	//// m_dynamicsWorld->addRigidBody(body); // Add to given dynamics world 

	//boid = new Boid(*body);
	//boid->targetToMoveTo = btVector3(10, 30, 60);


	clientResetScene();		
}

void FlockingDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	//simple dynamics world doesn't handle fixed-time-stepping
	float ms = getDeltaTimeMicroseconds();

	float minFPS = 1000000.f/60.f;
	float deltaTime = ms / 1000000.f;

	if (ms > minFPS)
		ms = minFPS;

	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->stepSimulation(ms / 1000000.f);
		
		//optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();

		// 
		// Steering test
		// boid->acceleration = btVector3(50, 0, 0);
		// boid->Update();

		if (currentMoveToResetTimer >= timeToResetMoveToPoint)
		{
			moveToPoint->setValue(std::rand() % ((stayMaxX - stayMinX + 1) + stayMinX), std::rand() % ((stayMaxX - stayMinX + 1) + stayMinX), std::rand() % ((stayMaxX - stayMinX + 1) + stayMinX));
			currentMoveToResetTimer = 0.f;
		}
		else
			currentMoveToResetTimer += deltaTime;

		if (flock1)
			flock1->Update(deltaTime);

		//if (flock2)
		//	flock2->Update(deltaTime);
	}

	renderme(); 

	glFlush();

	glutSwapBuffers();
}

void FlockingDemo::displayCallback()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	renderme();

	//optional but useful: debug drawing
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	glFlush();
	glutSwapBuffers();
}

void FlockingDemo::keyboardCallback(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 'e':
		{
		//btVector3 startOffset(0,2,0);
		//spawnRagdoll(startOffset);
		break;
		}
	default:
		DemoApplication::keyboardCallback(key, x, y);
	}

	
}



void	FlockingDemo::exitPhysics()
{

	int i;

	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}

	//delete dynamics world
	delete m_dynamicsWorld;

	//delete solver
	delete m_solver;

	//delete broadphase
	delete m_broadphase;

	//delete dispatcher
	delete m_dispatcher;

	delete m_collisionConfiguration;

	delete flock1;
	// delete flock2;

}





