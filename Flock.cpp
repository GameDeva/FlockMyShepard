#include "Flock.h"
#include "FlockBehaviour.h"

Flock::Flock(btDynamicsWorld & dynamicsWorld, FlockBehaviour &flockBehaviour, btVector3 startPos, btVector3 startCount, btConvexHullShape *boidShape) : _dynamicsWorld(&dynamicsWorld), _flockBehaviour(&flockBehaviour), _startPos(startPos), _startCount(startCount)
{
	// Initialise object
	squareMaxSpeed = _maxSpeed * _maxSpeed;
	squareNeighbourRaidus = _neighbourRadius * _neighbourRadius;
	squareSeparationRadius = squareNeighbourRaidus * _separationRadiusMultiplier * _separationRadiusMultiplier;

	boids = std::vector<Boid*>();

	// 
	// Create boids
	// Re-using the same collision is better for memory usage and performance
	// btCollisionShape* boidColl = new btConeShape(btScalar(scaling * 1.), btScalar(5.)); // 

	// 
	btTransform startTransform;
	startTransform.setIdentity();
	btScalar mass(1.f);
	btVector3 localInertia(0, 0, 0);
	boidShape->calculateLocalInertia(mass, localInertia); // Assuming all boids are dynamic 

	int scaling = 1;
	for (int k = 0; k < startCount.y(); k++)
	{
		for (int i = 0; i < startCount.x(); i++)
		{
			for (int j = 0; j < startCount.z(); j++)
			{
				startTransform.setOrigin(scaling*btVector3(
					btScalar(3.0*i + startPos.x()),
					btScalar(3.0*k + startPos.y()),
					btScalar(3.0*j + startPos.z())));

				//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
				btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
				btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, boidShape, localInertia);
				btRigidBody* body = new btRigidBody(rbInfo);

				dynamicsWorld.addRigidBody(body); // Add to given dynamics world 
				boids.push_back(new Boid(*body)); // Create new boid, add to vectorlist
			}
		}
	}

	//// Spawn boids randomly inside sphere
	//for (int i = 0; i < boidCount; ++i)
	//{
	//	// startTransform.setOrigin(GetRandomPointInSphere(flockSphereStartPoint, startSphereRadius));

	//	//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
	//	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	//	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, boidShape, localInertia);
	//	btRigidBody* body = new btRigidBody(rbInfo);

	//	body->getWorldTransform().setOrigin(GetRandomPointInSphere(flockSphereStartPoint, startSphereRadius));

	//	dynamicsWorld.addRigidBody(body); // Add to given dynamics world 
	//	boids.push_back(new Boid(*body)); // Create new boid, add to vectorlist
	//}	

	// 
	// Use ghost object to identify all boids entering each others' trigger regions, neighbourRadius and separationNeighbourRadius
	btSphereShape* sphereShape = new btSphereShape(_neighbourRadius);
	ghost = new btPairCachingGhostObject();
	ghost->setCollisionShape(sphereShape);
	ghost->setCollisionFlags(btCollisionObject::CF_NO_CONTACT_RESPONSE);
	_dynamicsWorld->getBroadphase()->getOverlappingPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());


}

void Flock::Update(float deltaTime)
{
	// Iterate through each boid
	for (auto it = boids.begin(); it != boids.end(); ++it)
	{
		// Get all rigidbodies in the neighbour radius using ghostobject
		btAlignedObjectArray<btRigidBody*> nearbyRigbods = OverlapAllRigBod((*it)->getBody(), _neighbourRadius, (*it)->getBody().getWorldTransform().getOrigin(), *ghost);
		// Method 2 with distances
		// btAlignedObjectArray<btRigidBody*> nearbyRigbods = GetNeighbours(**it);

		// Apply the given flock behaviour
		btVector3 movement = _flockBehaviour->CalculateMovement(**it, nearbyRigbods, *this);
		// Update the boid
		(*it)->Update(movement);

	}

}

// METHOD1
// USING btGHOSTOBJECT, TRIGGER REGION, more efficient
// Finds all rigibodies at a given position within a radius
btAlignedObjectArray<btRigidBody*> Flock::OverlapAllRigBod(btRigidBody &mainRigbod, float radius, btVector3 pos, btPairCachingGhostObject &ghost) 
{
	// Vector to return 
	btAlignedObjectArray<btRigidBody*> outputRigbods; // = std::vector<btRigidBody*>();

	btTransform t;
	t.setIdentity();
	t.setOrigin(pos);
	ghost.setWorldTransform(t);

	_dynamicsWorld->addCollisionObject(&ghost, btBroadphaseProxy::SensorTrigger, btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::SensorTrigger);


	// Loop through the overlapped ghost objects 
	int size = ghost.getNumOverlappingObjects();
	for (int i = 0; i < size; ++i)
	{
		// Dynamic cast to make sure its a rigid body
		btRigidBody *pRigidBody = (btRigidBody *)(ghost.getOverlappingObject(i));
		
		// If cast was successful, and not the main rigbod 
		if (pRigidBody && pRigidBody != &mainRigbod)
		{
			outputRigbods.push_back(pRigidBody);
		}
	}

	_dynamicsWorld->removeCollisionObject(&ghost);


	return outputRigbods;
}

// METHOD2
// Using distance between each boid
//Iterates through flock members and returns list near given boid
btAlignedObjectArray<btRigidBody*> Flock::GetNeighbours(Boid &boid)
{
	btAlignedObjectArray<btRigidBody*> neighbourBoids;

	for (auto other : boids)
	{
		if (&boid != other && boid.getBody().getWorldTransform().getOrigin().distance2(other->getBody().getWorldTransform().getOrigin()) < squareNeighbourRaidus)
		{
			// Add to output list
			neighbourBoids.push_back(other->getBodyPointer());
		}
	}

	return neighbourBoids;
}

// Gets random point, inside a sphere of given position and radius
btVector3 Flock::GetRandomPointInSphere(btVector3 position, float radius)
{
	float theta = RandomFloat(0.f, 2.f * SIMD_PI);
	float phi = RandomFloat(-(2.f / SIMD_PI), 2.f / SIMD_PI);
	float r = RandomFloat(0, radius);
	
	float sinTheta = sin(theta);
	float cosTheta = cos(theta);
	float sinPhi = sin(phi);
	float cosPhi = cos(phi);

	float x = r * sinPhi * cosTheta;
	float y = r * sinPhi * sinTheta;
	float z = r * cosPhi;

	return btVector3(x, y, z) + position;
}

// Gets random float between 2 floats, excluding b
float Flock::RandomFloat(float a, float b) 
{
	float random = ((float)rand()) / (float)RAND_MAX;
	float diff = b - a;
	float r = random * diff;
	return a + r;
}

Flock::~Flock()
{
	delete _flockBehaviour;
}
