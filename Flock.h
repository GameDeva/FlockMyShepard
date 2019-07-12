#pragma once
#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionDispatch/btGhostObject.h"
#include <vector>
#include "Boid.h"

class FlockBehaviour;
struct FlockInfo;

// A group of boids following the same flock behaviours
class Flock
{
public:
	Flock(btDynamicsWorld & dynamicsWorld, FlockBehaviour &flockBehaviour, btVector3 startPos, btVector3 startCount, btConvexHullShape *boidShape);
	~Flock();

	void Update(float deltaTime);
	float getSquareSeparationRadius() { return squareSeparationRadius; }

private:

	btPairCachingGhostObject *ghost; // Used for triggers 

	std::vector<Boid*> boids; // All boids part of this flock
	FlockBehaviour *_flockBehaviour; // The type of flock behaviour used 

	// Refence to the dyanmic world, to add bodies
	btDynamicsWorld *_dynamicsWorld;

	//
	// Flock Data 
	
	btVector3 _startPos;
	btVector3 _startCount;

	btScalar _thrustFactor = 200.f; // Thrust of each boid in any direction
	btScalar _maxSpeed = 60; // 
	btScalar _neighbourRadius = 2; // 
	btScalar _separationRadiusMultiplier = 0.7; // Between 0 and 1, since separationRadius will be the inner part of the neighbourRadius

	btScalar squareMaxSpeed;
	btScalar squareNeighbourRaidus; // Size of trigger area
	btScalar squareSeparationRadius; // Separation radius based on squareNeighbourRadius

	// Methods
	btAlignedObjectArray<btRigidBody*> GetNeighbours(Boid &boid);
	float RandomFloat(float a, float b);
	btVector3 GetRandomPointInSphere(btVector3 position, float radius);

	//
	// //
	// Not used, for spawning at random point in sphere
	int boidCount = 20;
	int scaling = 1;
	btVector3 flockSphereStartPoint = btVector3(0, 20, 0);
	float startSphereRadius = 30.f;

	btAlignedObjectArray<btRigidBody*> OverlapAllRigBod(btRigidBody &mainRigBod, float radius, btVector3 pos, btPairCachingGhostObject &ghost);

};

// TODO: move flock data here if more convinient
// Info/Data for each flock
struct FlockInfo
{
public:

};