#include "FlockingSteeringBehaviors.h"
#include "Flock.h"
#include "../SteeringAgent.h"
#include "../SteeringHelpers.h"


//*******************
//COHESION (FLOCKING)
SteeringOutput Cohesion::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	SteeringOutput cohesionSteering = {};
	FVector2D avgNeighbourPosition = pFlock->GetAverageNeighborPos();

	cohesionSteering.LinearVelocity = avgNeighbourPosition - pAgent.GetPosition();

	return cohesionSteering;
}

//*********************
//SEPARATION (FLOCKING)

SteeringOutput Separation::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	SteeringOutput separationSteering = {};

	const TArray<ASteeringAgent*>& neighbors = pFlock->GetNeighbors();
	const int nrOfNeighbors = pFlock->GetNrOfNeighbors();
	
	for (int i = 0; i < nrOfNeighbors; ++i)
	{
		ASteeringAgent* pSteeringAgent = neighbors[i];

		if (!pSteeringAgent) continue;

		FVector2D neigbourVector = pAgent.GetPosition() - pSteeringAgent->GetPosition();
		separationSteering.LinearVelocity += neigbourVector / neigbourVector.Length();
	}
	
	return separationSteering;
}

//*************************
//VELOCITY MATCH (FLOCKING)
SteeringOutput VelocityMatch::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	SteeringOutput velocityMatchSteering = {};
	FVector2D avgNeighbourVelocity = pFlock->GetAverageNeighborVelocity();

	velocityMatchSteering.LinearVelocity = avgNeighbourVelocity;

	return velocityMatchSteering;
}