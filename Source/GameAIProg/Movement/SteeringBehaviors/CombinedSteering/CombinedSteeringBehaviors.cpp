
#include "CombinedSteeringBehaviors.h"
#include <algorithm>
#include "../SteeringAgent.h"

BlendedSteering::BlendedSteering(const std::vector<WeightedBehavior>& WeightedBehaviors)
	:WeightedBehaviors(WeightedBehaviors)
{};

//****************
//BLENDED STEERING
SteeringOutput BlendedSteering::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput BlendedSteering = {};
	
	for (WeightedBehavior weighedBehavior : WeightedBehaviors)
	{
		//weighedBehavior.pBehavior->SetTarget(Target);
		FVector2D linVelIncrement = weighedBehavior.pBehavior->CalculateSteering(DeltaT, Agent).LinearVelocity;
		linVelIncrement.Normalize();
		linVelIncrement *= weighedBehavior.Weight;
		BlendedSteering.LinearVelocity += linVelIncrement;
		BlendedSteering.AngularVelocity += weighedBehavior.pBehavior->CalculateSteering(DeltaT, Agent).AngularVelocity * weighedBehavior.Weight;
	}

	return BlendedSteering;
}

float* BlendedSteering::GetWeight(ISteeringBehavior* const SteeringBehavior)
{
	auto it = find_if(WeightedBehaviors.begin(),
		WeightedBehaviors.end(),
		[SteeringBehavior](const WeightedBehavior& Elem)
		{
			return Elem.pBehavior == SteeringBehavior;
		}
	);

	if(it!= WeightedBehaviors.end())
		return &it->Weight;
	
	return nullptr;
}
void BlendedSteering::SetWeight(ISteeringBehavior* const SteeringBehavior, float weight)
{
	auto it = find_if(WeightedBehaviors.begin(),
		WeightedBehaviors.end(),
		[SteeringBehavior](const WeightedBehavior& Elem)
		{
			return Elem.pBehavior == SteeringBehavior;
		}
	);

	if(it!= WeightedBehaviors.end())
		it->Weight = weight;
	
	return;
}

//*****************
//PRIORITY STEERING
SteeringOutput PrioritySteering::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering = {};

	for (ISteeringBehavior* const pBehavior : m_PriorityBehaviors)
	{
		Steering = pBehavior->CalculateSteering(DeltaT, Agent);
		
		if (Steering.IsValid)
			break;
	}

	//If none of the behaviors returns a valid output, last behavior is returned
	return Steering;
}