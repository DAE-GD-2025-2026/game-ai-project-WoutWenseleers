#include "SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"
#include <iostream>
//SEEK
//*******
// TODO: Do the Week01 assignment :^)
SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};

	Steering.LinearVelocity = Target.Position - Agent.GetPosition();
	Agent.SetMaxLinearSpeed(MaxSpeed);

	return Steering;
}

SteeringOutput Flee::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};

	Steering.LinearVelocity = -(Target.Position - Agent.GetPosition());
	Agent.SetMaxLinearSpeed(MaxSpeed);

	return Steering;
}

SteeringOutput Arrive::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};

	Steering.LinearVelocity = Target.Position - Agent.GetPosition();

	float Distance = (Target.Position - Agent.GetPosition()).Length();

	const float SpeedFactor = std::clamp(((Distance - TargetRadius) / (SlowRadius)), 0.f, 1.f);

	Agent.SetMaxLinearSpeed(MaxSpeed * SpeedFactor);
	
	return Steering;
}

SteeringOutput Face::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};

	Steering.AngularVelocity = 50.f;
	std::cout << "???????????????\n";

	return Steering;
}

SteeringOutput Pursuit::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};

	FVector2D finalTarget{ Target.Position + Target.LinearVelocity };

	Steering.LinearVelocity = finalTarget - Agent.GetPosition();
	Agent.SetMaxLinearSpeed(0.f);

	return Steering;
}
