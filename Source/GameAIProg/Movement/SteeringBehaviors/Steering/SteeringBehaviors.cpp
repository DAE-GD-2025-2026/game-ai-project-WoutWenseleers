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
	//Agent.SetMaxLinearSpeed(MaxSpeed);

	return Steering;
}

SteeringOutput Flee::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};

	Steering.LinearVelocity = -(Target.Position - Agent.GetPosition());
	//Agent.SetMaxLinearSpeed(MaxSpeed);

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
	const FVector2D targetVect{ Target.Position - Agent.GetPosition() };
	const FVector2D facingVect{ Agent.GetActorForwardVector() };


	Steering.AngularVelocity = acosf(facingVect.Dot(targetVect) / facingVect.Length() / targetVect.Length()) * 3.1415f / 180;

	return Steering;
}

SteeringOutput Pursuit::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};
	double distance{ (Target.Position - Agent.GetPosition()).Length()};
	double timeToReachTarget{ distance / Agent.GetMaxLinearSpeed() };

	FVector2D finalTarget{ Target.Position + Target.LinearVelocity * timeToReachTarget };

	Steering.LinearVelocity = finalTarget - Agent.GetPosition();
	//Agent.SetMaxLinearSpeed(MaxSpeed/1.5f);

	return Steering;
}

SteeringOutput Evade::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};
	double distance{ (Target.Position - Agent.GetPosition()).Length()};
	double timeToReachTarget{ distance / Agent.GetMaxLinearSpeed() };

	FVector2D finalTarget{ Target.Position + Target.LinearVelocity * timeToReachTarget };

	Steering.LinearVelocity = -(finalTarget - Agent.GetPosition());
	//Agent.SetMaxLinearSpeed(MaxSpeed/1.5f);
	if (Steering.LinearVelocity.Length() > m_EvadeRadius)
	{
		Steering.IsValid = false;
	}
	else
	{
		Steering.IsValid = true;
	}
	return Steering;
}

SteeringOutput Wander::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};
	const FVector2D facingVect{ Agent.GetActorForwardVector() };
	const FVector2D actorPosition{ Agent.GetActorLocation() };

	float angleIncrement{ rand() / (float)RAND_MAX * m_MaxAngleChange * 2.f - m_MaxAngleChange};
	m_WanderAngle = m_WanderAngle + angleIncrement;
	FVector2D finalTarget{ actorPosition +
		facingVect * m_OffsetDistance + 
		FVector2D(cosf(m_WanderAngle), sinf(m_WanderAngle)) * m_Radius };

	Steering.LinearVelocity = (finalTarget - Agent.GetPosition());
	//Agent.SetMaxLinearSpeed(MaxSpeed);

	return Steering;
}
