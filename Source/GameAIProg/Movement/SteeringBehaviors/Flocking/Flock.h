#pragma once

// Toggle this define to enable/disable spatial partitioning
// #define GAMEAI_USE_SPACE_PARTITIONING

#include "FlockingSteeringBehaviors.h"
#include "Movement/SteeringBehaviors/SteeringAgent.h"
#include "Movement/SteeringBehaviors/SteeringHelpers.h"
#include "Movement/SteeringBehaviors/CombinedSteering/CombinedSteeringBehaviors.h"
#include <memory>
#include "imgui.h"
#ifdef GAMEAI_USE_SPACE_PARTITIONING
#include "../SpacePartitioning/SpacePartitioning.h"
#endif
class CellSpace;

class Flock final
{
public:
	Flock(
	UWorld* pWorld,
	TSubclassOf<ASteeringAgent> AgentClass,
	int FlockSize = 10, 
	float WorldSize = 100.f, 
	ASteeringAgent* const pAgentToEvade = nullptr, 
	bool bTrimWorld = false);

	~Flock();

	void Tick(float DeltaTime);
	void RenderDebug();
	void ImGuiRender(ImVec2 const& WindowPos, ImVec2 const& WindowSize);

	void RegisterNeighbors(ASteeringAgent* const Agent);

	int GetNrOfNeighbors() const;
	const TArray<ASteeringAgent*>& GetNeighbors() const;

	FVector2D GetAverageNeighborPos() const;
	FVector2D GetAverageNeighborVelocity() const;

	void SetTarget_Seek(FSteeringParams const& Target);

private:
	// For debug rendering purposes
	UWorld* pWorld{nullptr};
	
	int FlockSize{0};
	TArray<ASteeringAgent*> Agents{};
	
	std::unique_ptr<CellSpace> pPartitionedSpace{};
	int NrOfCellsX{ 10 };
	int NrOfCellsY{ 10 };
	TArray<FVector2D> OldPositions{};
	bool bUseSpacePartitioning{ true };

	
	TArray<ASteeringAgent*> Neighbors{};
	
	float NeighborhoodRadius{200.f};
	int NrOfNeighbors{0};

	ASteeringAgent* pAgentToEvade{nullptr};
	
	//Steering Behaviors
	std::unique_ptr<Separation> pSeparationBehavior{std::make_unique<Separation>(this)};
	std::unique_ptr<Cohesion> pCohesionBehavior{std::make_unique<Cohesion>(this)};
	std::unique_ptr<VelocityMatch> pVelMatchBehavior{std::make_unique<VelocityMatch>(this)};
	std::unique_ptr<Seek> pSeekBehavior{std::make_unique<Seek>()};
	std::unique_ptr<Wander> pWanderBehavior{std::make_unique<Wander>()};
	std::unique_ptr<Evade> pEvadeBehavior{std::make_unique<Evade>()};
	
	std::unique_ptr<BlendedSteering> pBlendedSteering{};
	std::unique_ptr<PrioritySteering> pPrioritySteering{};

	// UI and rendering
	bool DebugRenderSteering{false};
	bool DebugRenderNeighborhood{true};
	bool DebugRenderPartitions{true};

	void UpdateEvadeTarget();

	void RenderNeighborhood();
};
