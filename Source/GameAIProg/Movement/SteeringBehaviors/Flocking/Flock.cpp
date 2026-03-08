#include "Flock.h"
#include "FlockingSteeringBehaviors.h"
#include "Shared/ImGuiHelpers.h"
#include "../SpacePartitioning/SpacePartitioning.h"
#include "GameAIProg/Shared/Level_Base.h"


Flock::Flock(
	UWorld* pWorld,
	TSubclassOf<ASteeringAgent> AgentClass,
	int FlockSize,
	float WorldSize,
	ASteeringAgent* const pAgentToEvade,
	bool bTrimWorld)
	: pWorld{pWorld}
	, FlockSize{ FlockSize }
	, pAgentToEvade{pAgentToEvade}
{
	pBlendedSteering = std::make_unique<BlendedSteering>(
		std::vector<BlendedSteering::WeightedBehavior>{
			{ pCohesionBehavior.get(), 5.f }
		}
	);
	pBlendedSteering->AddBehaviour({ pSeparationBehavior.get(), 10.f });
	pBlendedSteering->AddBehaviour({ pVelMatchBehavior.get(), 5.f });
	pBlendedSteering->AddBehaviour({ pSeekBehavior.get(), 1.f });
	pBlendedSteering->AddBehaviour({ pWanderBehavior.get(), 10.f });

	pEvadeBehavior->SetEvadeRadius(350.f);

	pPrioritySteering = std::make_unique<PrioritySteering>(
		std::vector<ISteeringBehavior*>{ pEvadeBehavior.get(), pBlendedSteering.get() });

	Agents.Init(nullptr, FlockSize);
	Neighbors.Init(nullptr, FlockSize);
	OldPositions.Init(FVector2D::ZeroVector, FlockSize);

	NrOfCellsX = FMath::Max(1, FMath::CeilToInt(WorldSize / NeighborhoodRadius));
	NrOfCellsY = NrOfCellsX;

	pPartitionedSpace = std::make_unique<CellSpace>(
		pWorld,
		WorldSize,
		WorldSize,
		NrOfCellsY,
		NrOfCellsX,
		FlockSize
	);

	for (int i{}; i < FlockSize; ++i)
	{
		ASteeringAgent* pAgent = pWorld->SpawnActor<ASteeringAgent>(
			AgentClass,
			FVector{ -1000.f + 100.f * float(i / 10), -1000.f + 100.f * float(i % 10), 90.f },
			FRotator::ZeroRotator
		);

		if (!pAgent) continue;
		pAgent->PrimaryActorTick.bCanEverTick = false;
		pAgent->SetSteeringBehavior(pPrioritySteering.get());
		Agents[i] = pAgent;
		OldPositions[i] = pAgent->GetPosition();

		pPartitionedSpace->AddAgent(*pAgent);
	}
}

Flock::~Flock()
{
 // TODO: Cleanup any additional data
}

void Flock::Tick(float DeltaTime)
{
 // TODO: update the flock
 // TODO: for every agent:
  // TODO: register the neighbors for this agent (-> fill the memory pool with the neighbors for the currently evaluated agent)
  // TODO: update the agent (-> the steeringbehaviors use the neighbors in the memory pool)
  // TODO: trim the agent to the world
	UpdateEvadeTarget();
	for (int i = 0; i < Agents.Num(); ++i)
	{
		ASteeringAgent* pAgent = Agents[i];
		if (!pAgent) continue;

		RegisterNeighbors(pAgent);
		pAgent->Tick(DeltaTime);

		if (bUseSpacePartitioning)
		{
			pPartitionedSpace->UpdateAgentCell(*pAgent, OldPositions[i]);
			OldPositions[i] = pAgent->GetPosition();
		}
	}
	
}

void Flock::RenderDebug()
{
 // TODO: Render all the agents in the flock
}

void Flock::ImGuiRender(ImVec2 const& WindowPos, ImVec2 const& WindowSize)
{
#ifdef PLATFORM_WINDOWS
#pragma region UI
	//UI
	{
		//Setup
		bool bWindowActive = true;
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Gameplay Programming", &bWindowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

		//Elements
		ImGui::Text("CONTROLS");
		ImGui::Indent();
		ImGui::Text("LMB: place target");
		ImGui::Text("RMB: move cam.");
		ImGui::Text("Scrollwheel: zoom cam.");
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();
		ImGui::Spacing();

		ImGui::Text("STATS");
		ImGui::Indent();
		ImGui::Text("%.3f ms/frame", 1000.0f / ImGui::GetIO().Framerate);
		ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

		ImGui::Text("Flocking");
		ImGui::Spacing();

		ImGui::Text("Behavior Weights");
		ImGui::Spacing();
		
		ImGui::SliderFloat("WCohesion", pBlendedSteering.get()->GetWeight(pCohesionBehavior.get()), 0.f, 100.f, "%.2f");
		ImGui::SliderFloat("WSeparation", pBlendedSteering.get()->GetWeight(pSeparationBehavior.get()), 0.f, 100.f, "%.2f");
		ImGui::SliderFloat("WVelMatch", pBlendedSteering.get()->GetWeight(pVelMatchBehavior.get()), 0.f, 100.f, "%.2f");
		ImGui::SliderFloat("WSeek", pBlendedSteering.get()->GetWeight(pSeekBehavior.get()), 0.f, 100.f, "%.2f");
		ImGui::SliderFloat("WWander", pBlendedSteering.get()->GetWeight(pWanderBehavior.get()), 0.f, 100.f, "%.2f");

		ImGui::Text("Settings");
		ImGui::Spacing();
		ImGui::Checkbox("Space Partitioning", &bUseSpacePartitioning);
		//End
		ImGui::End();
	}
#pragma endregion
#endif
}

void Flock::RenderNeighborhood()
{
 // TODO: Debugrender the neighbors for the first agent in the flock
}


void Flock::RegisterNeighbors(ASteeringAgent* const pAgent)
{
	if (!pAgent) return;

	if (bUseSpacePartitioning && pPartitionedSpace)
	{
		pPartitionedSpace->RegisterNeighbors(*pAgent, NeighborhoodRadius);
		return;
	}

	NrOfNeighbors = 0;
	for (ASteeringAgent* pSteeringAgent : Agents)
	{
		if (!pSteeringAgent) continue;
		if (pAgent == pSteeringAgent) continue;

		if ((pAgent->GetPosition() - pSteeringAgent->GetPosition()).Length() < NeighborhoodRadius)
		{
			Neighbors[NrOfNeighbors++] = pSteeringAgent;
		}
	}
}

FVector2D Flock::GetAverageNeighborPos() const
{
	FVector2D avgPosition = FVector2D::ZeroVector;

	const TArray<ASteeringAgent*>& neighbors = GetNeighbors();
	const int nrOfNeighbors = GetNrOfNeighbors();

	if (nrOfNeighbors == 0)
		return avgPosition;

	for (int i = 0; i < nrOfNeighbors; ++i)
	{
		if (!neighbors[i]) continue;
		avgPosition += neighbors[i]->GetPosition();
	}

	return avgPosition / float(nrOfNeighbors);
}

FVector2D Flock::GetAverageNeighborVelocity() const
{
	FVector2D avgVelocity = FVector2D::ZeroVector;

	const TArray<ASteeringAgent*>& neighbors = GetNeighbors();
	const int nrOfNeighbors = GetNrOfNeighbors();

	if (nrOfNeighbors == 0)
		return avgVelocity;

	for (int i = 0; i < nrOfNeighbors; ++i)
	{
		if (!neighbors[i]) continue;
		avgVelocity += FVector2D(neighbors[i]->GetVelocity());
	}

	return avgVelocity / float(nrOfNeighbors);
}

void Flock::SetTarget_Seek(FSteeringParams const& Target)
{
	pSeekBehavior.get()->SetTarget(Target);
}

void Flock::UpdateEvadeTarget()
{
	if (!pAgentToEvade)
		return;

	FTargetData EvadeTarget{};
	EvadeTarget.Position = pAgentToEvade->GetPosition();
	EvadeTarget.Orientation = pAgentToEvade->GetRotation();
	EvadeTarget.LinearVelocity = pAgentToEvade->GetLinearVelocity();
	EvadeTarget.AngularVelocity = pAgentToEvade->GetAngularVelocity();

	pEvadeBehavior->SetTarget(EvadeTarget);
}

int Flock::GetNrOfNeighbors() const
{
	return bUseSpacePartitioning
		? pPartitionedSpace->GetNrOfNeighbors()
		: NrOfNeighbors;
}

const TArray<ASteeringAgent*>& Flock::GetNeighbors() const
{
	return bUseSpacePartitioning
		? pPartitionedSpace->GetNeighbors()
		: Neighbors;
}