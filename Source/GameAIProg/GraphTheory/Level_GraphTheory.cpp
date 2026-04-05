// Fill out your copyright notice in the Description page of Project Settings.


#include "Level_GraphTheory.h"

#include "Algorithms/EulerianPath.h"
#include "Shared/GameAISpectator.h"

using namespace GameAI;

// Sets default values
ALevel_GraphTheory::ALevel_GraphTheory()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void ALevel_GraphTheory::BeginPlay()
{
	Super::BeginPlay();

	Renderer = GameAI::GraphRenderer(GetWorld());
	
	// Add the graph editor to our player
	if (PlayerController = Cast<APlayerController>(GetWorld()->GetFirstLocalPlayerFromController()->PlayerController); 
		GraphEditorClass && PlayerController)
	{
		PlayerGraphEditor = NewObject<UGraphEditorComponent>(PlayerController->GetPawn(), GraphEditorClass);
		PlayerGraphEditor->RegisterComponent();
		PlayerGraphEditor->SetEditedGraph(&Graph);
		PlayerGraphEditor->SetNodeFactory(&NodeFactory);
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT("Unable to get PlayerController from LocalPlayer or GraphEditorClass is null"))
		return;
	}
	
	// Make the view orthogonal for less perspective issues
	if (AGameAISpectator* Player = Cast<AGameAISpectator>(PlayerController->GetPawnOrSpectator()); Player)
	{
		Player->SetCameraProjection(ECameraProjectionMode::Orthographic);
	}
	
	// Create a small graph
	const int N0 = Graph.AddNode(NodeFactory.CreateNode(FVector2D{-600.f, -200.f}));
	const int N1 = Graph.AddNode(NodeFactory.CreateNode(FVector2D{-200.f, -600.f}));
	const int N2 = Graph.AddNode(NodeFactory.CreateNode(FVector2D{-600.f, -600.f}));
	const int N3 = Graph.AddNode(NodeFactory.CreateNode(FVector2D{-200.f, -200.f}));
	const int N4 = Graph.AddNode(NodeFactory.CreateNode(FVector2D{-400.f, 0.f}));


	// Add edges
	Graph.AddConnection(N0, N2);
	Graph.AddConnection(N2, N1);
	Graph.AddConnection(N1, N3);
	Graph.AddConnection(N3, N0);
	Graph.AddConnection(N0, N4); 
	Graph.AddConnection(N4, N3); 

	// Set edge weights to geometric distances
	Graph.SetConnectionCostsToDistances();
	
	// Spawn the Agent
	Agent = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, 
	FVector{0,0,90}, FRotator::ZeroRotator);
	Agent->SetSteeringBehavior(&PathFollow);

	EulerianPath eulerPath{ &Graph };

	Eulerianity eulerianity = eulerPath.IsEulerian();
	std::vector<Node*> nodes = eulerPath.FindPath(eulerianity);
	if (!nodes.empty())
	{
		UpdateAgentPath(nodes);
	}
}

void ALevel_GraphTheory::BeginDestroy()
{
	Super::BeginDestroy();
}

void ALevel_GraphTheory::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	
#pragma region UI
	{
		//Setup
		bool windowActive = true;
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Gameplay Programming", &windowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
		ImGui::SetWindowFocus();
		ImGui::PushItemWidth(70);
		//Elements
		ImGui::Text("CONTROLS");
		ImGui::Indent();
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
		ImGui::Spacing();

		ImGui::Text("Graph Theory");
		ImGui::Spacing();
		ImGui::Spacing();

		//End
		ImGui::End();
	}
#pragma endregion UI
	
	Renderer.RenderGraph(Graph);


	
	
	// TODO Check if the graph has updated
	// TODO if so, run the EulerianPath algorithm
	// TODO if a path is found, have the agent follow it
}

void ALevel_GraphTheory::UpdateAgentPath(std::vector<Node*> const& Trail)
{
	std::vector<FVector2D> path{};


	for (Node* pNode : Trail)
	{
		path.push_back(pNode->GetPosition());
	}
	
	PathFollow.SetPath(path);
	if (path.size() > 0)
	{
		Agent->SetPosition(path[0]);
	}
}




