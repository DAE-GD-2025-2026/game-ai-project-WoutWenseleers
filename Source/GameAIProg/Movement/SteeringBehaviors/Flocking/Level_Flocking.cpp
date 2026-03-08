// Fill out your copyright notice in the Description page of Project Settings.


#include "Level_Flocking.h"


// Sets default values
ALevel_Flocking::ALevel_Flocking()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void ALevel_Flocking::BeginPlay()
{
	Super::BeginPlay();

	TrimWorld->SetTrimWorldSize(1000.f);
	TrimWorld->bShouldTrimWorld = true;

	pAgentToEvade = GetWorld()->SpawnActor<ASteeringAgent>(
		SteeringAgentClass,
		FVector{ 200.f, 200.f, 90.f },
		FRotator::ZeroRotator
	);

	if (pAgentToEvade)
	{
		pAgentToEvade->SetActorScale3D(FVector(2.f, 2.f, 2.f)); // easy to identify

		pEvadeTargetBehavior->SetWanderOffset(250.f);
		pEvadeTargetBehavior->SetWanderRadius(150.f);
		pEvadeTargetBehavior->SetMaxAngleChange(PI / 6.f);

		pAgentToEvade->SetSteeringBehavior(pEvadeTargetBehavior.get());
	}
	
	pFlock = TUniquePtr<Flock>(
		new Flock(
			GetWorld(),
			SteeringAgentClass,
			FlockSize,
			TrimWorld->GetTrimWorldSize(),
			pAgentToEvade,
			true)
			);
}

// Called every frame
void ALevel_Flocking::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	pFlock->ImGuiRender(WindowPos, WindowSize);
	pFlock->Tick(DeltaTime);
	pFlock->RenderDebug();
	if (bUseMouseTarget)
		pFlock->SetTarget_Seek(MouseTarget);
}

