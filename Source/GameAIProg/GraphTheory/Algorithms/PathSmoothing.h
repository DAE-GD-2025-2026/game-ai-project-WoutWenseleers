#pragma once
#include <vector>

#include "NavGraphPathfinding.h"
#include "Movement/Pathfinding/Navmesh/TriPolygon.h"
#include "Shared/Graph/Graph.h"
#include "Shared/Graph/NavGraph/NavGraphNode.h"

namespace GameAI
{
	class SSFA final
{
public:
	//=== SSFA Functions ===
	//--- References ---
	//http://digestingduck.blogspot.be/2010/03/simple-stupid-funnel-algorithm.html
	//https://gamedev.stackexchange.com/questions/68302/how-does-the-simple-stupid-funnel-algorithm-work
	static std::vector<NavLine> FindPortals(std::vector<Node*> const & Path, TriPolygon const & NavPoly)
	{
		//Container
		std::vector<NavLine> Portals = {};
		if (Path.size() < 2) return Portals;

		//Add degenerate portal for start
		auto pStartNode = static_cast<NavGraphNode*>(Path[0]);
		NavLine startPortal;
		startPortal.P1 = pStartNode->GetPosition();
		startPortal.P2 = pStartNode->GetPosition();
		Portals.push_back(startPortal);
		
		//For each node received, get it's corresponding line
		for (size_t i = 1; i < Path.size() - 1; ++i)
		{
			auto pCurrentNode = static_cast<NavGraphNode*>(Path[i]);
			auto pPrevNode = static_cast<NavGraphNode*>(Path[i - 1]);
                
			int edgeIdx = pCurrentNode->GetEdgeIdx();
			if (edgeIdx != -1)
			{
				auto edge = NavPoly.GetEdges()[edgeIdx];
				FVector2D edgeP1 = FVector2D(edge.GetP1(NavPoly));
				FVector2D edgeP2 = FVector2D(edge.GetP2(NavPoly));
				
				FVector2D travelDir = pCurrentNode->GetPosition() - pPrevNode->GetPosition();
				FVector2D edgeDir = edgeP1 - edgeP2;
				//Redetermine it's "orientation" based on the required path (left-right vs right-left) - p1 should be right point
				NavLine portal;
				if (FVector2D::CrossProduct(travelDir, edgeDir) > 0.0f)
				{
					portal.P1 = edgeP2;
					portal.P2 = edgeP1;
				}
				else
				{
					portal.P1 = edgeP1;
					portal.P2 = edgeP2;
				}
				//Store portal
				Portals.push_back(portal);
			}
		}

		//Add degenerate portal to force end evaluation
		auto pEndNode = static_cast<NavGraphNode*>(Path.back());
		NavLine endPortal;
		endPortal.P1 = pEndNode->GetPosition();
		endPortal.P2 = pEndNode->GetPosition();
		Portals.push_back(endPortal);
		
		return Portals;
	}

	static std::vector<FVector2D> OptimizePortals( std::vector<NavLine> const & Portals, TriPolygon const & NavPoly)
	{
		std::vector<FVector2D> Path{};
		if (Portals.empty()) return Path;

		FVector2D apexPoint = Portals[0].P1;
		Path.push_back(apexPoint);
		
		int apexIndex = 0;
		int leftLegIndex = 1;
		int rightLegIndex = 1;
		
		//P1 == right point of portal, P2 == left point of portal
		FVector2D rightLeg = Portals[rightLegIndex].P1 - apexPoint;
		FVector2D leftLeg = Portals[leftLegIndex].P2 - apexPoint;
		for (int portalIdx = 1; portalIdx < Portals.size(); ++portalIdx)
		{
			const NavLine& currentPortal = Portals[portalIdx];
			//--- RIGHT CHECK ---
			FVector2D newRightLeg = currentPortal.P1 - apexPoint;
			//1. See if moving funnel inwards - RIGHT
			if (FVector2D::CrossProduct(rightLeg, newRightLeg) >= 0.0f)
			{
				//2. See if new line degenerates a line segment - RIGHT
				if (FVector2D::CrossProduct(leftLeg, newRightLeg) <= 0.0f)
				{
					rightLeg = newRightLeg;
					rightLegIndex = portalIdx;
				}
				else
				{
					apexPoint = Portals[leftLegIndex].P2;
					Path.push_back(apexPoint);

					apexIndex = leftLegIndex;
					
					portalIdx = apexIndex;
					
					leftLegIndex = apexIndex;
					rightLegIndex = apexIndex;
					leftLeg = FVector2D::ZeroVector;
					rightLeg = FVector2D::ZeroVector;

					continue;
				}
			}


			//--- LEFT CHECK ---
			FVector2D newLeftLeg = currentPortal.P2 - apexPoint;
			//1. See if moving funnel inwards - LEFT
			if (FVector2D::CrossProduct(leftLeg, newLeftLeg) <= 0.0f)
			{
				//2. See if new line degenerates a line segment - LEFT
				if (FVector2D::CrossProduct(rightLeg, newLeftLeg) >= 0.0f)
				{
					leftLeg = newLeftLeg;
					leftLegIndex = portalIdx;
				}
				else
				{
					apexPoint = Portals[rightLegIndex].P1;
					Path.push_back(apexPoint);

					apexIndex = rightLegIndex;
					portalIdx = apexIndex;

					leftLegIndex = apexIndex;
					rightLegIndex = apexIndex;
					leftLeg = FVector2D::ZeroVector;
					rightLeg = FVector2D::ZeroVector;
				}
			}
		}

		// Add last path point
		Path.push_back(Portals.back().P1);
		
		return Path;
	}
private:
	SSFA() {};
	~SSFA() {};
};
}
