#include "NavGraph.h"

#include "NavGraphNode.h"

GameAI::NavGraph::NavGraph(std::unique_ptr<TriPolygon> && NavPoly)
	: Graph{false}
	, pNavPoly{std::move(NavPoly)}
{
	CreateNavigationGraph();
}

GameAI::NavGraph::NavGraph(const NavGraph& Other)
	: Graph(false)
{
	Nodes.reserve(Other.Nodes.size());
	for (std::unique_ptr<Node> const & OtherNode : Other.Nodes)
	{
		Nodes.push_back(std::make_unique<NavGraphNode>(*dynamic_cast<NavGraphNode*>(OtherNode.get())));
	}
        
	Connections.reserve(Other.Connections.size());
	for (std::unique_ptr<Connection> const & OtherConnection : Other.Connections)
	{
		Connections.push_back(std::make_unique<Connection>(*OtherConnection.get()));
	}
}

std::unique_ptr<GameAI::NavGraph> GameAI::NavGraph::Clone() const
{
	return std::make_unique<NavGraph>(*this);
}

int GameAI::NavGraph::GetNodeIdFromEdgeIndex(int EdgeIdx) const
{
	if (EdgeIdx >= 0)
	{
		for (auto const & pNode : Nodes)
		{
			if (reinterpret_cast<NavGraphNode*>(pNode.get())->GetEdgeIdx() == EdgeIdx)
			{
				return pNode->GetId();
			}
		}
	}
	
	return Graphs::InvalidNodeId;
}

void GameAI::NavGraph::CreateNavigationGraph()
{
	//1. Go over all the edges of the navigation mesh and create nodes
	const std::vector<TriPolygon::Edge>& edges = pNavPoly->GetEdges();
	const std::vector<TriPolygon::Triangle>& triangles = pNavPoly->GetTriangles();

	for (int i = 0; i < static_cast<int>(edges.size()); ++i)
	{
		const TriPolygon::Edge& currentEdge = edges[i];
		
		int connectedTrianglesCount = 0;
		for (const TriPolygon::Triangle& triangle : triangles)
		{
			if (triangle.HasEdge(currentEdge))
			{
				connectedTrianglesCount++;
			}
		}

		if (connectedTrianglesCount > 1)
		{
			FVector2D p1 = (FVector2D) currentEdge.GetP1(*pNavPoly);
			FVector2D p2 = (FVector2D) currentEdge.GetP2(*pNavPoly);
			FVector2D middlePos = (p1 + p2) / 2.0f; 
			
			auto pNode = std::make_unique<NavGraphNode>(middlePos, i);
			AddNode(std::move(pNode));
		}
	}
	//2. Create connections now that every node is created	
		//2 valid nodes -> 1 connection
		//3 valid nodes -> 3 connections
	for (const TriPolygon::Triangle& triangle : triangles)
	{
		std::vector<int> validNodeIds;
		
		std::array<TriPolygon::Edge, 3> triEdges = triangle.GetEdges();
		for (const TriPolygon::Edge& edge : triEdges)
		{
			std::optional<int> edgeIdxOpt = pNavPoly->FindEdgeIndex(edge);
            
			if (edgeIdxOpt.has_value())
			{
				int edgeIdx = edgeIdxOpt.value();
				int nodeId = GetNodeIdFromEdgeIndex(edgeIdx);
				
				if (nodeId != -1) 
				{
					validNodeIds.push_back(nodeId);
				}
			}
		}
		
		if (validNodeIds.size() == 2)
		{
			AddConnection(validNodeIds[0], validNodeIds[1]);
		}
		else if (validNodeIds.size() == 3)
		{
			AddConnection(validNodeIds[0], validNodeIds[1]);
			AddConnection(validNodeIds[1], validNodeIds[2]);
			AddConnection(validNodeIds[2], validNodeIds[0]);
		}
	}
	//3. Set the connections cost to the actual distance
	for (const auto& pConnection : GetConnections())
	{
		if (pConnection == nullptr || pConnection->GetWeight() == 0) continue;
		
		auto pNodeFrom = GetNode(pConnection->GetFromId()).get();
		auto pNodeTo = GetNode(pConnection->GetToId()).get();
		
		float distance = FVector2D::Distance(pNodeFrom->GetPosition(), pNodeTo->GetPosition());
		pConnection->SetWeight(distance);
	}
}
