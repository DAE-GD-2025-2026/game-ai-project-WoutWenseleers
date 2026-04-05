#include "NavGraphPathfinding.h"

#include "AStar.h"
#include "PathSmoothing.h"
#include "VectorTypes.h"
#include "Shared/Graph/NavGraph/NavGraph.h"
#include "Shared/Graph/NavGraph/NavGraphNode.h"
#include "Util/IndexUtil.h"

using namespace GameAI;

std::vector<FVector2D> NavMeshPathfinding::FindPath(const FVector2D& startPos, const FVector2D& endPos,
	NavGraph* const pNavGraph, std::vector<FVector2D>& debugNodePositions, std::vector<NavLine>& debugPortals) 
{
	//Create the path to return
	std::vector<FVector2D> finalPath{};

	//Get the start and endTriangle
	const TriPolygon::Triangle* startTriangle = pNavGraph->GetNavPolygon()->GetTriangleAtPosition(startPos, true);
	const TriPolygon::Triangle* endTriangle = pNavGraph->GetNavPolygon()->GetTriangleAtPosition(endPos, true);
	//We have valid start/end triangles and they are not the same
	if (startTriangle == nullptr || endTriangle == nullptr)
	{
		return finalPath;
	}
	if (*startTriangle == *endTriangle)
	{
		finalPath.push_back(startPos);
		finalPath.push_back(endPos);
		return finalPath;
	}
	//=> Start looking for a path
	//Copy the graph
	std::shared_ptr<NavGraph> clonedGraph = pNavGraph->Clone();
	
	//Create Extra node for the Start Node (Agent's position
	auto pStartNode = std::make_unique<NavGraphNode>(startPos, -1);
	int startNodeId = clonedGraph->AddNode(std::move(pStartNode));

	std::array<TriPolygon::Edge, 3> startEdges = startTriangle->GetEdges();
	for (const TriPolygon::Edge& edge : startEdges)
	{

		if (auto edgeIdxOpt = pNavGraph->GetNavPolygon()->FindEdgeIndex(edge); edgeIdxOpt.has_value())
		{
			int nodeId = clonedGraph->GetNodeIdFromEdgeIndex(edgeIdxOpt.value());
			if (nodeId != -1)
			{
				auto pEdgeNode = clonedGraph->GetNode(nodeId).get();
				float cost = FVector2D::Distance(startPos, pEdgeNode->GetPosition());
				
				auto connection = std::make_unique<Connection>(startNodeId, nodeId);
				connection->SetWeight(cost);
				clonedGraph->AddConnection(std::move(connection));
			}
		}
	}
	
	
	//Create extra node for the endNode
	auto pEndNode = std::make_unique<NavGraphNode>(endPos, -1);
	int endNodeId = clonedGraph->AddNode(std::move(pEndNode));

	std::array<TriPolygon::Edge, 3> endEdges = endTriangle->GetEdges();
	for (const TriPolygon::Edge& edge : endEdges)
	{
		if (auto edgeIdxOpt = pNavGraph->GetNavPolygon()->FindEdgeIndex(edge); edgeIdxOpt.has_value())
		{
			int nodeId = clonedGraph->GetNodeIdFromEdgeIndex(edgeIdxOpt.value());
			if (nodeId != -1)
			{
				auto pEdgeNode = clonedGraph->GetNode(nodeId).get();
				float cost = FVector2D::Distance(endPos, pEdgeNode->GetPosition());
				
				auto connection = std::make_unique<Connection>(nodeId, endNodeId);
				connection->SetWeight(cost);
				clonedGraph->AddConnection(std::move(connection));
			}
		}
	}
	//Run A star on new graph
	AStar pathfinder = AStar(clonedGraph.get(), HeuristicFunctions::Euclidean);
	std::vector<Node*> pathNodes = pathfinder.FindPath(clonedGraph->GetNode(startNodeId).get(), clonedGraph->GetNode(endNodeId).get());

	//Debug Visualisation
	for (const Node* pNode : pathNodes)
	{
		if (pNode != nullptr)
		{
			finalPath.push_back(pNode->GetPosition());
			debugNodePositions.push_back(pNode->GetPosition());
		}
	}
	// Extra: Run optimiser on new graph (First check if everything works without SSFA!)
	debugPortals = SSFA::FindPortals(pathNodes, *pNavGraph->GetNavPolygon());
	finalPath = SSFA::OptimizePortals(debugPortals, *pNavGraph->GetNavPolygon());
	
	return finalPath;
}

std::vector<FVector2D> NavMeshPathfinding::FindPath(const FVector2D& startPos, const FVector2D& endPos, NavGraph* const pNavGraph)
{
	std::vector<FVector2D> debugNodePositions{};
	std::vector<NavLine> debugPortals{};

	return FindPath(startPos, endPos, pNavGraph, debugNodePositions, debugPortals);
}