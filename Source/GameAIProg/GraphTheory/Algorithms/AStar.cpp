#include "AStar.h"

using namespace GameAI;
struct NodeRecord
{
	Node* pNode = nullptr;
	Connection* pConnection = nullptr;
	float costSoFar = 0.0f; // G-Cost
	float estimatedTotalCost = 0.0f; // F-Cost
	
	bool operator<(const NodeRecord& other) const
	{
		return estimatedTotalCost < other.estimatedTotalCost;
	}
};

AStar::AStar(Graph* const pGraph, HeuristicFunctions::Heuristic hFunction)
	: pGraph(pGraph)
	, HeuristicFunction(hFunction)
{
}

std::vector<Node*>AStar::FindPath(Node* const pStartNode, Node* const pGoalNode)
{
    std::vector<Node*> path{};

    if (pStartNode == nullptr || pGoalNode == nullptr)
    {
        return path;
    }
    // 1. Kickstart the loop
    std::vector<NodeRecord> openList;
    std::vector<NodeRecord> closedList;
    NodeRecord currentNodeRecord;

    NodeRecord startRecord;
    startRecord.pNode = pStartNode;
    startRecord.pConnection = nullptr;
    startRecord.costSoFar = 0.0f;
    startRecord.estimatedTotalCost = GetHeuristicCost(pStartNode, pGoalNode);

    openList.push_back(startRecord);

    // 2. The While Loop (part 1)
    while (!openList.empty())
    {
        // A. Get record from the open list with lowest F-score
        currentNodeRecord = *std::min_element(openList.begin(), openList.end());

        // B. Check if that record refers to the end node
        if (currentNodeRecord.pNode == pGoalNode)
        {
            break; 
        }

        // C. Else, we get all the connections of the NodeRecord’s node
        for (const auto& pConnection : pGraph->GetConnections())
        {
            if (pConnection.get()->GetFromId() != currentNodeRecord.pNode->GetId())
            {
                continue;
            }

            Node* pNextNode = pGraph->GetNode(pConnection->GetToId()).get();

            float gCost = currentNodeRecord.costSoFar + pConnection.get()->GetWeight();

            // D. Check if the connection leads to a node already on the closedlist 
            auto closedIt = std::find_if(closedList.begin(), closedList.end(),
                [pNextNode](const NodeRecord& r) { return r.pNode == pNextNode; });

            if (closedIt != closedList.end())
            {
                if (closedIt->costSoFar <= gCost)
                {
                    continue;
                }
                else
                {
                    closedList.erase(closedIt);
                }
            }

            // E. Check if the connection leads to a node already on the openlist
            auto openIt = std::find_if(openList.begin(), openList.end(),
                [pNextNode](const NodeRecord& r) { return r.pNode == pNextNode; });

            if (openIt != openList.end())
            {
                if (openIt->costSoFar <= gCost)
                {
                    continue;
                }
                else
                {
                    openList.erase(openIt);
                }
            }

            // F. At this point any expensive connection should be removed (if it existed). We create a new NodeRecord and add it to the openList.

            NodeRecord newRecord;
            newRecord.pNode = pNextNode;
            newRecord.pConnection = pConnection.get();
            newRecord.costSoFar = gCost;
            newRecord.estimatedTotalCost = gCost + GetHeuristicCost(pNextNode, pGoalNode);
            
            openList.push_back(newRecord);
        }

        // G. remove the currentNodeRecord from the openList and add it to the closedList
        openList.erase(std::find_if(openList.begin(), openList.end(),
            [&currentNodeRecord](const NodeRecord& r) { return r.pNode == currentNodeRecord.pNode; }));
        
        closedList.push_back(currentNodeRecord);
    }

    // 3. Backtracking
    // 3. Reconstruct path from last connection to start node
    while (currentNodeRecord.pNode != pStartNode)
    {
        path.push_back(currentNodeRecord.pNode);
        
        Node* pParentNode = pGraph->GetNode(currentNodeRecord.pConnection->GetFromId()).get();

        auto parentIt = std::find_if(closedList.begin(), closedList.end(),
            [pParentNode](const NodeRecord& r) { return r.pNode == pParentNode; });

        if (parentIt != closedList.end())
        {
            currentNodeRecord = *parentIt;
        }
    }
    
    path.push_back(pStartNode);
    
    std::reverse(path.begin(), path.end());

    return path;
}

float AStar::GetHeuristicCost(Node* const pStartNode, Node* const pEndNode) const
{
    if (!pStartNode || !pEndNode) return 0.0f;
	FVector2D toDestination = pGraph->GetNode(pEndNode->GetId())->GetPosition() - pGraph->GetNode(pStartNode->GetId())->GetPosition();
	return HeuristicFunction(abs(toDestination.X), abs(toDestination.Y));
}