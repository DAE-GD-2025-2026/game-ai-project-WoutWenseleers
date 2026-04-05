#include "BFS.h"

#include <map>
#include <vector>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>

#include "GeometryCollection/Facades/CollectionConnectionGraphFacade.h"
#include "Shared/Graph/Graph.h"

using namespace GameAI;

BFS::BFS(Graph* const pGraph)
	: pGraph(pGraph)
{
}

// TODO Breath First Search Algorithm searches for a path from the startNode to the destinationNode
std::vector<Node*> BFS::FindPath(Node* const pStartNode, Node* const pDestinationNode) const
{
	std::vector<Node*> path;
	
	if (pStartNode == nullptr || pDestinationNode == nullptr) 
	{
		return path;
	}

	std::queue<Node*> queue;
	std::unordered_set<Node*> visited;
	std::unordered_map<Node*, Node*> parent;
	
	queue.push(pStartNode);
	visited.insert(pStartNode);

	bool pathFound = false;
	
	while (!queue.empty()) 
	{
		Node* current = queue.front();
		queue.pop();
		
		if (current == pDestinationNode) 
		{
			pathFound = true;
			break;
		}
		
		for (const auto& connection : pGraph->GetConnections()) 
		{
			if (connection.get()->GetFromId() == current->GetId())
			{
				Node* neighbor = pGraph->GetNode(connection.get()->GetToId()).get();
				if (visited.find(neighbor) == visited.end()) 
				{
					visited.insert(neighbor);
					parent[neighbor] = current;
					queue.push(neighbor);
				}
			}

		}
	}
	
	if (pathFound) 
	{
		Node* current = pDestinationNode;
		while (current != pStartNode) 
		{
			path.push_back(current);
			current = parent[current];
		}
		path.push_back(pStartNode);
		
		std::reverse(path.begin(), path.end());
	}

	return path;
}
