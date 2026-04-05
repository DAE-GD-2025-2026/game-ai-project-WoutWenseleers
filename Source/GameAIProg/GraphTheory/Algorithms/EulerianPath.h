#pragma once
#include <stack>

#include "AudioMixerBlueprintLibrary.h"
#include "Shared/Graph/Graph.h"

namespace GameAI
{
	enum class Eulerianity
	{
		notEulerian,
		semiEulerian,
		eulerian,
	};

	class EulerianPath final
	{
	public:
		EulerianPath(Graph* const pGraph);

		Eulerianity IsEulerian() const;
		std::vector<Node*> FindPath(Eulerianity& eulerianity) const;

	private:
		void VisitAllNodesDFS(const std::vector<Node*>& pNodes, std::vector<bool>& visited, int startIndex) const;
		bool IsConnected() const;

		Graph* m_pGraph;
	};

	inline EulerianPath::EulerianPath(Graph* const pGraph)
		: m_pGraph(pGraph)
	{
	}

	inline Eulerianity EulerianPath::IsEulerian() const
	{
		std::vector<Node*> nodes = m_pGraph->GetActiveNodes();
		if (nodes.empty())
			return Eulerianity::notEulerian;

		const auto& connections = m_pGraph->GetConnections();
		if (connections.empty())
			return Eulerianity::notEulerian;
		
		int oddDegreeCount = 0;
		for (Node* pNode : nodes)
		{
			int nrConnections{};
			for (int i{}; i < connections.size(); ++i)
			{
				if (connections[i].get()->GetFromId() == pNode->GetId())
				{
					++nrConnections;
				}
			}
			
			if (nrConnections % 2 != 0)
			{
				++oddDegreeCount;
			}
		}

		if (oddDegreeCount > 2)
			return Eulerianity::notEulerian;

		if (oddDegreeCount == 2)
			return Eulerianity::semiEulerian;

		if (oddDegreeCount == 0)
			return Eulerianity::eulerian;

		return Eulerianity::notEulerian;
	}

	inline std::vector<Node*> EulerianPath::FindPath(Eulerianity& eulerianity) const
	{
		// Get a copy of the graph because this algorithm involves removing edges
		Graph graphCopy = m_pGraph->Clone();
		std::vector<Node*> Path = {};
		std::vector<Node*> Nodes = graphCopy.GetActiveNodes();
		int currentNodeId{ Graphs::InvalidNodeId };

		switch (eulerianity)
		{
		case Eulerianity::notEulerian:
			return Path;
			break;
		case Eulerianity::eulerian:
			currentNodeId = Nodes[0]->GetId();
			break;
		case Eulerianity::semiEulerian:
			for (Node* pNode: Nodes)
			{
				if (graphCopy.FindConnectionsFrom(pNode->GetId()).size() % 2 != 0)
				{
					currentNodeId = pNode->GetId();
					break;
				}
			}
			break;
		}
		// TODO Check if there can be an Euler path
		// TODO If this graph is not eulerian, return the empty path
		
		// TODO Start algorithm loop
		std::stack<int> nodeStack{};
		while (graphCopy.GetConnections().size() > 0)
		{
			if (!graphCopy.FindConnectionsFrom(currentNodeId).empty())
			{
				nodeStack.push(currentNodeId);
				int toId{ graphCopy.FindConnectionsFrom(currentNodeId)[0]->GetToId() };
				graphCopy.RemoveConnection(currentNodeId, toId);
				currentNodeId = toId;
			}
			else
			{
				return Path;
			}
		}
		
		while (!nodeStack.empty())
		{
			Path.push_back(m_pGraph->GetNode(nodeStack.top()).get());
			nodeStack.pop();
		}
		Path.push_back(m_pGraph->GetNode(currentNodeId).get());

		std::reverse(Path.begin(), Path.end());
		return Path;
	}

	inline void EulerianPath::VisitAllNodesDFS(const std::vector<Node*>& Nodes, std::vector<bool>& visited, int startIndex ) const
	{
		// TODO Mark the visited node

		// TODO Ask the graph for the connections from that node
		// TODO recursively visit any valid connected nodes that were not visited before
		// TODO Tip: use an index-based for-loop to find the correct index
		visited[startIndex] = true;

		std::vector<Connection*> connectedNodes = m_pGraph->FindConnectionsFrom(Nodes[startIndex]->GetId());

		for (Connection* pConnection: connectedNodes)
		{
			for (int i{}; i< Nodes.size(); ++i)
			{
				if (pConnection->GetToId() == Nodes[i]->GetId())
				{
					if (visited[i] == false)
					{
						VisitAllNodesDFS(Nodes, visited, i);
					}
				}
			}
		}
	}

	inline bool EulerianPath::IsConnected() const
	{
		std::vector<Node*> Nodes = m_pGraph->GetActiveNodes();
		if (Nodes.size() == 0)
			return false;

		std::vector<bool> visited(Nodes.size(), false);
		
		int startIndex = -1;
		for (int i = 0; i < Nodes.size(); ++i)
		{
			if (!m_pGraph->FindConnectionsFrom(Nodes[i]->GetId()).empty())
			{
				startIndex = i;
				break;
			}
		}
		
		if (startIndex == -1)
			return false;

		VisitAllNodesDFS(Nodes, visited, startIndex);
		
		for (int i = 0; i < Nodes.size(); ++i)
		{
			const bool hasConnections = !m_pGraph->FindConnectionsFrom(Nodes[i]->GetId()).empty();
			if (hasConnections && !visited[i])
			{
				return false;
			}
		}

		return true;
	}
}