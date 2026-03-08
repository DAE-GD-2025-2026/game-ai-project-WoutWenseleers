#include "SpacePartitioning.h"

// --- Cell ---
// ------------
Cell::Cell(float Left, float Bottom, float Width, float Height)
{
	BoundingBox.Min = { Left, Bottom };
	BoundingBox.Max = { BoundingBox.Min.X + Width, BoundingBox.Min.Y + Height };
}

std::vector<FVector2D> Cell::GetRectPoints() const
{
	const float left = BoundingBox.Min.X;
	const float bottom = BoundingBox.Min.Y;
	const float width = BoundingBox.Max.X - BoundingBox.Min.X;
	const float height = BoundingBox.Max.Y - BoundingBox.Min.Y;

	std::vector<FVector2D> rectPoints =
	{
		{ left , bottom  },
		{ left , bottom + height  },
		{ left + width , bottom + height },
		{ left + width , bottom  },
	};

	return rectPoints;
}

// --- Partitioned Space ---
// -------------------------
CellSpace::CellSpace(UWorld* pWorld, float Width, float Height, int Rows, int Cols, int MaxEntities)
	: pWorld{pWorld}
	, SpaceWidth{Width}
	, SpaceHeight{Height}
	, NrOfRows{Rows}
	, NrOfCols{Cols}
	, NrOfNeighbors{0}
{
	Neighbors.Init(nullptr, MaxEntities);

	// calculate bounds of a cell
	CellWidth = Width / Cols;
	CellHeight = Height / Rows;
	
	CellOrigin = FVector2D{ -Width * 0.5f, -Height * 0.5f };

	Cells.reserve(Rows * Cols);

	for (int row = 0; row < NrOfRows; ++row)
	{
		for (int col = 0; col < NrOfCols; ++col)
		{
			const float left = CellOrigin.X + col * CellWidth;
			const float bottom = CellOrigin.Y + row * CellHeight;
			Cells.emplace_back(left, bottom, CellWidth, CellHeight);
		}
	}
}

void CellSpace::AddAgent(ASteeringAgent& Agent)
{
	const int idx = PositionToIndex(Agent.GetPosition());
	Cells[idx].Agents.push_back(&Agent);
}

void CellSpace::UpdateAgentCell(ASteeringAgent& Agent, const FVector2D& OldPos)
{
	const int oldIdx = PositionToIndex(OldPos);
	const int newIdx = PositionToIndex(Agent.GetPosition());

	if (oldIdx == newIdx)
		return;

	Cells[oldIdx].Agents.remove(&Agent);
	Cells[newIdx].Agents.push_back(&Agent);
}

void CellSpace::RegisterNeighbors(ASteeringAgent& Agent, float QueryRadius)
{
	NrOfNeighbors = 0;

	const FVector2D pos = Agent.GetPosition();

	FRect queryBox;
	queryBox.Min = { pos.X - QueryRadius, pos.Y - QueryRadius };
	queryBox.Max = { pos.X + QueryRadius, pos.Y + QueryRadius };

	for (const Cell& cell : Cells)
	{
		if (!DoRectsOverlap(queryBox, cell.BoundingBox))
			continue;

		for (ASteeringAgent* pOther : cell.Agents)
		{
			if (!pOther) continue;
			if (pOther == &Agent) continue;

			const FVector2D diff = pos - pOther->GetPosition();
			if (diff.Length() <= QueryRadius)
			{
				Neighbors[NrOfNeighbors++] = pOther;
			}
		}
	}
}

void CellSpace::EmptyCells()
{
	for (Cell& c : Cells)
		c.Agents.clear();
}

void CellSpace::RenderCells() const
{
	if (!pWorld) return;

	for (const Cell& cell : Cells)
	{
		const auto rect = cell.GetRectPoints();

		DrawDebugLine(pWorld, FVector(rect[0], 5.f), FVector(rect[1], 5.f), FColor::Green, false, -1.f, 0, 1.f);
		DrawDebugLine(pWorld, FVector(rect[1], 5.f), FVector(rect[2], 5.f), FColor::Green, false, -1.f, 0, 1.f);
		DrawDebugLine(pWorld, FVector(rect[2], 5.f), FVector(rect[3], 5.f), FColor::Green, false, -1.f, 0, 1.f);
		DrawDebugLine(pWorld, FVector(rect[3], 5.f), FVector(rect[0], 5.f), FColor::Green, false, -1.f, 0, 1.f);

		const FVector2D center = (cell.BoundingBox.Min + cell.BoundingBox.Max) * 0.5f;
		DrawDebugString(
			pWorld,
			FVector(center, 10.f),
			FString::FromInt(static_cast<int>(cell.Agents.size())),
			nullptr,
			FColor::White,
			0.f,
			true
		);
	}
}

int CellSpace::PositionToIndex(FVector2D const & Pos) const
{
	const float localX = Pos.X - CellOrigin.X;
	const float localY = Pos.Y - CellOrigin.Y;

	int col = FMath::FloorToInt(localX / CellWidth);
	int row = FMath::FloorToInt(localY / CellHeight);

	col = FMath::Clamp(col, 0, NrOfCols - 1);
	row = FMath::Clamp(row, 0, NrOfRows - 1);

	return row * NrOfCols + col;
}

bool CellSpace::DoRectsOverlap(FRect const & RectA, FRect const & RectB)
{
	// Check if the rectangles are separated on either axis
	if (RectA.Max.X < RectB.Min.X || RectA.Min.X > RectB.Max.X) return false;
	if (RectA.Max.Y < RectB.Min.Y || RectA.Min.Y > RectB.Max.Y) return false;
    
	// If they are not separated, they must overlap
	return true;
}