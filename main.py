from dataclasses import dataclass
from typing import Set,List

import heapq

@dataclass
class CTNode:
    constraints: Set
    solutions: List
    cost: float
    conflicts: int

    def __lt__(self,other):
        return self.conflicts < other.conflicts if self.cost != other.cost else self.cost < other.cost


def highlevel():
    OPEN = []
    
    root - CTNode()
    
    heapq.heappush(root)
