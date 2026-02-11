"""
MA-CBS (Meta-Agent Conflict-Based Search) Implementation
Based on: Sharon et al., "Conflict-based search for optimal multi-agent pathfinding"
          Artificial Intelligence 219 (2015) 40-66

Low-level solver: A* with SIC heuristic
Cost function: Sum of Costs
Graph: NetworkX DiGraph with unit-weight edges, integer-labeled nodes
"""

import heapq
import itertools
from copy import deepcopy
from collections import defaultdict
from typing import Dict, List, Tuple, Set, Optional, FrozenSet

import networkx as nx


# ---------------------------------------------------------------------------
# Heuristic Precomputation (Reverse Dijkstra / BFS for unit weights)
# ---------------------------------------------------------------------------

def precompute_heuristics(graph: nx.DiGraph, goals: Tuple[int, ...]) -> Dict[int, Dict[int, float]]:
    """
    For each unique goal node, compute shortest distances from ALL nodes to that goal.
    Uses reverse graph + Dijkstra (BFS-equivalent for unit weights).
    Returns: {goal_node: {node: distance, ...}, ...}
    """
    reverse_graph = graph.reverse()
    h_values = {}
    for goal in set(goals):
        if goal in h_values:
            continue
        # Dijkstra on reverse graph from goal gives shortest dist from every node to goal
        lengths = nx.single_source_dijkstra_path_length(reverse_graph, goal, weight='weight')
        h_values[goal] = dict(lengths)
    return h_values


# ---------------------------------------------------------------------------
# Data Structures
# ---------------------------------------------------------------------------

class Constraint:
    """
    A vertex constraint: (agent, vertex, timestep)
    Agent `agent` is prohibited from occupying `vertex` at `timestep`.
    """
    __slots__ = ('agent', 'vertex', 'timestep')

    def __init__(self, agent: int, vertex: int, timestep: int):
        self.agent = agent
        self.vertex = vertex
        self.timestep = timestep

    def __eq__(self, other):
        return (self.agent == other.agent and self.vertex == other.vertex
                and self.timestep == other.timestep)

    def __hash__(self):
        return hash((self.agent, self.vertex, self.timestep))

    def __repr__(self):
        return f"Constraint(a{self.agent}, v{self.vertex}, t{self.timestep})"


class EdgeConstraint:
    """
    An edge constraint: (agent, v1, v2, timestep)
    Agent `agent` cannot move from v1 to v2 starting at `timestep`.
    """
    __slots__ = ('agent', 'v1', 'v2', 'timestep')

    def __init__(self, agent: int, v1: int, v2: int, timestep: int):
        self.agent = agent
        self.v1 = v1
        self.v2 = v2
        self.timestep = timestep

    def __eq__(self, other):
        return (self.agent == other.agent and self.v1 == other.v1
                and self.v2 == other.v2 and self.timestep == other.timestep)

    def __hash__(self):
        return hash((self.agent, self.v1, self.v2, self.timestep))

    def __repr__(self):
        return f"EdgeConstraint(a{self.agent}, {self.v1}->{self.v2}, t{self.timestep})"


class Conflict:
    """Represents a conflict between two agents."""
    VERTEX = 'vertex'
    EDGE = 'edge'

    def __init__(self, agent1: int, agent2: int, vertex: int, timestep: int,
                 conflict_type: str = 'vertex', vertex2: int = None):
        self.agent1 = agent1
        self.agent2 = agent2
        self.vertex = vertex
        self.timestep = timestep
        self.conflict_type = conflict_type
        self.vertex2 = vertex2  # For edge conflicts: agent1 moves v->v2, agent2 moves v2->v

    def __repr__(self):
        if self.conflict_type == self.VERTEX:
            return f"Conflict(a{self.agent1}, a{self.agent2}, v{self.vertex}, t{self.timestep})"
        else:
            return (f"EdgeConflict(a{self.agent1}, a{self.agent2}, "
                    f"{self.vertex}<->{self.vertex2}, t{self.timestep})")


class MetaAgent:
    """
    Represents a meta-agent: a group of individual agents solved together.
    A single agent is a meta-agent of size 1.
    """

    def __init__(self, agents: FrozenSet[int]):
        self.agents = agents  # frozenset of individual agent ids

    @property
    def size(self):
        return len(self.agents)

    def __contains__(self, agent_id):
        return agent_id in self.agents

    def __eq__(self, other):
        return self.agents == other.agents

    def __hash__(self):
        return hash(self.agents)

    def __repr__(self):
        return f"MetaAgent({sorted(self.agents)})"


class CTNode:
    """A node in the Constraint Tree (CT)."""

    _id_counter = 0

    def __init__(self):
        CTNode._id_counter += 1
        self.id = CTNode._id_counter

        # Constraints per individual agent: {agent_id: set of Constraint/EdgeConstraint}
        self.constraints: Dict[int, Set] = defaultdict(set)

        # Solution: {agent_id: [path as list of nodes]}
        self.solution: Dict[int, List[int]] = {}

        # Cost = sum of individual path costs (sum-of-costs)
        self.cost: float = float('inf')

        # Meta-agents: list of MetaAgent objects
        self.meta_agents: List[MetaAgent] = []

        # Meta-agent constraints: for merged agents, track which sub-agents
        # are constrained. {meta_agent: [(sub_agents_frozenset, vertex, timestep), ...]}
        self.meta_constraints: Dict[FrozenSet[int], List] = defaultdict(list)

    def __lt__(self, other):
        # For heap: lower cost first, then fewer conflicts
        return (self.cost, self.id) < (other.cost, other.id)

    def count_conflicts(self) -> int:
        """Count number of conflicts in current solution."""
        conflicts = find_all_conflicts(self.solution)
        return len(conflicts)


# ---------------------------------------------------------------------------
# Conflict Detection
# ---------------------------------------------------------------------------

def get_location(path: List[int], t: int) -> int:
    """Get agent location at time t. If t >= len(path), agent stays at goal."""
    if t < len(path):
        return path[t]
    return path[-1]


def find_first_conflict(solution: Dict[int, List[int]]) -> Optional[Conflict]:
    """Find the first conflict in the solution. Returns None if no conflict."""
    agents = sorted(solution.keys())
    max_t = max(len(p) for p in solution.values())

    for t in range(max_t):
        # Check vertex conflicts
        locations = {}
        for a in agents:
            loc = get_location(solution[a], t)
            if loc in locations:
                return Conflict(locations[loc], a, loc, t, Conflict.VERTEX)
            locations[loc] = a

        # Check edge conflicts (swap conflicts)
        if t > 0:
            for i in range(len(agents)):
                for j in range(i + 1, len(agents)):
                    ai, aj = agents[i], agents[j]
                    prev_i = get_location(solution[ai], t - 1)
                    curr_i = get_location(solution[ai], t)
                    prev_j = get_location(solution[aj], t - 1)
                    curr_j = get_location(solution[aj], t)
                    # Swap: ai goes from prev_i to curr_i, aj goes from prev_j to curr_j
                    if prev_i == curr_j and prev_j == curr_i and prev_i != curr_i:
                        return Conflict(ai, aj, prev_i, t - 1, Conflict.EDGE, curr_i)
    return None


def find_all_conflicts(solution: Dict[int, List[int]]) -> List[Conflict]:
    """Find all conflicts in the solution."""
    conflicts = []
    agents = sorted(solution.keys())
    max_t = max(len(p) for p in solution.values())

    for t in range(max_t):
        locations = {}
        for a in agents:
            loc = get_location(solution[a], t)
            if loc in locations:
                conflicts.append(Conflict(locations[loc], a, loc, t, Conflict.VERTEX))
            else:
                locations[loc] = a

        if t > 0:
            for i in range(len(agents)):
                for j in range(i + 1, len(agents)):
                    ai, aj = agents[i], agents[j]
                    prev_i = get_location(solution[ai], t - 1)
                    curr_i = get_location(solution[ai], t)
                    prev_j = get_location(solution[aj], t - 1)
                    curr_j = get_location(solution[aj], t)
                    if prev_i == curr_j and prev_j == curr_i and prev_i != curr_i:
                        conflicts.append(
                            Conflict(ai, aj, prev_i, t - 1, Conflict.EDGE, curr_i))
    return conflicts


def count_pairwise_conflicts(solution: Dict[int, List[int]], a1: int, a2: int) -> int:
    """Count conflicts between two specific agents."""
    count = 0
    path1, path2 = solution[a1], solution[a2]
    max_t = max(len(path1), len(path2))

    for t in range(max_t):
        loc1 = get_location(path1, t)
        loc2 = get_location(path2, t)
        if loc1 == loc2:
            count += 1
        if t > 0:
            prev1 = get_location(path1, t - 1)
            prev2 = get_location(path2, t - 1)
            if prev1 == loc2 and prev2 == loc1 and prev1 != loc1:
                count += 1
    return count


# ---------------------------------------------------------------------------
# Conflict Avoidance Table (CAT) for tie-breaking
# ---------------------------------------------------------------------------

def build_cat(solution: Dict[int, List[int]], exclude_agent: int) -> Dict[Tuple[int, int], int]:
    """
    Build a Conflict Avoidance Table.
    Returns: {(vertex, timestep): count_of_agents_there}
    """
    cat = defaultdict(int)
    for a, path in solution.items():
        if a == exclude_agent:
            continue
        max_t = len(path)
        # Also extend to some future time steps for agents at goal
        extended_max = max_t + 50
        for t in range(extended_max):
            loc = get_location(path, t)
            cat[(loc, t)] += 1
    return dict(cat)


def build_cat_multi(solution: Dict[int, List[int]],
                    exclude_agents: FrozenSet[int]) -> Dict[Tuple[int, int], int]:
    """Build CAT excluding a set of agents (for meta-agent searches)."""
    cat = defaultdict(int)
    for a, path in solution.items():
        if a in exclude_agents:
            continue
        max_t = len(path)
        extended_max = max_t + 50
        for t in range(extended_max):
            loc = get_location(path, t)
            cat[(loc, t)] += 1
    return dict(cat)


# ---------------------------------------------------------------------------
# Low-Level: A* for Single Agent with Constraints
# ---------------------------------------------------------------------------

def low_level_a_star(graph: nx.DiGraph,
                     start: int,
                     goal: int,
                     h_values: Dict[int, float],
                     constraints: Set,
                     cat: Dict[Tuple[int, int], int] = None,
                     max_time: int = None) -> Optional[List[int]]:
    """
    A* search for a single agent from start to goal, respecting constraints.
    Uses precomputed h_values (shortest distance to goal ignoring other agents).

    Args:
        graph: The underlying DiGraph
        start: Start node
        goal: Goal node
        h_values: {node: distance_to_goal}
        constraints: Set of Constraint and EdgeConstraint objects for this agent
        cat: Conflict Avoidance Table for tie-breaking
        max_time: Maximum timestep to search up to

    Returns:
        Path as list of nodes, or None if no path found
    """
    if max_time is None:
        # Reasonable upper bound
        max_time = len(graph.nodes) * 3

    # Build constraint lookup tables for fast access
    vertex_constraints = set()  # (vertex, timestep)
    edge_constraints = set()    # (v1, v2, timestep)
    for c in constraints:
        if isinstance(c, Constraint):
            vertex_constraints.add((c.vertex, c.timestep))
        elif isinstance(c, EdgeConstraint):
            edge_constraints.add((c.v1, c.v2, c.timestep))

    # State: (vertex, timestep)
    h_start = h_values.get(start, float('inf'))
    if h_start == float('inf'):
        return None  # Goal unreachable

    # Priority queue: (f, cat_conflicts, counter, vertex, timestep)
    counter = itertools.count()
    open_list = []
    start_cat = cat.get((start, 0), 0) if cat else 0
    heapq.heappush(open_list, (h_start, start_cat, next(counter), start, 0))

    # closed: {(vertex, timestep): g_value}
    closed = {}
    # parent: {(vertex, timestep): (parent_vertex, parent_timestep)}
    parent = {}

    g_values = {(start, 0): 0}

    while open_list:
        f, cat_val, _, v, t = heapq.heappop(open_list)

        if v == goal and (v, t) not in vertex_constraints:
            # Check if agent can stay at goal from time t onwards without constraint violations
            can_stay = True
            for ct in range(t, t + max_time - t):
                if (goal, ct) in vertex_constraints:
                    can_stay = False
                    break
            # Actually just check up to a reasonable horizon
            # For sum-of-costs, once at goal we count 0 extra cost if staying
            # So we just need the agent to reach goal; they stay there
            # But if there's a future constraint at goal, agent might need to leave & return
            # For simplicity: check constraints at goal for future times
            future_blocked = False
            for cc in constraints:
                if isinstance(cc, Constraint) and cc.vertex == goal and cc.timestep >= t:
                    future_blocked = True
                    break
            if not future_blocked:
                # Reconstruct path
                path = []
                state = (v, t)
                while state in parent:
                    path.append(state[0])
                    state = parent[state]
                path.append(state[0])
                path.reverse()
                return path

        if (v, t) in closed:
            continue
        closed[(v, t)] = g_values.get((v, t), float('inf'))

        if t >= max_time:
            continue

        # Generate successors: move to neighbors + wait
        successors = []
        for neighbor in graph.successors(v):
            successors.append(neighbor)
        successors.append(v)  # Wait action

        for next_v in successors:
            next_t = t + 1

            # Check vertex constraint
            if (next_v, next_t) in vertex_constraints:
                continue

            # Check edge constraint
            if next_v != v:  # move action
                if (v, next_v, t) in edge_constraints:
                    continue

            # g-cost: if at goal and waiting, cost is 0 for sum-of-costs
            # Actually for sum-of-costs: cost counts until agent reaches goal for last time
            # The low-level just finds shortest path; the cost accounting is at high level
            new_g = g_values[(v, t)] + 1

            state = (next_v, next_t)
            if state in closed:
                continue
            if state in g_values and g_values[state] <= new_g:
                continue

            g_values[state] = new_g
            parent[state] = (v, t)
            h = h_values.get(next_v, float('inf'))
            if h == float('inf'):
                continue
            f_val = new_g + h
            cat_conflicts = cat.get((next_v, next_t), 0) if cat else 0
            heapq.heappush(open_list, (f_val, cat_conflicts, next(counter), next_v, next_t))

    return None  # No path found


# ---------------------------------------------------------------------------
# Low-Level: A* for Meta-Agent (Joint A* for group of agents)
# ---------------------------------------------------------------------------

def low_level_joint_a_star(graph: nx.DiGraph,
                           agents: List[int],
                           starts: Dict[int, int],
                           goals: Dict[int, int],
                           h_values_all: Dict[int, Dict[int, float]],
                           constraints: Dict[int, Set],
                           meta_constraints_list: List,
                           cat: Dict[Tuple[int, int], int] = None,
                           max_time: int = None) -> Optional[Dict[int, List[int]]]:
    """
    Joint A* for a group of agents (meta-agent).
    State: tuple of (location_agent_0, location_agent_1, ...) at timestep t.

    Args:
        graph: The underlying DiGraph
        agents: List of agent IDs in the meta-agent (sorted)
        starts: {agent_id: start_node}
        goals: {agent_id: goal_node}
        h_values_all: {goal_node: {node: dist}}
        constraints: {agent_id: set of constraints}
        meta_constraints_list: list of (sub_agents_frozenset, vertex, timestep) meta constraints
        cat: Conflict Avoidance Table (excluding these agents)
        max_time: Maximum search time

    Returns:
        {agent_id: path} or None
    """
    if max_time is None:
        max_time = len(graph.nodes) * 3

    k = len(agents)
    agents = sorted(agents)

    # Build per-agent constraint lookups
    agent_vertex_constraints = {}
    agent_edge_constraints = {}
    for a in agents:
        vc = set()
        ec = set()
        for c in constraints.get(a, set()):
            if isinstance(c, Constraint):
                vc.add((c.vertex, c.timestep))
            elif isinstance(c, EdgeConstraint):
                ec.add((c.v1, c.v2, c.timestep))
        agent_vertex_constraints[a] = vc
        agent_edge_constraints[a] = ec

    # Meta constraints: (sub_agents_frozenset, vertex, timestep)
    # sub_agents are the ones prohibited from vertex at timestep
    meta_con_lookup = set()
    for mc in meta_constraints_list:
        sub_agents, vertex, timestep = mc
        for sa in sub_agents:
            if sa in agents:
                meta_con_lookup.add((sa, vertex, timestep))

    def is_constrained(agent, vertex, timestep):
        if (vertex, timestep) in agent_vertex_constraints.get(agent, set()):
            return True
        if (agent, vertex, timestep) in meta_con_lookup:
            return True
        return False

    def is_edge_constrained(agent, v1, v2, timestep):
        return (v1, v2, timestep) in agent_edge_constraints.get(agent, set())

    # Initial state
    start_state = tuple(starts[a] for a in agents)
    goal_state = tuple(goals[a] for a in agents)

    # Heuristic: SIC
    def heuristic(state):
        h = 0
        for i, a in enumerate(agents):
            goal = goals[a]
            h_val = h_values_all[goal].get(state[i], float('inf'))
            if h_val == float('inf'):
                return float('inf')
            h += h_val
        return h

    h_start = heuristic(start_state)
    if h_start == float('inf'):
        return None

    counter = itertools.count()
    # (f, cat_val, counter, state, timestep)
    open_list = []
    heapq.heappush(open_list, (h_start, 0, next(counter), start_state, 0))

    g_values = {(start_state, 0): 0}
    closed = set()
    parent = {}

    def get_successors(state, t):
        """Generate all valid joint moves from current state at time t."""
        # For each agent, compute possible next locations
        agent_moves = []
        for i, a in enumerate(agents):
            v = state[i]
            moves = []
            # Wait
            if not is_constrained(a, v, t + 1):
                moves.append(v)
            # Move to neighbors
            for nb in graph.successors(v):
                if not is_constrained(a, nb, t + 1) and not is_edge_constrained(a, v, nb, t):
                    moves.append(nb)
            if not moves:
                return []  # This agent has no valid move
            agent_moves.append(moves)

        # Generate all combinations (Cartesian product)
        results = []
        for combo in itertools.product(*agent_moves):
            # Check internal conflicts: no two agents at same vertex
            if len(set(combo)) != len(combo):
                continue

            # Check edge conflicts between agents in the group
            edge_ok = True
            for i in range(k):
                for j in range(i + 1, k):
                    # Check swap
                    if state[i] == combo[j] and state[j] == combo[i] and state[i] != combo[i]:
                        edge_ok = False
                        break
                if not edge_ok:
                    break
            if not edge_ok:
                continue

            results.append(combo)
        return results

    while open_list:
        f, _, _, state, t = heapq.heappop(open_list)

        if (state, t) in closed:
            continue
        closed.add((state, t))

        # Goal check: all agents at their goals, no future constraints blocking
        if state == goal_state:
            future_blocked = False
            for i, a in enumerate(agents):
                for c in constraints.get(a, set()):
                    if isinstance(c, Constraint) and c.vertex == goals[a] and c.timestep >= t:
                        future_blocked = True
                        break
                if future_blocked:
                    break
                # Also check meta constraints
                for sa, vertex, timestep in meta_con_lookup:
                    if sa == a and vertex == goals[a] and timestep >= t:
                        future_blocked = True
                        break
                if future_blocked:
                    break

            if not future_blocked:
                # Reconstruct paths
                paths = {a: [] for a in agents}
                s = (state, t)
                while s in parent:
                    for i, a in enumerate(agents):
                        paths[a].append(s[0][i])
                    s = parent[s]
                for i, a in enumerate(agents):
                    paths[a].append(s[0][i])
                for a in agents:
                    paths[a].reverse()
                return paths

        if t >= max_time:
            continue

        for next_state in get_successors(state, t):
            next_t = t + 1
            new_g = g_values[(state, t)] + sum(
                1 for i in range(k)
                # Cost: 1 for every agent not yet at goal or not staying at goal
                # Actually for sum-of-costs low-level: each timestep costs k
                # But we need the sum of individual costs
                # Each agent's cost = timesteps until reaching goal for last time
                # For joint A*: g = sum of costs so far for each agent
            )

            # Actually, let's compute g properly for sum-of-costs
            # g = sum over agents of (number of timesteps so far)
            # But we want the total cost at the end = sum of individual path lengths
            # For joint A*, each step adds k (one timestep per agent)
            # But agents at goal with no future constraints add 0
            # Actually simpler: g = total timesteps elapsed * k is wrong
            # 
            # The correct approach: track individual costs
            # For sum-of-costs: cost of agent = timestep when it arrives at goal (last time)
            # In joint A*: g is the sum of (t+1) for agents not yet settled at goal
            #
            # Simpler approach: just count g = t+1 for each agent and the SIC heuristic 
            # will handle it. At the end, the path length for each agent minus 1 is the cost
            # (since path includes start at t=0).
            # g = sum of (current_timestep) = (t+1) * k... no that's not right either.
            #
            # Let me think again. In standard joint-space A* for sum-of-costs:
            # g(state, t) = t * k  ... no
            # 
            # Actually the simplest: g = number of edges traversed so far by ALL agents combined
            # = (t+1) * k  NO... that counts waits too.
            #
            # For sum-of-costs, the cost of a solution is:
            # sum over agents of (timestep when agent last arrives at goal)
            # Equivalently: sum of path_lengths - k (since path includes start node)
            #
            # In A* for MAPF with SIC heuristic:
            # g = sum of individual g-values = sum of current timestep for each agent
            # h = sum of individual heuristics
            # At goal state at time T: cost = T * k ... no
            #
            # Actually let me simplify. For each agent, path cost = len(path) - 1.
            # In joint A*, we step all agents together. At timestep t:
            # g(t) = t * k (each agent has used t steps)
            # But agents already at goal incur wait costs too...
            # For sum-of-costs: wait at goal after final arrival = 0 cost.
            # So we need to track the latest time each agent is NOT at goal.
            # 
            # The standard approach in joint A* for sum-of-costs:
            # g-value = t * k - (accumulated free waits at goal)
            # This is complex.
            #
            # Simpler: just use g = t (timestep), h = max(individual heuristics) for makespan
            # or g = some function for sum-of-costs.
            #
            # OK let me use the proper formulation:
            # g(state, t) for sum-of-costs = sum_i max(t, h_i*) 
            # where h_i* is optimal cost for agent i... no that doesn't work either.
            #
            # The standard way: each move costs 1, each wait costs 1 EXCEPT
            # waiting at goal costs 0 if agent never leaves goal after.
            # Since we don't know the future, we just compute g = sum of steps per agent.
            # At the end, we trim trailing goal-waits to get actual cost.
            #
            # For A* to work optimally with SIC, we use:
            # g = timestep_count for all agents = sum of (t+1) = (t+1)*k... 
            # No: g should be additive per step.
            # Each joint step: cost_increment = number of agents for which this step is "useful"
            # i.e., agent is not yet at goal, OR agent is at goal but will move again later.
            # Since we don't know the future, the common approach:
            # cost_increment = k (charge 1 per agent per step)
            # And subtract trailing goal-waits when computing final cost.
            #
            # For the A* heuristic to be consistent and correct, the simplest way:
            # g = t * k, h = SIC(state) = sum of h_i(state_i)
            # f = g + h = t*k + sum(h_i)
            # This is admissible because: optimal cost for agent i >= t + h_i,
            # so optimal total cost >= sum(t + h_i) = t*k + sum(h_i) = f. 
            # Wait, but agent i might already be at goal with h_i = 0, 
            # and then its true cost is just t (or less if it arrived earlier).
            # So t*k might overcount for agents already at goal.
            #
            # Better standard approach:
            # For each agent i, track the cost so far: g_i = t (timesteps taken)
            # Total g = sum(g_i) = t * k
            # Total h = sum(h_i(current_location_i))
            # But agents at goal have h_i = 0, and their true remaining cost = 0
            # Their cost so far = t, but their actual contribution to sum-of-costs
            # should be the time they FIRST arrived at goal (permanently).
            # 
            # OK this is getting complicated for joint A*. Let me use a simpler approach
            # that is known to be correct:
            #
            # g(state, t) = sum_i of (t)  = t * k  [overestimates for agents at goal early]
            # This makes g inadmissible (overestimates cost-so-far) which breaks A*.
            #
            # Correct approach: track per-agent costs individually in the state.
            # But that explodes the state space.
            #
            # Actually the simplest CORRECT approach for sum-of-costs joint A*:
            # The cost of each joint step = k (one per agent)
            # g(root) = 0
            # g(child) = g(parent) + k
            # h = SIC(state)
            # When goal is reached at time T, total cost from this formula = T*k
            # But actual sum-of-costs < T*k because agents that arrived early 
            # have lower individual costs.
            #
            # WAIT. Actually, the path itself tells us the cost. The path for agent i
            # is the sequence of locations. Path cost = length_of_path - 1 
            # (but trailing goal waits don't count). So we just find the paths and
            # compute cost post-hoc. For A* we just need the heuristic to be admissible.
            #
            # Use g = t (just the timestep) and h = max(h_i) for makespan.
            # For sum-of-costs: h = sum(h_i) and g = t*k... still the same problem.
            #
            # Let me just use a different g:
            # For each transition, increment g by the number of agents that are NOT at goal
            # in the NEXT state. This correctly charges 0 for agents staying at goal.
            # And SIC as heuristic remains admissible.

            new_g_val = g_values.get((state, t), float('inf'))
            # Cost of this step: for each agent, if they are NOT at their goal in next_state,
            # or if they ARE at goal but moved from a non-goal position, cost += 1
            step_cost = 0
            for i, a in enumerate(agents):
                # Agent's next position
                next_pos = next_state[i]
                goal_pos = goals[a]
                # If agent is at goal and was at goal (staying at goal) -> 0 cost
                if next_pos == goal_pos and state[i] == goal_pos:
                    step_cost += 0
                else:
                    step_cost += 1

            actual_new_g = new_g_val + step_cost

            key = (next_state, next_t)
            if key in closed:
                continue
            if key in g_values and g_values[key] <= actual_new_g:
                continue

            g_values[key] = actual_new_g
            parent[key] = (state, t)
            h = heuristic(next_state)
            if h == float('inf'):
                continue
            f_val = actual_new_g + h
            cat_val = 0
            if cat:
                for i in range(k):
                    cat_val += cat.get((next_state[i], next_t), 0)
            heapq.heappush(open_list, (f_val, cat_val, next(counter), next_state, next_t))

    return None


# ---------------------------------------------------------------------------
# Path Cost Computation (Sum of Costs)
# ---------------------------------------------------------------------------

def compute_path_cost(path: List[int]) -> int:
    """
    Compute cost of a single-agent path under sum-of-costs.
    Cost = timestep when agent reaches goal for the last time.
    Trailing waits at goal are free.
    """
    if not path:
        return 0
    goal = path[-1]
    cost = len(path) - 1
    # Trim trailing goal waits
    while cost > 0 and path[cost] == goal and path[cost - 1] == goal:
        cost -= 1
    return cost


def compute_solution_cost(solution: Dict[int, List[int]]) -> int:
    """Compute total sum-of-costs for a multi-agent solution."""
    return sum(compute_path_cost(path) for path in solution.values())


# ---------------------------------------------------------------------------
# MA-CBS Algorithm
# ---------------------------------------------------------------------------

class MACBS:
    """
    Meta-Agent Conflict-Based Search (MA-CBS).

    Parameters:
        graph: nx.DiGraph with integer-labeled nodes and unit-weight edges
        starts: tuple of start nodes, one per agent (indexed 0..k-1)
        goals: tuple of goal nodes, one per agent (indexed 0..k-1)
        B: conflict bound parameter
            B = 0  -> equivalent to Independence Detection (ID)
            B = inf -> equivalent to basic CBS
            0 < B < inf -> MA-CBS with merging threshold B
    """

    def __init__(self, graph: nx.DiGraph, starts: Tuple[int, ...], goals: Tuple[int, ...],
                 B: float = float('inf')):
        self.graph = graph
        self.k = len(starts)
        assert len(starts) == len(goals), "starts and goals must have same length"

        self.starts = {i: starts[i] for i in range(self.k)}
        self.goals = {i: goals[i] for i in range(self.k)}
        self.B = B

        # Precompute heuristics (reverse Dijkstra from each goal)
        self.h_values = precompute_heuristics(graph, goals)

        # Conflict matrix: CM[i][j] = number of conflicts seen between agents i and j
        self.conflict_matrix = defaultdict(lambda: defaultdict(int))

        # Track which agents belong to which meta-agent
        # Initially each agent is its own meta-agent
        self.max_time = len(graph.nodes) * 3

    def get_h(self, agent: int, node: int) -> float:
        """Get heuristic value for agent at node."""
        goal = self.goals[agent]
        return self.h_values[goal].get(node, float('inf'))

    def find_individual_path(self, agent: int, constraints: Set,
                             solution: Dict[int, List[int]]) -> Optional[List[int]]:
        """Find optimal path for single agent with constraints using A*."""
        cat = build_cat(solution, agent)
        goal = self.goals[agent]
        return low_level_a_star(
            self.graph, self.starts[agent], goal,
            self.h_values[goal], constraints, cat, self.max_time
        )

    def find_meta_agent_path(self, meta_agent: MetaAgent,
                             constraints: Dict[int, Set],
                             meta_constraints_list: List,
                             solution: Dict[int, List[int]]) -> Optional[Dict[int, List[int]]]:
        """Find optimal paths for a meta-agent using joint A*."""
        agents = sorted(meta_agent.agents)
        cat = build_cat_multi(solution, meta_agent.agents)
        agent_constraints = {a: constraints.get(a, set()) for a in agents}
        return low_level_joint_a_star(
            self.graph, agents, self.starts, self.goals,
            self.h_values, agent_constraints, meta_constraints_list,
            cat, self.max_time
        )

    def get_meta_agent_for(self, agent_id: int, meta_agents: List[MetaAgent]) -> MetaAgent:
        """Find which meta-agent contains the given individual agent."""
        for ma in meta_agents:
            if agent_id in ma:
                return ma
        return MetaAgent(frozenset([agent_id]))

    def should_merge(self, a1: int, a2: int) -> bool:
        """Check if two agents should be merged based on conflict count > B."""
        # Get all individual agents in the meta-agents containing a1 and a2
        return self.conflict_matrix[min(a1, a2)][max(a1, a2)] > self.B

    def merge_meta_agents(self, ma1: MetaAgent, ma2: MetaAgent,
                          meta_agents: List[MetaAgent]) -> Tuple[MetaAgent, List[MetaAgent]]:
        """Merge two meta-agents into one. Returns (new_meta_agent, updated_list)."""
        new_agents = ma1.agents | ma2.agents
        new_ma = MetaAgent(new_agents)
        new_list = [ma for ma in meta_agents if ma != ma1 and ma != ma2]
        new_list.append(new_ma)
        return new_ma, new_list

    def solve(self, max_iterations: int = 100000) -> Optional[Dict[int, List[int]]]:
        """
        Run MA-CBS and return optimal solution.

        Args:
            max_iterations: Maximum number of CT nodes to expand before giving up.

        Returns:
            {agent_id: [path]} or None if no solution exists
        """
        CTNode._id_counter = 0
        self.conflict_matrix = defaultdict(lambda: defaultdict(int))
        iterations = 0

        # Initialize root CT node
        root = CTNode()
        root.meta_agents = [MetaAgent(frozenset([i])) for i in range(self.k)]

        # Find initial paths for each agent (no constraints)
        root.solution = {}
        for i in range(self.k):
            goal = self.goals[i]
            path = low_level_a_star(
                self.graph, self.starts[i], goal,
                self.h_values[goal], set(), None, self.max_time
            )
            if path is None:
                return None  # Agent can't reach goal at all
            root.solution[i] = path

        root.cost = compute_solution_cost(root.solution)

        # OPEN list
        open_list = []
        heapq.heappush(open_list, root)

        while open_list:
            iterations += 1
            if iterations > max_iterations:
                print(f"MA-CBS: max iterations ({max_iterations}) reached, giving up.")
                return None

            node = heapq.heappop(open_list)

            # Validate: find first conflict
            conflict = find_first_conflict(node.solution)

            if conflict is None:
                # No conflicts -> goal node
                return node.solution

            a1, a2 = conflict.agent1, conflict.agent2

            # Update conflict matrix
            key = (min(a1, a2), max(a1, a2))
            self.conflict_matrix[key[0]][key[1]] += 1

            # Get the meta-agents containing a1 and a2
            ma1 = self.get_meta_agent_for(a1, node.meta_agents)
            ma2 = self.get_meta_agent_for(a2, node.meta_agents)

            # Check if we should merge
            should_merge = False
            if self.B != float('inf'):
                # Check all pairs across the two meta-agents
                for x in ma1.agents:
                    for y in ma2.agents:
                        k1, k2 = min(x, y), max(x, y)
                        if self.conflict_matrix[k1][k2] > self.B:
                            should_merge = True
                            break
                    if should_merge:
                        break

            if should_merge and ma1 != ma2:
                # MERGE action
                new_ma, new_meta_agents = self.merge_meta_agents(ma1, ma2, node.meta_agents)

                # Build merged constraints:
                # Internal constraints (between a1 and a2 groups) are dropped
                # External constraints are preserved with sub-agent specificity
                merged_constraints = deepcopy(node.constraints)

                # Build meta constraints list for the merged agent
                meta_con_list = list(node.meta_constraints.get(new_ma.agents, []))

                # Also copy over meta constraints from the original meta agents
                for ma_key, mc_list in node.meta_constraints.items():
                    if ma_key != new_ma.agents:
                        meta_con_list.extend(mc_list)

                # Solve the merged meta-agent
                cat = build_cat_multi(node.solution, new_ma.agents)
                result = self.find_meta_agent_path(
                    new_ma, merged_constraints, meta_con_list, node.solution
                )

                if result is not None:
                    new_node = CTNode()
                    new_node.constraints = merged_constraints
                    new_node.solution = dict(node.solution)
                    for a, path in result.items():
                        new_node.solution[a] = path
                    new_node.cost = compute_solution_cost(new_node.solution)
                    new_node.meta_agents = new_meta_agents
                    new_node.meta_constraints = deepcopy(node.meta_constraints)
                    heapq.heappush(open_list, new_node)
                continue

            # BRANCH action: create two children
            if conflict.conflict_type == Conflict.VERTEX:
                # Child 1: constrain a1 from vertex at timestep
                # Child 2: constrain a2 from vertex at timestep
                for constrained_agent in [a1, a2]:
                    child = CTNode()
                    child.constraints = deepcopy(node.constraints)
                    child.constraints[constrained_agent].add(
                        Constraint(constrained_agent, conflict.vertex, conflict.timestep)
                    )
                    child.solution = dict(node.solution)
                    child.meta_agents = list(node.meta_agents)
                    child.meta_constraints = deepcopy(node.meta_constraints)

                    # Re-plan for the constrained agent's meta-agent
                    ma = self.get_meta_agent_for(constrained_agent, child.meta_agents)

                    if ma.size == 1:
                        # Single agent: use single-agent A*
                        cat = build_cat(child.solution, constrained_agent)
                        goal = self.goals[constrained_agent]
                        new_path = low_level_a_star(
                            self.graph, self.starts[constrained_agent], goal,
                            self.h_values[goal],
                            child.constraints[constrained_agent],
                            cat, self.max_time
                        )
                        if new_path is None:
                            continue  # No valid path, skip this child
                        child.solution[constrained_agent] = new_path
                    else:
                        # Meta-agent: use joint A*
                        meta_con_list = list(child.meta_constraints.get(ma.agents, []))
                        result = self.find_meta_agent_path(
                            ma, child.constraints, meta_con_list, child.solution
                        )
                        if result is None:
                            continue
                        for a, path in result.items():
                            child.solution[a] = path

                    child.cost = compute_solution_cost(child.solution)
                    heapq.heappush(open_list, child)

            elif conflict.conflict_type == Conflict.EDGE:
                # Edge conflict: a1 moves v->v2, a2 moves v2->v at timestep t
                for idx, constrained_agent in enumerate([a1, a2]):
                    child = CTNode()
                    child.constraints = deepcopy(node.constraints)

                    if idx == 0:
                        child.constraints[constrained_agent].add(
                            EdgeConstraint(constrained_agent,
                                           conflict.vertex, conflict.vertex2,
                                           conflict.timestep)
                        )
                    else:
                        child.constraints[constrained_agent].add(
                            EdgeConstraint(constrained_agent,
                                           conflict.vertex2, conflict.vertex,
                                           conflict.timestep)
                        )

                    child.solution = dict(node.solution)
                    child.meta_agents = list(node.meta_agents)
                    child.meta_constraints = deepcopy(node.meta_constraints)

                    ma = self.get_meta_agent_for(constrained_agent, child.meta_agents)

                    if ma.size == 1:
                        cat = build_cat(child.solution, constrained_agent)
                        goal = self.goals[constrained_agent]
                        new_path = low_level_a_star(
                            self.graph, self.starts[constrained_agent], goal,
                            self.h_values[goal],
                            child.constraints[constrained_agent],
                            cat, self.max_time
                        )
                        if new_path is None:
                            continue
                        child.solution[constrained_agent] = new_path
                    else:
                        meta_con_list = list(child.meta_constraints.get(ma.agents, []))
                        result = self.find_meta_agent_path(
                            ma, child.constraints, meta_con_list, child.solution
                        )
                        if result is None:
                            continue
                        for a, path in result.items():
                            child.solution[a] = path

                    child.cost = compute_solution_cost(child.solution)
                    heapq.heappush(open_list, child)

        return None  # No solution found


# ---------------------------------------------------------------------------
# Convenience wrapper
# ---------------------------------------------------------------------------

def solve_mapf(graph: nx.DiGraph,
               starts: Tuple[int, ...],
               goals: Tuple[int, ...],
               B: float = float('inf'),
               max_iterations: int = 100000) -> Optional[Dict[int, List[int]]]:
    """
    Solve a MAPF instance using MA-CBS.

    Args:
        graph: nx.DiGraph with integer-labeled nodes and unit weight edges.
               All edges should have weight=1 (or no weight attribute).
        starts: Tuple of start nodes for each agent (agent 0, 1, ..., k-1)
        goals: Tuple of goal nodes for each agent
        B: Conflict bound parameter for MA-CBS
           B = inf  -> basic CBS
           B = 0    -> equivalent to ID
           0 < B < inf -> MA-CBS
        max_iterations: Maximum CT nodes to expand before giving up.

    Returns:
        Dictionary mapping agent_id to path (list of nodes), or None
    """
    # Ensure all edges have weight=1
    for u, v in graph.edges():
        if 'weight' not in graph[u][v]:
            graph[u][v]['weight'] = 1

    solver = MACBS(graph, starts, goals, B)
    return solver.solve(max_iterations=max_iterations)


# ---------------------------------------------------------------------------
# Testing / Demo
# ---------------------------------------------------------------------------

def create_grid_graph(rows: int, cols: int, obstacles: Set[Tuple[int, int]] = None) -> nx.DiGraph:
    """
    Create a 4-connected grid graph as a DiGraph.
    Node labels: row * cols + col (integers).
    """
    if obstacles is None:
        obstacles = set()

    G = nx.DiGraph()
    for r in range(rows):
        for c in range(cols):
            if (r, c) in obstacles:
                continue
            node = r * cols + c
            G.add_node(node)

    directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]
    for r in range(rows):
        for c in range(cols):
            if (r, c) in obstacles:
                continue
            node = r * cols + c
            for dr, dc in directions:
                nr, nc = r + dr, c + dc
                if 0 <= nr < rows and 0 <= nc < cols and (nr, nc) not in obstacles:
                    neighbor = nr * cols + nc
                    G.add_edge(node, neighbor, weight=1)

    return G


def print_solution(solution: Dict[int, List[int]], rows: int = None, cols: int = None):
    """Pretty print a solution."""
    if solution is None:
        print("No solution found!")
        return

    total_cost = compute_solution_cost(solution)
    max_t = max(len(p) for p in solution.values())

    print(f"Solution found! Total sum-of-costs: {total_cost}")
    print(f"Number of agents: {len(solution)}")
    print()

    for agent in sorted(solution.keys()):
        path = solution[agent]
        cost = compute_path_cost(path)
        print(f"  Agent {agent}: cost={cost}, path={path}")

    print()
    print("Timeline:")
    for t in range(max_t):
        positions = {}
        for agent in sorted(solution.keys()):
            pos = get_location(solution[agent], t)
            positions[agent] = pos
        print(f"  t={t}: {positions}")


if __name__ == "__main__":
    print("=" * 60)
    print("Test 1: Paper example (Fig. 1) - Bottleneck")
    print("=" * 60)

    # Recreate the example from the paper (Fig. 1)
    # Graph: S1 -- A1 -- D -- G1
    #        S2 -- B1 -- D -- G2
    # Nodes: S1=0, S2=1, A1=2, B1=3, D=4, G1=5, G2=6
    G1 = nx.DiGraph()
    edges = [(0, 2), (2, 0), (2, 4), (4, 2), (4, 5), (5, 4),
             (1, 3), (3, 1), (3, 4), (4, 3), (4, 6), (6, 4)]
    G1.add_edges_from(edges, weight=1)

    starts1 = (0, 1)  # S1, S2
    goals1 = (5, 6)   # G1, G2

    print("\n--- CBS (B=inf) ---")
    sol = solve_mapf(G1, starts1, goals1, B=float('inf'))
    print_solution(sol)

    print("\n--- MA-CBS (B=0, equivalent to ID) ---")
    sol = solve_mapf(G1, starts1, goals1, B=0)
    print_solution(sol)

    print("\n--- MA-CBS (B=5) ---")
    sol = solve_mapf(G1, starts1, goals1, B=5)
    print_solution(sol)

    print()
    print("=" * 60)
    print("Test 2: 4x4 Grid with 3 agents")
    print("=" * 60)

    G2 = create_grid_graph(4, 4)
    starts2 = (0, 3, 12)    # top-left, top-right, bottom-left
    goals2 = (15, 12, 3)    # bottom-right, bottom-left, top-right

    print("\n--- CBS (B=inf) ---")
    sol = solve_mapf(G2, starts2, goals2, B=float('inf'))
    print_solution(sol)

    print("\n--- MA-CBS (B=5) ---")
    sol = solve_mapf(G2, starts2, goals2, B=5)
    print_solution(sol)

    print()
    print("=" * 60)
    print("Test 3: Corridor with bypass (solvable)")
    print("=" * 60)

    # 0 -- 1 -- 2 -- 3 -- 4
    #           |
    #           5 (bypass)
    G3 = nx.DiGraph()
    for i in range(4):
        G3.add_edge(i, i + 1, weight=1)
        G3.add_edge(i + 1, i, weight=1)
    G3.add_edge(2, 5, weight=1)
    G3.add_edge(5, 2, weight=1)

    starts3 = (0, 4)
    goals3 = (4, 0)

    print("\n--- CBS (B=inf) ---")
    sol = solve_mapf(G3, starts3, goals3, B=float('inf'))
    print_solution(sol)

    print()
    print("=" * 60)
    print("Test 4: 5x5 Grid with 4 agents")
    print("=" * 60)

    G4 = create_grid_graph(5, 5)
    starts4 = (0, 4, 20, 24)
    goals4 = (24, 20, 4, 0)

    print("\n--- MA-CBS (B=10) ---")
    sol = solve_mapf(G4, starts4, goals4, B=10)
    print_solution(sol)

    print()
    print("All tests completed!")
