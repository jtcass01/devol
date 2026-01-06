from __future__ import annotations
from numpy import ndarray, inf, array
from typing import List, Tuple, Dict

__author__ = "Jacob Taylor Cassady"
__email__ = "jcassad1@jh.edu"

def euclidean_distance(p1: Tuple[int, int], p2: Tuple[int, int]) -> float:
    x1, y1 = p1
    x2, y2 = p2
    return ((x2-x1)**2+(y2-y1)**2)**0.5

def list_neighbors(map: ndarray, p: Tuple[int, int]) -> List[Tuple[int, int]]:
    x, y = p
    l, w = map.shape
    neighbors: List[Tuple[int, int]] = []

    # 8 directions
    directions = [
        (-1,  0),  # up
        ( 1,  0),  # down
        ( 0, -1),  # left
        ( 0,  1),  # right
        (-1, -1),  # up-left
        (-1,  1),  # up-right
        ( 1, -1),  # down-left
        ( 1,  1)   # down-right
    ]

    for dx, dy in directions:
        nx, ny = x + dx, y + dy
        if 0 <= nx < l and 0 <= ny < w and map[nx, ny] == 0:
            neighbors.append((nx, ny))

    return neighbors

class Edge: 
    """
    This class provides a basic data structure for representing
    a directional edge in a graph. Travel is possible between
    the starting node to the ending node at the given cost
    but travel in the opposite direction is not allowed.
    """
    def __init__(self,starting_node, ending_node, cost):
        self.start = starting_node
        self.end = ending_node 
        self.cost = cost 

    def __repr__(self):
        return 'Node'+self.__str__()
    def __str__(self):
        return f'({self.start.name},{self.end.name},{self.cost})'

    def __eq__(self, obj):
        if  isinstance(obj, Edge):
            return self.start == obj.start and self.end == obj.end and self.cost == obj.cost 
        return False

class Node:
    """
    This class provides a basic data structure for representing
    a node in A* Graph
    """
    def __init__(self, name, h):
        #The name of the node (can be anything, just for human readable output)
        self.name = name
        #The current best cost-to-come for the node
        self.g = inf 
        #The current best estimate of the node's total cost
        self.f = inf 
        #The heuristic estimate of the cost-to-go for the node
        self.h = h 
        #The list of edges which connect the node 
        self.edges = []
        #The previous node in path to the goal
        self.previous = None

    def add_neighbor(self, node: Node, cost: int):
        new_edge = Edge(self, node, cost)
        self.edges.append(new_edge)

    def add_neighbor_bidirectional(self, node: Node, cost: int):
        self.add_neighbor(node, cost)
        node.add_neighbor(self, cost)

    def __str__(self):
        return f'({self.name},{self.f},{self.g},{self.h})'

    def __eq__(self, obj):
        if  isinstance(obj, Node):
            return self.name == obj.name and self.f == obj.f  and self.g == obj.g and self.h == obj.h 
        return False
    
    def __hash__(self):
        return hash(self.name)

    def __ge__(self, other: Node):
        return self.f >= other.f

    def __lt__(self, other: Node):
        return self.f < other.f

def a_star_grid(map: ndarray, start:Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
    """
    This function will compute the optimal path between a start point and an end point given a grid-based
    map. It is up to the student to implement the heuristic function and cost function. Assume a cell's 
    indices represent it's position in cartesian space. (e.g. cells [1,3] and [1,5] are 2 units apart). 

    If no path exists then this function should return an empty list.

    Worth 50 pts
    
    Input
      :param map: An ndarray representing free space and occupied space
      :param start: A tuple of indicies indicating the starting cell of the search
      :param goal: A tuple of indicies indicating the goal cell of the search

    Output
      :return: path: a list of Tuples indicating the indicies of the cells that make up the path with 
                    the starting cell as the first element of the list and the ending cell as the last
                    element in the list
    """
    open_list: List[Tuple[int, int]] = [start]
    closed_list: List[Tuple[int, int]] = []
    g_cost: Dict[Tuple[int, int], float] = {start: 0.0}
    came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}

    while open_list:
        # pick n_best from O such that f(n_best) <= f(n)
        n_best: Tuple[int, int] = min(open_list, key=lambda n: g_cost[n] + euclidean_distance(n, goal))

        if n_best == goal:
            path = [n_best]
            while n_best in came_from:
                n_best = came_from[n_best]
                path.append(n_best)
            return path[::-1] # reverse

        # Remove n_best from O and add it to C
        open_list.remove(n_best)
        closed_list.append(n_best)

        for neighbor in list_neighbors(map, n_best):
            if neighbor in closed_list:
                continue

            tentative_g: float = g_cost[n_best] + euclidean_distance(n_best, neighbor)

            if neighbor not in open_list or tentative_g < g_cost.get(neighbor, inf):
                came_from[neighbor] = n_best
                g_cost[neighbor] = tentative_g
                if neighbor not in open_list:
                    open_list.append(neighbor)

    return []

def a_star_graph(start: Node, goal: Node) -> List[Node]:
    """
    This function will compute the optimal path between a starting node and an ending node.
    The result should be a list of the Edges that represent the optimal path to the goal. 
    For this function the cost and heuristic functions are defined when the node is originally created.

    
    If no path exists then this function should return an empty list.

    Worth 50 pts
    
    Input
      :param start: The starting node of the search
      :param goal: The ending node of the search

    Output
      :return: path: a list of Node objects representing the optimal path to the goal 
    """
    open_list: List[Node] = [start]
    closed_list: List[Node] = []
    came_from: Dict[Node, Node] = {}

    start.g = 0.0
    start.f = start.h

    while open_list:
        # pick n_best from O such that f(n_best) <= f(n)
        n_best: Node = min(open_list, key=lambda n: n.f)
    
        if n_best == goal:
            path = [n_best]
            while n_best in came_from:
                n_best = came_from[n_best]
                path.append(n_best)
            return path[::-1]

        # Remove n_best from O and add it to C
        open_list.remove(n_best)
        closed_list.append(n_best)

        for edge in n_best.edges:
            neighbor: Node = edge.end
            tentative_g: float = n_best.g + edge.cost

            if neighbor in closed_list and tentative_g >= neighbor.g:
                continue

            if tentative_g < neighbor.g or neighbor not in open_list:
                came_from[neighbor] = n_best
                neighbor.g = tentative_g
                neighbor.f = neighbor.g + neighbor.h
                if neighbor not in open_list:
                    open_list.append(neighbor)

    return []

def graph_demo():
    nodes: List[Node] = []

    nodes.append(Node('A', 10)) # A
    nodes.append(Node('B', 5))  # B
    nodes.append(Node('C', 6))  # C
    nodes.append(Node('D', 2))  # D
    nodes.append(Node('E', 3))  # E
    nodes.append(Node('F', 0))  # F

    nodes[0].add_neighbor(nodes[1],3)
    nodes[0].add_neighbor(nodes[2],4)
    nodes[1].add_neighbor(nodes[3],2)
    nodes[1].add_neighbor(nodes[4],2)
    nodes[2].add_neighbor(nodes[4],4)
    nodes[3].add_neighbor(nodes[5],5)
    nodes[3].add_neighbor(nodes[4],4)
    nodes[4].add_neighbor(nodes[5],4)

    path = a_star_graph(nodes[0],nodes[-1])

    for e_i in path:
        print(e_i)

def grid_demo():

    map = array([[0,0,1,0,0,0,0,0,0],
                 [0,0,1,0,0,0,0,0,0],
                 [0,0,1,0,0,1,1,1,0],
                 [0,0,1,0,0,1,0,0,0],
                 [0,0,1,0,0,1,0,1,1],
                 [0,0,1,0,0,1,0,0,0],
                 [0,0,0,0,0,1,0,0,0],
                 [0,0,0,0,0,1,0,0,0],
                 [0,0,0,0,0,1,0,0,0],
        ])

    start = (0,0)
    goal =  (8,8)
    path = a_star_grid(map, start, goal)
    for c_i in path:
        print(c_i)

def main():
    graph_demo()
    grid_demo()

if __name__ == '__main__':
    main()
