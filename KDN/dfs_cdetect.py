from collections import defaultdict
import copy
import numpy as np
from atpg import uniform_agent_aug_tpg, get_agent
from tpg import TPG, sample_tpg, sample_tpg_2, sample_tpg_h16

class Graph:
    def __init__(self):
        self.graph = defaultdict(list)
        self.hascycle = False

    def addEdge(self, u, v):
        self.graph[u].append(v)

    def addEdgesFromTPG(self, tpg_data):
        # Apply uniform_agent_aug_tpg to get augmented TPG
        atpg = uniform_agent_aug_tpg(tpg_data, dp=None, dp_safe=None)

        # Iterate over the augmented TPG and add directed edges
        for agent in atpg:
            for node in agent:
                node_name = node.name
                next_node = node.next_node

                if next_node:
                    next_node_name = next_node.name
                    self.addEdge(node_name, next_node_name)

                type2_edge = node.type2
                if type2_edge:
                    type2_edge_name = type2_edge.name
                    self.addEdge(node_name, type2_edge_name)

    def DFSUtil(self, v, visited, recursionStack):
        visited.add(v)
        recursionStack.add(v)
        print(v, end=" ")

        for neighbour in self.graph[v]:
            if neighbour not in visited:
                self.DFSUtil(neighbour, visited, recursionStack)
            elif neighbour in recursionStack:
                print("\n")
                print(f"Cycle detected involving node {neighbour}")
                return True

        recursionStack.remove(v)
        return False

    def DFSAll(self):
        visited = set()
        recursionStack = set()

        # Create a copy of the keys to avoid "RuntimeError: dictionary changed size during iteration"
        nodes = list(self.graph.keys())  # No need to deepcopy, just create a list

        has_cycle = False

        for node in nodes:
            if node not in visited:
                print("\n")
                print(f"DFS starting from node {node}:")
                if self.DFSUtil(node, visited, recursionStack):
                    has_cycle = True
                    break  # Exit the loop if a cycle is detected
        
        self.hascycle = has_cycle  # Store the value in the instance variable

        print("\n")
        if has_cycle:
            print("The TPG contains a cycle.")
        else:
            print("The TPG does NOT contain any cycles.")

    def getHasCycle(self):
        return self.hascycle

    def generate_control_points(self, num_control_points):
        control_points = []

        for node in self.graph:
            neighbours = self.graph[node]
            num_neighbours = len(neighbours)

            if num_neighbours == 0:
                continue

            positions = np.linspace(0, 1, num_control_points+2)[1:-1]
            control_points.extend(positions)

        return control_points

if __name__ == "__main__":
    g = Graph()

    tpg_data = sample_tpg()  # Load your TPG data here
    # tpg_data = sample_tpg_2() 
    # tpg_data = sample_tpg_h16() 

    g.addEdgesFromTPG(tpg_data)

    print(tpg_data)
    g.DFSAll()
    has_cycle = g.getHasCycle()
    # print(has_cycle)

    # Generate control points
    num_control_points = 20
    control_points = g.generate_control_points(num_control_points)
    # print("Control Points:", control_points)

    # How to read output:
    # A^1_0 represents node A for agent 1 at time step 0
    # D^1_2 (D^2_3) represents node D for agent 1 at time step 2, which has a type 2 edge connecting it to node D for agent 2 at time step 3
