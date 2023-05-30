from collections import defaultdict
import logging
from tpg import TPG, TPG_node  # Assuming the TPG classes are defined in tpg.py

class Graph: 
    def __init__(self):
        self.graph = defaultdict(list)
 
    def addEdge(self, u, v):
        self.graph[u].append(v)
 
    def DFSUtil(self, v, visited, recursionStack):
        visited.add(v)
        recursionStack.add(v)
        print(v, end=" ")
 
        for neighbour in self.graph[v]:
            if neighbour not in visited:
                if self.DFSUtil(neighbour, visited, recursionStack):
                    return True
            elif neighbour in recursionStack:
                return True

        recursionStack.remove(v)
        return False
    
    def DFS(self, v):
        visited = set()
        recursionStack = set()

        if self.DFSUtil(v, visited, recursionStack):
            print("\n")
            print("Graph contains a cycle")
        else:
            print("\n")
            print("Graph does NOT contain a cycle") 


if __name__ == "__main__":
    g = Graph()

    # Read the TPG from tpg.py
    from tpg import sample_tpg
    tpg = sample_tpg()

    # Iterate over the TPG and add directed edges
    for agent in tpg:
        for node in agent:
            node_name = node.name
            next_node = node.next_node

            if next_node:
                next_node_name = next_node.name
                g.addEdge(node_name, next_node_name)

            type2_edge = node.type2
            if type2_edge:
                type2_edge_name = type2_edge.name
                g.addEdge(node_name, type2_edge_name)

    # Perform DFS on the graph
    g.DFS(1)
