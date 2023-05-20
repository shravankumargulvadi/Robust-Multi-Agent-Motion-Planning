from collections import defaultdict

# This class represents a directed graph using adjacency list representation
class Graph: 
    
    # Constructor
    def __init__(self):
 
        # default dictionary to store graph
        self.graph = defaultdict(list)
 
    # function to add an edge to graph
    def addEdge(self, u, v):
        self.graph[u].append(v)
 
    # A function used by DFS
    def DFSUtil(self, v, visited, recursionStack):
 
        # Mark the current node as visited and print it
        visited.add(v)
        recursionStack.add(v)
        print(v, end=" ")
 
        # Recur for all the vertices adjacent to this vertex
        for neighbour in self.graph[v]:
            if neighbour not in visited:
                if self.DFSUtil(neighbour, visited, recursionStack):
                    return True
            elif neighbour in recursionStack:
                return True

        recursionStack.remove(v)
        return False
    
    # The function to do DFS traversal. It uses recursive DFSUtil()
    
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
    # Create the TPG
    g.addEdge(5, 9)
    g.addEdge(5, 10)
    g.addEdge(2, 6)
    g.addEdge(4, 7)
    g.addEdge(7, 11)
    g.addEdge(7, 12)
    g.addEdge(4, 8)
    g.addEdge(1, 2)
    g.addEdge(1, 3)
    g.addEdge(1, 4)
    g.addEdge(2, 5)
    
    # Function call
    g.DFS(1)