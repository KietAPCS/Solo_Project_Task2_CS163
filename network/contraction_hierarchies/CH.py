import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../')))

from network.optimize import *
from shortestPath.dijkstra import *

"""
Optimizing graph by Contraction Hierarchies

[module] : [graph] -> [contraction_hierarchies]
"""

class Contraction_Hierarchies(Dijkstra):
    def __init__ (self, vars, stops, paths):
        super().__init__(vars, stops, paths)
        
        self.contracted_order = {}
        self.edge_difference = {}
            
        # Build G_star graph
        self.G_star_shortcuts = {}
        self.G_star_edges = {}
        self.G_star_parents = {}
        self.G_star_children = {}
        for u in self.numVertices:
            self.G_star_shortcuts[u] = {}
            self.G_star_edges[u] = []
            self.G_star_parents[u] = self.parents[u].copy()
            self.G_star_children[u] = self.children[u].copy()
            for v in self.children[u]:
                self.G_star_shortcuts[u][v] = None
                self.G_star_edges[u].append([self.timeAll[u][v], v])
                
    # Preprocessing stage
    """
    Simulating each node contraction and consider its edge difference. Then do a real
    contraction from the node with the least edge difference. Using lazy update to
    optimize the process.

    """
    def count_edge_diff_one(self, u):
        cnt_sides = len(self.children[u]) + len(self.parents[u])
        cnt_shortcuts = self.count_shortcuts(u)
        self.edge_difference[u] = cnt_shortcuts - cnt_sides
    
    def count_edge_diff_all(self):
        for u in self.numVertices:
            self.count_edge_diff_one(u)
                
    def add_shortcuts(self, v):
        # Incoming edges
        U = self.parents[v]  
        # Outgoing edges
        W = self.children[v]
        
        for u in U:
            P = {}
            Pmax = -float('inf')
            for w in W:
                Pw = self.timeAll[u][v] + self.timeAll[v][w]
                P[w] = Pw
                Pmax = max(Pmax, Pw)

            dist = self.dijkstra_excluding_v(u, v, Pmax)
            for w in W:
                if dist[w] > P[w]:
                    self.G_star_edges[u].append([P[w], w])
                    self.G_star_shortcuts[u][w] = v
                    self.G_star_children[u].add(w)
                    self.G_star_parents[w].add(u)
                    
                    self.timeAll[u][w] = P[w]
                    self.parents[w].add(u)
                    self.children[u].add(w)
                    
                
    def count_shortcuts(self, v):
        cnt = 0
        # Incoming edges
        U = self.parents[v]  
        # Outgoing edges
        W = self.children[v]
        
        for u in U:
            P = {}
            Pmax = -float('inf')
            for w in W:
                Pw = self.timeAll[u][v] + self.timeAll[v][w]
                P[w] = Pw
                Pmax = max(Pmax, Pw)

            dist = self.dijkstra_excluding_v(u, v, Pmax)
            for w in W:
                if dist[w] > P[w]:
                    cnt += 1
    
        return cnt
    
    def remove_node(self, v):
        # Remove all edges to and from node v
        for u in list(self.parents[v]):
            self.children[u].remove(v)
        for u in list(self.children[v]):
            self.parents[u].remove(v)

        # Clear the parents and children lists of node v
        self.parents[v].clear()
        self.children[v].clear()

    
    def dijkstra_excluding_v(self, start, exclude, Pmax):
        dist = {i: float('inf') for i in self.numVertices}
        dist[start] = 0
        priority_queue = [(0, start)]
        settled = set()
        
        while priority_queue:
            current_dist, current_node = heapq.heappop(priority_queue)
            
            if current_node in settled:
                continue
            
            settled.add(current_node)
            
            if current_dist > Pmax:
                break
            
            for neighbor in self.children[current_node]:
                if neighbor == exclude or neighbor in settled:
                    continue
                
                new_dist = current_dist + self.timeAll[current_node][neighbor]
                
                if new_dist < dist[neighbor]:
                    dist[neighbor] = new_dist
                    heapq.heappush(priority_queue, (new_dist, neighbor))
        
        return dist
    
    """
    Lazy Updating
    
    """    
    def pre_processing(self):
        self.count_edge_diff_all()
        pq = []
        
        for u in self.numVertices:
            heapq.heappush(pq, (self.edge_difference[u], u))
            
        while pq:
            cur, u = heapq.heappop(pq)
            self.count_edge_diff_one(u)
            
            if pq and self.edge_difference[u] > pq[0][0]:
                heapq.heappush(pq, (self.edge_difference[u], u))
                continue
            
            self.add_shortcuts(u)
            self.remove_node(u)
            self.contracted_order[u] = len(self.contracted_order) + 1
        
        print("Preprocessing done!")

    def dijkstra_modified(self, source, forward):
        dist = {i: float('inf') for i in self.numVertices}
        trace = {i: None for i in self.numVertices}
        dist[source] = 0
        priority_queue = [(0, source)]
        
        settled = set()
        
        while priority_queue:
            current_dist, current_node = heapq.heappop(priority_queue)
            
            if current_dist > dist[current_node]:
                continue
            
            settled.add(current_node)
            
            if forward:
                neighbors = self.G_star_children[current_node]
            else:
                neighbors = self.G_star_parents[current_node]
                
            for neighbor in neighbors:
                if neighbor in settled:
                    continue
                
                if self.contracted_order[neighbor] <= self.contracted_order[current_node]:
                    continue
                
                if forward:
                    new_dist = current_dist + self.timeAll[current_node][neighbor]
                else:
                    new_dist = current_dist + self.timeAll[neighbor][current_node]
                    
                if new_dist < dist[neighbor]:
                    dist[neighbor] = new_dist
                    trace[neighbor] = current_node
                    heapq.heappush(priority_queue, (new_dist, neighbor))
            
        return dist, settled, trace 
        
    def bidirectional_dijkstra(self, source, des):
        dist_foward, settled_foward, trace_forward = self.dijkstra_modified(source, forward=True)
        dist_backward, settled_backward, trace_backward = self.dijkstra_modified(des, forward=False)
        
        v = None
        min_dist = float('inf')
        
        L = [x for x in settled_foward if x in settled_backward]
        
        for u in L:
            if dist_foward[u] + dist_backward[u] < min_dist:
                min_dist = dist_foward[u] + dist_backward[u]
                v = u
        
        return min_dist, v, trace_forward, trace_backward
    
    def query(self, source, des):
        min_dist, v, trace_forward, trace_backward = self.bidirectional_dijkstra(source, des)
            
        path1 = []
        cur = v
        while cur is not None: 
            path1.append(cur)
            cur = trace_forward[cur]
            
            if cur == source:
                path1.append(cur)
                break
        
        path1.reverse()
        
        haveShortcuts = True
        
        while haveShortcuts:
            check = True
            for i in range(len(path1) - 1):
                if (path1[i] in self.G_star_shortcuts) and (path1[i+1] in self.G_star_shortcuts[path1[i]]) and (self.G_star_shortcuts[path1[i]][path1[i+1]] is not None):
                    check = False
                    path1 = path1[:i+1] + [self.G_star_shortcuts[path1[i]][path1[i+1]]] + path1[i+1:]
            
            if check:
                haveShortcuts = False
                
        path2 = []
        cur = v
        while cur is not None:
            path2.append(cur)
            cur = trace_backward[cur]
            
            if cur == des:
                path2.append(cur)
                break
        
        haveShortcuts = True
        while haveShortcuts:
            check = True
            for i in range(len(path2) - 1):
                if (path2[i] in self.G_star_shortcuts) and (path2[i+1] in self.G_star_shortcuts[path2[i]]) and (self.G_star_shortcuts[path2[i]][path2[i+1]] is not None):
                    check = False
                    path2 = path2[:i+1] + [self.G_star_shortcuts[path2[i]][path2[i+1]]] + path2[i+1:]
            
            if check:
                haveShortcuts = False
                
        return min_dist, path1 + path2[1:]
    
    def saveOneSP(self, time, path, start, end):
        if time == self.INF or time == 0:
            print("No path found!")
            return
        
        tracePath = {}
        for i in path:
            tracePath[i] = {}
        
        for i in range(len(path) - 1):
            tracePath[path[i]][path[i+1]] = self.pathAll[path[i]][path[i+1]]
            
        try:
            with open('output/shortestPath.txt', 'w') as file:
                file.write(f"Shortest path from {start} to {end}: ")
                for i in path:
                    file.write(f"{i}")
                    if i != end:
                        file.write("->")
                file.write("\n")
                file.write(f"Total time: {time} seconds")
                file.write("\n")
                for i in range(len(path) - 1):
                    if i != end:
                        file.write(f"From {path[i]}->{path[i+1]} : ")
                        for x, y in tracePath[path[i]][path[i+1]]:
                            file.write(f"[{y},{x}]")
                            if [x,y] != tracePath[path[i]][path[i+1]][-1]:
                                file.write("->")
                        file.write("\n")
            print("Find shortest path and save successfully.")
            file.close()
        except Exception as e:
            print("Error: " + str(e))
            
        try:
            with open("output/geoJson.json", "w", encoding="utf-8") as file:
                file.write('{\n')
                file.write('    "type": "FeatureCollection",\n')
                file.write('    "features": [\n')
                file.write('    {\n')
                file.write('      "type": "Feature",\n')
                file.write('      "properties": {},\n')
                file.write('      "geometry": {\n')
                file.write('        "coordinates": [')

                for i in range(len(path) - 1):
                    #print(path[i], path[i+1])
                    for x, y in tracePath[path[i]][path[i+1]]:
                        file.write(f"[{y},{x}]")
                        if [x,y] != tracePath[path[len(path) - 2]][path[len(path) - 1]][-1]:
                            file.write(',')
                file.write('],\n')
                file.write('        "type": "LineString"\n')
                file.write('      }\n')
                file.write('    }')
                file.write('\n')
                file.write('                ]\n')
                file.write('}\n')
            print("GeoJSON file created successfully.")
        except Exception as e:
            print(f"Error creating GeoJSON file: {str(e)}")      

