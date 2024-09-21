from network.optimize import *

"""
Dijkstra algorithm for finding 
shortest path in graph.

[module]: [dijkstra]
"""
class Dijkstra(Graph_Optimized):
    def __init__ (self, vars, stops, paths):
        super().__init__(vars, stops, paths)
    
        self.dist = {} 
        self.trace = {} 
        self.cnt = {}
        self.impo = {}
        
        for i in tqdm(self.numVertices):
            self.dist[i] = {}
            self.trace[i] = {}
            self.cnt[i] = {}
            self.impo[i] = 0

            for j in self.numVertices:              
                self.dist[i][j] = self.INF
                self.trace[i][j] = -1
                self.cnt[i][j] = 0
    
    def dijkstraOne(self, start):
        dist = {}
        trace = {}
        dist[start] = {}
        trace[start] = {}
        for j in self.numVertices:
            dist[start][j] = self.INF
            trace[start][j] = -1

        dist[start][start] = 0
        minHeap = []
        heapq.heappush(minHeap,(0, start))
        visited = set()

        while minHeap:
            w, u = heapq.heappop(minHeap)
            visited.add(u)
            
            if (w > dist[start][u]):
                continue
                
            for dis, v, f in self.vertices[u]:
                if v in visited:
                    continue
                if w + dis[0] < dist[start][v]:
                    dist[start][v] = w + dis[0]
                    trace[start][v] = u
                    heapq.heappush(minHeap, (w + dis[0], v))
        
        return dist, trace
    
    def dijkstraAll(self):
        for st in tqdm(self.numVertices):
            self.dist[st][st] = 0
            self.cnt[st][st] = 1 #count important
            minHeap = []
            heapq.heappush(minHeap,(0, st))
            visited = set()
            while minHeap:
                w, u = heapq.heappop(minHeap)
                visited.add(u)
                if (w > self.dist[st][u]):
                    continue
                
                for dis, v, f in self.vertices[u]:
                    if v not in visited:
                        if w + dis[0] < self.dist[st][v]:
                            self.cnt[st][v] = self.cnt[st][u] #count important
                            self.dist[st][v] = w + dis[0]
                            self.trace[st][v] = u
                            heapq.heappush(minHeap, (w + dis[0], v))  
                        elif w + dis[0] == self.dist[st][v]:
                            self.cnt[st][v] += self.cnt[st][u] #count important
                            
    def find_sp_all_pairs(self, source, destination):
        time = self.dist[source][destination]
        path = []
        
        current = destination
        while current != -1:
            path.append(current)
            current = self.trace[source][current]
            if current == source:
                path.append(source)
                break
        
        if current != source:
            return 0, []
        
        path.reverse()
        
        return time, path
    
    def countImportantStops(self):
        global topo
        global visited
        global edges

        for st in tqdm(self.numVertices):
            topo = []
            for i in self.numVertices:
                edges[i] = []
                visited[i] = False
            
            #Remove all edges not shortest path
            for i in self.numVertices:
                for j in self.vertices[i]:
                    if (self.dist[st][i] + j[0][0] == self.dist[st][j[1]]):
                        edges[i].append(j[1])

            for i in self.numVertices:
                if not visited[i]:
                    dfs(i)
            
            dp = {}
            for i in topo:
                dp[i] = 1
                for j in edges[i]:
                    dp[i] += dp[j]

                self.cnt[st][i] *= dp[i]
        
        for v in tqdm(self.numVertices):
            for u in self.numVertices:
                self.impo[v] += self.cnt[u][v]
    
    def saveManySP(self, totalTime, trace, start, end):
        path = []
        path.append(end)
        x = trace[start][end]
        while (x != start):
            path.append(x)
            x = trace[start][x]
        path.append(start)
        path.reverse()

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
                file.write(f"Total time: {totalTime} seconds")
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