from network.optimize import *
from shortestPath.dijkstra import *

"""
Improving dijkstra algorithm with A* algorithm 
by introducing heuristic function.

[module] : [a_star]
"""

class A_Star(Graph_Optimized):
    def __init__ (self, vars, stops, paths):
        super().__init__(vars, stops, paths)
        
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
            
    # 1) A* search algorithm is considered an improvement of Dijkstra's algorithm.
    def get_heuristic(self, source, destination):
        lat1, lng1 = self.coordinatesAll[source]
        lat2, lng2 = self.coordinatesAll[destination]
        return self.distanceLL(lat1, lng1, lat2, lng2) / self.maxSpeed
    def a_star_search(self, source, destination):    
        g_score = {v: self.INF for v in self.numVertices}
        f_score = {v: self.INF for v in self.numVertices}
        trace = {v: -1 for v in self.numVertices}
        
        g_score[source] = 0
        f_score[source] = self.get_heuristic(source, destination)
        
        minHeap = []
        heapq.heappush(minHeap, (f_score[source], source))
        visited = set()

        while minHeap:
            f_cur, u = heapq.heappop(minHeap)
            if u == destination:
                break
            visited.add(u)
            
            # Optimization algorithm
            if (f_cur > f_score[u]): 
                continue
            
            for info, neighbor, routeInfo in self.vertices[u]:
                if neighbor in visited:
                    continue
                
                time = info[0]
                distance = info[1]
                routeId = routeInfo[0]
                routeVarId = routeInfo[1]
                
                inherited_cost = g_score[u] + time
                f = inherited_cost + self.get_heuristic(neighbor, destination)
                
                if f < f_score[neighbor]:
                    g_score[neighbor] = inherited_cost
                    f_score[neighbor] = f
                    trace[neighbor] = u
                    heapq.heappush(minHeap, (f, neighbor))
                    
        path = []
        current = destination
        while current != -1:
            path.append(current)
            current = trace[current]
            if current == source:
                path.append(source)
                break
    
        if current != source:
            return self.INF, []
        
        path.reverse()
            
        return f_score[destination], path
    # 2) Caching some segments of important routes to reduce the number of calculations - The first 1000 important routes.
    # 2.1) Precompute important routes and cache them.
    def calculate_importance(self):
        self.dijkstraAll()
        self.countImportantStops()
    def cache_thousand_paths(self):
        self.top_thousand_impo = self.loadTopImpo()
    def init_impo(self):
        impo_cache = {}
        
        for u in self.top_thousand_impo:
            if u not in impo_cache:
                impo_cache[u] = {}
            for v in self.top_thousand_impo:
                if v not in impo_cache[u]:
                    impo_cache[u][v] = {}
                impo_cache[u][v]['time'] = 0
                impo_cache[u][v]['path'] = []
                impo_cache[u][v]['cache'] = False    

        with open('input/impo_cache.json', 'w') as file:
            json_object = json.dumps(impo_cache, indent = 4)
            file.write(json_object)
            file.close()
    # 2.2) Get the cached important routes while searching for the shortest path.
    def a_star_search_cache(self, source, destination):
        if str(source) in self.cache and str(destination) in self.cache:
            if (self.cache[f'{source}'][f'{destination}']['cache'] == True):
                print('Found cache important route! No need to calculate [important routes].')
                return self.cache[f'{source}'][f'{destination}']['time'], self.cache[f'{source}'][f'{destination}']['path']
        
        g_score = {v: self.INF for v in self.numVertices}
        f_score = {v: self.INF for v in self.numVertices}
        trace = {v: -1 for v in self.numVertices}
        
        g_score[source] = 0
        f_score[source] = self.get_heuristic(source, destination)
        
        minHeap = []
        heapq.heappush(minHeap, (f_score[source], source))
        visited = set()

        while minHeap:
            f_cur, u = heapq.heappop(minHeap)
            if u == destination:
                break
            visited.add(u)
            
            # Optimization algorithm
            if (f_cur > f_score[u]): 
                continue
            
            for info, neighbor, routeInfo in self.vertices[u]:
                if neighbor in visited:
                    continue
                
                time = info[0]
                distance = info[1]
                routeId = routeInfo[0]
                routeVarId = routeInfo[1]
                
                inherited_cost = g_score[u] + time
                f = inherited_cost + self.get_heuristic(neighbor, destination)
                
                if f < f_score[neighbor]:
                    g_score[neighbor] = inherited_cost
                    f_score[neighbor] = f
                    trace[neighbor] = u
                    heapq.heappush(minHeap, (f, neighbor))
                    
        path = []
        current = destination
        while current != -1:
            path.append(current)
            current = trace[current]
            if current == source:
                path.append(source)
                break
    
        if current != source:
            return self.INF, []
        
        path.reverse()
                    
        if str(source) in self.cache and str(destination) in self.cache:
            print('Found new important route! Caching [important routes].')
            self.cache[f'{source}'][f'{destination}']['time'] = f_score[destination]
            self.cache[f'{source}'][f'{destination}']['path'] = path
            self.cache[f'{source}'][f'{destination}']['cache'] = True
            
        return f_score[destination], path
    
    def haversine(lat1, lon1, lat2, lon2):
        # Radius of the Earth in kilometers
        R = 6371.0

        # Convert latitude and longitude from degrees to radians
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)

        # Haversine formula
        a = math.sin(delta_phi / 2.0)**2 + \
            math.cos(phi1) * math.cos(phi2) * \
            math.sin(delta_lambda / 2.0)**2

        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        # Distance in kilometers
        distance = R * c
        
        return distance * 1000
