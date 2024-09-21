from shortestPath.fixedPoints import Fixed_Caching
import heapq

"""
Benchmark [performance] module for A* algoritm.

[module] : [a_star_benchmark]
"""

class A_Star_BenchMark(Fixed_Caching):
    def __init__ (self, vars, stops, paths):
        super().__init__(vars, stops, paths)
       
    def a_star_search_cache_benchmark(self, source, destination):
        if str(source) in self.cache and str(destination) in self.cache:
            if (self.cache[f'{source}'][f'{destination}']['cache'] == True):
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
            self.cache[f'{source}'][f'{destination}']['time'] = f_score[destination]
            self.cache[f'{source}'][f'{destination}']['path'] = path
            self.cache[f'{source}'][f'{destination}']['cache'] = True
            
        return f_score[destination], path
    
    def shortest_path_fixed_benchmark(self, source, destination):
        u_zone = self.zoneAll[source]
        v_zone = self.zoneAll[destination]
        
        if (u_zone == v_zone):
            time, path = self.a_star_search_cache_benchmark(source, destination)
            #time, path = self.find_sp_all_pairs(source, destination)
            return time, path
        elif (self.fixed_route[u_zone][v_zone]['have_fixed'] == False):
            time, path = self.a_star_search_cache_benchmark(source, destination)
            #time, path = self.find_sp_all_pairs(source, destination)
            return time, path
        elif (self.fixed_route[u_zone][v_zone]['have_fixed'] == True):
            new_time, new_path = self.a_star_search_cache_benchmark(source, destination)   
            #new_time, new_path = self.find_sp_all_pairs(source, destination)      
            cache_time = self.fixed_route[u_zone][v_zone]['time']
            cache_path = self.fixed_route[u_zone][v_zone]['path']
            time1, path1 = self.a_star_search_cache_benchmark(source, cache_path[0])
            time2, path2 = self.a_star_search_cache_benchmark(cache_path[-1], destination)
            #time1, path1 = self.find_sp_all_pairs(source, cache_path[0])
            #time2, path2 = self.find_sp_all_pairs(cache_path[-1], destination)
            curTime = time1 + cache_time + time2
            curPath = path1 + cache_path + path2
            
            if new_time >= curTime:
                return curTime, curPath
            else:
                tmp_time, update = self.update_fix_route(curPath, new_path, curTime, new_time)
                if (update):
                    self.fixed_route[u_zone][v_zone]['time'] = tmp_time
                    self.fixed_route[u_zone][v_zone]['path'] = update
                    self.fixed_route[u_zone][v_zone]['have_fixed'] = True
                else:
                    self.fixed_route[u_zone][v_zone]['time'] = 0
                    self.fixed_route[u_zone][v_zone]['path'] = []
                    self.fixed_route[u_zone][v_zone]['have_fixed'] = False    
                return new_time, new_path    
        elif (self.fixed_route[u_zone][v_zone]['have_fixed'] == ""):
            time, path = self.a_star_search_cache_benchmark(source, destination)  
            # time, path = self.find_sp_all_pairs(source, destination)
            for v in path:
                if (self.zoneAll[v] != u_zone and self.zoneAll[v] != v_zone):
                    self.fixed_route[u_zone][v_zone]['time'] = time
                    self.fixed_route[u_zone][v_zone]['path'] = path
                    self.fixed_route[u_zone][v_zone]['have_fixed'] = True
                    return time, path
            self.fixed_route[u_zone][v_zone]['time'] = 0
            self.fixed_route[u_zone][v_zone]['path'] = []
            self.fixed_route[u_zone][v_zone]['have_fixed'] = False
            return time, path  