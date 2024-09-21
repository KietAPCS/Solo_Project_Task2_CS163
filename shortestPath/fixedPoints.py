from network.optimize import *
from shortestPath.a_star import *
from collections import Counter

"""
Improving A* algorithm with caching fixed routes.
Diving shortest paths into smaller becauons

[modules] : [a_star] -> [fixed_points]
"""
class Fixed_Caching(A_Star):
    def __init__ (self, vars, stops, paths):
        super().__init__(vars, stops, paths)
        #self.cache_thousand_paths() 
        self.loadImpoCache()  # Load cache 1000 important paths
        self.fixed_route = {}
        self.load_fixed_route() # Load cache fixed route
    # 2.3 Fixed point integrated with caching routes - segments.
    def save_stop_json(self):
        stop_zone = {}
        for i in tqdm(self.numVertices):
            stop_zone[i] = {}
            stop_zone[i]['lat'] = self.coordinatesAll[i][0]
            stop_zone[i]['lng'] = self.coordinatesAll[i][1]
            stop_zone[i]['zone'] = self.zoneAll[i]
            
        with open('output/stop_zone.json', 'w', encoding='utf-8') as file:
            json_object = json.dumps(stop_zone, indent = 4, ensure_ascii = False)
            file.write(json_object)
            file.close()
        print("Stop zone saved successfully.")
    
    def init_cache(self):
        with open('input/stop_zone.json', 'r', encoding='utf-8') as file:
            stop_zone = json.load(file)
            file.close()
        
        fixed_route = {}
        for u in tqdm(self.numVertices):
            for v in self.numVertices:
                u_zone = stop_zone[str(u)]['zone']
                v_zone = stop_zone[str(v)]['zone']
                if u_zone not in fixed_route:
                    fixed_route[u_zone] = {}
                if v_zone not in fixed_route[u_zone]:
                    fixed_route[u_zone][v_zone] = {}
                
                fixed_route[u_zone][v_zone]['time'] = 0
                fixed_route[u_zone][v_zone]['path'] = []
                fixed_route[u_zone][v_zone]['have_fixed'] = ''
        
        with open('input/fixed_route.json', 'w', encoding='utf-8') as file:
            json_object = json.dumps(fixed_route, indent = 4, ensure_ascii = False)
            file.write(json_object)
            file.close()
        print("Init caching for fixed routes successfully.")
        
    def shortest_path_fixed(self, source, destination):
        u_zone = self.zoneAll[source]
        v_zone = self.zoneAll[destination]
        
        # print(f"Zone of {source} is:", u_zone)
        # print(f"Zone of {destination} is:", v_zone)
        
        if (u_zone == v_zone):
            print("Same zone, no need caching [fixed routes].")
            time, path = self.a_star_search_cache(source, destination)
            #time, path = self.find_sp_all_pairs(source, destination)
            return time, path
        elif (self.fixed_route[u_zone][v_zone]['have_fixed'] == False):
            print("Two zones have no fixed route, no need caching [fixed routes].")
            time, path = self.a_star_search_cache(source, destination)
            #time, path = self.find_sp_all_pairs(source, destination)
            return time, path
        elif (self.fixed_route[u_zone][v_zone]['have_fixed'] == True):
            print("Two zones have fixed route, use caching [fixed routes].")
            
            new_time, new_path = self.a_star_search_cache(source, destination)   
            #new_time, new_path = self.find_sp_all_pairs(source, destination)      
            cache_time = self.fixed_route[u_zone][v_zone]['time']
            cache_path = self.fixed_route[u_zone][v_zone]['path']
            time1, path1 = self.a_star_search_cache(source, cache_path[0])
            time2, path2 = self.a_star_search_cache(cache_path[-1], destination)
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
            print("Two zones have no fixed route, consider for caching [fixed routes].")
            time, path = self.a_star_search_cache(source, destination)  
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
                      
    def update_fix_route(self, path1, path2, time1, time2):  
        counter1 = Counter(path1)
        counter2 = Counter(path2)
    
        common = counter1 & counter2
        common = list(common.elements())
        
        new_time = 0

        if len(common) < 2:
            return new_time, common  
    
        for u, v in zip(common, common[1:]):
            new_time += self.timeAll[u][v]
            
        return new_time, common
     
    def precompute_all_pairs(self):
        self.dijkstraAll()
        
        for u in tqdm(self.numVertices):
            for v in self.numVertices:
                self.shortest_path_fixed(u, v)
                
        self.save_fixed_route()
        print("Precompute all pairs successfully.")
        
            
        
        
