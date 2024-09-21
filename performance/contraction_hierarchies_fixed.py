from shortestPath.fixedPoints import *
from network.contraction_hierarchies.CH import *

class CH_Fixed(Fixed_Caching, Contraction_Hierarchies):
    def __init__ (self, vars, stops, paths):
        super().__init__(vars, stops, paths)
        self.pre_processing()
        
    def shortest_path_CH_fixed(self, source, destination):
        u_zone = self.zoneAll[source]
        v_zone = self.zoneAll[destination]
        
        if (u_zone == v_zone):
            time, path = self.query(source, destination)
            return time, path
        elif (self.fixed_route[u_zone][v_zone]['have_fixed'] == False):
            time, path = self.query(source, destination)
            return time, path
        elif (self.fixed_route[u_zone][v_zone]['have_fixed'] == True):
            new_time, new_path = self.query(source, destination) 
            
            cache_time = self.fixed_route[u_zone][v_zone]['time']
            cache_path = self.fixed_route[u_zone][v_zone]['path']
            
            time1, path1 = self.query(source, cache_path[0])
            time2, path2 = self.query(cache_path[-1], destination)

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
            time, path = self.query(source, destination)  
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