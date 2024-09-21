import heapq
import math
from constructions.var import *
from constructions.stop import *
from constructions.path import *
from math import sin, cos, sqrt, atan2, radians
from pyproj import Transformer
from time import sleep
from tqdm import tqdm
from collections import deque

"""
Optimize graph from the last semester by 
deleting unnecessary variables and splitting
files into smaller modules.

[module] : [graph] -> [optimize]
"""

lat_lng_crs = "EPSG:4326"  
target_crs = "EPSG:3405"  
transformer = Transformer.from_crs(lat_lng_crs, target_crs)

topo = []
visited = {}
edges = {}

def dfs(u):
    global topo
    global visited
    global edges
    visited[u] = True
    for v in edges[u]:
        if not visited[v]:
            dfs(v)
    topo.append(u)

class Graph_Optimized():
    #-------------------------------------------------GRAPH-INITIALIZATION-------------------------------------------------
    def __init__(self, fileName1, fileName2, fileName3):
        varQuery = RouteVarQuery(fileName1)
        stopQuery = StopQuery(fileName2)
        pathQuery = PathQuery(fileName3)

        self.INF = 10**9
        self.maxSpeed = -1e9

        # Initialize graph
        self.numVertices = set([i.StopId for i in tqdm(stopQuery.StopList)])
        self.vertices = {}
        self.pathAll = {}
        self.timeAll = {}
        self.zoneAll = {}
        self.coordinatesAll = {} 
        
        # For Contraction Hierarchies
        self.children = {}
        self.parents = {}
        
        # Cache 1000 important routes    
        self.cache = {}
        self.top_thousand_impo = [] 
        
        for i in tqdm(self.numVertices):
            self.vertices[i] = []
            self.children[i] = set()
            self.parents[i] = set()
            self.pathAll[i] = {}
            self.timeAll[i] = {}
            
            for j in self.numVertices:
                self.timeAll[i][j] = self.INF


        for routeVarObject in tqdm(varQuery.routeVarList):
            averageSpeed = routeVarObject.Distance / (routeVarObject.RunningTime * 60)
            routeId = routeVarObject.RouteId
            routeVarId = routeVarObject.RouteVarId

            stopObject = stopQuery.searchByABC(RouteId = str(routeId), RouteVarId = str(routeVarId))
            pathObject = pathQuery.searchByABC(RouteId = str(routeId), RouteVarId = str(routeVarId))[0]

            stops = [[stop.StopId, stop.Lat, stop.Lng, stop.Zone] for stop in stopObject]
            coordinates = [[x, y] for x, y in zip(pathObject.lat, pathObject.lng)]

            startStopId = stops[0][0]
            
            self.coordinatesAll[startStopId] = stops[0][1], stops[0][2]
            self.zoneAll[startStopId] = stops[0][3]
            self.maxSpeed = max(self.maxSpeed, averageSpeed)
            
            stops = stops[1:]

            for stop in stops:
                endStopId = stop[0]
                
                self.coordinatesAll[endStopId] = stop[1], stop[2]
                self.zoneAll[endStopId] = stop[3]

                x = stop[1]
                y = stop[2]

                minDist = self.INF

                for i in range(len(coordinates)):
                    curDist = self.distanceLL(x, y, coordinates[i][0], coordinates[i][1])
                    if curDist < minDist:
                        minDist = curDist
                        closestPoint = i
                
                distance = 0
                path = coordinates[0:closestPoint + 1]
                pathXY = [[*transformer.transform(point[0],point[1])] for point in path]
                coordinates = coordinates[closestPoint:]

                for p1, p2 in zip(pathXY, pathXY[1:]):
                    distance += self.distanceXY(p1[0], p1[1], p2[0], p2[1])

                time = distance / averageSpeed
                
                # Save edges from one stop to another (with time and distance).
                self.vertices[startStopId].append(((time, distance), endStopId, (routeId, routeVarId)))          
                   
                """ 
                Opimization step:
                - For each pair vertices, we will find the shortest edges between them. Excluding all
                the unnecessary edges between them.
                
                """  
                self.children[startStopId].add(endStopId)
                self.parents[endStopId].add(startStopId)
                
                self.timeAll[startStopId][endStopId] = min(self.timeAll[startStopId][endStopId], time)
                
                # Save path between two stops.
                if endStopId not in self.pathAll[startStopId]:
                    self.pathAll[startStopId][endStopId] = path

                startStopId = endStopId
    #-------------------------------------------------GRAPH-INITIALIZATION-------------------------------------------------
    
    #---------------------------------------------------HELPER-FUNCTIONS---------------------------------------------------
     
    def distanceXY(self, x1, y1, x2, y2):
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

    def distanceLL(self, lat1, lng1, lat2, lng2):
        R = 6371.0
        lat1 = radians(lat1)   
        lng1 = radians(lng1)
        lat2 = radians(lat2)
        lng2 = radians(lng2)
        dislng = lng2 - lng1
        dislat = lat2 - lat1
        a = sin(dislat / 2)**2 + cos(lat1) * cos(lat2) * sin(dislng / 2)**2
        c = 2 * atan2(sqrt(a), sqrt(1 - a))
        return R * c * 1000

    def getLatLon(self, stopId):
        stopQuery = StopQuery("stops.json")
        stopObject = stopQuery.searchByABC(StopId=str(stopId))[0]
        return stopObject.Lat, stopObject.Lng
    
    #---------------------------------------------------HELPER-FUNCTIONS---------------------------------------------------
    
    #---------------------------------------------------SAVING-FUNCTIONS---------------------------------------------------
    
    # Load 1000 important routes
    def loadTopImpo(self):
        top_thousand_id = []
        try:
            with open('input/top_thousand_impo.txt', 'r') as file:
                for line in file:
                    line = line.strip().split(" ")
                    stopId = int(line[2])
                    stopImpo = " ".join(line[4:])
                    top_thousand_id.append([stopId, stopImpo])
            file.close()
        except Exception as e:
            print("Error: " + str(e))
        return top_thousand_id
    
    def saveTopImpo(self, top_thousand_id):
        cnt = 1
        try:
            with open('input/top_thousand_impo.txt', 'w') as file:
                for i in top_thousand_id:
                    file.write(f"{cnt}. StopID: {i[0]} - {i[1]}\n")
                    cnt += 1
            file.close()
            print("Top important stops saved successfully.")
        except Exception as e:
            print("Error: " + str(e))
                       
    # Important routes caching 
    def saveImpoCache(self):
        try:
            with open('input/impo_cache.json', 'w') as file:
                json_object = json.dumps(self.cache, indent = 4)
                file.write(json_object)
            file.close()
        except Exception as e:
            print("Error: " + str(e))
            
    def loadImpoCache(self):
        try:
            with open('input/impo_cache.json', 'r') as file:
                self.cache = json.load(file)
            print("Load cache [important routes] successfully.")
            file.close()
        except Exception as e:
            print("Error: " + str(e))
    
    # Fixed points caching
    def save_fixed_route(self):
        with open('input/fixed_route.json', 'w', encoding='utf-8') as file:
            json_object = json.dumps(self.fixed_route, indent = 4, ensure_ascii = False)
            file.write(json_object)
            file.close()
    
    def load_fixed_route(self):
        with open('input/fixed_route.json', 'r', encoding='utf-8') as file:
            self.fixed_route = json.load(file)
            print("Load cache [fixed routes] successfully.")
            file.close()
            
    # Save important routes caching and fixed points caching
    def save_all_cache(self):
        print("Cache saving...")
        self.saveImpoCache()
        self.save_fixed_route()  
        print("Cache saved successfully.")     
    
    #---------------------------------------------------SAVING-FUNCTIONS---------------------------------------------------
