from constructions.var import *
from constructions.stop import *
from constructions.path import *
from network.optimize import *
from shortestPath.fixedPoints import *
from shortestPath.a_star import *
from shortestPath.dijkstra import *
from performance.a_star_benchmark import *
import time

graph = Fixed_Caching("vars.json", "stops.json", "paths.json")

while True:
    print("-------------SHORTEST_PATH_CACHING-------------")
    
    print("Enter -1 to exit.")

    x = int(input("Enter source: "))
    y = int(input("Enter destination: "))
    
    if x == -1 or y == -1:
        graph.save_all_cache()
        break
    
    time1 = time.time()
    totalTime, path = graph.a_star_search_cache(x, y)
    #print(path)
    time2 = time.time()
    graph.saveOneSP(totalTime, path, x, y)
    print("Total time processing:", time2 - time1)
    







    
    












 


