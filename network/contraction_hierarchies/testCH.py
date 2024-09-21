from CH import *
import time

graph = Contraction_Hierarchies("vars.json", "stops.json", "paths.json")

time1 = time.time()
graph.pre_processing()
time2 = time.time()
print("Preprocessing time:", time2 - time1)

totalTime, path = graph.query(1, 35)
graph.saveOneSP(totalTime, path, 1, 35)

