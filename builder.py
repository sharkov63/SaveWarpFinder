import math, sys, queue, threading, multiprocessing, re, signal, time
from collections import defaultdict

WAYPOINTS_FILE = 'waypoints_hl2.txt'
GRAPH_FILE = 'landmark_hl2.txt'

DESTINATION = 'd1_canals_01 at: 763 x, 3327 y, 668 z'
WAYPOINT_DISTANCE = 4000

# This transitions will not be used in routes (one way direction ban!)
BANNED_TRANSITIONS = [
	# from_map via_landmark to_map
	
	# backward transitions
	["d1_trainstation_02","trainstation_01_to_02","d1_trainstation_01"],
	["d1_trainstation_04","landmark_trainstation_04-05","d1_trainstation_03"],
	["d1_trainstation_05","landmark_trainstation_04-05","d1_trainstation_04"],
	["d1_canals_01a","canals_trans_0101a","d1_canals_01"],
	["d1_canals_02","canals_trans_01a02","d1_canals_01a"],
	["d1_canals_06","canals_05_06","d1_canals_05"],
	["d1_eli_01","canals_trans_13_eli","d1_canals_13"],
	# yet all
	
	# transitions to self (?)
	["d1_trainstation_02","landmark_trainstation_02-03","d1_trainstation_02"],
	["d1_trainstation_03","landmark_trainstation_03-04","d1_trainstation_03"],
	["d1_trainstation_03","landmark_trainstation_02-03","d1_trainstation_03"],
	["d1_trainstation_05","landmark_trainstation_04-05","d1_trainstation_05"],
	["d1_canals_06","canals_05_06","d1_canals_06"],
	["d1_canals_06","canals_06_07","d1_canals_06"],
	["d1_canals_07","trans_canals_07_08","d1_canals_07"],
	["d1_canals_08","trans_canals_07_08","d1_canals_08"],
	["d1_canals_09","canals_trans_0809","d1_canals_09"],
	["d1_canals_09","trans_canals_09_10","d1_canals_09"],
	["d1_canals_10","trans_canals_09_10","d1_canals_10"],
	["d1_canals_11","canals_trans_10_11","d1_canals_11"],
	["d1_canals_11","landmark_canals_11_12","d1_canals_11"],
	["d1_canals_12","landmark_canals_11_12","d1_canals_12"],
	["d1_canals_12","trans_canals_12_13","d1_canals_12"],
	["d1_canals_13","trans_canals_12_13","d1_canals_13"],
	["d2_coast_04","landmark_d2_coast_04-05","d2_coast_04"],
	["d2_coast_10","landmark_d2_coast_10-11","d2_coast_10"],
	["d2_coast_11","landmark_d2_coast_10-11","d2_coast_11"],
	["d2_coast_11","landmark_d2_coast_11-12","d2_coast_11"],
	["d2_coast_12","landmark_d2_coast_11-12","d2_coast_12"],
	["d2_prison_06","landmark_d2_prison_06-07","d2_prison_06"],
	["d2_prison_07","landmark_d2_prison_06-07","d2_prison_07"],
	["d2_prison_07","landmark_d2_prison_07-08","d2_prison_07"],
	["d2_prison_08","landmark_d2_prison_07-08","d2_prison_08"],
	["d3_citadel_02","trans_cit02_cit03","d3_citadel_02"],
	["d3_citadel_03","trans_cit02_cit03","d3_citadel_03"],
	["d3_citadel_04","trans_citadel_0304","d3_citadel_04"],
	["d3_citadel_04","trans_cit04_cit05","d3_citadel_04"],
	["d3_citadel_05","trans_cit04_cit05","d3_citadel_05"]
]

VOID_MAPS = [
	"d1_trainstation_01",
	"d1_trainstation_02",
	"d1_trainstation_03",
	"d1_trainstation_04",
	"d1_trainstation_05",
	"d1_trainstation_06",
	"d1_canals_01",
	"d1_canals_01a",
	"d1_canals_02",
	"d1_canals_03",
	"d1_canals_05",
	"d1_canals_06",
	"d1_canals_07",
	"d1_canals_08",
	"d1_canals_09",
	"d1_canals_10",
	"d1_canals_11",
	"d1_canals_12",
	"d1_canals_13",
	"d1_eli_01",
	"d1_eli_02",
	"d1_town_01",
	"d1_town_01a",
	"d1_town_02",
	"d1_town_03",
	"d1_town_04",
	"d1_town_05",
	"d2_coast_01",
	"d2_coast_03",
	"d2_coast_04",
	"d2_coast_05",
	"d2_coast_07",
	"d2_coast_08",
	"d2_coast_09",
	"d2_coast_10",
	"d2_coast_11",
	"d2_coast_12",
	"d2_prison_01",
	"d2_prison_02",
	"d2_prison_03",
	"d2_prison_04",
	"d2_prison_05",
	"d2_prison_06",
	"d2_prison_07",
	"d2_prison_08",
	"d3_c17_01",
	"d3_c17_02",
	"d3_c17_03",
	"d3_c17_04",
	"d3_c17_05",
	"d3_c17_06a",
	"d3_c17_06b",
	"d3_c17_07",
	"d3_c17_08",
	"d3_c17_09",
	"d3_c17_10a",
	"d3_c17_10b",
	"d3_c17_11",
	"d3_c17_12",
	"d3_c17_12b",
	"d3_c17_13",
	"d3_citadel_01",
	"d3_citadel_02",
	"d3_citadel_03",
	"d3_citadel_04",
	"d3_citadel_05",
	"d3_breen_01"
]

MAX_PATH_LENGTH = 35

INT_MAX = 1 << 32

DST_MAP = ''
DST_ORIGIN = []

WARP_START_MAP = 'd1_trainstation_01'
MAIN_COORD_MAP = WARP_START_MAP

THREAD_COUNT = multiprocessing.cpu_count()

def InitDestination(x):
	global DST_MAP, DST_ORIGIN
	part = x.split(' at: ')
	DST_MAP = part[0]
	DST_ORIGIN = [int(e[:-2]) for e in part[1].split(', ')]

#
# Common
#
	
def intersection(listA, listB):
	return [x for x in listA if x in listB]

#
# Vector
#

def vec_len(vec):
	tmp = 0
	for val in vec:
		tmp += val**2
	return math.sqrt(tmp)
	
def vec_add(vec1, vec2):
	assert(len(vec1) == len(vec2))
	tmp = []
	for i in range(len(vec1)):
		tmp.append(vec1[i] + vec2[i])
	return tmp
	
def vec_neg(vec):
	tmp = []
	for val in vec:
		tmp.append(-val)
	return tmp
	
def vec_sub(vec1, vec2):
	return vec_add(vec1, vec_neg(vec2))
	
def vec_distance(vec1, vec2):
	return vec_len(vec_sub(vec1, vec2))
	
def vec_mul_scalar(vec, c):
	tmp = []
	for i in range(len(vec)):
		tmp.append(vec[i] * c)
	return tmp	
	
def vec_str(vec):
	return ' '.join([str(x) for x in vec])
	
def distance(x, y):
	return vec_len(vec_sub(x, y))

#
# Graph, maps
#
Graph = []
Maps = []

def ReadGraph(filename):
	global Graph, Maps
	
	fp = open(filename, 'r')
	if not fp:
		print('No graph in ' + filename)
		exit(-1)
	lines = fp.read().split('\n')
	for line in lines:
		data = line.split(',')
		data = [e[1:-1] for e in data]
		if data[0] == data[2]:
			continue # weird self trigger
		Graph += [(data[0], data[1], data[2], [float(e) for e in data[3].split(' ')])]
	fp.close()		
	#
	Maps = set()
	landmarks = set()
	for node in Graph:
		Maps.add(node[0])
		Maps.add(node[2])	
	Maps = list(Maps)

def GetConnections(map_1, map_2):
	global Graph
	result = []
	for node in Graph:
		if node[0] == map_1 and node[2] == map_2 and [node[1]] not in result:
			result.append(node[1])
	return result
	
def GetOutConnections(map_1):
	global Graph
	result = []
	for node in Graph:
		if node[0] == map_1 and [node[2], node[1]] not in result:
			result.append([node[2], node[1]])
	return result
	
def GetInConnections(map_1):
	global Graph
	result = []
	for node in Graph:
		if node[2] == map_1 and [node[0], node[1]] not in result:
			result.append([node[0], node[1]])
	return result

def GetStrongPairConnections(map_1, map_2):
	return intersection(GetConnections(map_1, map_2), GetConnections(map_2, map_1))

def GetStrongConnections(map):
	result = []
	out_connections = GetOutConnections(map)
	for connect in out_connections:
		strong_connections = GetStrongPairConnections(map, connect[0])
		for x in strong_connections:
			result += [[connect[0], x]]
	return result
	
def GetAnyStrongConnection(map_1, map_2):
	result = GetStrongPairConnections(map_1, map_2)
	return result[0] if result != [] else None

def GetLandmarkOrigin(mapName, landmarkName):
	global Graph
	for node in Graph:
		if node[0] == mapName and node[1] == landmarkName:
			return node[3]
	return [0, 0, 0]

#
# Graph d,p
#
Maps_d = []
Maps_p = []

def BuildWays():
	global Maps, Maps_d, Maps_p
	Maps_d = [[INT_MAX for i in range(len(Maps))] for j in range(len(Maps))]
	Maps_p = [[-1 for i in range(len(Maps))] for j in range(len(Maps))]	
	for i in range(len(Maps)):
		for j in range(len(Maps)):
			lm = GetAnyStrongConnection(Maps[i], Maps[j])
			if lm and i != j:
				Maps_d[i][j] = 1;
				Maps_d[j][i] = 1
				Maps_p[i][j] = j
				Maps_p[j][i] = i
				
	for i in range(len(Maps)):
		for u in range(len(Maps)):
			for v in range(len(Maps)):
				if u != v:
					if Maps_d[u][i] < INT_MAX and Maps_d[i][v] < INT_MAX:
						if Maps_d[u][i] + Maps_d[i][v] < Maps_d[u][v]:
							Maps_d[u][v] = Maps_d[u][i] + Maps_d[i][v]
							Maps_p[u][v] = Maps_p[u][i]
												

def FixOrigin(map_from, map_to, coord):
	global Maps, Maps_d, Maps_p
	START_IDX = Maps.index(map_from)
	FINISH_IDX = Maps.index(map_to)
	current = START_IDX
	while Maps_d[current][FINISH_IDX] != INT_MAX:	
		next = Maps_p[current][FINISH_IDX]
		landmark = GetAnyStrongConnection(Maps[next], Maps[current])
		assert(landmark != None)
		coord = vec_add(coord, vec_sub(
			GetLandmarkOrigin(Maps[next], landmark), 
			GetLandmarkOrigin(Maps[current], landmark))) 
		current = next	
	return coord
	
#	
# Map_order
#
Maps_order = []

def BuildMapOrder(map_start, map_finish):
	global Maps, Maps_d, Maps_p, Maps_order
	START_IDX = Maps.index(map_start)
	FINISH_IDX = Maps.index(map_finish)
	current = START_IDX
	while Maps_d[current][FINISH_IDX] != INT_MAX:
		Maps_order += [Maps[current]]
		current = Maps_p[current][FINISH_IDX]
	Maps_order += [Maps[current]]
	# print('\n'.join(Maps_order))						
												
def GetMapWarps(save_map):
	global Maps_order
	result = []

	save_map_idx = Maps_order.index(save_map)
	X = Maps_order[save_map_idx]
	# landmarks
	array_Y_landmark_XY = GetStrongConnections(X)
	for Y, landmark_XY in array_Y_landmark_XY:
		if Y not in Maps_order:
			continue  # map outside of our way
		in_connections_Y = GetInConnections(Y)
		for Z, landmark_ZY in in_connections_Y:
			if Z not in Maps_order:
				continue  # map outside of our way
			if [Z, landmark_ZY, Y] in BANNED_TRANSITIONS:
				continue
			# SWT: changelevel2 Y landmark_ZY
			A = GetLandmarkOrigin(X, landmark_XY)
			B = GetLandmarkOrigin(X, landmark_ZY)	
			C = GetLandmarkOrigin(Y, landmark_ZY)	
			D = GetLandmarkOrigin(Y, landmark_XY)		
			vec_result = vec_add(vec_sub(C, B), vec_sub(A, D))
			if vec_result == [0, 0, 0]:
				continue # save warp
			direction = -1 if Maps_order.index(Y) < Maps_order.index(X) else 1
			result += [[vec_result, Y, X, landmark_ZY, direction, landmark_XY]]
	return result
	
#
# Waypoints
# 
Waypoints = []
	
def ReadWaypoints(filename, main_map):
	global Waypoints, Maps_order
	fp = open(filename, 'r')
	if not fp:
		print('No waypoints in ' + filename)
		return None
		exit(-1);
	for line in fp:
		data = line.split()
		if len(data) == 8:
			coord = [float(data[2]), float(data[4]), float(data[6])]			
			if data[0] not in Maps_order[:-1]:
				continue
			coord_global = FixOrigin(data[0], main_map, coord)
			Waypoints += [[coord_global, data[0]]]
	print('\n'.join([str(x) for x in Waypoints]))
	fp.close()
	
def HowCloseToWaypoint(vec, isVoidMapInPath):
	global Waypoints	
	for point in Waypoints[::-1]:
		info = 0
		shift_z = point[0][2] - vec[2]
		if isVoidMapInPath and shift_z > 0:
			dist = distance(point[0][:2], vec[:2])
			info = shift_z
		else:
			dist = distance(point[0], vec)
		if dist < WAYPOINT_DISTANCE: 
			return dist, point[1], info # doesn't look for a mimimum, might be a problem?
	return -1, '', 0
	
#
# Warp Vectors
#
WarpVectors = []
WarpDict = defaultdict(list) # dst -> [[vec_result, X, landmark_ZY, direction]]

def GetWarpVectors():
	global Maps_order, WarpVectors
	for map in Maps_order:
		WarpVectors += GetMapWarps(map)
	print('\n'.join([str(x) for x in WarpVectors]))
	for warp in WarpVectors:
		new_warp = [warp[0], warp[2], warp[3], warp[4]]
		if new_warp not in WarpDict[warp[1]]:
			WarpDict[warp[1]] += [new_warp]
#
# Threading
#

counterLock = threading.Lock()
lastRouteLen = 1

def simplifyRoute(route):
	return re.sub(r'\(.+?\)', '', ''.join(route))

def calcRouteLen(route):
	return len(route)
	
def reverseRoute(route):
	return route[::-1]

processedSaved = set()
stdout_lock = threading.Lock()
	
def PathFinder(q, startMapIndex, dstMapIndex):
	global BANNED_TRANSITIONS, MAIN_COORD_MAP, MAX_PATH_LENGTH, Maps_order, lastRouteLen, WarpDict, stdout_lock
	
	while q.qsize():
		currentMapIndex, currentCoordinates, isVoidMapCrossed, route = q.get()

		if str((currentCoordinates, currentMapIndex)) not in processedSaved:
			processedSaved.add(str((currentCoordinates, currentMapIndex)))
		else:
			q.task_done()
			continue
		
		routeLen = calcRouteLen(route)
		if lastRouteLen / THREAD_COUNT < routeLen:
			with counterLock:
				lastRouteLen += 1
			if lastRouteLen % THREAD_COUNT == 0:
				with stdout_lock:
					print('Search for path not longer than ' + str(int(lastRouteLen / THREAD_COUNT)) + ' steps complete!')
					sys.stdout.flush()
					
		if currentMapIndex < startMapIndex or currentMapIndex > dstMapIndex:
			q.task_done()
			continue
		
		dist, save_map, shift_z = HowCloseToWaypoint(currentCoordinates, isVoidMapCrossed)
		if dist >= 0:
			if save_map == Maps_order[currentMapIndex] and routeLen > 0:
				currentCoordinates_save_map = FixOrigin(MAIN_COORD_MAP, save_map, currentCoordinates)
				with stdout_lock:
					print("="*20)
					print("   Destination: " + DESTINATION)
					print("   " + str(int(dist)) + " - " + save_map)	
					shift_vec = [0, 0, -shift_z]
					print("   map "+save_map+";setpos " + vec_str(vec_sub(currentCoordinates_save_map, shift_vec)))
					if shift_vec != [0, 0, 0]:
						print("   bxt_ch_set_pos_offset " + vec_str(shift_vec))
					print("   // " + simplifyRoute(reverseRoute(route)))
					print("   // " + ' '.join(reverseRoute(route)))
					print("="*20)
					sys.stdout.flush()
		
							
		if routeLen == MAX_PATH_LENGTH:
			q.task_done()
			continue
		if not isVoidMapCrossed:
			if Maps_order[currentMapIndex] in VOID_MAPS:
				isVoidMapCrossed = True	
				
		# sw
		if currentMapIndex > startMapIndex:
			for lm in GetStrongPairConnections(Maps_order[currentMapIndex], Maps_order[currentMapIndex - 1]):
				if [Maps_order[currentMapIndex - 1], lm, Maps_order[currentMapIndex]] in BANNED_TRANSITIONS:
					continue
				q.put([currentMapIndex - 1, currentCoordinates, isVoidMapCrossed, route + ['l(' + Maps_order[currentMapIndex] + ',' + lm +')']])
				break  # any save warp
		# bsw
		if currentMapIndex < dstMapIndex:
			for lm in GetStrongPairConnections(Maps_order[currentMapIndex], Maps_order[currentMapIndex + 1]):
				if [Maps_order[currentMapIndex + 1], lm, Maps_order[currentMapIndex]] in BANNED_TRANSITIONS:
					continue
				q.put([currentMapIndex + 1, currentCoordinates, isVoidMapCrossed, route + ['k(' + Maps_order[currentMapIndex] + ',' + lm +')']])
				break  # any save warp
		
		if Maps_order[currentMapIndex] in WarpDict:
			for x in WarpDict[Maps_order[currentMapIndex]]:
				if x[3] == +1: # ww
					q.put([Maps_order.index(x[1]), vec_sub(currentCoordinates, x[0]), isVoidMapCrossed, route + ['x(' + Maps_order[currentMapIndex] + ',' + x[2]+')']])
				if x[3] == -1: # bww
					q.put([Maps_order.index(x[1]), vec_sub(currentCoordinates, x[0]), isVoidMapCrossed, route + ['v(' + Maps_order[currentMapIndex] + ',' + x[2]+')']])
				
		q.task_done()

def SearchPathTo(startMapIndex, dstMapIndex, coordinates):
	q = queue.Queue()
	q.put([dstMapIndex, coordinates, False, []])
	for i in range(THREAD_COUNT):
		t = threading.Thread(target=PathFinder, args=(q, startMapIndex, dstMapIndex))
		t.daemon = True
		t.start()
	while q.unfinished_tasks:
		time.sleep(1)
	print('DIE', q.qsize())

def signal_handler(signal, frame):
	print('Ctrl+C triggered!')
	sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
	
InitDestination(DESTINATION)
ReadGraph(GRAPH_FILE)
BuildWays()
BuildMapOrder(WARP_START_MAP, DST_MAP)
ReadWaypoints(WAYPOINTS_FILE, MAIN_COORD_MAP)
GetWarpVectors()

print('')
print('Warps:')
for x in WarpDict:
    print ('+ ', x)
    for y in WarpDict[x]:
        print ('    ', y)

print('')
print('Config:')
print(' swarp_ban_clear')
for x in BANNED_TRANSITIONS:
	print(' swarp_ban ' + ' '.join(x))
print(' swarp_rebuild_graph')
print('')

print('Searching for path to ' + DESTINATION)
print('Waypoints loaded: ' + str(len(Waypoints)))
# print(*Waypoints)
print('Warps loaded: ' + str(len(WarpVectors)))
print('Thread count: ' + str(THREAD_COUNT))

SearchPathTo(Maps_order.index(WARP_START_MAP), Maps_order.index(DST_MAP), FixOrigin(DST_MAP, MAIN_COORD_MAP, DST_ORIGIN))
time.sleep(2)
print('Complete! Terminating...')