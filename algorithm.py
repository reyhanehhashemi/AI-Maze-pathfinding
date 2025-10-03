"""
This file contains function headers for various pathfinding algorithms.
Students are responsible for implementing these algorithms.

IMPORTANT:
- Do NOT include any graphical elements in this file.
- Use `update_visualization function` to update the visualization.
"""

import heapq
import time
from collections import deque
from graphics import update_visualization, reconstruct_path
from graphics import update_visualization


def heuristic(cell, goal):
   
    return abs(cell[0] - goal[0]) + abs(cell[1] - goal[1])

def A_star(graph, start, goal, pathMap, ax2, ax3):
    
     
    start_time = time.time()
    
    
    open_list = [start]      
    closed_list = []         
    
    
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    
    came_from = {}  
    expanded = 0
    
    while open_list:
        
        current = min(open_list, key=lambda node: f_score[node])
        open_list.remove(current)
        closed_list.append(current)
        
        
        if current == goal:
            final_path = reconstruct_path(came_from, current, pathMap, ax3, "A* Final Path", start, goal)
            elapsed = time.time() - start_time
            return final_path, expanded, elapsed
        
        
        x, y = current
        if current != start and current != goal:
            pathMap[x, y] = 13  
        expanded += 1
        update_visualization(pathMap, ax2, "A* Search Progress")
        
       
        for neighbor in graph[current]:
            
            if pathMap[neighbor] == 0:
                continue
            
            if neighbor in closed_list:
                continue
            
            tentative_g = g_score[current] + 1 
            
            if neighbor not in open_list or tentative_g < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                if neighbor not in open_list:
                    open_list.append(neighbor)
    
   
    elapsed = time.time() - start_time
    return None, expanded, elapsed


def bfs(graph, start, goal, pathMap, ax2, ax3):
 
    start_time = time.time()
    queue = deque([start])
    came_from = {start: None}
    visited = set([start])
    expanded = 0

    while queue:
        current = queue.popleft()
        
        
        if current == goal:
            final_path = reconstruct_path(came_from, current, pathMap, ax3, "BFS Final Path", start, goal)
            elapsed = time.time() - start_time
            return final_path, expanded, elapsed
        
       
        x, y = current
        if current != start and current != goal:
            pathMap[x, y] = 13  
        update_visualization(pathMap, ax2, "BFS Search Progress")
            
        expanded += 1
        
        
        update_visualization(pathMap, ax2, "BFS Search Progress")
        
       
        for neighbor in graph[current]:
            if neighbor not in visited:
                visited.add(neighbor)
                came_from[neighbor] = current
                queue.append(neighbor)
    
    
    elapsed = time.time() - start_time
    return None, expanded, elapsed


def dfs(graph, start, goal, pathMap, ax2, ax3):
 
    start_time = time.time()
    
    
    stack = [start]
   
    came_from = {start: None}
   
    visited = set([start])
    expanded = 0  

    while stack:
        current = stack.pop()

    
        if current == goal:
            final_path = reconstruct_path(came_from, current, pathMap, ax3, "DFS Final Path", start, goal)
            elapsed = time.time() - start_time
            return final_path, expanded, elapsed
        
        
        x, y = current
        if current != start and current != goal:
            pathMap[x, y] = 13  
        update_visualization(pathMap, ax2, "DFS Search Progress")
        
        expanded += 1
        
       
        update_visualization(pathMap, ax2, "DFS Search Progress")
        
      
        for neighbor in graph[current]:
            if neighbor not in visited:
                visited.add(neighbor)
                came_from[neighbor] = current
                stack.append(neighbor)
    
 
    elapsed = time.time() - start_time
    return None, expanded, elapsed

def greedy(graph, start, goal, pathMap, ax2, ax3):

    start_time = time.time()
    
    
    open_set = []
    heapq.heappush(open_set, (heuristic(start, goal), start))
    

    came_from = {start: None}
    
    
    visited = set([start])
    
    expanded = 0 
    
    while open_set:
        current_priority, current = heapq.heappop(open_set)
        
     
        if current == goal:
            final_path = reconstruct_path(came_from, current, pathMap, ax3, "Greedy Final Path", start, goal)
            elapsed = time.time() - start_time
            return final_path, expanded, elapsed
        
      
        x, y = current
        if current != start and current != goal:
            pathMap[x, y] = 13 
        update_visualization(pathMap, ax2, "Greedy Search Progress")
        expanded += 1
        
       
        update_visualization(pathMap, ax2, "Greedy Search Progress")
        
        
        for neighbor in graph[current]:
            if neighbor not in visited:
                visited.add(neighbor)
                came_from[neighbor] = current
                
                heapq.heappush(open_set, (heuristic(neighbor, goal), neighbor))
    
   
    elapsed = time.time() - start_time
    return None, expanded, elapsed


def depth_limited_search(graph, current, goal, depth, came_from, pathMap, ax2, expanded_nodes):

    x, y = current
    if came_from.get(current) is not None and current != goal:
        
        pathMap[x, y] = 13  

   
    update_visualization(pathMap, ax2, "IDS Search Progress")
    
   
    expanded_nodes[0] += 1


    if current == goal:
        return current


    if depth == 0:
        return "cutoff"

    cutoff_occurred = False

   
    for neighbor in graph[current]:
     
        if neighbor not in came_from:
            came_from[neighbor] = current
            result = depth_limited_search(graph, neighbor, goal, depth - 1, came_from, pathMap, ax2, expanded_nodes)
            if result == "cutoff":
                cutoff_occurred = True
            elif result is not None:
                return result


    if cutoff_occurred:
        return "cutoff"
    else:
        return None

def iterative_deepening_search(graph, start, goal, pathMap, ax2, ax3):

    start_time = time.time()
    depth = 0
    total_expanded = 0

    while True:
      
        came_from = {start: None}
       
        expanded_nodes = [0]
        
  
        result = depth_limited_search(graph, start, goal, depth, came_from, pathMap, ax2, expanded_nodes)
        total_expanded += expanded_nodes[0]

   
        if result != "cutoff" and result is not None:
            final_path = reconstruct_path(came_from, result, pathMap, ax3, "IDS Final Path", start, goal)
            elapsed = time.time() - start_time
            return final_path, total_expanded, elapsed
        
       
        if result is None:
            elapsed = time.time() - start_time
            return None, total_expanded, elapsed
        
     
        depth += 1
        
        

