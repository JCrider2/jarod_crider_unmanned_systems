from Dijkstras import DijkstraFun as Dft
from AStar import AStarFun as Ast

# Map sizing, (xmin,xmax),(ymin,ymax),gs
map = [(0,10),(0,10),0.5]

# Obstacle position, list of x,y tuples
obstacle_positions = [(1,1),(4,4),(3,4),(5,0),(5,1),(0,7),(1,7),(2,7),(3,7)]
obstacle_radius = 0.25

# Start x,y tuple
start = (0,0)

# Finish x,y tuple
finish = (8,9)

robot_radius = 0.5




x2,y2 = Ast.AStarFun(map,obstacle_positions,obstacle_radius,start,finish,robot_radius)
