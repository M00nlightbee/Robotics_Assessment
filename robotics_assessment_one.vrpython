#region VEXcode Generated Robot Configuration
import math
import random
from vexcode_vr import *

# Brain should be defined by default
brain=Brain()

drivetrain = Drivetrain("drivetrain", 0)
pen = Pen("pen", 8)
pen.set_pen_width(THIN)
left_bumper = Bumper("leftBumper", 2)
right_bumper = Bumper("rightBumper", 3)
front_eye = EyeSensor("frontEye", 4)
down_eye = EyeSensor("downEye", 5)
front_distance = Distance("frontdistance", 6)
distance = front_distance
magnet = Electromagnet("magnet", 7)
location = Location("location", 9)

#endregion VEXcode Generated Robot Configuration
# ------------------------------------------
# 
# 	Project:      A mouse in a maze
#	Author:       Owajigbana Blessing Uloyok
#	Last Updated: 17/02/2026     
#	Description:  VEXcode VR Python Project of a simulated mouse escaping a maze
# 
# ------------------------------------------

"""The Playgrounds range from -1000mm to 1000mm on both the X and Y axes. 
This makes the dimensions of the VEXcode VR Playground 2000mm x 2000mm."""

# https://kb.vex.com/hc/en-us/articles/360041366072-Understanding-the-Coordinate-System-in-VEXcode-VR#directions-header-4

# ------------------------------------------
#  Robot status tracking
# ------------------------------------------

# list Position 1 = x_cord, 2 = y_cord, 3 = heading
# Initialise the status list: [x_cord, y_cord, heading]
robot_status = [130, -900, 0]

# Lists to store the path history
path_history = [] 

# Initialise 8x8 maze grid
maze_grid = [[0 for _ in range(8)] for _ in range(8)]

# Store walls for each tile
maze_walls = [[{"N": False, "E": False, "S": False, "W": False}
               for _ in range(8)] for _ in range(8)]

# # Create a wall between (6,0) and (6,1)
# maze_walls[6][0]["E"] = True
# maze_walls[6][1]["W"] = True

# ------------------------------------------
#  Update the position and heading
# ------------------------------------------
def refresh_status():
    global robot_status
    robot_status[0] = location.position(X, MM)
    robot_status[1] = location.position(Y, MM)
    robot_status[2] = drivetrain.heading(DEGREES)

# ------------------------------------------
#  Update maze grid and detect walls
# ------------------------------------------
def update_maze_list():
    global path_history

    col = int((robot_status[0] + 1000) / 250)
    row = int((robot_status[1] + 1000) / 250)

    col = max(0, min(7, col))
    row = max(0, min(7, row))

    heading = int(drivetrain.heading(DEGREES)) % 360

    if maze_grid[row][col] == 0:
        maze_grid[row][col] = 1
        path_history.append(list(robot_status))

    # Detect front wall
    if front_distance.get_distance(MM) < 250:
        # heading = int(drivetrain.heading(DEGREES)) % 360
        if heading == 0:   # North
            maze_walls[row][col]["N"] = True
            if row < 7:
                maze_walls[row+1][col]["S"] = True
        elif heading == 90:  # East
            maze_walls[row][col]["E"] = True
            if col < 7:
                maze_walls[row][col+1]["W"] = True
        elif heading == 180:  # South
            maze_walls[row][col]["S"] = True
            if row > 0:
                maze_walls[row-1][col]["N"] = True
        elif heading == 270:  # West
            maze_walls[row][col]["W"] = True
            if col > 0:
                maze_walls[row][col-1]["E"] = True
    else:
        if heading == 0:   # North
            maze_walls[row][col]["N"] = False
            if row < 7:
                maze_walls[row+1][col]["S"] = False
        elif heading == 90:  # East
            maze_walls[row][col]["E"] = False
            if col < 7:
                maze_walls[row][col+1]["W"] = False
        elif heading == 180:  # South
            maze_walls[row][col]["S"] = False
            if row > 0:
                maze_walls[row-1][col]["N"] = False
        elif heading == 270:  # West
            maze_walls[row][col]["W"] = False
            if col > 0:
                maze_walls[row][col-1]["E"] = False

    return row, col, heading

# ------------------------------------------
#   Count Visited Cell
# ------------------------------------------
def visited_count(): 
    count = 0 
    for r in range(8): 
        for c in range(8): 
            if maze_grid[r][c] == 1: 
                count += 1 
    return count

# ------------------------------------------
#   Map the maze
# ------------------------------------------
#----- Displaying the full maze Path in the console -------#
START_TILE = (0, 4)  
FINISH_TILE = (7, 3)  

last_printed_tile = (-1, -1)

def display_data():
    global last_printed_tile

    col = int((robot_status[0] + 1000) / 250)
    row = int((robot_status[1] + 1000) / 250)

    col = max(0, min(7, col))
    row = max(0, min(7, row))

    # Only print to console when entering new tile
    if (row, col) == last_printed_tile:
        return

    last_printed_tile = (row, col)
    current_count = visited_count()

    brain.print("Visited: " + str(current_count) + " / 64\n")
    brain.print("Robot Position: Row " + str(row) + " Col " + str(col) + "\n")

    for r in range(7, -1, -1):

        # ---- Print Top Walls ----#
        line = ""
        for c in range(8):
            line += "+"
            if maze_walls[r][c]["N"]:
                line += "---"
            else:
                line += "   "
        line += "+"
        brain.print(line + "\n")

        # ---- Print Cell Contents + Vertical Walls ----#
        line = ""
        for c in range(8):

            if maze_walls[r][c]["W"]:
                line += "|"
            else:
                line += " "

            if (r, c) == (row, col):
                line += " R "
            elif (r, c) == START_TILE:
                line += " S "
            elif (r, c) == FINISH_TILE:
                line += " F "
            # elif maze_grid[r][c] == 1:
            #     line += " . "
            else:
                line += "   "

        if maze_walls[r][7]["E"]:
            line += "|"
        else:
            line += " "

        brain.print(line + "\n")

    # ---- Bottom Border ----#
    line = ""
    for c in range(8):
        line += "+---"
    line += "+"
    brain.print(line + "\n")

    wait(3, MSEC)

# ------------------------------------------
#   Shortest Path
# ------------------------------------------
# A* pathfinding algorithm
def astar(start, goal):
    """Calculates shortest path using Manhattan distance heuristic."""
    open_set = [start]
    came_from = {}
    g_score = { (r, c): float('inf') for r in range(8) for c in range(8) }
    g_score[start] = 0
    f_score = { (r, c): float('inf') for r in range(8) for c in range(8) }
    f_score[start] = abs(start[0] - goal[0]) + abs(start[1] - goal[1])

    while open_set:
        current = min(open_set, key=lambda x: f_score[x])
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            return path[::-1]

        open_set.remove(current)
        r, c = current
        
        # Valid Neighbors based on mapped maze_walls
        neighbors = []
        if not maze_walls[r][c]["N"] and r < 7: 
            neighbors.append((r + 1, c))
        if not maze_walls[r][c]["E"] and c < 7: 
            neighbors.append((r, c + 1))
        if not maze_walls[r][c]["S"] and r > 0: 
            neighbors.append((r - 1, c))
        if not maze_walls[r][c]["W"] and c > 0: 
            neighbors.append((r, c - 1))

        for n in neighbors:
            temp_g = g_score[current] + 1
            if temp_g < g_score[n]:
                came_from[n] = current
                g_score[n] = temp_g
                f_score[n] = temp_g + abs(n[0] - goal[0]) + abs(n[1] - goal[1])
                if n not in open_set: 
                    open_set.append(n)
    return []

# Drive quickest route after calculating the shortest path
def drive_quickest_route(target_tile):
    curr_row, curr_col, curr_heading = update_maze_list()
    path = astar((curr_row, curr_col), target_tile)
    
    if not path:
        brain.print("\nNo path found to target!\n")
        return

    # Highlight the travel path
    pen.move(DOWN)
    pen.set_pen_color(GREEN) 
    pen.set_pen_width(MEDIUM)

    for next_row, next_col in path:
        if next_row > curr_row: 
            target_h = 0
        elif next_row < curr_row: 
            target_h = 180
        elif next_col > curr_col: 
            target_h = 90
        elif next_col < curr_col: 
            target_h = 270
        
        drivetrain.turn_to_heading(target_h, DEGREES)
        drivetrain.drive_for(FORWARD, 250, MM)
        curr_row, curr_col = next_row, next_col

# ------------------------------------------
#   Main
# ------------------------------------------
def main():
    drivetrain.set_drive_velocity(200, PERCENT)
    drivetrain.set_turn_velocity(100, PERCENT)
    onStart = True
    onFinish = False

    current_count = visited_count()

    # For maze with no false walls 
    while current_count < 64:
        refresh_status()
        row, col, heading = update_maze_list()

        directions = {0: "N", 90: "E", 180: "S", 270: "W"}
        facing = directions.get(heading, "N")
        
        # left-wall following to explore and map the entire maze first
        if onStart == True and front_distance.get_distance(MM) >= 3000:
            drivetrain.turn_for(RIGHT, 90, DEGREES)
        elif onFinish == True and front_distance.get_distance(MM) >= 3000:
            drivetrain.turn_for(RIGHT, 90, DEGREES)  
        elif maze_walls[row][col][facing]:
            drivetrain.turn_for(RIGHT, 90, DEGREES)
        else:
            drivetrain.drive_for(FORWARD, 250, MM)
            drivetrain.turn_for(LEFT,90, DEGREES)

        # Check location again for onStart status
        if location.position(X, MM)>=115 and location.position(X, MM)<=145 and \
           location.position(Y,MM)>=-915 and location.position(Y,MM)<=-885:
           onStart = True
        else:
           onStart = False
        
        if location.position(X, MM)>=-135 and location.position(X, MM)<=-115 and \
           location.position(Y,MM)>=845 and location.position(Y,MM)<=875:
           onFinish = True
        else:
           onFinish = False

        # Update count of visited cells
        current_count = visited_count()
        wait(5,MSEC)

    # Uncomment the code below to use for the map with a false wall

    # # False wall:
    # while current_count < 64:
    #     refresh_status()
    #     # Get row, col AND the current heading from your updated function
    #     row, col, heading = update_maze_list()

    #     # Create a wall between (6,0) and (6,1)
    #     maze_walls[6][0]["E"] = True
    #     maze_walls[6][1]["W"] = True

    #     # Convert heading to the dictionary key
    #     directions = {0: "N", 90: "E", 180: "S", 270: "W"}
    #     facing = directions.get(heading, "N")
        
    #     # left-wall following to explore and map the entire maze first
    #     if onStart == True and front_distance.get_distance(MM) >= 3000:
    #         drivetrain.turn_for(RIGHT, 90, DEGREES)
    #     elif onFinish == True and front_distance.get_distance(MM) >= 3000:
    #         drivetrain.turn_for(RIGHT, 90, DEGREES)
    #     # The robot now "sees" your manual walls at (6,0) and (6,1)
    #     elif maze_walls[row][col][facing]:
    #         drivetrain.turn_for(RIGHT, 90, DEGREES)
    #     else:
    #         drivetrain.drive_for(FORWARD, 250, MM)
    #         drivetrain.turn_for(LEFT, 90, DEGREES)

    #     # Check location again for onStart status
    #     if location.position(X, MM)>=115 and location.position(X, MM)<=145 and \
    #        location.position(Y,MM)>=-915 and location.position(Y,MM)<=-885:
    #        onStart = True
    #     else:
    #        onStart = False
        
    #     if location.position(X, MM)>=-135 and location.position(X, MM)<=-115 and \
    #        location.position(Y,MM)>=845 and location.position(Y,MM)<=875:
    #        onFinish = True
    #     else:
    #        onFinish = False

    #     # Update count of visited cells
    #     current_count = visited_count()
    #     wait(5,MSEC)

    brain.clear()
    # Prints the final map
    brain.print("=== FINAL MOVEMENT MAPPING ===\n\n")
    display_data() 
    wait(2, SECONDS)

    # Return to the start coordinate, to reset the robot position before travelling the quickest route
    while not onStart:  
        refresh_status()  
        update_maze_list()  
        
        if front_distance.get_distance(MM) >= 3000 and onFinish == False:  
            drivetrain.drive_for(FORWARD, 250, MM)  
            drivetrain.turn_for(LEFT, 90, DEGREES)  
        elif front_distance.get_distance(MM) > 250:  
            drivetrain.drive_for(FORWARD, 250, MM)  
            drivetrain.turn_for(LEFT, 90, DEGREES)  
        else: # 
                drivetrain.turn_for(RIGHT, 90, DEGREES)  
                
        # Check location again for onStart status
        if location.position(X, MM)>=115 and location.position(X, MM)<=145 and \
           location.position(Y,MM)>=-915 and location.position(Y,MM)<=-885:
           onStart = True
        else:
           onStart = False
        
        if location.position(X, MM)>=-135 and location.position(X, MM)<=-115 and \
           location.position(Y,MM)>=845 and location.position(Y,MM)<=875:
           onFinish = True
        else:
           onFinish = False
        
        wait(5,MSEC)
    wait(2, SECONDS)

    refresh_status()
        
    brain.print("\n=== PATH HISTORY (X, Y, Heading) ===\n")
    # for step in path_history:
    #     # Rounds coordinates to 1 decimal place 
    #     clean_step = [round(step[0], 1), round(step[1], 1), int(step[2])]
    #     brain.print(str(clean_step) + "\n")
    #     wait(5, MSEC)
    
    for step in path_history:
        brain.print(str(step) + "\n")
        wait(5, MSEC)
    
    brain.print("\n=== A* SHORTEST PATH ===\n")
    # Drive quickest route to the finish
    drive_quickest_route(FINISH_TILE)
    brain.print("\nEscaped the maze by the fastest possible route\n")
    wait(1, SECONDS)

    refresh_status()
    update_maze_list()

    # Return to the start
    drive_quickest_route(START_TILE)
    brain.print("\nReturned to Start safely.\n")

    wait(1, SECONDS)

# VR threads
vr_thread(main)