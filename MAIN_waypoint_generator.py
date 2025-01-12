import math
import matplotlib.pyplot as plt
import numpy as np
import Delaunay_Triangulation_v2 # Import Delaunay triangulation algorithm
import Obstacle_Reader_v2 # Import obstacle reading function
import csv

"""
This script performs path planning for drones using Delaunay Triangulation.
It generates a roadmap by considering an outer boundary, obstacles, and start/goal points.
The output includes the triangulation, visualized roadmap, and a calculated optimal path.

### Description:
The script enables drones to navigate between two predefined helipads (Bristol Royal Infirmary and Southmead Hospital), 
avoiding obstacles using Delaunay triangulation. It also generates waypoints compatible with Mission Planner.

### User Input:
- `start_hospital`: Choose the starting helipad ('BRI' for Bristol Royal Infirmary, or 'SH' for Southmead Hospital).
- `alt`: Set the flight altitude above the terrain (in meters).

### Features:
- Computes the distance and bearing between helipads.
- Reads obstacles from `.poly` files and visualizes them.
- Generates a triangulated map and determines the shortest obstacle-free path.
- Outputs waypoints for Mission Planner.

### Outputs:
1. Plots:
   - Workspace boundary and obstacles.
   - Delaunay triangulation and the optimal path.
2. Waypoint File:
   - A `.txt` file named `MissionPlanner_Waypoints.txt` for use with Mission Planner.
"""

####################################### USER INPUT ########################################
# Define the hospital from which the drone will take-off:
start_hospital = 'BRI' #'BRI' for Bristol Royal Infirmary, or 'SH' for Southmead Hospital
alt = 100 #chosen flight altitude above terrain (m)  
###########################################################################################

############################ Rest of the code (DO NOT CHANGE) #############################

## Coordinates of BRI and SH helipads ##

BRI_lat = 51.4589706947779
BRI_lon = -2.59624421596527

SH_lat = 51.4937088
SH_lon = -2.5925803

# Define where the drone will start (i.e. take-off)
def start_choose(start):
    """
    Function to choose the starting point (either BRI or SH) and determine the corresponding triangle.
    
    Parameters:
    - start (str): The starting hospital, 'BRI' or 'SH'.
    
    Returns:
    - start_point (np.array): Coordinates of the starting point [longitude, latitude].
    - start_tri (int): Triangle index of the starting point (manually determined).
    - goal_point (np.array): Coordinates of the goal point [longitude, latitude].
    - goal_tri (int): Triangle index of the goal point (manually determined).
    """
    
    if start == 'BRI':
        start_point = np.array([BRI_lon,BRI_lat])
        start_tri = 18 #triangle in which the start point is - manually determined from Delaunay. (!DO NOT CHANGE!)
        
    
        goal_point = np.array([SH_lon,SH_lat])
        goal_tri = 92 #triangle in which the goal point is - manually determined from Delaunay. (!DO NOT CHANGE!)
    
    elif start == 'SH':
        start_point = np.array([SH_lon,SH_lat])
        start_tri = 92 #triangle in which the start point is - manually determined from Delaunay. (!DO NOT CHANGE!)
    
        goal_point = np.array([BRI_lon,BRI_lat])
        goal_tri = 18 #triangle in which the goal point is - manually determined from Delaunay. (!DO NOT CHANGE!)
        
    return start_point, start_tri, goal_point, goal_tri
    
start_point, start_tri, goal_point, goal_tri = start_choose(start_hospital)


## Find the distance and bearing between helipads using the haversine formula ##
def haversine(lat1, lon1, lat2, lon2):
    """
    Calculate the distance and bearing between two geographical points using the Haversine formula.
    
    Parameters:
    - lat1 (float): Latitude of the first point.
    - lon1 (float): Longitude of the first point.
    - lat2 (float): Latitude of the second point.
    - lon2 (float): Longitude of the second point.
    
    Returns:
    - distance (float): The great-circle distance between the two points (in meters).
    - bearing (float): The bearing angle from the first point to the second point (in degrees).
    """
    
    # Radius of the Earth in meters
    earth_radius = 6371000  # Approximate value, can vary slightly depending on the Earth model

    # Convert latitude and longitude from degrees to radians
    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)

    # Haversine formula
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = earth_radius * c

    # Calculate the bearing (angle) from the first point to the second point
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    bearing = math.degrees(math.atan2(y, x))

    # Normalize the bearing to a compass bearing between 0 and 360 degrees
    if bearing < 0:
        bearing += 360

    return distance, bearing

# Calculate and print distance and bearing between BRI and SH
distance, bearing = haversine(BRI_lat,BRI_lon,SH_lat,SH_lon)
print(f"Straight line distance between the two helipads: {round(distance,1)} metres")
print(f"Bearing from Bristol Royal Infirmary to Southmead Hospital: {round(bearing,2)} degrees")


## Plot to illustrate ##
plt.figure('Obstacle Visualisation')
# Plot the two points
plt.scatter([BRI_lon, SH_lon], [BRI_lat, SH_lat], color='green', label='Helipads')
plt.gca().set_aspect('equal', adjustable='datalim')

# Draw a line to represent the distance between the points (optional)
#plt.plot([BRI_lon, SH_lon], [BRI_lat, SH_lat], linestyle='--', color='blue', label='Distance')

# Set labels and legend
plt.xlabel('Longitude')
plt.ylabel('Latitude')
plt.grid()
# Don't show the plot yet (will show obstacles & workspace as well)


## Define the workspace & obstacles ##
# First, define the boundary of the workspace (as [lon,lat]])
lat_margin = 0.0065 #i.e. ~700m
lon_margin = 0.0125 #i.e. ~1100m
outer_points = np.array([[BRI_lon-lon_margin,BRI_lat-lat_margin],[BRI_lon-lon_margin,SH_lat+lat_margin]
                        ,[SH_lon+lon_margin,SH_lat+lat_margin],[SH_lon+lon_margin,BRI_lat-lat_margin]])
# Extract outer points in a .poly file to visualise in MissionPlanner
output_file = "outer_boundary.poly"

# Write the coordinates to the text file
with open(output_file, 'w') as file:
    for lon, lat in outer_points:
        file.write(f"{lat} {lon}\n")


# Then define the obstacles. A function is created to read the obstacles from .poly
# files defined in MissionPlanner, using data from https://dronesafetymap.com
obstacles = obstacle_coord = obstacle_reader_v2.obstacle_read()


## Plot to show the obstacles
def plot_poly(points,fmt='b-',**kwargs):
    """
    Function to plot a polygon on a map.
    
    Parameters:
    - points (np.array): Coordinates of the polygon vertices [[lon, lat], ...]
    - fmt (str): Format for plotting the polygon (default 'b-' for blue).
    - **kwargs: Additional keyword arguments passed to plt.plot().
    """
    
    plt.plot(np.append(points[:,0],points[0,0]),np.append(points[:,1],points[0,1]),fmt,**kwargs)

plot_poly(outer_points, 'b-', label='Workspace')
legend_flag = 0
for ob in obstacles:
    legend_flag = legend_flag + 1
    if legend_flag == 1:
        plot_poly(ob,'r-',label='Obstacles')
    else:
        plot_poly(ob,'r-')
plt.legend()
plt.title('Obstacle Visualisation')
plt.xlim(-2.63, -2.561)
plt.ylim(51.45, 51.505)
#plt.show() # Uncomment to visualize this plot (optional)

### Now ready for Delaunay Algorithm ###
# Call the Delaunay pathfinding algorithm
all_points, tri, path_nodes = Delaunay_Triangulation_v2.delaunay_path(outer_points,obstacles,start_point,start_tri,goal_point,goal_tri)

## Mission Planner Waypoints ## 
# Note: The waypoints yield sharp turns. This can be overcome by enabling 'spline waypoints' in the waypoint file [at MissionPlanner]

# Constants (DO NOT CHANGE)
NAV_WAYPOINT = 16
NAV_SPLINE_WAYPOINT = 82 
NAV_LAND = 21
TAKEOFF = 22
MAV_FRAME_GLOBAL_TERRAIN_ALT = 10
MAV_FRAME_GLOBAL = 0

# Function to write waypoints in a suitable format for Mission Planner
def waypoint_write(alt,filename):
    """
    Function to generate waypoints for Mission Planner.
    
    Parameters:
    - alt (int): Flight altitude above terrain (in meters).
    - filename (str): Name of the file where waypoints will be saved.
    
    Returns:
    - waypoints (list): A list of waypoints with their respective details.
    """

    home_lat = path_nodes[0][0]
    home_lon = path_nodes[0][1]
    
    waypoints = [[0,1,MAV_FRAME_GLOBAL_TERRAIN_ALT,NAV_WAYPOINT,0,0,0,0,home_lat,home_lon,0,1]]
    
    
    wp_idx = 0
    for lat,lon in path_nodes:
        wp_idx = wp_idx + 1
        
        if wp_idx == 1:
            waypoints.append([wp_idx,0,MAV_FRAME_GLOBAL_TERRAIN_ALT,TAKEOFF,0,0,0,0,lat,lon,alt,1])
            
        elif wp_idx == len(path_nodes):
            waypoints.append([wp_idx,0,MAV_FRAME_GLOBAL_TERRAIN_ALT,NAV_LAND,0,0,0,0,lat,lon,0,1])
            
        else:
            waypoints.append([wp_idx,0,MAV_FRAME_GLOBAL_TERRAIN_ALT,NAV_SPLINE_WAYPOINT,0,0,0,0,lat,lon,alt,1])
    
    # Save waypoints
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f, delimiter='\t')
        writer.writerow(['QGC WPL 110'])
        writer.writerows(waypoints)
        
    return waypoints      

waypoints = waypoint_write(alt,'MissionPlanner_Waypoints.txt')

plt.show() # visualise all open plots
