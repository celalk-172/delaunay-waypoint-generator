import numpy as np
"""
This script reads obstacle coordinates from multiple files for use in drone path planning.

Functions:
- read_obstacle_file: Reads obstacle coordinates from a specified file.
- obstacle_read: Reads obstacle data from multiple files and returns a list of arrays.
"""

def read_obstacle_file(file_path):
    """
    Reads obstacle coordinates from a specified file.

    Args:
        file_path (str): Path to the obstacle file.

    Returns:
        numpy.ndarray: Array of obstacle coordinates as [longitude, latitude].
    """
    obstacle_coord = []
    # Open the file for reading
    with open(file_path, 'r') as file:
        # Read all lines except the first and last
        lines = file.readlines()[1:-1]
        for line in lines:
            # Split the line into latitude and longitude
            lat, lon = map(float, line.split())
            obstacle_coord.append([lon, lat])
    return np.array(obstacle_coord)

def obstacle_read():
    """
    Reads obstacle data from multiple files and returns a list of arrays.

    Each file contains coordinates defining the boundaries of an obstacle area.
    The coordinates are expected to be in a specific format:
    - The first line is skipped.
    - The last line is skipped.
    - Remaining lines contain latitude and longitude values.

    Returns:
        list: List of numpy arrays, where each array represents an obstacle's coordinates.
    """
    # List of obstacle files
    obstacle_files = [
        'obstacle_files/obstacle_prison.poly',
        'obstacle_files/obstacle_university.poly',
        'obstacle_files/obstacle_cothamschool.poly',
        'obstacle_files/obstacle_cothamgardens.poly',
        'obstacle_files/obstacle_montpeillerparkandschools.poly',
        'obstacle_files/obstacle_redlandgreen.poly',
        'obstacle_files/obstacle_cityofbristolcollege.poly',
        'obstacle_files/obstacle_centralbrisandcastlepark.poly',
        'obstacle_files/obstacle_standrewsgarden.poly',
        'obstacle_files/obstacle_memorialstadium.poly',
        'obstacle_files/obstacle_bishopplayfield.poly',
        'obstacle_files/obstacle_stbonaventures.poly',
        'obstacle_files/obstacle_horfieldcommon.poly',
        'obstacle_files/obstacle_henleazeschools.poly',
        'obstacle_files/obstacle_badockswood.poly',
        'obstacle_files/obstacle_armyreserve.poly'
    ]

    # Read all obstacle files and append their data to the obstacles list
    obstacles = [read_obstacle_file(file_path) for file_path in obstacle_files]

    return obstacles
