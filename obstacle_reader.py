import numpy as np

def obstacle_read():
    
    obstacle_coord = []
    # Open the file for reading
    with open('obstacle_prison.poly', 'r') as file:
        # Read each line in the file
        lines = file.readlines()  # Read all lines into a list
        for line in lines[1:-1]:
            # Split each line into latitude and longitude values
            lat, lon = map(float, line.split())
            obstacle_coord.append([lon,lat])
    
    # Convert the list to a NumPy array
    obstacles = [np.array(obstacle_coord)]


    # Repeat  
    obstacle_coord = []
    # Open the file for reading
    with open('obstacle_university.poly', 'r') as file:
        # Read each line in the file
        lines = file.readlines()  # Read all lines into a list
        for line in lines[1:-1]:
            # Split each line into latitude and longitude values
            lat, lon = map(float, line.split())
            obstacle_coord.append([lon,lat])
    
    # Add to the obstacle array
    obstacles.append(np.array(obstacle_coord))
    
    # Repeat  
    obstacle_coord = []
    # Open the file for reading
    with open('obstacle_cothamschool.poly', 'r') as file:
        # Read each line in the file
        lines = file.readlines()  # Read all lines into a list
        for line in lines[1:-1]:
            # Split each line into latitude and longitude values
            lat, lon = map(float, line.split())
            obstacle_coord.append([lon,lat])
            
    obstacles.append(np.array(obstacle_coord))

    # Repeat  
    obstacle_coord = []
    # Open the file for reading
    with open('obstacle_cothamgardens.poly', 'r') as file:
        # Read each line in the file
        lines = file.readlines()  # Read all lines into a list
        for line in lines[1:-1]:
            # Split each line into latitude and longitude values
            lat, lon = map(float, line.split())
            obstacle_coord.append([lon,lat])
            
    obstacles.append(np.array(obstacle_coord))
    
    # Repeat  
    obstacle_coord = []
    # Open the file for reading
    with open('obstacle_montpeillerparkandschools.poly', 'r') as file:
        # Read each line in the file
        lines = file.readlines()  # Read all lines into a list
        for line in lines[1:-1]:
            # Split each line into latitude and longitude values
            lat, lon = map(float, line.split())
            obstacle_coord.append([lon,lat])
            
    obstacles.append(np.array(obstacle_coord))
    
    # Repeat  
    obstacle_coord = []
    # Open the file for reading
    with open('obstacle_redlandgreen.poly', 'r') as file:
        # Read each line in the file
        lines = file.readlines()  # Read all lines into a list
        for line in lines[1:-1]:
            # Split each line into latitude and longitude values
            lat, lon = map(float, line.split())
            obstacle_coord.append([lon,lat])
            
    obstacles.append(np.array(obstacle_coord))
    
    
    # Repeat  
    obstacle_coord = []
    # Open the file for reading
    with open('obstacle_cityofbristolcollege.poly', 'r') as file:
        # Read each line in the file
        lines = file.readlines()  # Read all lines into a list
        for line in lines[1:-1]:
            # Split each line into latitude and longitude values
            lat, lon = map(float, line.split())
            obstacle_coord.append([lon,lat])
            
    obstacles.append(np.array(obstacle_coord))
    
    # Repeat  
    obstacle_coord = []
    # Open the file for reading
    with open('obstacle_centralbrisandcastlepark.poly', 'r') as file:
        # Read each line in the file
        lines = file.readlines()  # Read all lines into a list
        for line in lines[1:-1]:
            # Split each line into latitude and longitude values
            lat, lon = map(float, line.split())
            obstacle_coord.append([lon,lat])
            
    obstacles.append(np.array(obstacle_coord))
    
    # Repeat  
    obstacle_coord = []
    # Open the file for reading
    with open('obstacle_standrewsgarden.poly', 'r') as file:
        # Read each line in the file
        lines = file.readlines()  # Read all lines into a list
        for line in lines[1:-1]:
            # Split each line into latitude and longitude values
            lat, lon = map(float, line.split())
            obstacle_coord.append([lon,lat])
            
    obstacles.append(np.array(obstacle_coord))
    
    # Repeat  
    obstacle_coord = []
    # Open the file for reading
    with open('obstacle_memorialstadium.poly', 'r') as file:
        # Read each line in the file
        lines = file.readlines()  # Read all lines into a list
        for line in lines[1:-1]:
            # Split each line into latitude and longitude values
            lat, lon = map(float, line.split())
            obstacle_coord.append([lon,lat])
            
    obstacles.append(np.array(obstacle_coord))
    
    # Repeat  
    obstacle_coord = []
    # Open the file for reading
    with open('obstacle_bishopplayfield.poly', 'r') as file:
        # Read each line in the file
        lines = file.readlines()  # Read all lines into a list
        for line in lines[1:-1]:
            # Split each line into latitude and longitude values
            lat, lon = map(float, line.split())
            obstacle_coord.append([lon,lat])
            
    obstacles.append(np.array(obstacle_coord))
    
    # Repeat  
    obstacle_coord = []
    # Open the file for reading
    with open('obstacle_stbonaventures.poly', 'r') as file:
        # Read each line in the file
        lines = file.readlines()  # Read all lines into a list
        for line in lines[1:-1]:
            # Split each line into latitude and longitude values
            lat, lon = map(float, line.split())
            obstacle_coord.append([lon,lat])
            
    obstacles.append(np.array(obstacle_coord))
    
    # Repeat  
    obstacle_coord = []
    # Open the file for reading
    with open('obstacle_horfieldcommon.poly', 'r') as file:
        # Read each line in the file
        lines = file.readlines()  # Read all lines into a list
        for line in lines[1:-1]:
            # Split each line into latitude and longitude values
            lat, lon = map(float, line.split())
            obstacle_coord.append([lon,lat])
            
    obstacles.append(np.array(obstacle_coord))
    
    # Repeat  
    obstacle_coord = []
    # Open the file for reading
    with open('obstacle_henleazeschools.poly', 'r') as file:
        # Read each line in the file
        lines = file.readlines()  # Read all lines into a list
        for line in lines[1:-1]:
            # Split each line into latitude and longitude values
            lat, lon = map(float, line.split())
            obstacle_coord.append([lon,lat])
            
    obstacles.append(np.array(obstacle_coord))
    
    # Repeat  
    obstacle_coord = []
    # Open the file for reading
    with open('obstacle_badockswood.poly', 'r') as file:
        # Read each line in the file
        lines = file.readlines()  # Read all lines into a list
        for line in lines[1:-1]:
            # Split each line into latitude and longitude values
            lat, lon = map(float, line.split())
            obstacle_coord.append([lon,lat])
            
    obstacles.append(np.array(obstacle_coord))
    
    # Repeat  
    obstacle_coord = []
    # Open the file for reading
    with open('obstacle_armyreserve.poly', 'r') as file:
        # Read each line in the file
        lines = file.readlines()  # Read all lines into a list
        for line in lines[1:-1]:
            # Split each line into latitude and longitude values
            lat, lon = map(float, line.split())
            obstacle_coord.append([lon,lat])
            
    obstacles.append(np.array(obstacle_coord))
    
    
    return obstacles
