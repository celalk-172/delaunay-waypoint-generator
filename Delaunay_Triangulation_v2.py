import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import Delaunay

################## Created by Dr. Arthur Richards, Adapted by Celal Konuklu #############################

"""
This script performs path planning for drones using Delaunay Triangulation.
It generates a roadmap by considering an outer boundary, obstacles, and start/goal points.
The output includes the triangulation, visualized roadmap, and a calculated optimal path.

Functions:
- delaunay_path: Main function to compute and visualize the triangulated region and optimal path.
"""

def delaunay_path(outer_points, obstacles, start_point, start_tri, goal_point, goal_tri):
    """
    Computes a Delaunay Triangulation-based roadmap for drone path planning.
    The function considers workspace boundaries, obstacles, and start/goal points to determine an optimal path.

    Parameters:
    - outer_points (ndarray): Coordinates of the workspace boundary vertices.
    - obstacles (list of ndarrays): List of obstacle polygons, each defined by vertices.
    - start_point (ndarray): Coordinates of the starting point.
    - start_tri (int): Index of the starting triangle in the triangulation.
    - goal_point (ndarray): Coordinates of the goal point.
    - goal_tri (int): Index of the goal triangle in the triangulation.

    Returns:
    - all_points (ndarray): Array of all vertices used in the triangulation.
    - tri (Delaunay object): Delaunay triangulation object.
    - path_nodes (list of ndarrays): List of waypoints along the computed path.
    """
    
    ## Problem set-up
    def plot_poly(points, fmt='b-', **kwargs):
        """
        Utility function to plot a polygon defined by its vertices.

        Parameters:
        - points (ndarray): Coordinates of the polygon vertices.
        - fmt (str): Line format for plotting (default is blue lines).
        """
        plt.plot(np.append(points[:, 0], points[0, 0]), np.append(points[:, 1], points[0, 1]), fmt)

    ## Roadmap generation
    # Combine all vertices (outer boundary + obstacles) for triangulation
    all_points = outer_points
    obst_edges = set([])
    for ob in obstacles:
        all_points = np.append(all_points, ob[:], 0)
        num_ob_points = len(ob)
        for ii in range(num_ob_points):
            new_edge = (len(all_points) - num_ob_points + ii, len(all_points) - num_ob_points + (ii + 1) % num_ob_points)
            obst_edges.add((min(new_edge), max(new_edge)))

    tri = Delaunay(all_points)

    # Identify missing edges (edges in obstacles but not in triangulation)
    tri_edges = set([(min(a, b), max(a, b)) for (a, b, c) in tri.simplices])
    tri_edges = tri_edges.union(set([(min(a, c), max(a, c)) for (a, b, c) in tri.simplices]))
    tri_edges = tri_edges.union(set([(min(c, b), max(c, b)) for (a, b, c) in tri.simplices]))
    missed_edges = obst_edges - tri_edges

    # Correct triangulation by splitting missing edges
    for prob_edge in missed_edges:
        obst_edges.discard(prob_edge)
        new_point = 0.5 * all_points[min(prob_edge)] + 0.5 * all_points[max(prob_edge)]
        all_points = np.append(all_points, [new_point], 0)
        obst_edges.add((min(prob_edge), len(all_points) - 1))
        obst_edges.add((max(prob_edge), len(all_points) - 1))

    ## Build roadmap
    # Create entry/exit points for neighboring triangles
    num_tris = len(tri.simplices)
    nx_points = []
    d = np.inf + np.zeros((num_tris, num_tris))
    tri_nx = [[] for _ in tri.simplices]

    for ii in range(num_tris):
        for jj in tri.neighbors[ii]:
            if jj > ii:
                common_edge = set(tri.simplices[ii]).intersection(set(tri.simplices[jj]))
                if (min(common_edge), max(common_edge)) in obst_edges:
                    continue  # Skip edges that are obstacle edges
                mid_point = sum(all_points[list(common_edge)]) / 2.0
                nx_points.append(mid_point)
                new_nx_idx = len(nx_points) - 1
                tri_nx[ii].append(new_nx_idx)
                tri_nx[jj].append(new_nx_idx)

                # Update distance matrix
                for nx in tri_nx[ii] + tri_nx[jj]:
                    d[new_nx_idx, nx] = np.linalg.norm(nx_points[nx] - nx_points[new_nx_idx])
                    d[nx, new_nx_idx] = d[new_nx_idx, nx]

    ## Add start and goal points to the roadmap
    def append_dist_matrix(d):
        """
        Utility function to expand a distance matrix with a new row and column.

        Parameters:
        - d (ndarray): Original distance matrix.

        Returns:
        - ndarray: Expanded distance matrix.
        """
        (r, c) = d.shape
        d2 = np.vstack((d, np.inf + np.zeros((1, c))))
        return np.hstack((d2, np.inf + np.zeros((r + 1, 1))))

    ## Visualization and further computation...
    
    # copy the NX points before adding start and goal to the list
    nx_points_aug = nx_points[:]
    
    plt.figure('Triangulated Region')

    nx_points_aug.append(start_point)
    d_aug = append_dist_matrix(d)
    start_idx = len(nx_points_aug)-1
    plt.plot(start_point[0],start_point[1],'go')
    for nx in tri_nx[start_tri]:
        plt.plot([nx_points_aug[start_idx][0],nx_points_aug[nx][0]],[nx_points_aug[start_idx][1],nx_points_aug[nx][1]],'c-')
        d_aug[start_idx,nx]=np.linalg.norm(nx_points_aug[start_idx]-nx_points_aug[nx])
        d_aug[nx,start_idx]=d_aug[start_idx,nx]
    
    
    nx_points_aug.append(goal_point)
    d_aug = append_dist_matrix(d_aug)
    goal_idx = len(nx_points_aug)-1
    plt.plot(goal_point[0],goal_point[1],'g*')
    for nx in tri_nx[goal_tri]:
        plt.plot([nx_points_aug[goal_idx][0],nx_points_aug[nx][0]],[nx_points_aug[goal_idx][1],nx_points_aug[nx][1]],'c-')
        d_aug[goal_idx,nx]=np.linalg.norm(nx_points_aug[goal_idx]-nx_points_aug[nx])
        d_aug[nx,goal_idx]=d_aug[goal_idx,nx]
        
    plot_poly(outer_points,'b-')
    for ob in obstacles:
        plot_poly(ob,'r-')
    plt.triplot(all_points[:,0], all_points[:,1], tri.simplices.copy(),'k--')
    plt.gca().set_aspect('equal', adjustable='box')
    plt.xlim(-2.63, -2.561)
    plt.ylim(51.45, 51.505)
    plt.xticks(np.arange(-2.63, -2.561, 0.01))  # Adjust as needed
    plt.yticks(np.arange(51.45, 51.505, 0.005))  # Adjust as needed
    plt.title('Start and Goal Points within the Triangulated Region')
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    #plt.show()
    
    ## Finding the path using graph search
    
    from scipy.sparse.csgraph import shortest_path
    distance,predecessors = shortest_path(d_aug, return_predecessors=True)
    
    # Finally, reconstruct the path by traversing from `start_idx` to `goal_idx`.
    plt.figure('Resulting Path')
    
    path_nodes = [np.array([nx_points_aug[start_idx][1],nx_points_aug[start_idx][0]])]
    # Note: path_nodes find the coordinates of each waypoint!
    
    curr_node = start_idx
    for kk in range(len(nx_points)):
        next_node = predecessors[goal_idx,curr_node]
        plt.plot([nx_points_aug[curr_node][0],nx_points_aug[next_node][0]],[nx_points_aug[curr_node][1],nx_points_aug[next_node][1]],'m-')
        curr_node=next_node
        path_nodes.append(np.array([nx_points_aug[curr_node][1],nx_points_aug[curr_node][0]]))
        if curr_node==goal_idx:
            break
    path_nodes.append(np.array([nx_points_aug[goal_idx][1],nx_points_aug[goal_idx][0]]))      
           
    plot_poly(outer_points,'b-')
    for ob in obstacles:
        plot_poly(ob,'r-')
    plt.plot(start_point[0],start_point[1],'go',goal_point[0],goal_point[1],'gx')
    plt.gca().set_aspect('equal', adjustable='box')
    plt.grid()
    plt.title('Drone Path')
    plt.xlim(-2.63, -2.561)
    plt.ylim(51.45, 51.505)
    plt.xticks(np.arange(-2.63, -2.561, 0.01))  # Adjust as needed
    plt.yticks(np.arange(51.45, 51.505, 0.005))  # Adjust as needed
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    #plt.show()
    
    return all_points, tri, path_nodes
    # Note: all_points and tri are output for manual triangle selection
