import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import Delaunay

################## Created by Dr. Arthur Richards, Adapted by Celal Konuklu #############################
def delaunay_path(outer_points,obstacles,start_point,start_tri,goal_point,goal_tri):
    ## Problem set-up

    # Workspace is defined in terms of four corners, and obstacles are represented as polygons.  
    # -> Delaunay needs an outer boundary as well as obstacles.

    def plot_poly(points,fmt='b-',**kwargs):
            plt.plot(np.append(points[:,0],points[0,0]),np.append(points[:,1],points[0,1]),fmt)
        # plot_poly(outer_points)
        # for ob in obstacles:
            #     plot_poly(ob,'r-')
            # plt.show()


    ## Roadmap generation

    # Take all vertices together - those of the workspace and of the obstacles - and run a Delaunay triangulation.
    # The Delaunay triangles don't necessary align with the edges of our obstacles. 
    # Hence, 'Constrained Delaunay triangulation' is implemented 

    #plt.figure()
    #plot_poly(outer_points,'g-')
    #for ob in obstacles:
        #plot_poly(ob,'r-')
    all_points = outer_points
    obst_edges = set([])
    for ob in obstacles:
        all_points = np.append(all_points,ob[:],0)
        num_ob_points = len(ob)
        for ii in range(num_ob_points):
            new_edge = (len(all_points)-num_ob_points+ii,len(all_points)-num_ob_points+(ii+1)%num_ob_points)
            obst_edges.add((min(new_edge),max(new_edge)))
    tri = Delaunay(all_points)
    #plt.triplot(all_points[:,0], all_points[:,1], tri.simplices.copy(),'k--')
    #plt.show()

    # Identify the missing edges as those in the set of obstacle edges that are not included in the triangle edges.
    
    tri_edges = set([(min(a,b),max(a,b)) for (a,b,c) in tri.simplices])
    tri_edges = tri_edges.union(set([(min(a,c),max(a,c)) for (a,b,c) in tri.simplices]))
    tri_edges = tri_edges.union(set([(min(c,b),max(c,b)) for (a,b,c) in tri.simplices]))
    missed_edges = obst_edges-tri_edges
    #print(missed_edges)
    
    # To correct the problem, take each missed edge and split it in two, adding the mid point to the 
    # set of all points and including the two new edges created in the set of obstacle edges.
    
    for prob_edge in missed_edges:
        obst_edges.discard(prob_edge)
        new_point = 0.5*all_points[min(prob_edge)]+0.5*all_points[max(prob_edge)]
        all_points=np.append(all_points,[new_point],0)
        obst_edges.add((min(prob_edge),len(all_points)-1))
        obst_edges.add((max(prob_edge),len(all_points)-1))
    
    
    ## Now to build the roadmap. The triangulation tells the neighbour information about each triangle.  
    # An 'entry/exit' point is included for each pair of neighbouring triangles if their shared edge 
    # isn't an obstacle edge.  A distance matrix is also built, capturing the distance between 
    # entry/exit points across their common triangle.
    
    num_tris = len(tri.simplices)
    
    nx_points = []
    d = np.inf+np.zeros((num_tris,num_tris))
    tri_nx = [[] for t in tri.simplices]
    new_nx_idx = 0
    
    #plt.figure('All Nodes')
    for ii in range(num_tris):
        for jj in tri.neighbors[ii]:
            if jj>ii:
                common_edge = set(tri.simplices[ii]).intersection(set(tri.simplices[jj]))
                # ignore neighbour if the common edge is an obstacle edge
                if (min(common_edge),max(common_edge)) in obst_edges:
                    pass
                else:
                    mid_point = sum(all_points[list(common_edge)])/2.0
                    #print(mid_point)
                    #plt.plot(mid_point[0],mid_point[1],'c*')
                    nx_points.append(mid_point)
                    new_nx_idx = len(nx_points)-1
                    tri_nx[ii].append(new_nx_idx)
                    tri_nx[jj].append(new_nx_idx)
                    for nx in tri_nx[ii]:
                        d[new_nx_idx,nx] = np.linalg.norm(nx_points[nx]-nx_points[new_nx_idx])
                        d[nx,new_nx_idx] = np.linalg.norm(nx_points[nx]-nx_points[new_nx_idx])
                        #plt.plot([nx_points[nx][0],nx_points[new_nx_idx][0]],[nx_points[nx][1],nx_points[new_nx_idx][1]],'c-')
                    for nx in tri_nx[jj]:
                        d[new_nx_idx,nx] = np.linalg.norm(nx_points[nx]-nx_points[new_nx_idx])
                        d[nx,new_nx_idx] = np.linalg.norm(nx_points[nx]-nx_points[new_nx_idx])
                        #plt.plot([nx_points[nx][0],nx_points[new_nx_idx][0]],[nx_points[nx][1],nx_points[new_nx_idx][1]],'c-')
    
    num_nx_points = len(nx_points)
    d = d[0:num_nx_points,0:num_nx_points]
                        
    #plot_poly(outer_points,'b-')
    #for ob in obstacles:
    #    plot_poly(ob,'r-')
    #plt.triplot(all_points[:,0], all_points[:,1], tri.simplices.copy(),'k--')
    #plt.title('All Possible Nodes')
    #plt.xlabel('Longitude')
    #plt.ylabel('Latitude')
    #plt.show()
    
    ## Adding the start and goal
    
    # That's the roadmap done, and is included in the distance matrix.
    def append_dist_matrix(d):
        (r,c)=d.shape
        d2 = np.vstack((d,np.inf+np.zeros((1,c))))
        d3 = np.hstack((d2,np.inf+np.zeros((r+1,1))))
        return(d3)
    
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
    
