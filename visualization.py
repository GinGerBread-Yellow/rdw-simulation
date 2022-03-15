
import matplotlib.pyplot as plt
from matplotlib import colors
import numpy as np

from randomPath import generatePath
# Use for visualizing the virtual and physical paths of the users defined in the users variable.   

def visualize_paths(virtual_paths, rdw_paths, grids, jx,jy):


    myColors = ['blue', 'red', 'green', 'orange', 'yellow']
    plt.figure(1) 
    plt.grid(True)
    plt.title("Virtual paths: random walk")

    for ind, virtual_path in enumerate(virtual_paths):

        x = [p[0] for p in virtual_path]
        y = [p[1] for p in virtual_path]

        plt.plot(x, y, '.-', color=myColors[ind])
        plt.plot(virtual_path[0][0], virtual_path[0][1], 'o', color="black")
      
    plt.figure(2) 
    plt.grid(True)
    plt.title("physical paths")

    for ind,rdw_path in enumerate(rdw_paths):
        rx = [p[0] for p in rdw_path]
        ry = [p[1] for p in rdw_path]
        w, h = grids.shape
        # _, plt = plt.subplots()
        # plt.set_xticks(range(w))
        # plt.set_yticks(range(h))
        plt.plot(rx, ry, '.-', color=myColors[ind])
        plt.plot(rdw_path[0][0], rdw_path[0][1], 'o', color="black")

    # cmap = colors.ListedColormap(['black', 'white'])
    # plt.pcolormesh(grids, cmap=cmap)
    w,h = grids.shape
    for i in range(w):
        for j in range(h):
            if not grids[i,j]:
                plt.plot(i,j, 'x', color="black")
    
    vec_x, vec_y = rdw_path[0][0]-jx, rdw_path[0][1]-jy
    plt.arrow(rdw_path[0][0], rdw_path[0][1], vec_x, vec_y, width=0.05)



    plt.show()


if __name__ == "__main__":
    testpath = generatePath(5, 10)
    grids = np.ones([30,50], dtype=bool)
    grids[10:, 20:50] = 0
    rdwpath = generatePath(5, 10)
    visualize_paths(testpath, rdwpath, grids)