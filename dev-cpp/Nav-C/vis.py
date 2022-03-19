import matplotlib.pyplot as plt
import numpy as np

## Count the width
row = 0
col = 0
f = open("/Users/vermeulenlucas/Documents/Master/SPOT/code/dev-cpp/Nav-C/a_star.txt", "r")
for x in f:
    row += 1
    col = 0
    for y in x:
        col += 1
f.close() 

# Maps
map_a_star = np.zeros((row,col-1))
map_jump_point_search = np.zeros((row,col-1))
map_d_star_lite = np.zeros((row,col-1))
map_lpa_star = np.zeros((row,col-1))
# Files
f_a_star = open("/Users/vermeulenlucas/Documents/Master/SPOT/code/dev-cpp/Nav-C/a_star.txt", "r")
f_jump_point_search = open("/Users/vermeulenlucas/Documents/Master/SPOT/code/dev-cpp/Nav-C/jump_point_search.txt", "r")
f_d_star_lite = open("/Users/vermeulenlucas/Documents/Master/SPOT/code/dev-cpp/Nav-C/d_star_lite.txt", "r")
f_lpa_star = open("/Users/vermeulenlucas/Documents/Master/SPOT/code/dev-cpp/Nav-C/lpa_star.txt", "r")
# Map to fill
i = 0
for x in f_a_star:
    j = 0
    for y in x:
        if( y != "\n"):
            map_a_star[i][j] = int(y)
        j += 1
    i += 1
f.close() 

i = 0
for x in f_jump_point_search:
    j = 0
    for y in x:
        if( y != "\n"):
            map_jump_point_search[i][j] = int(y)
        j += 1
    i += 1
f.close() 

i = 0
for x in f_d_star_lite:
    j = 0
    for y in x:
        if( y != "\n"):
            map_d_star_lite[i][j] = int(y)
        j += 1
    i += 1
f.close() 

i = 0
for x in f_lpa_star:
    j = 0
    for y in x:
        if( y != "\n"):
            map_lpa_star[i][j] = int(y)
        j += 1
    i += 1
f.close() 

#plot
fig, axs = plt.subplots(2, 2)
axs[0, 0].matshow(map_a_star)
axs[0, 0].set_title('A_star')
axs[0, 1].matshow(map_jump_point_search)
axs[0, 1].set_title('JPS')
axs[1, 0].matshow(map_d_star_lite)
axs[1, 0].set_title('d_star_lite')
axs[1, 1].matshow(map_lpa_star)
axs[1, 1].set_title('lpa_star')


# Hide x labels and tick labels for top plots and y ticks for right plots.
for ax in axs.flat:
    ax.label_outer()

plt.show()
