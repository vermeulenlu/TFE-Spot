import matplotlib.pyplot as plt
import numpy as np

## Count the width
row = 0
col = 0
f = open("/Users/vermeulenlucas/Documents/Master/SPOT/code/dev-cpp/Nav-C/JPS.txt", "r")
for x in f:
    row += 1
    col = 0
    for y in x:
        col += 1
f.close() 

map_JPS = np.zeros((row,col-1))
f_JPS= open("/Users/vermeulenlucas/Documents/Master/SPOT/code/dev-cpp/Nav-C/JPS.txt", "r")
i = 0
for x in f_JPS:
    j = 0
    for y in x:
        if( y != "\n"):
            map_JPS[j][i] = int(y)
        j += 1
    i += 1
f.close() 

#plot
fig, axs = plt.subplots()
axs.matshow(map_JPS)
axs.set_title('JPS')
plt.show()

# ## Count the width
# row = 0
# col = 0
# f = open("/Users/vermeulenlucas/Documents/Master/SPOT/code/dev-cpp/Nav-C/a_star.txt", "r")
# for x in f:
#     row += 1
#     col = 0
#     for y in x:
#         col += 1
# f.close() 

# # Maps
# map_a_star = np.zeros((row,col-1))
# map_jump_point_search = np.zeros((row,col-1))
# map_d_star_lite = np.zeros((row,col-1))
# map_lpa_star = np.zeros((row,col-1))
# # Files
# f_a_star = open("/Users/vermeulenlucas/Documents/Master/SPOT/code/dev-cpp/Nav-C/a_star.txt", "r")
# f_jump_point_search = open("/Users/vermeulenlucas/Documents/Master/SPOT/code/dev-cpp/Nav-C/jump_point_search.txt", "r")
# f_d_star_lite = open("/Users/vermeulenlucas/Documents/Master/SPOT/code/dev-cpp/Nav-C/d_star_lite.txt", "r")
# f_lpa_star = open("/Users/vermeulenlucas/Documents/Master/SPOT/code/dev-cpp/Nav-C/lpa_star.txt", "r")
# # Map to fill
# i = 0
# for x in f_a_star:
#     j = 0
#     for y in x:
#         if( y != "\n"):
#             map_a_star[i][j] = int(y)
#         j += 1
#     i += 1
# f.close() 

# i = 0
# for x in f_jump_point_search:
#     j = 0
#     for y in x:
#         if( y != "\n"):
#             map_jump_point_search[i][j] = int(y)
#         j += 1
#     i += 1
# f.close() 

# i = 0
# for x in f_d_star_lite:
#     j = 0
#     for y in x:
#         if( y != "\n"):
#             map_d_star_lite[i][j] = int(y)
#         j += 1
#     i += 1
# f.close() 

# i = 0
# for x in f_lpa_star:
#     j = 0
#     for y in x:
#         if( y != "\n"):
#             map_lpa_star[i][j] = int(y)
#         j += 1
#     i += 1
# f.close() 

# #plot
# fig, axs = plt.subplots()
# axs.matshow(map_a_star)
# axs.set_title('A_star')
# plt.show()
# fig, axs = plt.subplots(2, 2)
# axs[0, 0].matshow(map_a_star)
# axs[0, 0].set_title('A_star')
# axs[0, 1].matshow(map_jump_point_search)
# axs[0, 1].set_title('JPS')
# axs[1, 0].matshow(map_d_star_lite)
# axs[1, 0].set_title('d_star_lite')
# axs[1, 1].matshow(map_lpa_star)
# axs[1, 1].set_title('lpa_star')


# # Hide x labels and tick labels for top plots and y ticks for right plots.
# for ax in axs.flat:
#     ax.label_outer()

# plt.show()

# x = [150,200,300,600]
# y_a_star = [0.007, 0.016,0.02,0.05]
# y_jps = [0.007, 0.012, 0.07,0.046]
# y_d_star = [0.3, 0.56, 1.35,4.53]
# y_lpa = [0.3, 0.56,1.25,4.55]
# y_grid = [0.06,0.14,0.35,1.04]

# fig, ax = plt.subplots()
# ax.plot(x, y_a_star, label='a_star')  # Plot some data on the axes.
# ax.plot(x, y_jps, label='jps')  # Plot more data on the axes...
# ax.plot(x, y_d_star, label='d_star')  # ... and some more.
# ax.plot(x, y_lpa, label='lpa')  # ... and some more.
# ax.plot(x, y_grid, label='grid process')  # ... and some more.
# ax.set_xlabel('Size of the map')  # Add an x-label to the axes.
# ax.set_ylabel('Time Path Planning')  # Add a y-label to the axes.
# ax.legend();  # Add a legend.
# plt.show()
