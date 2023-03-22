import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

TITLEFONTSIZE=20
LABELFONTSIZE=14
TICKFONTSIZE=13

data = np.array([
    [-28.86, -31.51, -20.52, -23.31, -15.55, -15.13, -5.75],
    # [-13.01, -24.15, -35.6, -40.94, -41.41, -45.57, -27.38], This is the true data but not good for plotting against diverging color scales
    [-13.01, -24.15, -35.6, -40.94, -41.41, -55.27, -27.38],
    [6.43, -1.25, -7.17, -13.09, -18.22, -28.62, -38.19],
    [17.42, 13.34, 3.58, -1.43, -8.3, -13.58, -20.66],
    [35.33, 42.38, 19.12, 12.79, 6.99, -2.46, -9.48],
    [55.27, 49.71, 54.32, 50.22, 40.96, 31.63, 32.05],
    [6.21, 18.06, 16.6, 34.68, 24.95, 31.52, 33.02],
])


fig, ax = plt.subplots()

im = ax.imshow(data, cmap=mpl.colormaps['PuOr'])
ax.set_title(r'SVM SeBSF Verification Map', fontsize=TITLEFONTSIZE)
ax.set_ylabel(r'$d$ [ft]', fontsize=LABELFONTSIZE)
ax.set_xlabel(r'$\theta$ [deg]', fontsize=LABELFONTSIZE)
ax.set_xticks(np.arange(7))
ax.set_yticks(np.arange(7))
ax.set_yticklabels([-3, -2, -1, 0, 1, 2, 3])
ax.set_xticklabels([45, 30, 15, 0, -15, -30, -45])
ax.tick_params(axis='both', which='major', labelsize=TICKFONTSIZE)
cb = fig.colorbar(im, ax=ax)
cb.ax.tick_params(labelsize=TICKFONTSIZE)
# ax.plot(X,Y)

# Plot polygon of SeBSF region
SEBSF_POLYGON = [(1,0),(4,0),(4,1),(5,1),(5,2),(6,2),(6,5),(5,5),(5,6),(2,6),(2,2),(1,2),(1,0)]
X_POLY = [x-0.45 for _,x in SEBSF_POLYGON]
Y_POLY = [y-0.5 for y,_ in SEBSF_POLYGON]
ax.plot(X_POLY,Y_POLY,linestyle='-',color='#8B0000',lw=3)
#ax.fill(X_POLY,Y_POLY,facecolor='r',edgecolor='k',lw=3,alpha=0.25)

# Plot directional derivative arrows
stats = []
max_d = float('-inf')
NUM_ROWS = NUM_COLS = 7
for t in range(NUM_ROWS-1):
    stats.append([])
    #for j in reversed(range(1,NUM_COLS)):
    for d in range(1,NUM_COLS):
        #dF_dt = data[i+1][j] - data[i][j]
        dF_dt = data[d][t+1] - data[d][t]
        dF_dd = data[d-1][t] - data[d][t]
        stats[-1].append((t,d,dF_dt,dF_dd))
        max_d = max(max_d, max(abs(dF_dt), abs(dF_dd)))

colors = {
    True: 'r',
    False: 'g',
}

# ax.plot(1,1,marker='o',markersize=10,color='r')
# ax.plot(2,1,marker='o',markersize=10,color='orange')
# ax.plot(1,2,marker='o',markersize=10,color='g')

# Point [1,1]:
# dF_dt = [2,1]_x - [1,1]_x
# dF_dd = [1,0]_y - [1,1]_y

# Now plot our lines on each pixel
MIN_ARROW_LEN = 0.3 # Between [0,1]
for stat_row in stats:
    for t, d, dF_dt, dF_dd in stat_row:
        # dF_dt arrow first
        print(f"Stats:\nt={t},d={d}\ndF_dt={dF_dt}\tdF_dd={dF_dd}\n\n")
        t_arr_len = min(abs(dF_dt/max_d), MIN_ARROW_LEN)
        ax.arrow(t,d,t_arr_len,0,color=colors[dF_dt<0], width=0.05)

        # dF_dd arrow
        d_arr_len = min(abs(dF_dd/max_d), MIN_ARROW_LEN)
        ax.arrow(t,d,0,-d_arr_len,color=colors[dF_dd<0], width=0.05)

        #ax.text(t,d,f"{data[d][t]}")


#fig.show()
#plt.show()
fig.savefig("./plots/dt-map2.png")
