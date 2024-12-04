
'''
This file makes a matplotlib plot which displays the runtimes from splat_neighborhood for 
different numbers of splats. 

'''
import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt("imgs/2_powers_neighborhood_splats/clock.txt")
x_values = np.arange(4, 21)
x_values = 2**x_values

plt.plot(x_values, data, marker='o')

plt.xscale('log')
plt.yscale("log")

tick_labels = [f'$2^{{{i}}}$' for i in range(4, 21)]
plt.xticks(x_values, tick_labels)
plt.minorticks_off()
plt.grid(True)

plt.title("Nearest Neighbor Times for 1 Forward Pass")
plt.xlabel("Number of Splats")
plt.ylabel("Time (seconds)")

plt.savefig("imgs/2_powers_neighborhood_splats/clock_plot.png")
plt.show()
