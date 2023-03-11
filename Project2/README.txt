Implementation of Dijkstra's Algorithm to find the shortest path:

1) In the terminal, type "python3 dijkstra_Sagar_Ojha.py" to run the code.
2) When prompted, enter the integer values of starting coordinates and then the goal coordinates as such:
	 a) If the starting coordinates are (460, 15), enter: 460 15
	 b) If the goal coordinates are (450, 15), enter: 450 15

3) Note: The code works as it is in my laptop.
	 But, if the animation does not work on your end, remove parameter 's = 0.35' from the plt.scatter().

	 a) Line 39: Change--> plt.scatter(x_obs, y_obs, marker = "s", s = 0.35, c = 'black')
		     To-->     plt.scatter(x_obs, y_obs, marker = "s", c = 'black')

	 b) Line 41: Change--> plt.scatter(x_vis[i], y_vis[i], marker = "s", s = 0.35, c = 'green')
		     To-->     plt.scatter(x_vis[i], y_vis[i], marker = "s", c = 'green')

	 c) Line 44: Change--> plt.scatter(x_opt[i], y_opt[i], marker = "s", s = 0.35, c = 'red')
		     To-->     plt.scatter(x_opt[i], y_opt[i], marker = "s", c = 'red')


Libraries Used:
1) heapq: Used to create a priority queue to store the coordinates/nodes according to their cost to come.
	  The nodes with least cost to come was explored first.
2) matplotlib: Used to plot the obstacles and animate the node exploration and optimal path.
3) time: Used to compute the total time taken to obtain the optimal path.