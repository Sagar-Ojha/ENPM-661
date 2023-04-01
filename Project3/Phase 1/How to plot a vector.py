import matplotlib.pyplot as plt
import numpy as np

def move(curr_node,degree,step_size):
    x = curr_node[0]
    y = curr_node[1]
    x_new = (step_size) * np.cos(np.deg2rad(degree)) + x  # - (y-0)*np.sin(np.deg2rad(degree)) + length
    y_new = (step_size) * np.sin(np.deg2rad(degree)) + y  # + (y-0)*np.cos(np.deg2rad(degree)) + length
    new_node = (round(x_new, 2), round(y_new, 2))
    return (new_node)


node = (0, 0)
x0 = node[0]
y0 = node[1]
print (node[0])
deg = 30
step = 2
x1, y1 = move(node,deg, step)
print(x1,",", y1)

fig, ax = plt.subplots()
q = ax.quiver(x0, y0, x1, y1,units='xy' ,scale=1)


'''X0 = np.array((0))
Y0= np.array((0))
U0 = np.array((2))
V0 = np.array((-2))

fig, ax = plt.subplots()
q0 = plt.quiver(X0, Y0, U0, V0,units='xy' ,scale=1,color= 'r',headwidth = 1,headlength=0)

Node=[X0+U0, Y0+V0]

print('Node0: ')

print(Node)

X1 = np.array((2))
Y1= np.array((-2))
U1 = np.array((3))
V1 = np.array((-2))

q1 = plt.quiver(X1, Y1, U1, V1,units='xy' ,scale=1, headwidth = 1,headlength=0)

Node1=[X1+U1, Y1+V1]

print('Node1: ')

print(Node1)

X2 = np.array((5))
Y2= np.array((-4))
U2 = np.array((3))
V2 = np.array((-1))

q2 = ax.quiver(X2, Y2, U2, V2,units='xy' ,scale=1)

Node2=[X2+U2, Y2+V2]

print('Node2: ')
print(Node2)



X3 = np.array((2))
Y3= np.array((-2))
U3 = np.array((1))
V3 = np.array((-3.5))

q3 = ax.quiver(X3, Y3, U3, V3,units='xy' ,scale=1)

Node3=[X3+U3, Y3+V3]

print('Node3: ')

print(Node3)'''




plt.grid()

ax.set_aspect('equal')

plt.xlim(-10,10)
plt.ylim(-10,10)

plt.title('How to plot a vector in matplotlib ?',fontsize=10)

plt.savefig('how_to_plot_a_vector_in_matplotlib_fig3.png', bbox_inches='tight')

plt.show()
plt.close()
    
