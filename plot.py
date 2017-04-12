import sys
import pandas
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

if len(sys.argv) < 2:
	print("Usage: <txt file path>");
	sys.exit(0)

df = pandas.read_csv(sys.argv[1], sep='\t', header=0)
ax.set_xlabel('std_a_')
ax.set_ylabel('std_yawdd_')
ax.scatter(xs=df['std_a_'], ys=df['std_yawdd_'], zs=df['2nis_total'])
plt.show()
