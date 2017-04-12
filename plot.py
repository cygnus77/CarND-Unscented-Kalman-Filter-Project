import pandas
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# with open("C:\\Temp\\CarND-Unscented-Kalman-Filter-Project\\good_values.txt") as f:
	# for line in csv.reader(f, dialect="excel-tab"):
		# print(line)

df = pandas.read_csv("C:\\Temp\\CarND-Unscented-Kalman-Filter-Project\\good_values.txt", sep='\t', header=0)
# print(df['std_a_'])
# print(df['std_yawdd_'])
# print(df['2nis_total'])
ax.set_xlabel('std_a_')
ax.set_ylabel('std_yawdd_')
ax.scatter(xs=df['std_a_'], ys=df['std_yawdd_'], zs=df['2nis_total'])
plt.show()
