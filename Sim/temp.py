import numpy as np
import matplotlib.pyplot as plt

def sign(a,b,c):
	return (a[0]-c[0])*(b[1]-c[1]) -(b[0]-c[0])*(a[1]-c[1])		

def ptin(x,a,b,c,d):
	d1=sign(x,a,b)
	d2=sign(x,b,c)
	d3=sign(x,c,d)
	d4=sign(x,d,a)

	has_neg=(d1<0) or (d2<0) or (d3<0) or(d4<0)
	has_pos=(d1>0) or (d2>0) or (d3>0) or(d4>0)

	return not(has_neg and has_pos)

# a=(-379.0748291015625, -14.44198989868164)
# b=(-378.80542201006585, -19.53388736613435)
# c=(-376.86772352962225, -19.038584833446983)
# x=(-378.4154357910156, -19.533935546875)
a=(-10,10)
b=(-10,-10)
c=(10,-10)
d=(10,10)
x=(8,8)
print(ptin(x,a,b,c,d))
# plt.scatter(a[0],a[1])
# plt.scatter(b[0],b[1])
# plt.scatter(c[0],b[1])
# plt.plot([a[0],b[0],c[0],a[0]],[a[1],b[1],c[1],a[1]])
# plt.scatter(x[0],x[1],c='b')
# plt.show()