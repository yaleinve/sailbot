
from math import sin,cos,atan2
from numpy import pi
import numpy as np

n=40
n_smooth=10
offsize=pi/8


from random import shuffle
def get_smoothed_data(apVecs,truVecs,apRe,truRe):
	def mean_of_circular_quantities(v):
		return atan2(1.0/len(v)*sum(sin(x) for x in v),1.0/len(v)*sum(cos(x) for x in v))%(2*pi)
	def distAngAbs(a,b):
		return min(abs((a-b)%(2*pi)),abs((b-a)%(2*pi)))
	def bin_smoothed(v):
		if(len(v)==0):
			return (0,.0001)#so doesnt cause any errors
		vSorted=sorted(v[:],key =lambda x:x[0])
		pairs=[]
		e=0
		s=0
		n=len(vSorted)
		while(distAngAbs(vSorted[s][0],vSorted[0][0])<offsize):
			s-=1
			if(s==-n):
				break
		while(distAngAbs(vSorted[e][0],vSorted[0][0])<offsize and e<s+n):
			e+=1
		pairs.append((vSorted[0],e-s))
		for i in range(1,n):
			while(not distAngAbs(vSorted[s][0],vSorted[i][0])<offsize):
				s+=1
			while(distAngAbs(vSorted[e%n][0],vSorted[i][0])<offsize and e<s+n):
				e+=1
			pairs.append((vSorted[i],e-s))
		shuffle(pairs)
		pairs.sort(key=lambda x:x[1])
		return pairs[0][0]
	def mean_smoothed(vs):
		a=mean_of_circular_quantities([v[0] for v in vs])
		s=1.0/len(vs)*sum(v[1] for v in vs)
	# print "APTRURE",apRe,ls
		return a,s
	if(len(apVecs)==0):
		return (0,.001),(0,.001)
	apRe.append(mean_smoothed(apVecs))
	if(len(apRe)>n_smooth):
		apRe.pop(0)
	truRe.append(mean_smoothed(truVecs))
	if(len(truRe)>n_smooth):
		truRe.pop(0)
	#apRe[-1],truRe[-1]#
	return mean_smoothed(apRe),mean_smoothed(truRe)


la,lv=1,3
las=[la]
windData=[(la,lv)]
for i in range(10000):
	nv=abs(lv+np.random.normal(scale=.04))
	if(np.random.random()*nv<.5):
		nv+=.1
	if(np.random.random()*nv>5):
		nv-=.3
	nv=abs(nv)
	lv=nv
	la=(la+np.random.normal(scale=.1))%(2*pi)
	na=(la+np.random.normal(scale=.25))%(2*pi)
	if(np.random.random()<.1):
		na=np.random.random()*2*pi
	windData.append((na,nv))
	las.append(la)
res=[]
res2=[]
swd=[]
for i in range(len(windData)):
	ls=windData[max(0,i-n):i]
	(sa,sv),(_,_)=get_smoothed_data(ls,ls,res,res2)
	if(i%10==0):
		print windData[i],sa,sv
	swd.append((sa,sv))
import matplotlib.pyplot as plt
plt.scatter(list(range(10001)),[a for a,s in windData])
plt.plot(list(range(10001)),[a for a,s in swd])
plt.plot(list(range(10001)),las)
plt.show()