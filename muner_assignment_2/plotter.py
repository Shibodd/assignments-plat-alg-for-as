import numpy as np
import matplotlib.pyplot as plt

x=[]
y=[]
xgt=[]
ygt=[]
xrmse=[]
yrmse=[]
fig, (ax1, ax2, ax3) = plt.subplots(3)

with open('res.txt','r') as file:
    # reading each line    
    next(file)
    for line in file:
        # reading each word        
        word = line.split()
        x.append(float(word[0]))
        y.append(float(word[1])) 
        xgt.append(float(word[2]))
        ygt.append(float(word[3])) 
        xrmse.append(float(word[4]))
        yrmse.append(float(word[5])) 

ax1.plot(x, y)
ax1.scatter(xgt, ygt,color='green', s=5)

t=[i for i in range(len(xrmse))]
ax2.plot(t, xrmse)
ax3.plot(t, yrmse)
plt.show()
