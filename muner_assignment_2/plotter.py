import numpy as np
import matplotlib.pyplot as plt
import pandas as pd



# x_est; y_est; x_gt; y_gt; squared_error
with open('res.txt','r') as file:
  rmse = float(next(file))
  data = pd.read_csv(file, delimiter=';')

print(rmse)
print(data)

exit()

fig, (ax1, ax2, ax3) = plt.subplots(3)
ax1.plot(x, y)
ax1.scatter(xgt, ygt,color='green', s=5)

t=np.arange(x)
ax2.plot(t, xrmse)
ax3.plot(t, yrmse)
plt.show()
