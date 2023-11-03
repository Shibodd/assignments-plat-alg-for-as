import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# x_est; y_est; x_gt; y_gt; squared_error
with open('res.txt','r') as file:
  rmse = float(next(file))
  data = pd.read_csv(file, delimiter=';')

n = data.shape[0]

fig, (ax1, ax2) = plt.subplots(2)
fig.text(0.2, 0.9, f"RMSE: {rmse}")
ax1.plot(data['x_est'], data['y_est'])
ax1.scatter(data['x_gt'], data['y_gt'],color='green', s=5)

t = np.arange(n)
ax2.plot(t, data['squared_error'])
plt.show()
