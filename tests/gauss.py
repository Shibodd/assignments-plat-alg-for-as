import numpy as np

def multivariate_normal(x, d, mean, covariance):
  """pdf of the multivariate normal distribution."""
  x_m = x - mean
  return (1. / (np.sqrt((2 * np.pi)**d * np.linalg.det(covariance))) * 
          np.exp(-(np.linalg.solve(covariance, x_m).T.dot(x_m)) / 2))

d = multivariate_normal(
  np.array([0.351, -0.1]),
  2,
  np.array([5, 3]),
  np.array([[7, 2],
            [1, 3]])
)

print(d)