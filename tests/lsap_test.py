import scipy.optimize
import numpy as np
import random
import math

def randn(n):
  return np.array([random.random() for _ in range(n)])

TRACKERS = np.array([randn(2) for _ in range(20)])
DETS = np.array([randn(2) for _ in range(20) ])

def cost_matrix(detections, trackers):
  def cost(det_idx, track_idx):
    det = detections[det_idx]
    track = trackers[track_idx]
    delta = det - track
    return -1 / math.sqrt(np.dot(delta, delta))
  return np.fromfunction(np.vectorize(cost), (detections.shape[0], trackers.shape[0]), dtype=int)


cost = cost_matrix(DETS, TRACKERS)
det_inds, track_inds = scipy.optimize.linear_sum_assignment(cost)
assignments = list(zip(det_inds, track_inds))

with open('cost.txt', 'wt') as f:
  f.write(str(cost))

with open('trackers.h', 'wt') as f:
  f.writelines(f"tracks.emplace_back({tracker[0]}, {tracker[1]});\n" for tracker in TRACKERS)

with open('detections.h', 'wt') as f:
  f.writelines(f"detections.emplace_back({det[0]}, {det[1]});\n" for det in DETS)

with open('assignments.csv', 'wt') as f:
  f.writelines(f"{ass[0]}, {ass[1]}\n" for ass in assignments)