/*
C++ friendly version of scipy.optimize.linear_sum_assignment

Credits to the scipy project, i only applied modifications to make it C++ friendly.
Author of the C++ edits: Shibodd
*/

/*
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above
   copyright notice, this list of conditions and the following
   disclaimer in the documentation and/or other materials provided
   with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived
   from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


This code implements the shortest augmenting path algorithm for the
rectangular assignment problem.  This implementation is based on the
pseudocode described in pages 1685-1686 of:

    DF Crouse. On implementing 2D rectangular assignment algorithms.
    IEEE Transactions on Aerospace and Electronic Systems
    52(4):1679-1696, August 2016
    doi: 10.1109/TAES.2016.140952

Author: PM Larsen
*/

#include <cmath>
#include <vector>
#include <numeric>
#include <algorithm>
#include "rectangular_lsap.hpp"

template <typename T>
std::vector<int> argsort_iter(const std::vector<T> &v)
{
  std::vector<int> index(v.size());
  std::iota(index.begin(), index.end(), 0);
  std::sort(index.begin(), index.end(), [&v](int i, int j)
            { return v[i] < v[j]; });
  return index;
}

static int
augmenting_path(int nc, const Eigen::MatrixXd& costs, std::vector<double> &u,
                std::vector<double> &v, std::vector<int> &path,
                std::vector<int> &row4col,
                std::vector<double> &shortestPathCosts, int i,
                std::vector<bool> &SR, std::vector<bool> &SC,
                std::vector<int> &remaining, double *p_minVal)
{
  double minVal = 0;

  // Crouse's pseudocode uses set complements to keep track of remaining
  // nodes.  Here we use a vector, as it is more efficient in C++.
  int num_remaining = nc;
  for (int it = 0; it < nc; it++)
  {
    // Filling this up in reverse order ensures that the solution of a
    // constant cost matrix is the identity matrix (c.f. #11602).
    remaining[it] = nc - it - 1;
  }

  std::fill(SR.begin(), SR.end(), false);
  std::fill(SC.begin(), SC.end(), false);
  std::fill(shortestPathCosts.begin(), shortestPathCosts.end(), INFINITY);

  // find shortest augmenting path
  int sink = -1;
  while (sink == -1)
  {

    int index = -1;
    double lowest = INFINITY;
    SR[i] = true;

    for (int it = 0; it < num_remaining; it++)
    {
      int j = remaining[it];

      double r = minVal + costs(i, j) - u[i] - v[j];
      if (r < shortestPathCosts[j])
      {
        path[j] = i;
        shortestPathCosts[j] = r;
      }

      // When multiple nodes have the minimum cost, we select one which
      // gives us a new sink node. This is particularly important for
      // integer cost matrices with small co-efficients.
      if (shortestPathCosts[j] < lowest ||
          (shortestPathCosts[j] == lowest && row4col[j] == -1))
      {
        lowest = shortestPathCosts[j];
        index = it;
      }
    }

    minVal = lowest;
    if (minVal == INFINITY)
    { // infeasible cost matrix
      return -1;
    }

    int j = remaining[index];
    if (row4col[j] == -1)
    {
      sink = j;
    }
    else
    {
      i = row4col[j];
    }

    SC[j] = true;
    remaining[index] = remaining[--num_remaining];
  }

  *p_minVal = minVal;
  return sink;
}


std::vector<std::pair<int, int>> lsap::solve(Eigen::MatrixXd costs, bool maximize)
{
  std::vector<std::pair<int, int>> assignments;

  // handle trivial inputs
  if (costs.rows() == 0 || costs.cols() == 0)
  {
    return assignments;
  }

  // tall rectangular cost matrix must be transposed
  bool transpose = costs.cols() < costs.rows();

  if (transpose)
    costs.transposeInPlace();
    
  if (maximize)
    costs = (-costs).eval();

  int nr = costs.rows();
  int nc = costs.cols();

  // test for NaN and -inf entries
  for (int i = 0; i < nr; ++i) {
    for (int j = 0; j < nc; ++j) {
      if (costs(i, j) != costs(i, j) || costs(i, j) == -INFINITY)  
        throw lsap::SolverException(lsap::RECTANGULAR_LSAP_INVALID);
    }
  }


  // initialize variables
  std::vector<double> u(nr, 0);
  std::vector<double> v(nc, 0);
  std::vector<double> shortestPathCosts(nc);
  std::vector<int> path(nc, -1);
  std::vector<int> col4row(nr, -1);
  std::vector<int> row4col(nc, -1);
  std::vector<bool> SR(nr);
  std::vector<bool> SC(nc);
  std::vector<int> remaining(nc);

  // iteratively build the solution
  for (int curRow = 0; curRow < nr; curRow++)
  {

    double minVal;
    int sink = augmenting_path(nc, costs, u, v, path, row4col,
                                    shortestPathCosts, curRow, SR, SC,
                                    remaining, &minVal);
    if (sink < 0)
    {
      throw lsap::SolverException(lsap::RECTANGULAR_LSAP_INFEASIBLE);
    }

    // update dual variables
    u[curRow] += minVal;
    for (int i = 0; i < nr; i++)
    {
      if (SR[i] && i != curRow)
      {
        u[i] += minVal - shortestPathCosts[col4row[i]];
      }
    }

    for (int j = 0; j < nc; j++)
    {
      if (SC[j])
      {
        v[j] -= minVal - shortestPathCosts[j];
      }
    }

    // augment previous solution
    int j = sink;
    while (1)
    {
      int i = path[j];
      row4col[j] = i;
      std::swap(col4row[i], j);
      if (i == curRow)
      {
        break;
      }
    }
  }

  if (transpose)
  {
    int i = 0;
    for (auto v : argsort_iter(col4row))
    {
      assignments.push_back(std::make_pair(col4row[v], v));
      i++;
    }
  }
  else
  {
    for (int i = 0; i < nr; i++)
    {
      assignments.push_back(std::make_pair(i, col4row[i]));
    }
  }

  return assignments;
}
