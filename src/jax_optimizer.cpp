/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    VisualISAM2Example.cpp
 * @brief   A visualSLAM example for the structure-from-motion problem on a simulated dataset
 * This version uses iSAM2 to solve the problem incrementally
 * @author  Duy-Nguyen Ta
 */

/**
 * A structure-from-motion example with landmarks
 *  - The landmarks form a 10 meter cube
 *  - The robot rotates around the landmarks, always facing towards the cube
 */

// In planar SLAM example we use Pose2 variables (x, y, theta) to represent the robot poses
#include <gtsam/geometry/Pose2.h>

// We will use simple integer Keys to refer to the robot poses.
#include <gtsam/inference/Key.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
// Here we will use Between factors for the relative motion described by odometry measurements.
// We will also use a Between Factor to encode the loop closure constraint
// Also, we will initialize the robot at the origin using a Prior factor.
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

// When the factors are created, we will add them to a Factor Graph. As the factors we are using
// are nonlinear factors, we will need a Nonlinear Factor Graph.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// Finally, once all of the factors have been added to our factor graph, we will want to
// solve/optimize to graph to find the best (Maximum A Posteriori) set of variable values.
// GTSAM includes several nonlinear optimizers to perform this step. Here we will use the
// a Gauss-Newton solver
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

// Once the optimized values have been calculated, we can also calculate the marginal covariance
// of desired variables
#include <gtsam/nonlinear/Marginals.h>

// The nonlinear solvers within GTSAM are iterative solvers, meaning they linearize the
// nonlinear functions around an initial linearization point, then solve the linear system
// to update the linearization point. This happens repeatedly until the solver converges
// to a consistent set of variable values. This requires us to specify an initial guess
// for each variable, held in a Values container.
#include <gtsam/nonlinear/Values.h>


#include <vector>

#include <fstream>
#include <stdio.h>
#include <libgen.h>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <time.h>
#include <map>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
int main(int argc, char* argv[])
{
  // Load G2O File
  ifstream g2o_file;
  std::string filename = "edges.g2o";
  g2o_file.open(filename);

  if (!g2o_file.is_open())
  {
    cout << "failed to open " << filename << "\n";
    return 0;
  }

  // Initialize Factor Graph
  NonlinearFactorGraph graph;

  // An initial Estimate data member to fill up when we get vertexes
  Values initialEstimate;

  // READ G2O File
  std::string type;
  int num_nodes = 0;
  int fixed_node_index = 0;
  int num_edges = 0;
  std::map<std::string, int> node_id_to_index_map;
  std::map<int, string> index_to_node_id_map;
  std::vector<std::vector<string>> edge_list;
  while (!g2o_file.eof())
  {

    g2o_file >> type;
    if (type.compare("FIX") == 0)
    {
      std::string node;
      g2o_file >> node;

      int index = node_id_to_index_map[node];

      // 2a. Add a prior on the first pose, setting it to the origin
      // A prior factor consists of a mean and a noise model (covariance matrix)
      noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.001, 0.001, 0.001));
      graph.emplace_shared<PriorFactor<Pose2> >(index, Pose2(0, 0, 0), priorNoise);
      fixed_node_index = index;
    }

    else if (type.compare("EDGE_SE2") == 0)
    {
      string from, to;
      double x, y, z, P11, P12, P13, P22, P23, P33;
      g2o_file >> from >> to >> x >> y >> z >> P11 >> P12 >> P13 >> P22 >> P23 >> P33;

      if(!from.empty() && !to.empty())
      {
        std::vector<string> edge = {from, to};
        edge_list.push_back(edge);
        // Create the Noise model for this edge
        noiseModel::Diagonal::shared_ptr model = noiseModel::Diagonal::Sigmas(Vector3(P11, P22, P33));

        // 2b. Add odometry factors
        // Create odometry (Between) factors for each edge
        graph.emplace_shared<BetweenFactor<Pose2> >(node_id_to_index_map[from], node_id_to_index_map[to],
                                                    Pose2(x, y, z), model);
        num_edges++;
      }
    }

    else if (type.compare("VERTEX_SE2") == 0)
    {
      std::string id;
      double x, y, z;
      g2o_file >> id >> x >> y >> z;
      node_id_to_index_map[id] = num_nodes;
      index_to_node_id_map[num_nodes] = id;
      initialEstimate.insert(num_nodes, Pose2(x, y,  z));
      num_nodes++;
    }
  }
  g2o_file.close();

  // 4. Optimize the initial values using a Gauss-Newton nonlinear optimizer
  // The optimizer accepts an optional set of configuration parameters,
  // controlling things like convergence criteria, the type of linear
  // system solver to use, and the amount of information displayed during
  // optimization. We will set a few parameters as a demonstration.
  GaussNewtonParams parameters;
  // Stop iterating once the change in error between steps is less than this value
  parameters.relativeErrorTol = 1e-5;
  // Do not perform more than N iteration steps
  parameters.maxIterations = 30;
  // Create the optimizer ...
  GaussNewtonOptimizer optimizer(graph, initialEstimate, parameters);

  clock_t start_time = clock();

  Values optimized_values = optimizer.optimize();
  clock_t time = clock() - start_time;
  cout << "took " << time << " ticks or " << ((float)time)/CLOCKS_PER_SEC << " seconds " << endl;
  cout << "optimized " << num_nodes << " nodes and " << num_edges << " edges " << endl;

  // Output as a g2o file
  ofstream output_file;
  output_file.open("output.g2o");

  for (int i = 0; i < num_nodes; i++)
  {
    Pose2 output = optimized_values.at<Pose2>(i);
    output_file << "VERTEX_SE2 " << index_to_node_id_map[i] << " " << output.x() << " " << output.y() << " " << output.theta() << " " << "\n";
  }
  output_file << "FIX " << fixed_node_index << "\n";
  for (int i = 0; i < edge_list.size(); i++)
  {
    output_file << "EDGE_SE2 " << edge_list[i][0] << " " << edge_list[i][1] << " \n";
  }
  output_file.close();

  return 0;
}
/* ************************************************************************* */
