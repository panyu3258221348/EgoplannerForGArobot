#ifndef _BSPLINE_OPTIMIZER_H_
#define _BSPLINE_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <path_searching/dyn_a_star.h>
#include <bspline_opt/uniform_bspline.h>
#include <plan_env/grid_map.h>
#include <ros/ros.h>
#include "bspline_opt/lbfgs.hpp"

// Gradient and elasitc band optimization

// Input: a signed distance field and a sequence of points
// Output: the optimized sequence of points
// The format of points: N x 3 matrix, each row is a point
namespace plan_manage
{

  class ControlPoints
  {
  public:
    double clearance;
    int size;
    Eigen::MatrixXd points;
    std::vector<std::vector<Eigen::Vector3d>> base_point; // The point at the statrt of the direction vector (collision point)
    std::vector<std::vector<Eigen::Vector3d>> direction;  // Direction vector, must be normalized.
    std::vector<bool> flag_temp;                          // A flag that used in many places. Initialize it everytime before using it.
    // std::vector<bool> occupancy;

    void resize(const int size_set)
    {
      size = size_set;

      base_point.clear();
      direction.clear();
      flag_temp.clear();
      // occupancy.clear();

      points.resize(3, size_set);
      base_point.resize(size);
      direction.resize(size);
      flag_temp.resize(size);
      // occupancy.resize(size);
    }

    void segment(ControlPoints &seg, const int start_idx, const int end_idx)
    {
      seg.resize(end_idx - start_idx + 1);
      seg.clearance = clearance;
      for (int i = start_idx; i <= end_idx; ++i)
      {
        seg.points.col(i - start_idx) = points.col(i);
        seg.base_point[i - start_idx] = base_point[i];
        seg.direction[i - start_idx] = direction[i];
      }
    }
  };

  class BsplineOptimizer
  {

  public:
    BsplineOptimizer() {}
    ~BsplineOptimizer() {}

    /* main API */
    void setEnvironment(const GridMap::Ptr &env);
    void setParam(ros::NodeHandle &nh);
    Eigen::MatrixXd BsplineOptimizeTraj(const Eigen::MatrixXd &points, const double &ts,
                                        const int &cost_function, int max_num_id, int max_time_id);

    /* helper function */

    // required inputs
    void setControlPoints(const Eigen::MatrixXd &points);
    void setBsplineInterval(const double &ts);
    void setCostFunction(const int &cost_function);
    void setTerminateCond(const int &max_num_id, const int &max_time_id);

    // optional inputs
    void setGuidePath(const vector<Eigen::Vector3d> &guide_pt);
    void setWaypoints(const vector<Eigen::Vector3d> &waypts,
                      const vector<int> &waypt_idx); // N-2 constraints at most

    void optimize();

    Eigen::MatrixXd getControlPoints();

    AStar::Ptr a_star_;
    std::vector<Eigen::Vector3d> ref_pts_;
    std::vector<std::vector<Eigen::Vector3d>> a_star_pathes_;
    std::vector<std::pair<int, int>> collision_segments_;

    std::vector<std::vector<Eigen::Vector3d>> initControlPoints(Eigen::MatrixXd &init_points, bool flag_first_init = true);
    std::vector<ControlPoints> distinctiveTrajs(void);
    bool BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double ts); // must be called after initControlPoints()
    bool BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double &final_cost, const ControlPoints &cps_set, double ts);
    bool BsplineOptimizeTrajRefine(const Eigen::MatrixXd &init_points, const double ts, Eigen::MatrixXd &optimal_points);

    inline int getOrder(void) { return order_; }

  private:
    GridMap::Ptr grid_map_;

    enum FORCE_STOP_OPTIMIZE_TYPE
    {
      DONT_STOP,
      STOP_FOR_REBOUND,
      STOP_FOR_ERROR
    } force_stop_type_;

    // main input
    // Eigen::MatrixXd control_points_;     // B-spline control points, N x dim
    double bspline_interval_; // B-spline knot span
    Eigen::Vector3d end_pt_;  // end of the trajectory
    // int             dim_;                // dimension of the B-spline
    //
    vector<Eigen::Vector3d> guide_pts_; // geometric guiding path points, N-6
    vector<Eigen::Vector3d> waypoints_; // waypts constraints
    vector<int> waypt_idx_;             // waypts constraints index
                                        //
    int max_num_id_, max_time_id_;      // stopping criteria
    int cost_function_;                 // used to determine objective function
    double start_time_;                 // global time for moving obstacles

    /* optimization parameters */
    int order_;                    // bspline degree
    double lambda1_;               // jerk smoothness weight
    double lambda2_, new_lambda2_; // distance weight
    double lambda3_;               // feasibility weight
    double lambda4_;               // curve fitting

    double lambda_height_;         // terrestrial-aerial height constraint weight
    double max_height_;            // maximum flight height (m)
    double ground_height_;         // ground surface height (m), lower bound for the trajectory
    double lambda_ground_;         // ground-attraction weight (pull traversable segments onto the ground)
    double lambda_nonholo_;        // ground non-holonomic (lateral) constraint weight
    double ground_judge_;          // z threshold (m); below it the vehicle is in ground mode
    double ground_clear_radius_;   // horizontal clearance (m); no ground-attraction near obstacles (allow climbing)

    int a;
    //
    double dist0_;             // safe distance
    double max_vel_, max_acc_; // dynamic limits

    int variable_num_;              // optimization variables
    int iter_num_;                  // iteration of the solver
    Eigen::VectorXd best_variable_; //
    double min_cost_;               //

    ControlPoints cps_;

    /* cost function */
    /* calculate each part of cost function with control points q as input */

    static double costFunction(const std::vector<double> &x, std::vector<double> &grad, void *func_data);
    void combineCost(const std::vector<double> &x, vector<double> &grad, double &cost);

    // q contains all control points
    void calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                            Eigen::MatrixXd &gradient, bool flag_use_jerk = true);
    void calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                             Eigen::MatrixXd &gradient);
    void calcDistanceCostRebound(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient, int iter_num, double smoothness_cost);
    void calcFitnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);
    void calcHeightCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);
    void calcGroundNonHolonomicCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);
    bool check_collision_and_rebound(void);
    bool findIntersectionPoint(const std::vector<Eigen::Vector3d> &a_star_path,
                               const Eigen::Vector3d &point,
                               const Eigen::Vector3d &ctrl_pts_law,
                               Eigen::Vector3d &intersection_point);

    static int earlyExit(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls);
    static double costFunctionRebound(void *func_data, const double *x, double *grad, const int n);
    static double costFunctionRefine(void *func_data, const double *x, double *grad, const int n);

    bool rebound_optimize();
    bool refine_optimize();
    void combineCostRebound(const double *x, double *grad, double &f_combine, const int n);
    void combineCostRefine(const double *x, double *grad, double &f_combine, const int n);

    /* for benckmark evaluation only */
  public:
    typedef unique_ptr<BsplineOptimizer> Ptr;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace plan_manage
#endif