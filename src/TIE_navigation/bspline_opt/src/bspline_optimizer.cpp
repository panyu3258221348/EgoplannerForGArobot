#include "bspline_opt/bspline_optimizer.h"
#include "bspline_opt/gradient_descent_optimizer.h"
// using namespace std;

namespace plan_manage
{

  void BsplineOptimizer::setParam(ros::NodeHandle &nh)
  {
    nh.param("optimization/lambda_smooth", lambda1_, -1.0);
    nh.param("optimization/lambda_collision", lambda2_, -1.0);
    nh.param("optimization/lambda_feasibility", lambda3_, -1.0);
    nh.param("optimization/lambda_fitness", lambda4_, -1.0);

    nh.param("optimization/dist0", dist0_, -1.0);
    nh.param("optimization/max_vel", max_vel_, -1.0);
    nh.param("optimization/max_acc", max_acc_, -1.0);

    nh.param("optimization/lambda_height", lambda_height_, 0.0);
    nh.param("optimization/max_height", max_height_, 2.0);
    nh.param("optimization/ground_height", ground_height_, 0.2);
    nh.param("optimization/lambda_ground", lambda_ground_, 0.0);
    nh.param("optimization/lambda_nonholo", lambda_nonholo_, 0.0);
    nh.param("optimization/ground_judge", ground_judge_, 0.3);
    nh.param("optimization/ground_clear_radius", ground_clear_radius_, 1.5);

    nh.param("optimization/order", order_, 3);
  }

  void BsplineOptimizer::setEnvironment(const GridMap::Ptr &env)
  {
    this->grid_map_ = env;
  }

  void BsplineOptimizer::setControlPoints(const Eigen::MatrixXd &points)
  {
    cps_.points = points;
  }

  void BsplineOptimizer::setBsplineInterval(const double &ts) { bspline_interval_ = ts; }

  /* This function is very similar to check_collision_and_rebound(). 
   * It was written separately, just because I did it once and it has been running stably since March 2020.
   * But I will merge then someday.*/
  std::vector<std::vector<Eigen::Vector3d>> BsplineOptimizer::initControlPoints(Eigen::MatrixXd &init_points, bool flag_first_init /*= true*/)
  {

    if (flag_first_init)
    {
      cps_.clearance = dist0_;
      cps_.resize(init_points.cols());
      cps_.points = init_points;
    }

    /*** Segment the initial trajectory according to obstacles ***/
    constexpr int ENOUGH_INTERVAL = 2;
    double step_size = grid_map_->getResolution() / ((init_points.col(0) - init_points.rightCols(1)).norm() / (init_points.cols() - 1)) / 2;
    int in_id, out_id;
    vector<std::pair<int, int>> segment_ids;
    int same_occ_state_times = ENOUGH_INTERVAL + 1;
    bool occ, last_occ = false;
    bool flag_got_start = false, flag_got_end = false, flag_got_end_maybe = false;
    int i_end = (int)init_points.cols() - order_ - ((int)init_points.cols() - 2 * order_) / 3; // only check closed 2/3 points.
    for (int i = order_; i <= i_end; ++i)
    {
      for (double a = 1.0; a >= 0.0; a -= step_size)
      {
        occ = grid_map_->getInflateOccupancy(a * init_points.col(i - 1) + (1 - a) * init_points.col(i));
        // cout << setprecision(5);
        // cout << (a * init_points.col(i-1) + (1-a) * init_points.col(i)).transpose() << " occ1=" << occ << endl;

        if (occ && !last_occ)
        {
          if (same_occ_state_times > ENOUGH_INTERVAL || i == order_)
          {
            in_id = i - 1;
            flag_got_start = true;
          }
          same_occ_state_times = 0;
          flag_got_end_maybe = false; // terminate in advance
        }
        else if (!occ && last_occ)
        {
          out_id = i;
          flag_got_end_maybe = true;
          same_occ_state_times = 0;
        }
        else
        {
          ++same_occ_state_times;
        }

        if (flag_got_end_maybe && (same_occ_state_times > ENOUGH_INTERVAL || (i == (int)init_points.cols() - order_)))
        {
          flag_got_end_maybe = false;
          flag_got_end = true;
        }

        last_occ = occ;

        if (flag_got_start && flag_got_end)
        {
          flag_got_start = false;
          flag_got_end = false;
          segment_ids.push_back(std::pair<int, int>(in_id, out_id));
        }
      }
    }

    /*** a star search ***/
    vector<vector<Eigen::Vector3d>> a_star_pathes;
    for (size_t i = 0; i < segment_ids.size(); ++i)
    {
      //cout << "in=" << in.transpose() << " out=" << out.transpose() << endl;
      Eigen::Vector3d in(init_points.col(segment_ids[i].first)), out(init_points.col(segment_ids[i].second));
      if (a_star_->AstarSearch(/*(in-out).norm()/10+0.05*/ 0.2, in, out))
      {
        auto path = a_star_->getPath();
        double zmin = 999, zmax = -999;
        for (auto& p : path) { zmin = std::min(zmin, p(2)); zmax = std::max(zmax, p(2)); }
        printf("[A*#%zu] seg[%zu] in=(%.1f,%.1f,%.1f) out=(%.1f,%.1f,%.1f) path=%zu pts  z=[%.2f..%.2f]\n",
               i, i, in(0), in(1), in(2), out(0), out(1), out(2), path.size(), zmin, zmax);
        a_star_pathes.push_back(path);
      }
      else
      {
        ROS_ERROR("a star error, force return!");
        return a_star_pathes;
      }
    }

    /*** calculate bounds ***/
    int id_low_bound, id_up_bound;
    vector<std::pair<int, int>> bounds(segment_ids.size());
    for (size_t i = 0; i < segment_ids.size(); i++)
    {

      if (i == 0) // first segment
      {
        id_low_bound = order_;
        if (segment_ids.size() > 1)
        {
          id_up_bound = (int)(((segment_ids[0].second + segment_ids[1].first) - 1.0f) / 2); // id_up_bound : -1.0f fix()
        }
        else
        {
          id_up_bound = init_points.cols() - order_ - 1;
        }
      }
      else if (i == segment_ids.size() - 1) // last segment, i != 0 here
      {
        id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
        id_up_bound = init_points.cols() - order_ - 1;
      }
      else
      {
        id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
        id_up_bound = (int)(((segment_ids[i].second + segment_ids[i + 1].first) - 1.0f) / 2);  // id_up_bound : -1.0f fix()
      }

      bounds[i] = std::pair<int, int>(id_low_bound, id_up_bound);
    }

    // cout << "+++++++++" << endl;
    // for ( int j=0; j<bounds.size(); ++j )
    // {
    //   cout << bounds[j].first << "  " << bounds[j].second << endl;
    // }

    /*** Adjust segment length ***/
    vector<std::pair<int, int>> final_segment_ids(segment_ids.size());
    constexpr double MINIMUM_PERCENT = 0.0; // Each segment is guaranteed to have sufficient points to generate sufficient thrust
    int minimum_points = round(init_points.cols() * MINIMUM_PERCENT), num_points;
    for (size_t i = 0; i < segment_ids.size(); i++)
    {
      /*** Adjust segment length ***/
      num_points = segment_ids[i].second - segment_ids[i].first + 1;
      //cout << "i = " << i << " first = " << segment_ids[i].first << " second = " << segment_ids[i].second << endl;
      if (num_points < minimum_points)
      {
        double add_points_each_side = (int)(((minimum_points - num_points) + 1.0f) / 2);

        final_segment_ids[i].first = segment_ids[i].first - add_points_each_side >= bounds[i].first ? segment_ids[i].first - add_points_each_side : bounds[i].first;

        final_segment_ids[i].second = segment_ids[i].second + add_points_each_side <= bounds[i].second ? segment_ids[i].second + add_points_each_side : bounds[i].second;
      }
      else
      {
        final_segment_ids[i].first = segment_ids[i].first;
        final_segment_ids[i].second = segment_ids[i].second;
      }

      //cout << "final:" << "i = " << i << " first = " << final_segment_ids[i].first << " second = " << final_segment_ids[i].second << endl;
    }

    /*** Assign data to each segment ***/
    for (size_t i = 0; i < segment_ids.size(); i++)
    {
      // step 1
      for (int j = final_segment_ids[i].first; j <= final_segment_ids[i].second; ++j)
        cps_.flag_temp[j] = false;

      // step 2
      int got_intersection_id = -1;
      for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
      {
        Eigen::Vector3d ctrl_pts_law(cps_.points.col(j + 1) - cps_.points.col(j - 1)), intersection_point;
        if (findIntersectionPoint(a_star_pathes[i], cps_.points.col(j), ctrl_pts_law, intersection_point))
        {
          got_intersection_id = j;
          cps_.flag_temp[j] = true;
          double length = (intersection_point - cps_.points.col(j)).norm();
          if (length > 1e-5)
          {
            for (double a = length; a >= 0.0; a -= grid_map_->getResolution())
            {
              occ = grid_map_->getInflateOccupancy((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));

              if (occ || a < grid_map_->getResolution())
              {
                if (occ)
                  a += grid_map_->getResolution();
                cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));
                cps_.direction[j].push_back((intersection_point - cps_.points.col(j)).normalized());
                break;
              }
            }
          }
        }
      }

      /* Corner case: the segment length is too short. */
      if (segment_ids[i].second - segment_ids[i].first == 1)
      {
        Eigen::Vector3d ctrl_pts_law(cps_.points.col(segment_ids[i].second) - cps_.points.col(segment_ids[i].first)), intersection_point;
        Eigen::Vector3d middle_point = (cps_.points.col(segment_ids[i].second) + cps_.points.col(segment_ids[i].first)) / 2;
        if (findIntersectionPoint(a_star_pathes[i], middle_point, ctrl_pts_law, intersection_point))
        {
          if ((intersection_point - middle_point).norm() > 0.01)
          {
            cps_.flag_temp[segment_ids[i].first] = true;
            cps_.base_point[segment_ids[i].first].push_back(cps_.points.col(segment_ids[i].first));
            cps_.direction[segment_ids[i].first].push_back((intersection_point - middle_point).normalized());

            got_intersection_id = segment_ids[i].first;
          }
        }
      }

      //step 3
      if (got_intersection_id >= 0)
      {
        for (int j = got_intersection_id + 1; j <= final_segment_ids[i].second; ++j)
          if (!cps_.flag_temp[j])
          {
            cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
            cps_.direction[j].push_back(cps_.direction[j - 1].back());
          }

        for (int j = got_intersection_id - 1; j >= final_segment_ids[i].first; --j)
          if (!cps_.flag_temp[j])
          {
            cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
            cps_.direction[j].push_back(cps_.direction[j + 1].back());
          }
      }
      else
      {
        // Just ignore, it does not matter ^_^.
        // ROS_ERROR("Failed to generate direction! segment_id=%d", i);
      }
    }

    a_star_pathes_ = a_star_pathes;
    collision_segments_.clear();
    for (size_t i = 0; i < segment_ids.size(); i++)
      if (a_star_pathes[i].size() > 1)
        collision_segments_.push_back(final_segment_ids[i]);

    return a_star_pathes;
  }

  bool BsplineOptimizer::findIntersectionPoint(const std::vector<Eigen::Vector3d> &a_star_path,
                                               const Eigen::Vector3d &point,
                                               const Eigen::Vector3d &ctrl_pts_law,
                                               Eigen::Vector3d &intersection_point)
  {
    int Astar_id = a_star_path.size() / 2, last_Astar_id;
    double val = (a_star_path[Astar_id] - point).dot(ctrl_pts_law), last_val = val;
    while (Astar_id >= 0 && Astar_id < (int)a_star_path.size())
    {
      last_Astar_id = Astar_id;

      if (val >= 0)
        --Astar_id;
      else
        ++Astar_id;

      val = (a_star_path[Astar_id] - point).dot(ctrl_pts_law);

      if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0))
      {
        intersection_point =
            a_star_path[Astar_id] +
            ((a_star_path[Astar_id] - a_star_path[last_Astar_id]) *
             (ctrl_pts_law.dot(point - a_star_path[Astar_id]) / ctrl_pts_law.dot(a_star_path[Astar_id] - a_star_path[last_Astar_id])));
        return true;
      }
    }
    return false;
  }

  std::vector<ControlPoints> BsplineOptimizer::distinctiveTrajs(void)
  {
    auto &segments = collision_segments_;
    if (segments.empty())
    {
      std::vector<ControlPoints> oneSeg;
      oneSeg.push_back(cps_);
      return oneSeg;
    }

    constexpr int MAX_TRAJS = 8;
    constexpr int VARIS = 2; // 2 variants per segment: original 0 and reversed 1
    int seg_upbound = std::min((int)segments.size(), static_cast<int>(floor(log(MAX_TRAJS) / log(VARIS))));
    const double RESOLUTION = grid_map_->getResolution();
    const double CTRL_PT_DIST = (cps_.points.col(0) - cps_.points.col(cps_.size - 1)).norm() / (cps_.size - 1);

    // Extract RichInfo for each segment: first=original direction, second=reversed direction
    std::vector<std::pair<ControlPoints, ControlPoints>> RichInfoSegs(seg_upbound);
    for (int i = 0; i < seg_upbound; i++)
    {
      cps_.segment(RichInfoSegs[i].first, segments[i].first, segments[i].second);
      RichInfoSegs[i].second = RichInfoSegs[i].first;

      if (RichInfoSegs[i].first.size > 1)
      {
        // Find start and end occupied control point indices
        int occ_start_id = -1, occ_end_id = -1;
        Eigen::Vector3d occ_start_pt, occ_end_pt;
        for (int j = 0; j < RichInfoSegs[i].first.size - 1; j++)
        {
          double step_size = RESOLUTION / (RichInfoSegs[i].first.points.col(j) - RichInfoSegs[i].first.points.col(j + 1)).norm() / 2;
          for (double a = 1; a > 0; a -= step_size)
          {
            Eigen::Vector3d pt(a * RichInfoSegs[i].first.points.col(j) + (1 - a) * RichInfoSegs[i].first.points.col(j + 1));
            if (grid_map_->getInflateOccupancy(pt)) { occ_start_id = j; occ_start_pt = pt; goto exit_loop1; }
          }
        }
      exit_loop1:
        for (int j = RichInfoSegs[i].first.size - 1; j >= 1; j--)
        {
          double step_size = RESOLUTION / (RichInfoSegs[i].first.points.col(j) - RichInfoSegs[i].first.points.col(j - 1)).norm();
          for (double a = 1; a > 0; a -= step_size)
          {
            Eigen::Vector3d pt(a * RichInfoSegs[i].first.points.col(j) + (1 - a) * RichInfoSegs[i].first.points.col(j - 1));
            if (grid_map_->getInflateOccupancy(pt)) { occ_end_id = j; occ_end_pt = pt; goto exit_loop2; }
          }
        }
      exit_loop2:

        if (occ_start_id < 0 || occ_end_id < 0) { seg_upbound = i; break; }

        // Reverse direction vectors and find new base points for the opposite side
        for (int j = occ_start_id; j <= occ_end_id; j++)
        {
          if (RichInfoSegs[i].first.base_point[j].size() != 1) continue;
          Eigen::Vector3d base_vec_reverse = -RichInfoSegs[i].first.direction[j][0];
          Eigen::Vector3d base_pt_reverse;
          if (j == occ_start_id) base_pt_reverse = occ_start_pt;
          else if (j == occ_end_id) base_pt_reverse = occ_end_pt;
          else base_pt_reverse = RichInfoSegs[i].first.points.col(j) + base_vec_reverse * (RichInfoSegs[i].first.base_point[j][0] - RichInfoSegs[i].first.points.col(j)).norm();

          // Search outward to find free space on the reversed side
          double l_upbound = 5 * CTRL_PT_DIST;
          double l = RESOLUTION;
          bool found = false;
          for (; l <= l_upbound; l += RESOLUTION)
          {
            Eigen::Vector3d base_pt_temp = base_pt_reverse + l * base_vec_reverse;
            if (!grid_map_->getInflateOccupancy(base_pt_temp))
            {
              RichInfoSegs[i].second.base_point[j][0] = base_pt_temp;
              RichInfoSegs[i].second.direction[j][0] = base_vec_reverse;
              found = true;
              break;
            }
          }
          if (!found) { seg_upbound = i; goto abandon_segment; }
        }

        // Propagate base points to surrounding control points
        for (int j = occ_start_id - 1; j >= 0; j--)
        {
          RichInfoSegs[i].second.base_point[j][0] = RichInfoSegs[i].second.base_point[occ_start_id][0];
          RichInfoSegs[i].second.direction[j][0] = RichInfoSegs[i].second.direction[occ_start_id][0];
        }
        for (int j = occ_end_id + 1; j < RichInfoSegs[i].second.size; j++)
        {
          RichInfoSegs[i].second.base_point[j][0] = RichInfoSegs[i].second.base_point[occ_end_id][0];
          RichInfoSegs[i].second.direction[j][0] = RichInfoSegs[i].second.direction[occ_end_id][0];
        }
      abandon_segment:;
      }
    }

    if (seg_upbound == 0)
    {
      std::vector<ControlPoints> oneSeg;
      oneSeg.push_back(cps_);
      return oneSeg;
    }

    // Generate all combinations (2 per segment → max 8 total)
    std::vector<ControlPoints> control_pts_buf;
    std::vector<int> selection(seg_upbound, 0);
    selection[0] = -1;
    int max_traj_nums = static_cast<int>(pow(VARIS, seg_upbound));
    for (int i = 0; i < max_traj_nums; i++)
    {
      int digit_id = 0;
      selection[digit_id]++;
      while (digit_id < seg_upbound && selection[digit_id] >= VARIS)
      {
        selection[digit_id] = 0;
        digit_id++;
        if (digit_id < seg_upbound) selection[digit_id]++;
      }

      ControlPoints cpsOneSample;
      cpsOneSample.resize(cps_.size);
      cpsOneSample.clearance = cps_.clearance;
      int cp_id = 0, seg_id = 0, cp_of_seg_id = 0;
      bool abandon = false;

      while (cp_id < cps_.size)
      {
        if (seg_id >= seg_upbound || cp_id < segments[seg_id].first || cp_id > segments[seg_id].second)
        {
          cpsOneSample.points.col(cp_id) = cps_.points.col(cp_id);
          cpsOneSample.base_point[cp_id] = cps_.base_point[cp_id];
          cpsOneSample.direction[cp_id] = cps_.direction[cp_id];
        }
        else if (cp_id >= segments[seg_id].first && cp_id <= segments[seg_id].second)
        {
          auto &src = (selection[seg_id] == 0) ? RichInfoSegs[seg_id].first : RichInfoSegs[seg_id].second;
          if (src.size == 0) { abandon = true; break; }
          cpsOneSample.points.col(cp_id) = src.points.col(cp_of_seg_id);
          cpsOneSample.base_point[cp_id] = src.base_point[cp_of_seg_id];
          cpsOneSample.direction[cp_id] = src.direction[cp_of_seg_id];
          cp_of_seg_id++;
          if (cp_id == segments[seg_id].second) { cp_of_seg_id = 0; seg_id++; }
        }
        cp_id++;
      }

      if (!abandon) control_pts_buf.push_back(cpsOneSample);
    }

    if (control_pts_buf.empty())
    {
      control_pts_buf.push_back(cps_);
    }

    return control_pts_buf;
  }

  int BsplineOptimizer::earlyExit(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls)
  {
    BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);
    // cout << "k=" << k << endl;
    // cout << "opt->flag_continue_to_optimize_=" << opt->flag_continue_to_optimize_ << endl;
    return (opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
  }

  double BsplineOptimizer::costFunctionRebound(void *func_data, const double *x, double *grad, const int n)
  {
    BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);

    double cost;
    opt->combineCostRebound(x, grad, cost, n);

    opt->iter_num_ += 1;
    return cost;
  }

  double BsplineOptimizer::costFunctionRefine(void *func_data, const double *x, double *grad, const int n)
  {
    BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);

    double cost;
    opt->combineCostRefine(x, grad, cost, n);

    opt->iter_num_ += 1;
    return cost;
  }

  void BsplineOptimizer::calcDistanceCostRebound(const Eigen::MatrixXd &q, double &cost,
                                                 Eigen::MatrixXd &gradient, int iter_num, double smoothness_cost)
  {
    cost = 0.0;
    int end_idx = q.cols() - order_;
    double demarcation = cps_.clearance;
    double a = 3 * demarcation, b = -3 * pow(demarcation, 2), c = pow(demarcation, 3);

    force_stop_type_ = DONT_STOP;
    if (iter_num > 3 && smoothness_cost / (cps_.size - 2 * order_) < 0.1) // 0.1 is an experimental value that indicates the trajectory is smooth enough.
    {
      check_collision_and_rebound();
    }

    /*** calculate distance cost and gradient ***/
    for (auto i = order_; i < end_idx; ++i)
    {
      for (size_t j = 0; j < cps_.direction[i].size(); ++j)
      {
        double dist = (cps_.points.col(i) - cps_.base_point[i][j]).dot(cps_.direction[i][j]);
        double dist_err = cps_.clearance - dist;
        Eigen::Vector3d dist_grad = cps_.direction[i][j];

        if (dist_err < 0)
        {
          /* do nothing */
        }
        else if (dist_err < demarcation)
        {
          cost += pow(dist_err, 3);
          gradient.col(i) += -3.0 * dist_err * dist_err * dist_grad;
        }
        else
        {
          cost += a * dist_err * dist_err + b * dist_err + c;
          gradient.col(i) += -(2.0 * a * dist_err + b) * dist_grad;
        }
      }
    }
  }

  void BsplineOptimizer::calcFitnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
  {

    cost = 0.0;

    int end_idx = q.cols() - order_;

    // def: f = |x*v|^2/a^2 + |x×v|^2/b^2
    double a2 = 25, b2 = 1;
    for (auto i = order_ - 1; i < end_idx + 1; ++i)
    {
      Eigen::Vector3d x = (q.col(i - 1) + 4 * q.col(i) + q.col(i + 1)) / 6.0 - ref_pts_[i - 1];
      Eigen::Vector3d v = (ref_pts_[i] - ref_pts_[i - 2]).normalized();

      double xdotv = x.dot(v);
      Eigen::Vector3d xcrossv = x.cross(v);

      double f = pow((xdotv), 2) / a2 + pow(xcrossv.norm(), 2) / b2;
      cost += f;

      Eigen::Matrix3d m;
      m << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
      Eigen::Vector3d df_dx = 2 * xdotv / a2 * v + 2 / b2 * m * xcrossv;

      gradient.col(i - 1) += df_dx / 6;
      gradient.col(i) += 4 * df_dx / 6;
      gradient.col(i + 1) += df_dx / 6;
    }
  }

  void BsplineOptimizer::calcHeightCost(const Eigen::MatrixXd &q, double &cost,
                                        Eigen::MatrixXd &gradient)
  {
    /* Terrestrial-aerial height constraint.
       - Hard bounds: keep the trajectory below max_height (ceiling) and above
         ground_height (floor).
       - Ground attraction: where the ground beneath a control point is
         traversable (free), pull the control point down onto the ground so the
         vehicle drives instead of flies. Above obstacles (ground occupied) no
         attraction is applied, so the vehicle is free to climb and fly over. */
    cost = 0.0;

    int end_idx = q.cols() - order_;
    for (int i = order_; i < end_idx; ++i)
    {
      double z = q(2, i);

      /* ceiling */
      if (z > max_height_)
      {
        double diff = z - max_height_;
        cost += lambda_height_ * diff * diff;
        gradient(2, i) += lambda_height_ * 2.0 * diff;
      }
      /* floor */
      if (z < ground_height_)
      {
        double diff = z - ground_height_;
        cost += lambda_height_ * diff * diff;
        gradient(2, i) += lambda_height_ * 2.0 * diff;
      }
      /* ground attraction over traversable ground */
      else if (lambda_ground_ > 0.0)
      {
        Eigen::Vector3d ground_probe(q(0, i), q(1, i), ground_height_);
        if (grid_map_->getInflateOccupancy(ground_probe) == 0) // ground below is free
        {
          /* Only pull down if there is no obstacle nearby. Near a wall/obstacle
             the vehicle must be free to climb over it, so ground-attraction is
             disabled within ground_clear_radius_ horizontally. The neighbourhood
             is scanned over several radii and heights so that even a thin, tall
             wall is reliably detected. */
          bool near_obstacle = false;
          double res = grid_map_->getResolution();
          for (double r = res; r <= ground_clear_radius_ + 1e-3 && !near_obstacle; r += res)
          {
            for (double ang = 0.0; ang < 6.28; ang += M_PI / 6)
            {
              double px = q(0, i) + r * cos(ang);
              double py = q(1, i) + r * sin(ang);
              for (double pz = ground_height_ + res; pz <= max_height_; pz += 3 * res)
              {
                if (grid_map_->getInflateOccupancy(Eigen::Vector3d(px, py, pz)) == 1)
                {
                  near_obstacle = true;
                  break;
                }
              }
              if (near_obstacle)
                break;
            }
          }

          if (!near_obstacle)
          {
            double diff = z - ground_height_;
            cost += lambda_ground_ * diff * diff;
            gradient(2, i) += lambda_ground_ * 2.0 * diff;
          }
        }
      }
    }
  }

  void BsplineOptimizer::calcGroundNonHolonomicCost(const Eigen::MatrixXd &q, double &cost,
                                                    Eigen::MatrixXd &gradient)
  {
    /* Ground non-holonomic (lateral) regularizer.
       For control points in ground mode, penalize the horizontal lateral term
       cross = vx*ay - vy*ax (proportional to |v| * lateral_acc). This discourages
       sideways / sharp swerving on the ground, approximating car-like motion.
       Only horizontal (x, y) components are involved. */
    cost = 0.0;

    if (lambda_nonholo_ <= 0.0)
      return;

    int end_idx = q.cols() - order_;
    for (int i = order_; i < end_idx - 1; ++i)
    {
      if (q(2, i + 1) >= ground_judge_) // only apply on ground segments
        continue;

      double vx = q(0, i + 1) - q(0, i);
      double vy = q(1, i + 1) - q(1, i);
      double ax = q(0, i + 2) - 2 * q(0, i + 1) + q(0, i);
      double ay = q(1, i + 2) - 2 * q(1, i + 1) + q(1, i);

      double cross = vx * ay - vy * ax;
      cost += lambda_nonholo_ * cross * cross;

      double g = lambda_nonholo_ * 2.0 * cross;

      gradient(0, i) += g * (-ay - vy);
      gradient(1, i) += g * (vx + ax);
      gradient(0, i + 1) += g * (ay + 2 * vy);
      gradient(1, i + 1) += g * (-2 * vx - ax);
      gradient(0, i + 2) += g * (-vy);
      gradient(1, i + 2) += g * (vx);
    }
  }

  void BsplineOptimizer::calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                                            Eigen::MatrixXd &gradient, bool flag_use_jerk /* = true*/)
  {
    cost = 0.0;

    if (flag_use_jerk)
    {
      Eigen::Vector3d jerk, temp_j;

      for (int i = 0; i < q.cols() - 3; i++)
      {
        /* evaluate jerk */
        jerk = q.col(i + 3) - 3 * q.col(i + 2) + 3 * q.col(i + 1) - q.col(i);
        cost += jerk.squaredNorm();
        temp_j = 2.0 * jerk;
        /* jerk gradient */
        gradient.col(i + 0) += -temp_j;
        gradient.col(i + 1) += 3.0 * temp_j;
        gradient.col(i + 2) += -3.0 * temp_j;
        gradient.col(i + 3) += temp_j;
      }
    }
    else
    {
      Eigen::Vector3d acc, temp_acc;

      for (int i = 0; i < q.cols() - 2; i++)
      {
        /* evaluate acc */
        acc = q.col(i + 2) - 2 * q.col(i + 1) + q.col(i);
        cost += acc.squaredNorm();
        temp_acc = 2.0 * acc;
        /* acc gradient */
        gradient.col(i + 0) += temp_acc;
        gradient.col(i + 1) += -2.0 * temp_acc;
        gradient.col(i + 2) += temp_acc;
      }
    }
  }

  void BsplineOptimizer::calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                                              Eigen::MatrixXd &gradient)
  {

    cost = 0.0;
    double ts, ts_inv2;
    ts = bspline_interval_;
    ts_inv2 = 1 / ts / ts;

    for (int i = 0; i < q.cols() - 1; i++)
    {
      Eigen::Vector3d vi = (q.col(i + 1) - q.col(i)) / ts;

      for (int j = 0; j < 3; j++)
      {
        if (vi(j) > max_vel_)
        {
          cost += pow(vi(j) - max_vel_, 2) * ts_inv2;

          gradient(j, i + 0) += -2 * (vi(j) - max_vel_) / ts * ts_inv2;
          gradient(j, i + 1) += 2 * (vi(j) - max_vel_) / ts * ts_inv2;
        }
        else if (vi(j) < -max_vel_)
        {
          cost += pow(vi(j) + max_vel_, 2) * ts_inv2;

          gradient(j, i + 0) += -2 * (vi(j) + max_vel_) / ts * ts_inv2;
          gradient(j, i + 1) += 2 * (vi(j) + max_vel_) / ts * ts_inv2;
        }
      }
    }

    for (int i = 0; i < q.cols() - 2; i++)
    {
      Eigen::Vector3d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;

      for (int j = 0; j < 3; j++)
      {
        if (ai(j) > max_acc_)
        {
          cost += pow(ai(j) - max_acc_, 2);

          gradient(j, i + 0) += 2 * (ai(j) - max_acc_) * ts_inv2;
          gradient(j, i + 1) += -4 * (ai(j) - max_acc_) * ts_inv2;
          gradient(j, i + 2) += 2 * (ai(j) - max_acc_) * ts_inv2;
        }
        else if (ai(j) < -max_acc_)
        {
          cost += pow(ai(j) + max_acc_, 2);

          gradient(j, i + 0) += 2 * (ai(j) + max_acc_) * ts_inv2;
          gradient(j, i + 1) += -4 * (ai(j) + max_acc_) * ts_inv2;
          gradient(j, i + 2) += 2 * (ai(j) + max_acc_) * ts_inv2;
        }
      }
    }
  }

  bool BsplineOptimizer::check_collision_and_rebound(void)
  {

    int end_idx = cps_.size - order_;

    /*** Check and segment the initial trajectory according to obstacles ***/
    int in_id, out_id;
    vector<std::pair<int, int>> segment_ids;
    bool flag_new_obs_valid = false;
    int i_end = end_idx - (end_idx - order_) / 3;
    for (int i = order_ - 1; i <= i_end; ++i)
    {

      bool occ = grid_map_->getInflateOccupancy(cps_.points.col(i));

      /*** check if the new collision will be valid ***/
      if (occ)
      {
        for (size_t k = 0; k < cps_.direction[i].size(); ++k)
        {
          cout.precision(2);
          if ((cps_.points.col(i) - cps_.base_point[i][k]).dot(cps_.direction[i][k]) < 1 * grid_map_->getResolution()) // current point is outside all the collision_points.
          {
            occ = false; // Not really takes effect, just for better hunman understanding.
            break;
          }
        }
      }

      if (occ)
      {
        flag_new_obs_valid = true;

        int j;
        for (j = i - 1; j >= 0; --j)
        {
          occ = grid_map_->getInflateOccupancy(cps_.points.col(j));
          if (!occ)
          {
            in_id = j;
            break;
          }
        }
        if (j < 0) // fail to get the obs free point
        {
          ROS_ERROR("ERROR! the drone is in obstacle. This should not happen.");
          in_id = 0;
        }

        for (j = i + 1; j < cps_.size; ++j)
        {
          occ = grid_map_->getInflateOccupancy(cps_.points.col(j));

          if (!occ)
          {
            out_id = j;
            break;
          }
        }
        if (j >= cps_.size) // fail to get the obs free point
        {
          ROS_WARN("WARN! terminal point of the current trajectory is in obstacle, skip this planning.");

          force_stop_type_ = STOP_FOR_ERROR;
          return false;
        }

        i = j + 1;

        segment_ids.push_back(std::pair<int, int>(in_id, out_id));
      }
    }

    if (flag_new_obs_valid)
    {
      vector<vector<Eigen::Vector3d>> a_star_pathes;
      for (size_t i = 0; i < segment_ids.size(); ++i)
      {
        /*** a star search ***/
        Eigen::Vector3d in(cps_.points.col(segment_ids[i].first)), out(cps_.points.col(segment_ids[i].second));
        if (a_star_->AstarSearch(/*(in-out).norm()/10+0.05*/ 0.2, in, out))
        {
          auto path = a_star_->getPath();
          double zmin = 999, zmax = -999;
          for (auto& p : path) { zmin = std::min(zmin, p(2)); zmax = std::max(zmax, p(2)); }
          printf("[A*-rebound#%zu] in=(%.1f,%.1f,%.1f) out=(%.1f,%.1f,%.1f) path=%zu pts  z=[%.2f..%.2f]\n",
                 i, in(0), in(1), in(2), out(0), out(1), out(2), path.size(), zmin, zmax);
          a_star_pathes.push_back(path);
        }
        else
        {
          ROS_ERROR("a star error");
          segment_ids.erase(segment_ids.begin() + i);
          i--;
        }
      }

      /*** Assign parameters to each segment ***/
      for (size_t i = 0; i < segment_ids.size(); ++i)
      {
        // step 1
        for (int j = segment_ids[i].first; j <= segment_ids[i].second; ++j)
          cps_.flag_temp[j] = false;

        // step 2
        int got_intersection_id = -1;
        for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
        {
          Eigen::Vector3d ctrl_pts_law(cps_.points.col(j + 1) - cps_.points.col(j - 1)), intersection_point;
          if (findIntersectionPoint(a_star_pathes[i], cps_.points.col(j), ctrl_pts_law, intersection_point))
          {
            got_intersection_id = j;
            cps_.flag_temp[j] = true;
            double length = (intersection_point - cps_.points.col(j)).norm();
            if (length > 1e-5)
            {
              for (double a = length; a >= 0.0; a -= grid_map_->getResolution())
              {
                bool occ = grid_map_->getInflateOccupancy((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));

                if (occ || a < grid_map_->getResolution())
                {
                  if (occ)
                    a += grid_map_->getResolution();
                  cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));
                  cps_.direction[j].push_back((intersection_point - cps_.points.col(j)).normalized());
                  break;
                }
              }
            }
            else
            {
              got_intersection_id = -1;
            }
          }
        }

        //step 3
        if (got_intersection_id >= 0)
        {
          for (int j = got_intersection_id + 1; j <= segment_ids[i].second; ++j)
            if (!cps_.flag_temp[j])
            {
              cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
              cps_.direction[j].push_back(cps_.direction[j - 1].back());
            }

          for (int j = got_intersection_id - 1; j >= segment_ids[i].first; --j)
            if (!cps_.flag_temp[j])
            {
              cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
              cps_.direction[j].push_back(cps_.direction[j + 1].back());
            }
        }
        else
          ROS_WARN("Failed to generate direction. It doesn't matter.");
      }

      force_stop_type_ = STOP_FOR_REBOUND;
      return true;
    }

    return false;
  }

  bool BsplineOptimizer::BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double ts)
  {
    setBsplineInterval(ts);

    bool flag_success = rebound_optimize();

    optimal_points = cps_.points;

    return flag_success;
  }

  bool BsplineOptimizer::BsplineOptimizeTrajRefine(const Eigen::MatrixXd &init_points, const double ts, Eigen::MatrixXd &optimal_points)
  {

    setControlPoints(init_points);
    setBsplineInterval(ts);

    bool flag_success = refine_optimize();

    optimal_points = cps_.points;

    return flag_success;
  }

  bool BsplineOptimizer::rebound_optimize()
  {
    iter_num_ = 0;
    int start_id = order_;
    int end_id = this->cps_.size - order_;
    variable_num_ = 3 * (end_id - start_id);
    double final_cost;

    ros::Time t0 = ros::Time::now(), t1, t2;
    int restart_nums = 0, rebound_times = 0;
    ;
    bool flag_force_return, flag_occ, success;
    new_lambda2_ = lambda2_;
    constexpr int MAX_RESART_NUMS_SET = 3;
    do
    {
      /* ---------- prepare ---------- */
      min_cost_ = std::numeric_limits<double>::max();
      iter_num_ = 0;
      flag_force_return = false;
      flag_occ = false;
      success = false;

      double q[variable_num_];
      memcpy(q, cps_.points.data() + 3 * start_id, variable_num_ * sizeof(q[0]));

      lbfgs::lbfgs_parameter_t lbfgs_params;
      lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
      lbfgs_params.mem_size = 32;
      lbfgs_params.max_iterations = 200;
      lbfgs_params.g_epsilon = 0.1;

      /* ---------- optimize ---------- */
      t1 = ros::Time::now();
      int result = lbfgs::lbfgs_optimize(variable_num_, q, &final_cost, BsplineOptimizer::costFunctionRebound, NULL, BsplineOptimizer::earlyExit, this, &lbfgs_params);
      t2 = ros::Time::now();
      double time_ms = (t2 - t1).toSec() * 1000;
      double total_time_ms = (t2 - t0).toSec() * 1000;

      /* ---------- success temporary, check collision again ---------- */
      if (result == lbfgs::LBFGS_CONVERGENCE ||
          result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
          result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
          result == lbfgs::LBFGS_STOP)
      {
        //ROS_WARN("Solver error in planning!, return = %s", lbfgs::lbfgs_strerror(result));
        flag_force_return = false;

        UniformBspline traj = UniformBspline(cps_.points, 3, bspline_interval_);
        double tm, tmp;
        traj.getTimeSpan(tm, tmp);
        double t_step = (tmp - tm) / ((traj.evaluateDeBoorT(tmp) - traj.evaluateDeBoorT(tm)).norm() / grid_map_->getResolution());
        for (double t = tm; t < tmp * 2 / 3; t += t_step) // Only check the closest 2/3 partition of the whole trajectory.
        {
          flag_occ = grid_map_->getInflateOccupancy(traj.evaluateDeBoorT(t));
          if (flag_occ)
          {
            //cout << "hit_obs, t=" << t << " P=" << traj.evaluateDeBoorT(t).transpose() << endl;

            if (t <= bspline_interval_) // First 3 control points in obstacles!
            {
              cout << cps_.points.col(1).transpose() << "\n"
                   << cps_.points.col(2).transpose() << "\n"
                   << cps_.points.col(3).transpose() << "\n"
                   << cps_.points.col(4).transpose() << endl;
              ROS_WARN("First 3 control points in obstacles! return false, t=%f", t);
              return false;
            }

            break;
          }
        }

        if (!flag_occ)
        {
          printf("\033[32miter(+1)=%d,time(ms)=%5.3f,total_t(ms)=%5.3f,cost=%5.3f\n\033[0m", iter_num_, time_ms, total_time_ms, final_cost);
          success = true;
        }
        else // restart
        {
          restart_nums++;
          initControlPoints(cps_.points, false);
          new_lambda2_ *= 2;

          printf("\033[32miter(+1)=%d,time(ms)=%5.3f,keep optimizing\n\033[0m", iter_num_, time_ms);
        }
      }
      else if (result == lbfgs::LBFGSERR_CANCELED)
      {
        flag_force_return = true;
        rebound_times++;
        cout << "iter=" << iter_num_ << ",time(ms)=" << time_ms << ",rebound." << endl;
      }
      else
      {
        ROS_WARN("Solver error. Return = %d, %s. Skip this planning.", result, lbfgs::lbfgs_strerror(result));
        // while (ros::ok());
      }

    } while ((flag_occ && restart_nums < MAX_RESART_NUMS_SET) ||
             (flag_force_return && force_stop_type_ == STOP_FOR_REBOUND && rebound_times <= 20));

    return success;
  }

  bool BsplineOptimizer::refine_optimize()
  {
    iter_num_ = 0;
    int start_id = order_;
    int end_id = this->cps_.points.cols() - order_;
    variable_num_ = 3 * (end_id - start_id);

    double q[variable_num_];
    double final_cost;

    memcpy(q, cps_.points.data() + 3 * start_id, variable_num_ * sizeof(q[0]));

    double origin_lambda4 = lambda4_;
    bool flag_safe = true;
    int iter_count = 0;
    do
    {
      lbfgs::lbfgs_parameter_t lbfgs_params;
      lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
      lbfgs_params.mem_size = 32;
      lbfgs_params.max_iterations = 200;
      lbfgs_params.g_epsilon = 0.01;

      int result = lbfgs::lbfgs_optimize(variable_num_, q, &final_cost, BsplineOptimizer::costFunctionRefine, NULL, NULL, this, &lbfgs_params);
      if (result == lbfgs::LBFGS_CONVERGENCE ||
          result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
          result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
          result == lbfgs::LBFGS_STOP)
      {
        //pass
      }
      else
      {
        ROS_ERROR("Solver error in refining!, return = %d, %s", result, lbfgs::lbfgs_strerror(result));
      }

      UniformBspline traj = UniformBspline(cps_.points, 3, bspline_interval_);
      double tm, tmp;
      traj.getTimeSpan(tm, tmp);
      double t_step = (tmp - tm) / ((traj.evaluateDeBoorT(tmp) - traj.evaluateDeBoorT(tm)).norm() / grid_map_->getResolution()); // Step size is defined as the maximum size that can passes throgth every gird.
      for (double t = tm; t < tmp * 2 / 3; t += t_step)
      {
        if (grid_map_->getInflateOccupancy(traj.evaluateDeBoorT(t)))
        {
          // cout << "Refined traj hit_obs, t=" << t << " P=" << traj.evaluateDeBoorT(t).transpose() << endl;

          Eigen::MatrixXd ref_pts(ref_pts_.size(), 3);
          for (size_t i = 0; i < ref_pts_.size(); i++)
          {
            ref_pts.row(i) = ref_pts_[i].transpose();
          }

          flag_safe = false;
          break;
        }
      }

      if (!flag_safe)
        lambda4_ *= 2;

      iter_count++;
    } while (!flag_safe && iter_count <= 1);

    lambda4_ = origin_lambda4;

    //cout << "iter_num_=" << iter_num_ << endl;

    return flag_safe;
  }

  void BsplineOptimizer::combineCostRebound(const double *x, double *grad, double &f_combine, const int n)
  {

    memcpy(cps_.points.data() + 3 * order_, x, n * sizeof(x[0]));

    /* ---------- evaluate cost and gradient ---------- */
    double f_smoothness, f_distance, f_feasibility;

    Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_distance = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(3, cps_.size);

    calcSmoothnessCost(cps_.points, f_smoothness, g_smoothness);
    calcDistanceCostRebound(cps_.points, f_distance, g_distance, iter_num_, f_smoothness);
    calcFeasibilityCost(cps_.points, f_feasibility, g_feasibility);

    double f_height = 0.0, f_nonholo = 0.0;
    Eigen::MatrixXd g_height = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_nonholo = Eigen::MatrixXd::Zero(3, cps_.size);
    calcHeightCost(cps_.points, f_height, g_height);
    calcGroundNonHolonomicCost(cps_.points, f_nonholo, g_nonholo);

    f_combine = lambda1_ * f_smoothness + new_lambda2_ * f_distance + lambda3_ * f_feasibility + f_height + f_nonholo;
    //printf("origin %f %f %f %f\n", f_smoothness, f_distance, f_feasibility, f_combine);

    Eigen::MatrixXd grad_3D = lambda1_ * g_smoothness + new_lambda2_ * g_distance + lambda3_ * g_feasibility + g_height + g_nonholo;
    memcpy(grad, grad_3D.data() + 3 * order_, n * sizeof(grad[0]));
  }

  void BsplineOptimizer::combineCostRefine(const double *x, double *grad, double &f_combine, const int n)
  {

    memcpy(cps_.points.data() + 3 * order_, x, n * sizeof(x[0]));

    /* ---------- evaluate cost and gradient ---------- */
    double f_smoothness, f_fitness, f_feasibility;

    Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(3, cps_.points.cols());
    Eigen::MatrixXd g_fitness = Eigen::MatrixXd::Zero(3, cps_.points.cols());
    Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(3, cps_.points.cols());

    //time_satrt = ros::Time::now();

    calcSmoothnessCost(cps_.points, f_smoothness, g_smoothness);
    calcFitnessCost(cps_.points, f_fitness, g_fitness);
    calcFeasibilityCost(cps_.points, f_feasibility, g_feasibility);

    double f_height = 0.0, f_nonholo = 0.0;
    Eigen::MatrixXd g_height = Eigen::MatrixXd::Zero(3, cps_.points.cols());
    Eigen::MatrixXd g_nonholo = Eigen::MatrixXd::Zero(3, cps_.points.cols());
    calcHeightCost(cps_.points, f_height, g_height);
    calcGroundNonHolonomicCost(cps_.points, f_nonholo, g_nonholo);

    /* ---------- convert to solver format...---------- */
    f_combine = lambda1_ * f_smoothness + lambda4_ * f_fitness + lambda3_ * f_feasibility + f_height + f_nonholo;
    // printf("origin %f %f %f %f\n", f_smoothness, f_fitness, f_feasibility, f_combine);

    Eigen::MatrixXd grad_3D = lambda1_ * g_smoothness + lambda4_ * g_fitness + lambda3_ * g_feasibility + g_height + g_nonholo;
    memcpy(grad, grad_3D.data() + 3 * order_, n * sizeof(grad[0]));
  }

} // namespace plan_manage