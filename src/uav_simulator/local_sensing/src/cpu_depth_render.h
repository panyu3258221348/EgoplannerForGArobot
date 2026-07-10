#ifndef _CPU_DEPTH_RENDER_H
#define _CPU_DEPTH_RENDER_H

#include <vector>
#include <Eigen/Dense>

class CpuDepthRender {
public:
  CpuDepthRender() {}
  ~CpuDepthRender() {}

  void set_para(double fx, double fy, double cx, double cy, int width, int height);
  void set_data(const std::vector<float> &cloud);
  void render(const Eigen::Matrix4d &T_wc, int *depth_hostptr);

private:
  double fx_, fy_, cx_, cy_;
  int width_, height_;
  std::vector<float> cloud_data_;
};

#endif
