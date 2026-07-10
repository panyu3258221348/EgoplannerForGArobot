#include "cpu_depth_render.h"
#include <algorithm>

void CpuDepthRender::set_para(double fx, double fy, double cx, double cy,
                               int width, int height) {
  fx_ = fx;
  fy_ = fy;
  cx_ = cx;
  cy_ = cy;
  width_ = width;
  height_ = height;
}

void CpuDepthRender::set_data(const std::vector<float> &cloud) {
  cloud_data_ = cloud;
}

void CpuDepthRender::render(const Eigen::Matrix4d &T_wc, int *depth_hostptr) {
  const size_t N = cloud_data_.size() / 3;
  const int total_pixels = width_ * height_;

  for (int i = 0; i < total_pixels; i++)
    depth_hostptr[i] = 0;

  for (size_t i = 0; i < N; i++) {
    Eigen::Vector4d Pw(cloud_data_[3*i], cloud_data_[3*i+1], cloud_data_[3*i+2], 1.0);
    Eigen::Vector4d Pc = T_wc * Pw;

    double zc = Pc(2);
    if (zc <= 0.1) continue;

    double inv_z = 1.0 / zc;
    int u = int(Pc(0) * inv_z * fx_ + cx_ + 0.5);
    int v = int(Pc(1) * inv_z * fy_ + cy_ + 0.5);

    if (u < 0 || u >= width_ || v < 0 || v >= height_) continue;

    int depth_mm = int(zc * 1000.0);
    int idx = v * width_ + u;

    if (depth_hostptr[idx] == 0 || depth_mm < depth_hostptr[idx])
      depth_hostptr[idx] = depth_mm;
  }
}
