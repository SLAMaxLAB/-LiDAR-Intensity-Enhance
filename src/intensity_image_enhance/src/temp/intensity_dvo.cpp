/******************************************************
 *  LiDAR Intensity + 3D Points DVO (Direct VO)
 *  Robust, Multithreaded, Pyramid, Photometric+Plane
 *  Author: (your name)
 *  License: MIT
 *
 *  依赖：
 *    - ROS (roscpp, sensor_msgs)
 *    - OpenCV
 *    - Ceres Solver (>=1.14 推荐)
 *    - OpenMP（可选；未启用也可编译运行）
 *
 *  主要特性：
 *    - 激光点云 → 强度图 + 每像素 3D 点
 *    - 法向估计（邻域差分），用于点到平面约束
 *    - 金字塔（粗→细），梯度驱动采样
 *    - 光度仿射模型 a,b（缓解亮度尺度/偏置漂移）
 *    - 多线程：残差构建并行 + Ceres 内部并行
 *    - 详细日志：每层残差数、初末 cost、耗时、(a,b)
 *    - 可视化输出：强度图、梯度图、帧日志
 ******************************************************/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <opencv2/opencv.hpp>

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/version.h>

#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>

#include <fstream>
#include <chrono>
#include <vector>
#include <string>
#include <cmath>
#include <thread>
#include <atomic>
#include <limits>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ======================= 工具函数 =======================
inline bool isFinite3(double x, double y, double z) {
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

template<typename T>
inline double toDouble(const T& x) { return static_cast<double>(x); }
template<int N>
inline double toDouble(const ceres::Jet<double,N>& x) { return x.a; }

static inline void ensureDir(const std::string& d) {
  if (mkdir(d.c_str(), 0777) && errno != EEXIST) {
    ROS_WARN("目录创建失败或已存在: %s", d.c_str());
  }
}

// ======================= 数据帧结构 =======================
struct Frame {
  int H{0}, W{0};
  double min_az{0}, max_az{0}, min_el{0}, max_el{0};

  // 强度（8U用于可视化，64F用于优化）
  cv::Mat intensity_8u;     // HxW, CV_8U, 0..255
  cv::Mat intensity_f64;    // HxW, CV_64F, 0..255

  // 3D 点与法向，行主序向量（与图像同分辨率）
  std::vector<double> X, Y, Z;        // size=H*W
  std::vector<double> Nx, Ny, Nz;     // size=H*W
  std::vector<uint8_t> mask;          // 1有效 0无效

  // 梯度（优化用）
  cv::Mat grad_mag;         // CV_32F

  bool valid() const { return (H>0 && W>0 && (int)X.size()==H*W); }
};

// ======================= 金字塔结构 =======================
struct Pyramid {
  std::vector<Frame> levels;  // 0: finest
};

// ======================= 残差：光度(1) + 点到平面(1) =======================
// 参数块：pose[6] (angle-axis + txyz), ab[2] (亮度仿射 a,b)
struct PhotoPlaneResidual {
  // 上一帧的像素与 3D
  double px, py, pz;  // prev 3D
  double I_prev;      // prev intensity (double)

  // 当前帧数据指针（均为连续内存）
  const double* cur_I; // double 强度图 HxW
  const double* cur_X; // 当前 3D
  const double* cur_Y;
  const double* cur_Z;
  const double* cur_Nx; // 当前法向
  const double* cur_Ny;
  const double* cur_Nz;
  int W, H;

  // 投影角度范围
  double min_az, max_az, min_el, max_el;

  // 权重
  double w_photo, w_geo;

  PhotoPlaneResidual(double _px,double _py,double _pz,double _Iprev,
                     const double* _I,const double* _X,const double* _Y,const double* _Z,
                     const double* _Nx,const double* _Ny,const double* _Nz,
                     int _W,int _H,
                     double _min_az,double _max_az,double _min_el,double _max_el,
                     double _w_photo,double _w_geo)
  : px(_px),py(_py),pz(_pz),I_prev(_Iprev),
    cur_I(_I),cur_X(_X),cur_Y(_Y),cur_Z(_Z),
    cur_Nx(_Nx),cur_Ny(_Ny),cur_Nz(_Nz),
    W(_W),H(_H),
    min_az(_min_az),max_az(_max_az),min_el(_min_el),max_el(_max_el),
    w_photo(_w_photo),w_geo(_w_geo) {}

  template<typename T>
  bool operator()(const T* const pose, const T* const ab, T* residual) const {
    // 1) 变换上一帧点
    T p[3] = {T(px), T(py), T(pz)};
    T p_t[3];
    ceres::AngleAxisRotatePoint(pose, p, p_t);
    p_t[0] += pose[3]; p_t[1] += pose[4]; p_t[2] += pose[5];

    // 原点附近用数值域判断（避免 Jet 在 0 处的奇异导数）
    if (std::abs(toDouble(p_t[0])) < 1e-12 && std::abs(toDouble(p_t[1])) < 1e-12) {
      residual[0] = T(0);
      residual[1] = T(0);
      return true;
    }

    // 2) 球面投影 -> 像素 (u,v)
    T X = p_t[0], Y = p_t[1], Z = p_t[2];
    T rho = ceres::sqrt(X*X + Y*Y);
    T az  = ceres::atan2(Y, X);
    T el  = ceres::atan2(Z, rho);

    T uf = (az - T(min_az)) / T(max_az - min_az) * T(W - 1);
    T vf = (T(max_el) - el)  / T(max_el - min_el) * T(H - 1);

    // 数值域检查越界
    const double ufd = toDouble(uf), vfd = toDouble(vf);
    if (!std::isfinite(ufd) || !std::isfinite(vfd)) {
      residual[0]=T(0); residual[1]=T(0); return true;
    }
    const int u0 = (int)std::floor(ufd);
    const int v0 = (int)std::floor(vfd);
    if (u0 < 0 || v0 < 0 || u0+1 >= W || v0+1 >= H) {
      residual[0]=T(0); residual[1]=T(0); return true;
    }

    // 3) 双线性插值
    const int idx00 = v0*W + u0;
    const int idx01 = v0*W + (u0+1);
    const int idx10 = (v0+1)*W + u0;
    const int idx11 = (v0+1)*W + (u0+1);

    const T du = uf - T(u0);
    const T dv = vf - T(v0);
    const T w00 = (T(1)-du)*(T(1)-dv);
    const T w01 = du*(T(1)-dv);
    const T w10 = (T(1)-du)*dv;
    const T w11 = du*dv;

    auto sfetch = [](const double* arr, int idx)->T {
      const double v = arr[idx];
      return std::isfinite(v) ? T(v) : T(0);
    };

    // 强度
    const T Icur = w00*sfetch(cur_I, idx00) + w01*sfetch(cur_I, idx01)
                 + w10*sfetch(cur_I, idx10) + w11*sfetch(cur_I, idx11);
    // 当前帧 3D
    const T Xs = w00*sfetch(cur_X, idx00) + w01*sfetch(cur_X, idx01)
               + w10*sfetch(cur_X, idx10) + w11*sfetch(cur_X, idx11);
    const T Ys = w00*sfetch(cur_Y, idx00) + w01*sfetch(cur_Y, idx01)
               + w10*sfetch(cur_Y, idx10) + w11*sfetch(cur_Y, idx11);
    const T Zs = w00*sfetch(cur_Z, idx00) + w01*sfetch(cur_Z, idx01)
               + w10*sfetch(cur_Z, idx10) + w11*sfetch(cur_Z, idx11);

    // 法向（关键修复：根号“内”加 eps，且对极小范数禁用几何项）
    T nx = w00*sfetch(cur_Nx, idx00) + w01*sfetch(cur_Nx, idx01)
         + w10*sfetch(cur_Nx, idx10) + w11*sfetch(cur_Nx, idx11);
    T ny = w00*sfetch(cur_Ny, idx00) + w01*sfetch(cur_Ny, idx01)
         + w10*sfetch(cur_Ny, idx10) + w11*sfetch(cur_Ny, idx11);
    T nz = w00*sfetch(cur_Nz, idx00) + w01*sfetch(cur_Nz, idx01)
         + w10*sfetch(cur_Nz, idx10) + w11*sfetch(cur_Nz, idx11);

    const double norm2d =
      toDouble(nx)*toDouble(nx) + toDouble(ny)*toDouble(ny) + toDouble(nz)*toDouble(nz);

    // 光度残差（总是可写）
    const T a = ab[0], b = ab[1];
    residual[0] = T(w_photo) * (a * Icur + b - T(I_prev));

    if (norm2d < 1e-12) {
      // 几何法向不可用 → 几何残差置零（雅可比也安全地为 0）
      residual[1] = T(0);
      return true;
    }

    const T n_norm = ceres::sqrt(nx*nx + ny*ny + nz*nz + T(1e-12));
    nx /= n_norm; ny /= n_norm; nz /= n_norm;

    // 点到平面几何残差
    residual[1] = T(w_geo) * ( nx*(X - Xs) + ny*(Y - Ys) + nz*(Z - Zs) );

    // 兜底
    for (int i=0;i<2;++i) if (!ceres::IsFinite(residual[i])) residual[i]=T(0);
    return true;
  }
};

// ======================= 主类 =======================
class LidarIntensityDVO {
public:
  LidarIntensityDVO(ros::NodeHandle& nh) : nh_(nh) {
    // ---- 参数读取 ----
    nh_.param<std::string>("output_dir", output_dir_, "/home/lb/Piont_cloudToImage/lidar_dvo");
    nh_.param<int>("v_res", H_, 128);
    nh_.param<int>("h_res", W_, 500);
    nh_.param<double>("h_fov_deg", h_fov_deg_, 120.0);
    nh_.param<double>("v_min_deg", v_min_deg_, -12.5);
    nh_.param<double>("v_max_deg", v_max_deg_, 12.9);

    nh_.param<int>("pyr_levels", pyr_levels_, 3);            // 金字塔层数
    nh_.param<int>("sample_step", sample_step_, 5);           // 细层采样步长（像素）
    nh_.param<double>("grad_thresh", grad_thresh_, 8.0);      // 梯度阈值(8U强度域)
    nh_.param<double>("w_photo", w_photo_, 0);
    nh_.param<double>("w_geo", w_geo_, 1.0);
    nh_.param<int>("max_iter_per_level", max_iter_per_level_, 50);
    nh_.param<int>("min_residuals", min_residuals_, 200);

    nh_.param<int>("num_threads", num_threads_, 28);
    if (num_threads_ <= 0) num_threads_ = std::max(1u, std::thread::hardware_concurrency());

    nh_.param<bool>("use_clahe", use_clahe_, false);          // 默认关闭，保持光度一致性
    nh_.param<double>("clahe_clip", clahe_clip_, 3.0);

    // 角度边界（弧度）
    min_az_ = -h_fov_deg_ / 2.0 * M_PI/180.0;
    max_az_ =  h_fov_deg_ / 2.0 * M_PI/180.0;
    min_el_ =  v_min_deg_ * M_PI/180.0;
    max_el_ =  v_max_deg_ * M_PI/180.0;

    ensureDir(output_dir_);
    clahe_ = cv::createCLAHE(clahe_clip_);
    cv::setNumThreads(num_threads_);

    sub_ = nh_.subscribe("/lidar_points", 1, &LidarIntensityDVO::callback, this);

    ROS_INFO("=== Lidar DVO 初始化 ===");
    ROS_INFO("输出目录: %s", output_dir_.c_str());
    ROS_INFO("分辨率: H=%d, W=%d, FOV_h=%.1fdeg, v=[%.1f, %.1f]deg, 线程=%d",
             H_, W_, h_fov_deg_, v_min_deg_, v_max_deg_, num_threads_);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;

  std::string output_dir_;
  int H_{128}, W_{500};
  double h_fov_deg_{120.0}, v_min_deg_{-12.5}, v_max_deg_{12.9};
  double min_az_{0}, max_az_{0}, min_el_{0}, max_el_{0};

  int pyr_levels_{3};
  int sample_step_{2};
  double grad_thresh_{8.0};
  double w_photo_{1.0}, w_geo_{1.0};
  int max_iter_per_level_{10};
  int min_residuals_{200};
  int num_threads_{1};
  bool use_clahe_{false};
  double clahe_clip_{3.0};

  cv::Ptr<cv::CLAHE> clahe_;

  bool has_prev_{false};
  Frame prev_;
  int frame_idx_{0};

  // ========== 回调 ==========
  void callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    auto t0 = std::chrono::high_resolution_clock::now();

    Frame cur = makeFrameFromCloud(*msg);
    if (!cur.valid()) {
      ROS_WARN("当前帧无效，跳过");
      return;
    }

    saveImage(cur.intensity_8u, "intensity_cur", frame_idx_);

    if (!has_prev_) {
      prev_ = std::move(cur);
      has_prev_ = true;
      frame_idx_++;
      ROS_INFO("首帧缓存完成");
      return;
    }

    // ===== 构建金字塔 =====
    Pyramid pyr_prev = buildPyramid(prev_, pyr_levels_);
    Pyramid pyr_cur  = buildPyramid(cur,  pyr_levels_);

    // ===== 多层优化（粗->细）=====
    double pose[6] = {0,0,0, 0,0,0};  // 角轴+平移，初始为单位
    double ab[2]   = {1.0, 0.0};      // 光度仿射参数 a,b

    double total_cost_init = 0.0, total_cost_final = 0.0;

    for (int lvl = pyr_levels_-1; lvl >= 0; --lvl) {
      const Frame& Fp = pyr_prev.levels[lvl];
      const Frame& Fc = pyr_cur.levels[lvl];
      ROS_INFO("---- 金字塔层 %d: H=%d W=%d ----", lvl, Fp.H, Fp.W);

      // 将 Fc.intensity_f64 拷贝为连续 std::vector<double>（避免潜在步长问题）
      std::vector<double> Ibuf(Fc.H * Fc.W, 0.0);
      #pragma omp parallel for num_threads(num_threads_)
      for (int v = 0; v < Fc.H; ++v) {
        const double* row = Fc.intensity_f64.ptr<double>(v);
        for (int u = 0; u < Fc.W; ++u) {
          Ibuf[v * Fc.W + u] = row[u];
        }
      }

      int added = 0;
      ceres::Problem problem;
      std::vector<ceres::CostFunction*> costs;
      std::vector<ceres::LossFunction*> losses;
      costs.reserve((Fp.H*Fp.W)/std::max(1, sample_step_*sample_step_));
      losses.reserve(costs.capacity());

      // 层级步长（粗层更大）
      int step = sample_step_ << std::max(0, lvl);
      step = std::max(1, step);

      // 梯度阈值（使用 8U 梯度的绝对阈值）
      double gthr = grad_thresh_;

      std::atomic<int> atomic_added(0);

      #pragma omp parallel num_threads(num_threads_)
      {
        std::vector<ceres::CostFunction*> local_costs;
        std::vector<ceres::LossFunction*> local_losses;
        int local_added = 0;

        #pragma omp for schedule(static)
        for (int v=1; v<Fp.H-1; v+=step) { // 避开边界1像素，便于法向/梯度
          for (int u=1; u<Fp.W-1; u+=step) {
            const int idx = v*Fp.W + u;
            if (!Fp.mask[idx]) continue;

            // 梯度筛选（prev）
            float gm = Fp.grad_mag.at<float>(v,u);
            if (gm < gthr) continue;

            // 当前像素处至少要有 3D/法向（周边插值才可能有效）
            if (!Fc.mask[idx]) continue;

            // 读取上一帧数据（double 强度，及其 3D）
            const double Iprev = Fp.intensity_f64.at<double>(v,u);
            const double Xp = Fp.X[idx], Yp = Fp.Y[idx], Zp = Fp.Z[idx];

            auto* functor = new PhotoPlaneResidual(
              Xp, Yp, Zp, Iprev,
              Ibuf.data(),                  // ★ 连续内存
              Fc.X.data(), Fc.Y.data(), Fc.Z.data(),
              Fc.Nx.data(), Fc.Ny.data(), Fc.Nz.data(),
              Fc.W, Fc.H,
              Fc.min_az, Fc.max_az, Fc.min_el, Fc.max_el,
              w_photo_, w_geo_
            );

            auto* cost = new ceres::AutoDiffCostFunction<PhotoPlaneResidual, 2, 6, 2>(functor);
            auto* loss = new ceres::HuberLoss(1.0); // 鲁棒核

            local_costs.push_back(cost);
            local_losses.push_back(loss);
            local_added++;
          }
        }

        #pragma omp critical
        {
          costs.insert(costs.end(), local_costs.begin(), local_costs.end());
          losses.insert(losses.end(), local_losses.begin(), local_losses.end());
          atomic_added += local_added;
        }
      }

      added = atomic_added.load();
      ROS_INFO("层 %d: 残差块 %d 个（步长=%d，梯度阈=%.1f）", lvl, added, step, gthr);
      if (added < min_residuals_) {
        ROS_WARN("层 %d: 残差过少（%d < %d），跳过该层", lvl, added, min_residuals_);
        continue;
      }

      for (size_t i=0;i<costs.size();++i) {
        problem.AddResidualBlock(costs[i], losses[i], pose, ab);
      }

      // Ceres 设置
      ceres::Solver::Options opt;
      opt.linear_solver_type = ceres::DENSE_QR;
      opt.max_num_iterations = std::max(3, max_iter_per_level_);
      opt.num_threads = num_threads_;                // ? 新版统一使用 num_threads
      opt.minimizer_progress_to_stdout = false;
      opt.logging_type = ceres::SILENT;

      ceres::Solver::Summary sum;
      double cost0 = 0.0, cost1 = 0.0;
      problem.Evaluate(ceres::Problem::EvaluateOptions(), &cost0, nullptr, nullptr, nullptr);
      auto t1 = std::chrono::high_resolution_clock::now();
      ceres::Solve(opt, &problem, &sum);
      auto t2 = std::chrono::high_resolution_clock::now();
      problem.Evaluate(ceres::Problem::EvaluateOptions(), &cost1, nullptr, nullptr, nullptr);

      double dt = std::chrono::duration<double, std::milli>(t2 - t1).count();
      ROS_INFO("层 %d: 迭代 %.2f ms, 初始cost=%.3f, 结束cost=%.3f, iters=%d, a=%.4f, b=%.4f",
               lvl, dt, cost0, cost1, sum.num_successful_steps, ab[0], ab[1]);

      if (lvl == pyr_levels_-1) total_cost_init = cost0;
      if (lvl == 0) total_cost_final = cost1;
    }

    // ===== 日志与保存 =====
    logAndSaveResult(cur, pose, ab, total_cost_init, total_cost_final);

    // 缓存当前帧
    prev_ = std::move(cur);
    frame_idx_++;

    auto t9 = std::chrono::high_resolution_clock::now();
    double total = std::chrono::duration<double,std::milli>(t9 - t0).count();
    ROS_INFO("本帧总耗时 %.1f ms", total);
  }

  // ========== 点云 -> Frame ==========
  Frame makeFrameFromCloud(const sensor_msgs::PointCloud2& msg) {
    Frame F;
    F.H = H_; F.W = W_;
    F.min_az = min_az_; F.max_az = max_az_;
    F.min_el = min_el_; F.max_el = max_el_;
    F.X.assign(F.H*F.W, std::numeric_limits<double>::quiet_NaN());
    F.Y.assign(F.H*F.W, std::numeric_limits<double>::quiet_NaN());
    F.Z.assign(F.H*F.W, std::numeric_limits<double>::quiet_NaN());
    F.Nx.assign(F.H*F.W, 0.0);
    F.Ny.assign(F.H*F.W, 0.0);
    F.Nz.assign(F.H*F.W, 0.0);
    F.mask.assign(F.H*F.W, 0);

    // 投影到强度图（保留 raw 强度量纲，随后线性映射到 0..255）
    std::vector<double> I(F.H*F.W, -1.0);

    sensor_msgs::PointCloud2ConstIterator<float> it_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(msg, "z");
    sensor_msgs::PointCloud2ConstIterator<float> it_i(msg, "intensity");

    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_i) {
      double x=*it_x, y=*it_y, z=*it_z, inten=*it_i;
      if (!isFinite3(x,y,z)) continue;

      double az = std::atan2(y,x);
      double el = std::atan2(z, std::sqrt(x*x + y*y));
      if (az < min_az_ || az > max_az_ || el < min_el_ || el > max_el_) continue;

      int u = (int)std::round((az - min_az_) / (max_az_ - min_az_) * (W_ - 1));
      int v = (int)std::round((max_el_ - el) / (max_el_ - min_el_) * (H_ - 1));
      int id = v*W_ + u;

      // 采用“强度更大”的点（减少遮挡伪影）
      if (!F.mask[id] || inten > I[id]) {
        F.X[id]=x; F.Y[id]=y; F.Z[id]=z;
        I[id] = inten;
        F.mask[id]=1;
      }
    }

    // 构建强度图（8U用可视化，F64用于优化；范围0..255）
    F.intensity_f64 = cv::Mat(F.H, F.W, CV_64F, cv::Scalar(0));
    double minI=1e30, maxI=-1e30;
    for (int i=0;i<F.H*F.W;++i) if (F.mask[i]) {
      minI = std::min(minI, I[i]);
      maxI = std::max(maxI, I[i]);
    }
    if (!(maxI>minI)) { minI=0; maxI=1; } // 防护

    const double scale = 255.0/(maxI-minI);
    F.intensity_8u.create(F.H, F.W, CV_8U);
    for (int v=0; v<F.H; ++v) {
      for (int u=0; u<F.W; ++u) {
        int id = v*F.W + u;
        if (F.mask[id]) {
          double val = (I[id]-minI)*scale;
          val = std::min(255.0,std::max(0.0,val));
          F.intensity_8u.at<uchar>(v,u) = (uchar)std::lround(val);
          F.intensity_f64.at<double>(v,u) = val;     // 直接用 0..255
        } else {
          F.intensity_8u.at<uchar>(v,u) = 0;
          F.intensity_f64.at<double>(v,u) = 0;
        }
      }
    }

    if (use_clahe_) {
      cv::Mat tmp; F.intensity_8u.copyTo(tmp);
      clahe_->apply(tmp, F.intensity_8u);
      F.intensity_8u.convertTo(F.intensity_f64, CV_64F);
    }

    // 计算法向（简单邻域差分）
    computeNormals(F);

    // 计算梯度 magnitude（Sobel）
    cv::Mat gx, gy;
    cv::Sobel(F.intensity_8u, gx, CV_32F, 1, 0, 3);
    cv::Sobel(F.intensity_8u, gy, CV_32F, 0, 1, 3);
    cv::magnitude(gx, gy, F.grad_mag);

    return F;
  }

  // ========== 法向估计（简单 + 有效性检查）==========
  void computeNormals(Frame& F) {
    const int H=F.H, W=F.W;
    auto id = [W](int u,int v){return v*W+u;};
    for (int v=1; v<H-1; ++v) {
      for (int u=1; u<W-1; ++u) {
        int i = id(u,v);
        if (!F.mask[i]) { F.Nx[i]=F.Ny[i]=F.Nz[i]=0; continue; }

        int il = id(u-1,v), ir = id(u+1,v), iu = id(u,v-1), idd = id(u,v+1);
        if (!(F.mask[il] && F.mask[ir] && F.mask[iu] && F.mask[idd])) {
          F.Nx[i]=F.Ny[i]=F.Nz[i]=0; continue;
        }

        // 邻域向量
        double vx1 = F.X[ir]-F.X[il], vy1=F.Y[ir]-F.Y[il], vz1=F.Z[ir]-F.Z[il];
        double vx2 = F.X[idd]-F.X[iu], vy2=F.Y[idd]-F.Y[iu], vz2=F.Z[idd]-F.Z[iu];

        // 叉乘
        double nx = vy1*vz2 - vz1*vy2;
        double ny = vz1*vx2 - vx1*vz2;
        double nz = vx1*vy2 - vy1*vx2;
        double nrm = std::sqrt(nx*nx+ny*ny+nz*nz) + 1e-12;

        F.Nx[i]=nx/nrm; F.Ny[i]=ny/nrm; F.Nz[i]=nz/nrm;
      }
    }
  }

  // ========== 构建金字塔 ==========
  Pyramid buildPyramid(const Frame& in, int L) {
    Pyramid P; P.levels.resize(L);
    P.levels[0]=in;

    for (int l=1; l<L; ++l) {
      const Frame& F0 = P.levels[l-1];
      Frame F1;
      F1.H = std::max(1, F0.H/2);
      F1.W = std::max(1, F0.W/2);
      F1.min_az=F0.min_az; F1.max_az=F0.max_az;
      F1.min_el=F0.min_el; F1.max_el=F0.max_el;

      // 强度图缩放
      cv::resize(F0.intensity_8u,  F1.intensity_8u,  cv::Size(F1.W,F1.H), 0,0, cv::INTER_AREA);
      cv::resize(F0.intensity_f64, F1.intensity_f64, cv::Size(F1.W,F1.H), 0,0, cv::INTER_AREA);
      cv::resize(F0.grad_mag,      F1.grad_mag,      cv::Size(F1.W,F1.H), 0,0, cv::INTER_AREA);

      // 3D/法向/Mask：2x2 block 平均（有效数>=2认为有效）
      F1.X.assign(F1.H*F1.W, std::numeric_limits<double>::quiet_NaN());
      F1.Y.assign(F1.H*F1.W, std::numeric_limits<double>::quiet_NaN());
      F1.Z.assign(F1.H*F1.W, std::numeric_limits<double>::quiet_NaN());
      F1.Nx.assign(F1.H*F1.W, 0.0);
      F1.Ny.assign(F1.H*F1.W, 0.0);
      F1.Nz.assign(F1.H*F1.W, 0.0);
      F1.mask.assign(F1.H*F1.W, 0);

      for (int v=0; v<F1.H; ++v) {
        for (int u=0; u<F1.W; ++u) {
          int u0=u*2, v0=v*2;
          double sx=0,sy=0,sz=0, snx=0,sny=0,snz=0; int cnt=0;
          for (int dv=0; dv<2; ++dv)
            for (int du=0; du<2; ++du) {
              int uu=u0+du, vv=v0+dv;
              if (uu>=F0.W || vv>=F0.H) continue;
              int id0 = vv*F0.W+uu;
              if (!F0.mask[id0]) continue;
              sx += F0.X[id0]; sy += F0.Y[id0]; sz += F0.Z[id0];
              snx+= F0.Nx[id0]; sny+= F0.Ny[id0]; snz+= F0.Nz[id0];
              cnt++;
            }
          int id1 = v*F1.W+u;
          if (cnt >= 2) {
            F1.X[id1]=sx/cnt; F1.Y[id1]=sy/cnt; F1.Z[id1]=sz/cnt;
            double nx=snx/cnt, ny=sny/cnt, nz=snz/cnt;
            double nrm=std::sqrt(nx*nx+ny*ny+nz*nz)+1e-12;
            F1.Nx[id1]=nx/nrm; F1.Ny[id1]=ny/nrm; F1.Nz[id1]=nz/nrm;
            F1.mask[id1]=1;
          }
        }
      }
      P.levels[l]=F1;
    }
    return P;
  }

  // ========== 日志与输出 ==========
  void logAndSaveResult(const Frame& cur, const double pose[6], const double ab[2],
                        double cost0, double cost1) {
    {
      std::ofstream ofs(output_dir_ + "/T_log.txt", std::ios::app);
      ofs << frame_idx_ << " "
          << pose[0] << " " << pose[1] << " " << pose[2] << " "
          << pose[3] << " " << pose[4] << " " << pose[5] << " "
          << ab[0] << " " << ab[1] << " "
          << cost0 << " " << cost1 << "\n";
    }

    saveImage(cur.intensity_8u, "intensity", frame_idx_);
    cv::Mat grad_u8; cv::normalize(cur.grad_mag, grad_u8, 0,255, cv::NORM_MINMAX, CV_8U);
    saveImage(grad_u8, "gradmag", frame_idx_);

    ROS_INFO("帧 %d 结果已保存：姿态(角轴+平移)=[%.4f %.4f %.4f | %.4f %.4f %.4f], a=%.3f, b=%.3f, cost: %.3f->%.3f",
             frame_idx_, pose[0],pose[1],pose[2], pose[3],pose[4],pose[5], ab[0],ab[1], cost0, cost1);
  }

  void saveImage(const cv::Mat& img, const std::string& tag, int idx) {
    std::string path = output_dir_ + "/" + tag + "_" + std::to_string(idx) + ".png";
    cv::imwrite(path, img);
  }
};

// ======================= 主函数 =======================
int main(int argc, char** argv) {
  ros::init(argc, argv, "lidar_intensity_dvo_full");
  ros::NodeHandle nh("~");
  LidarIntensityDVO node(nh);
  ros::spin();
  return 0;
}
