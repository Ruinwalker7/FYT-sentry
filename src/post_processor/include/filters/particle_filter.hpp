#ifndef POST_PROCESSOR__PARTICLE_FILTER_HPP
#define POST_PROCESSOR__PARTICLE_FILTER_HPP

#include <Eigen/Core>
#include <Eigen/Dense>
#include <EigenRand/EigenRand>

namespace post_processor
{
/**
* 粒子滤波器
*/
class ParticleFilter
{
public:
  explicit ParticleFilter(
    const int &particle_num,
    const int &param_num,
    const Eigen::MatrixXd &process_noise_cov,
    const Eigen::MatrixXd &observe_noise_cov,
    const int &rng_gen_id
  );

  Eigen::VectorXd update(const Eigen::VectorXd &measurement, const double &dt);

  Eigen::VectorXd predict(const double &dt);

  void initialize(const Eigen::VectorXd &x0);

private:
  void resample();

private:
    int particle_num_;
    int param_num_;

    Eigen::MatrixXd F_;
    Eigen::MatrixXd process_noise_cov_;
    Eigen::MatrixXd measure_noise_cov_;
    Eigen::MatrixXd weights_;
    Eigen::MatrixXd estimate_;
    Eigen::MatrixXd particles_;

    Eigen::Rand::P8_mt19937_64 rng_;
};

}//post_processor
#endif