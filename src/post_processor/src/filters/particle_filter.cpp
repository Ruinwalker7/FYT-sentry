#include "filters/particle_filter.hpp"
namespace post_processor
{
ParticleFilter::ParticleFilter(
  const int &particle_num,
  const int &param_num,
  const Eigen::MatrixXd &process_noise_cov,
  const Eigen::MatrixXd &measure_noise_cov,
  const int &rng_gen_id):
param_num_(param_num),
particle_num_(particle_num),
process_noise_cov_(process_noise_cov),
measure_noise_cov_(measure_noise_cov),
rng_(rng_gen_id)
{
  auto noise_genarator = Eigen::Rand::makeMvNormalGen(Eigen::VectorXd::Zero(param_num), process_noise_cov);
  F_ = Eigen::MatrixXd::Identity(param_num, param_num);
  //初始化粒子矩阵及粒子权重
  particles_ = Eigen::MatrixXd::Zero(particle_num, param_num);
  particles_ = (noise_genarator.generate(rng_,particle_num)).transpose();
  weights_ = Eigen::MatrixXd::Ones(particle_num, 1) / float(particle_num);
}

void ParticleFilter::initialize(const Eigen::VectorXd &x0)
{
  auto noise_genarator = Eigen::Rand::makeMvNormalGen(Eigen::VectorXd::Zero(param_num_),process_noise_cov_);
  particles_ = Eigen::MatrixXd::Zero(particle_num_, param_num_);
  particles_ = (noise_genarator.generate(rng_,particle_num_)).transpose();
  weights_ = Eigen::MatrixXd::Ones(particle_num_, 1) / float(particle_num_);

  Eigen::MatrixXd tmp = (x0.replicate(1, particle_num_).transpose());
  particles_ += tmp;
}


Eigen::VectorXd ParticleFilter::update(const Eigen::VectorXd &measurement, const double &dt)
{
  // F_(0,3) = F_(1,4) = F_(2,5) = dt;
  Eigen::MatrixXd predict = particles_* F_.transpose();
  Eigen::MatrixXd measure_mat = measurement.replicate(1, particle_num_).transpose();
  double err = (((measurement - predict.transpose() * weights_)).norm());

  weights_ = Eigen::MatrixXd::Ones(particle_num_, 1);
  for(int i=0; i<particles_.cols(); ++i)
  {
    double sigma = measure_noise_cov_(i,i);
    Eigen::MatrixXd weights_dist = (predict.col(i) - measure_mat.col(i)).rowwise().squaredNorm();
    Eigen::MatrixXd tmp = ((-(weights_dist / pow(sigma, 2)) / predict.cols()).array().exp() / (sqrt(6.283135) * sigma)).array();
    weights_ = weights_.array() * tmp.array();
  }
  weights_ /= weights_.sum();
  particles_ = predict;
  double n_eff = 1.0 / (weights_.transpose()*weights_).value();
  if(err > measure_noise_cov_(0,0) || (n_eff < 0.5*particle_num_))
  {
    resample();
  }
  
  Eigen::VectorXd filtered_state = predict.transpose() * weights_;

  return filtered_state;
}

void ParticleFilter::resample()
{
  //先生成particle_num * param_num的高斯噪声矩阵
  auto noise_genarator = Eigen::Rand::makeMvNormalGen(Eigen::VectorXd::Zero(param_num_), process_noise_cov_);
  Eigen::MatrixXd noise_mat = (noise_genarator.generate(rng_,particle_num_)).transpose();

  int i = 0;
  auto c = weights_(0,0);
  auto r = Eigen::Rand::balanced<Eigen::Matrix2f>(2,2,rng_)(0,0);
  r = (r+1)/2.0/particle_num_;
  Eigen::MatrixXd particle_tmp = particles_;
  for (int m = 0; m < particle_num_; m++)
  {
    auto u = r + m * (1.0 / particle_num_);
    // 当 u > c 不进行采样
    while (u > c)
    {
      i++;
      c = c + weights_(i,0);
    }
    particle_tmp.row(m) = particles_.row(i);
  }
  particles_ = particle_tmp + noise_mat;
  weights_ = Eigen::MatrixXd::Ones(particle_num_, 1) / float(particle_num_);

}

Eigen::VectorXd ParticleFilter::predict(const double &dt)
{
  // F_(0,3) = F_(1,4) = F_(2,5) = dt;
  return (F_ * (particles_.transpose() * weights_));
}

}