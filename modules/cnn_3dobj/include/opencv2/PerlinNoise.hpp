#ifndef __OPENCV_PERLINOISE_HPP__
#define __OPENCV_PERLINOISE_HPP__
#ifdef __cplusplus

#include <opencv2/viz/vizcore.hpp>
#include <opencv2/highgui.hpp>
#include <fstream>
#include <vector>
#include <set>
#include <iostream>

namespace cv
{
namespace cnn_3dobj
{

class CV_EXPORTS_W PerlinNoise
{

 public:
  int p[512];
  std::vector<int> permutation;
  std::default_random_engine engine;

  PerlinNoise();
  ~PerlinNoise()
  {
  }

  double noise( const double &src_x, const double &src_y, const double &src_z );
  cv::Mat CreatePerlinNoiseImage( const cv::Size &size, const double &scale = 0.05 );

 protected:
  double fade( double t );

  double lerp( double t, double a, double b );

  double grad( int hash, double x, double y, double z );
};

//! @}

}
}

#endif /* CNN_3DOBJ_HPP_ */
#endif
