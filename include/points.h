#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <vector>

class EstimateTransform {
 private:
  int readFile(const std::string &path, cv::Mat &out);
  cv::Mat computeCentroid(const cv::Mat &P);
  float computeCenteredVar(const cv::Mat &P);
  void computeResidual();
  void center(const cv::Mat &in, cv::Mat &out, cv::Mat &centroid);
  void generateDiag(const cv::Mat &U, const cv::Mat &V, cv::Mat &M);

  cv::Mat P_, Q_;
  int frame_loaded_;
  float res_;
  
 public:
  EstimateTransform(const std::string &path);
  EstimateTransform();
  ~EstimateTransform();
  int setSrc(const std::string &path);
  int setDst(const std::string &path);
  float getRes();
  bool compute();
  
  cv::Mat Rot_, t_;
  float c_;
};
