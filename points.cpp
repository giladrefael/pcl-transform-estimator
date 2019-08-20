#include <points.h>

EstimateTransform::EstimateTransform() : res_(0) {}
EstimateTransform::EstimateTransform(const std::string &folder) : res_(0) {
  frame_loaded_ = true;
  if (setSrc(folder + "/src.txt")){
    frame_loaded_ = false;
  }
  if (setDst(folder + "/dst.txt")){
    frame_loaded_ = false;
  }
}

EstimateTransform::~EstimateTransform() {}

int EstimateTransform::readFile(const std::string &path, cv::Mat &out) {
  std::vector<float> points;
  int sz;
  std::fstream file;
  file.open(path);
  std::string line;
  if (!file.is_open()) {
    std::cout << "File open failed!" << std::endl;
    return -1;
  }
  while (getline(file, line, '\n')) {
    std::istringstream temp(line);
    std::string data;
    while (getline(temp, data, ' ')) {
      if (data == "size:") {
        getline(temp, data, ' ');
        sz = std::stoi(data);
        continue;
      }
      try {
        float val = std::stof(data);
        points.emplace_back(val);
      } catch (const std::exception &e) {
        continue;
      }
    }
  }
  file.close();
  int len = points.size() / 3;
  out = cv::Mat(points).reshape(0, len).clone();
  if (sz == out.rows) {
    std::cout << path + " Mat loaded succesfully\n";
  } else {
    std::cout << path + " Input size does not match given size\n";
    return -1;
  }
  return 0;
}

int EstimateTransform::setDst(const std::string &path) {
  return readFile(path, Q_);
}
int EstimateTransform::setSrc(const std::string &path) {
  return readFile(path, P_);
}
float EstimateTransform::getRes() { return res_; }

cv::Mat EstimateTransform::computeCentroid(const cv::Mat &P) {
  cv::Mat sum = cv::Mat::zeros(1, 3, CV_32FC1);
  for (int row = 0; row < P.rows; row++) {
    sum += P.row(row);
  }
  return sum / static_cast<int>(P.rows);
}

void EstimateTransform::center(const cv::Mat &in, cv::Mat &out,
                               cv::Mat &centroid) {
  centroid = computeCentroid(in);
  for (int row = 0; row < in.rows; row++) {
    out.row(row) = in.row(row) - centroid;
  }
}

void EstimateTransform::generateDiag(const cv::Mat &U, const cv::Mat &Vt,
                                     cv::Mat &M) {
  float d = cv::determinant(U * Vt);
  M = cv::Mat::eye(3, 3, CV_32FC1);
  if (d < 0) {
    M.at<float>(2, 2) = -1;
  }
}
float EstimateTransform::computeCenteredVar(const cv::Mat &P) {
  float var = 0;
  for (int row = 0; row < P.rows; row++) {
    var += P.row(row).dot(P.row(row));
  }
  var /= P.rows;
  return var;
}

void EstimateTransform::computeResidual() {
  float residual = 0;
  for (int row = 0; row < P_.rows; row++) {
    residual += norm(c_ * P_.row(row) * Rot_ + t_ - Q_.row(row));
  }
  res_ = residual / P_.rows;
}

bool EstimateTransform::compute() {
  if (!frame_loaded_){
    return -1;
  }
  cv::Mat meanP, meanQ;
  cv::Mat P0(P_.size(), CV_32FC1), Q0(Q_.size(), CV_32FC1);
  center(P_, P0, meanP);
  center(Q_, Q0, meanQ);

  cv::Mat S = P0.t() * Q0 / P0.rows;
  cv::SVD svd(S);

  cv::Mat M;
  generateDiag(svd.u, svd.vt, M);
  Rot_ = svd.u * M * svd.vt;

  float varP = computeCenteredVar(P0);
  c_ = svd.w.dot(M.diag()) / varP;
  t_ = meanQ - meanP * c_ * Rot_.t();
  computeResidual();
  return 0;
}

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cout << "usage: ./points path/to/folder" << std::endl;
    return -1;
  }
  const std::string folder = argv[1];
  EstimateTransform est(folder);
  bool res = est.compute();
  if (res) return -1;
  
  std::cout << "\nOutput :\nRotation:\n"
            << est.Rot_ << "\ntranslation:\n"
            << est.t_ << "\nscale: " << est.c_
            << "\nresidual: " << est.getRes() << std::endl;

  return 0;
}
