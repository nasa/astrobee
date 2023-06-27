#include <DBoW2/DBoW2.h>      // BoW db that works with both float and binary descriptors
#include <DBoW2/FSurf64.h>
#include <sparse_mapping/sparse_map.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/imgproc.hpp>

typedef DBoW2::TemplatedVocabulary<DBoW2::FSurf64::TDescriptor, DBoW2::FSurf64> MyFloatVocab;
typedef DBoW2::TemplatedDatabase<DBoW2::FSurf64::TDescriptor, DBoW2::FSurf64> MyFloatDB;

using namespace std;
using namespace DBoW2;


void changeStructure(const cv::Mat& plain, std::vector<std::vector<float>>& out)
{
  out.resize(plain.rows);

  for (int i = 0; i < plain.rows; ++i)
  {
    const cv::Mat row = plain.row(i);
    for (int j = 0; j < row.cols; ++j)
    {
      out[i].push_back(row.at<float>(0, j));
    }
  }
}



int main(int argc, char** argv) {
    sparse_mapping::SparseMap map("/home/lmao/Documents/20210304_aach.surf.test_vocab2.map", true);
    std::string filename = "/home/lmao/Documents/1595953736.4697478.jpg";
    cv::Mat image = cv::imread(filename, 0);
    cv::Mat mask;
    std::vector<cv::KeyPoint> test_keypoints;
    cv::Mat test_descriptors;
    cv::Ptr<cv::Feature2D> fdetector=cv::xfeatures2d::SURF::create(400, 4, 2, false);
    fdetector->detectAndCompute(image, mask, test_keypoints, test_descriptors);
    std::vector<std::vector<float> > test_features;
    changeStructure(test_descriptors, test_features);
    DBoW2::QueryResults ret;
    MyFloatDB db("/home/lmao/Documents/db.yml.gz");
    db.query(test_features, ret, 20);
    std::cout << "Searching for image " << filename << "." << std::endl;
    for (size_t j = 0; j < ret.size(); j++) {
        cout<< ret[j].Id << " " << ret[j].Score <<" "<<map.GetFrameFilename(ret[j].Id)<<endl;
    }

    //-------------------------------------------------------------------------

    // sparse_mapping::VocabDB vocab_db = map.vocab_db_;
    std::vector<int> query_db_indices;
    sparse_mapping::QueryDB("SURF", &(map.vocab_db_), 20, test_descriptors, &query_db_indices);

    std::cout << "Testing protobuf loaded file" << std::endl;
    for (size_t j = 0; j < query_db_indices.size(); j++) {
      cout<< query_db_indices[j] << "  "<<map.GetFrameFilename(query_db_indices[j])<<endl;
    }

    // DBoW2::BowVector bowVector;
    // std::cout << "gz file vocab "<< *(db.getVocabulary()) << std::endl;
    // std::cout << "gz file db "<< db << std::endl;
    // db.getVocabulary()->transform(test_features, bowVector);
    // std::cout <<"gz bowvector "<<bowVector<<std::endl;
    // std::cout << "word 700007 "<< db.getVocabulary()->getWord(700007)[39]<<std::endl;
    // std::cout<<"transform single feature into word (desc, word) "<<test_features[0][0]<<" "<<db.getVocabulary()->transform(test_features[0])<<std::endl;
    // std::cout<<" row 0 of desc matrix "<<test_descriptors.row(0)<<std::endl;
    // std::cout<<" first element of features list :"<<std::endl;
    // for (size_t i = 0; i < test_features[0].size(); i++) {
    //   std::cout<<test_features[0][i]<<" ";
    // }
    std::cout << "Done" << std::endl;

}