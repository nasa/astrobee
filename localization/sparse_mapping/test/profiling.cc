#include <DBoW2/DBoW2.h>      // BoW db that works with both float and binary descriptors
#include <DBoW2/FSurf64.h>
#include <interest_point/matching.h>
#include <iostream>
#include <vector>
#include <typeinfo>
#include <ff_common/utils.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/imgproc.hpp>

#include <boost/filesystem.hpp>
#include <gflags/gflags.h>
#include <random>
#include <cmath>
#include <algorithm>
#include "json.hpp"

namespace fs = boost::filesystem;
using namespace DBoW2;
using namespace std;
using json = nlohmann::json;

typedef DBoW2::TemplatedVocabulary<DBoW2::FSurf64::TDescriptor, DBoW2::FSurf64> MyFloatVocab;
typedef DBoW2::TemplatedDatabase<DBoW2::FSurf64::TDescriptor, DBoW2::FSurf64> MyFloatDB;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// extended surf gives 128-dimensional vectors
const bool EXTENDED_SURF = false;
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

DEFINE_bool(iss_dir, true, "use ISS image directory of 2021 images");
DEFINE_int32(num_images, 500, "number of images to use from ISS directory; default 500");
DEFINE_bool(create_vocab, true, "create vocabulary from images; default true");
DEFINE_int32(branching_factor, 10, "branching factor; default 9");
DEFINE_int32(depth, 4, "depth; default 3");
DEFINE_int32(num_to_query, 4, "number to query; default 4");
DEFINE_int32(num_query_results, 10, "num query results; default 10");
DEFINE_bool(create_db, true, "create db; default true");
DEFINE_bool(test_query, true, "test_query; default true");
DEFINE_bool(verbose, false, "verbose; default false");
DEFINE_string(json_file, "data.json", "file to dump to; default data.json");
DEFINE_double(rbo_p, 0.95, "rbo p; default 0.95");
DEFINE_int32(threshold, 400, "detector threshold; default 400");

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void loadFeatures(vector<vector<vector<float> > > &features, vector<string> &cid_to_name);
void loadFeatures2(vector<vector<vector<float> > > &features, vector<string> &cid_to_name);

void changeStructure(const cv::Mat &plain, vector<vector<float>> &out);
void testVocCreation(const vector<vector<vector<float> > > &features);
void testDatabase(const vector<vector<vector<float>>> &features, vector<string> &cid_to_name);
void testQueryVsAllImages(const vector<vector<vector<float>>> &features, vector<string> &cid_to_name);

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// number of training images
const int NIMAGES = 4;


void wait()
{
  cout << endl << "Press enter to continue" << endl;
  getchar();
}

int getRandomNumber(int min, int max) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(min, max);
    return dis(gen);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 


int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  vector<vector<vector<float> > > features;
  vector<string> cid_to_name;

  if (FLAGS_iss_dir) {
    loadFeatures2(features, cid_to_name);
  } else {
    loadFeatures(features, cid_to_name);
  }

  // std::cout << "feature " << features[0][0] << std::endl;

  // num images, num features for each image, descriptor length for each feature
  // 4 319 [64 x 1]
  std::cout << features.size() << " " << features[0].size() << " " << features[0][0].size() << std::endl;
  if (FLAGS_create_vocab) {
    testVocCreation(features);
  }

  // wait();

  if (FLAGS_create_db) {
    testDatabase(features, cid_to_name);
  }

  // wait();
  if (FLAGS_test_query) {
    testQueryVsAllImages(features, cid_to_name);
  }

  return 0;
}

// ----------------------------------------------------------------------------

void loadFeatures(vector<vector<vector<float> > > &features, vector<string> &cid_to_name)
{
  features.clear();
  features.reserve(NIMAGES);

  cv::Ptr<cv::Feature2D> fdetector=cv::xfeatures2d::SURF::create(FLAGS_threshold, 4, 2, EXTENDED_SURF);

  if (FLAGS_verbose) {
    std::cout << "Extracting  features..." << endl;
  }
  for(int i = 0; i < NIMAGES; ++i)
  {
    stringstream ss;
    ss << "/home/lmao/Documents/images/image" << i << ".png";
    cid_to_name.push_back(ss.str());
    cv::Mat image = cv::imread(ss.str(), 0);
    cv::Mat mask;
    vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

    fdetector->detectAndCompute(image, mask, keypoints, descriptors);

    features.push_back(vector<vector<float> >());
    changeStructure(descriptors, features.back());
  }

  int num_features = 0;
  // iterate over features
  for (int i = 0; i < features.size(); i++) {
    num_features += features[i].size();
  }
  if (FLAGS_verbose) {
    std::cout << "num features: " << num_features << std::endl;
  }
}

// ----------------------------------------------------------------------------

void loadFeatures2(vector<vector<vector<float> > > &features, vector<string> &cid_to_name)
{
  features.clear();
  features.reserve(NIMAGES);

  // Specify the directory path
  std::string directoryPath = "/srv/novus_1/amoravar/data/images/latest_map_imgs/2020-09-24/";

  cv::Ptr<cv::Feature2D> fdetector=cv::xfeatures2d::SURF::create(FLAGS_threshold, 4, 2, EXTENDED_SURF);

  int num_images = 0;
  if (FLAGS_verbose) {
    cout << "Extracting  features..." << endl;
  }
  // Iterate over files in the directory
  for (const auto& entry : fs::directory_iterator(directoryPath)) {
      if (num_images >= FLAGS_num_images) {
        break;
      }

      // Check if the entry is a regular file
      if (fs::is_regular_file(entry)) {
          // Get the file path
          std::string filePath = entry.path().string();
          cid_to_name.push_back(filePath);

        cv::Mat image = cv::imread(filePath, 0);
        cv::Mat mask;
        vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;

        fdetector->detectAndCompute(image, mask, keypoints, descriptors);

        features.push_back(vector<vector<float> >());
        changeStructure(descriptors, features.back());
        num_images++;
      }
  }
  int num_features = 0;
  // iterate over features
  for (int i = 0; i < features.size(); i++) {
    num_features += features[i].size();
  }
  if (FLAGS_verbose) {
    std::cout << "num images: " << num_images << " num features " <<num_features<<std::endl;
  }
}

// ----------------------------------------------------------------------------


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

// ----------------------------------------------------------------------------

void testVocCreation(const vector<vector<vector<float> > > &features)
{
  // branching factor and depth levels 
  const int k = FLAGS_branching_factor;
  const int L = FLAGS_depth;
  const WeightingType weight = TF_IDF;
  const ScoringType scoring = L1_NORM;

  MyFloatVocab voc(k, L, weight, scoring);

  if (FLAGS_verbose) {
    std::cout << "Creating a small " << k << "^" << L << " vocabulary..." << endl;
  }
  voc.create(features);

  if (FLAGS_verbose) {
    std::cout << "... done!" << endl;
    cout << "Vocabulary information: " << endl
    << voc << endl << endl;
  }


  // save the vocabulary to disk
  if (FLAGS_verbose) {
    std::cout << endl << "Saving vocabulary..." << endl;
  }
  voc.save("/home/lmao/Documents/small_voc.yml.gz");
  if (FLAGS_verbose) {
    std::cout << "Done" << endl;
  }
}

// // ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------

void testDatabase(const vector<vector<vector<float>>> &features, vector<string> &cid_to_name)
{
  if (FLAGS_verbose) {
    std::cout << "Creating a small database..." << endl;
  }

  // load the vocabulary from disk
  MyFloatVocab voc("/home/lmao/Documents/small_voc.yml.gz");
  if (FLAGS_verbose) {
    std::cout << "Vocabulary information: " << endl
    << voc << endl << endl;
    std::cout << "total num features "<< features.size() << endl;
  }
  MyFloatDB db(voc, false, 0); // false = do not use direct index
  // (so ignore the last param)
  // The direct index is useful if we want to retrieve the features that 
  // belong to some vocabulary node.
  // db creates a copy of the vocabulary, we may get rid of "voc" now

  // add images to the database
  for(int i = 0; i < features.size(); i++)
  {
    db.add(features[i]);
  }

  if (FLAGS_verbose) {
    std::cout << "... done!" << endl;
    cout << "Database information: " << endl << db << endl;
    cout << "Saving database..." << endl;
  }

  // we can save the database. The created file includes the vocabulary
  // and the entries added
  db.save("/home/lmao/Documents/small_db.yml.gz");
  if (FLAGS_verbose) {
    std::cout << "... done!" << endl;
  }
  
  // once saved, we can load it again  
  // cout << "Retrieving database once again..." << endl;
  // MyFloatDB db2("/home/lmao/Documents/small_db.yml.gz");
  // cout << "... done! This is: " << endl << db2 << endl;
}

// ----------------------------------------------------------------------------
// compare with all images
cv::Mat convertVectorOfVectorsToMat(const std::vector<std::vector<float>>& input);
double rankBiasedOverlap(const std::vector<int>& list1, const std::vector<int>& list2, double p);

void testQueryVsAllImages(const vector<vector<vector<float>>> &features, vector<string> &cid_to_name) {

  // Create an empty JSON object
  json jsonObjectToDump;
  vector<json> jsonObjects;
  jsonObjectToDump["cid_to_filename"] = cid_to_name;


  MyFloatDB db("/home/lmao/Documents/small_db.yml.gz");

  // and query the database
  if (FLAGS_verbose) {
    cout << "Querying the database: " << endl;
  }

  QueryResults ret;
  for(int i = 0; i < FLAGS_num_to_query; i++)
  {
    json jsonObject;
    int idx = getRandomNumber(0, (int)features.size()-1);
    jsonObject["query_cid"] = idx;
    // int idx = i;
    db.query(features[idx], ret, FLAGS_num_query_results);

    // ret[0] is always the same image in this case, because we added it to the 
    // database. ret[1] is the second best match.

    if (FLAGS_verbose) {
      cout << "Searching for Image " << idx<< "  " << cid_to_name[idx] <<endl;
      for (size_t j = 0; j < ret.size(); j++) {
        cout<< ret[j].Id << " " << ret[j].Score <<" "<<cid_to_name[ret[j].Id]<<endl;
      }
    }
    // << ". " << ret << endl;
    vector<int> rank_query;
    vector<float> query_score_list;
    for (size_t j = 0; j < ret.size(); j++) {
      // cout<< ret[j].Id << " " << ret[j].Score <<" "<<cid_to_name[ret[j].Id]<<endl;
      rank_query.push_back(ret[j].Id);
      query_score_list.push_back(ret[j].Score);
    }
    jsonObject["query_results"] = rank_query;
    jsonObject["query_score_list"] = query_score_list;

  cout << endl;
 //----------------------------------------------
 // matching against all images
  if (FLAGS_verbose) {
    std::cout << "Matching against all images: " << endl;
  }
  std::vector<int> similarity_rank(features.size(), 0);
  std::vector<std::vector<cv::DMatch> > all_matches(features.size());
  int total = 0;
  for (size_t k = 0; k < features.size(); k++) {  // for each image
    cv::Mat test_descriptors = convertVectorOfVectorsToMat(features[idx]);
    cv::Mat test_descriptors2 = convertVectorOfVectorsToMat(features[k]);
    interest_point::FindMatches(test_descriptors,
                                test_descriptors2,
                                &all_matches[k]);

    for (size_t n = 0; n < all_matches[k].size(); n++) {  // for each match
      // if (cid_fid_to_pid[cid].count(all_matches[k][n].trainIdx) == 0)
      //   continue;
      similarity_rank[k]++;
    }

  }
  std::vector<int> highly_ranked = ff_common::rv_order(similarity_rank);
    vector<int> brute_force_score_list;

  std::vector<int> rank_all(highly_ranked.begin(), highly_ranked.begin() + FLAGS_num_query_results);
  if (FLAGS_verbose) {
    for (int n = 0; n < FLAGS_num_query_results; n++) {
      cout<< highly_ranked[n] << " " << similarity_rank[highly_ranked[n]] << " " << cid_to_name[highly_ranked[n]] << endl;
    }

    cout << "Rank Biased Overlap: " << rankBiasedOverlap(rank_query, rank_all, FLAGS_rbo_p) << endl;

    cout << "--------------------------------------------" << endl;
  } 
    for (int n : rank_all) {
      brute_force_score_list.push_back(similarity_rank[highly_ranked[n]]);
    }
    jsonObject["brute_force_results"] = rank_all;
    jsonObject["brute_force_score_list"] = brute_force_score_list;
    jsonObject["RBO_score"] = rankBiasedOverlap(rank_query, rank_all, FLAGS_rbo_p);
    jsonObjects.push_back(jsonObject);
  }

  // Define the file path
  std::string filePath = "/home/lmao/Documents/"+FLAGS_json_file;

  // Check if the file exists
  std::ifstream inputFile(filePath);
  bool fileExists = inputFile.good();
  inputFile.close();

  // If the file exists, read its contents into a JSON object
  json existingObject;
  if (fileExists) {
    std::ifstream existingFile(filePath);
    existingFile >> existingObject;
    existingFile.close();
  }

  // Append the new JSON object to the existing JSON object
  stringstream ss;
  int depth = db.getVocabulary()->getDepthLevels();
  int branching_factor = db.getVocabulary()->getBranchingFactor();
  ss << "voc_" << branching_factor << "^" << depth;
  jsonObjectToDump["data"] = jsonObjects;
  existingObject[ss.str()] = (jsonObjectToDump);

  // Write the updated JSON object back to the file
  std::ofstream outputFile(filePath);
  outputFile << existingObject.dump();
  outputFile.close();
  

  // per vocab:
    // cid to filename
    // data
      // list of num query:
        // query_results (cid)
        // query_scores
        // brute_force_results (cid)
        // brute_force_scores
        // rank biased overlap

}

cv::Mat convertVectorOfVectorsToMat(const std::vector<std::vector<float>>& input)
{
    // Check if the input vector is empty
    if (input.empty())
        return cv::Mat();

    // Get the dimensions of the input vector
    size_t rows = input.size();
    size_t cols = input[0].size();

    // Create a new cv::Mat with the appropriate size
    cv::Mat output(rows, cols, CV_32F);

    // Copy the data from the input vector to the cv::Mat
    for (size_t i = 0; i < rows; i++)
    {
        // Check if the size of each inner vector is consistent
        if (input[i].size() != cols)
        {
            // Throw an exception or handle the error accordingly
            // ...
        }

        // Copy the data from the input vector to the cv::Mat row by row
        memcpy(output.ptr<float>(i), input[i].data(), cols * sizeof(float));
    }

    return output;
}

double rankBiasedOverlap(const std::vector<int>& list1, const std::vector<int>& list2, double p) {
    double rbo = 0.0;
    double weight = 1.0;

    int commonItems = 0;
    std::vector<int> intersect;
    std::set_intersection(list1.begin(), list1.end(), list2.begin(), list2.end(), std::back_inserter(intersect));

    for (int i = 0; i < intersect.size(); i++) {
        if (i < list1.size() && i < list2.size() && list1[i] == list2[i]) {
            commonItems++;
        }
        rbo += weight * (double)(commonItems) / (double)(i + 1);
        weight *= p;
    }

    rbo = (1 - p) * rbo + (p / std::min(list1.size(), list2.size())) * (commonItems);
    return rbo;
}