#include <time.h>
#include <string>
#include <iostream>
#include <ros/package.h>

#include <ORB_SLAM3/ORBVocabulary.h>
using namespace std;

bool load_as_text(ORB_SLAM3::ORBVocabulary* voc, const std::string infile) {
  clock_t tStart = clock();
  bool res = voc->loadFromTextFile(infile);
  printf("Loading fom text: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
  return res;
}

void load_as_xml(ORB_SLAM3::ORBVocabulary* voc, const std::string infile) {
  clock_t tStart = clock();
  voc->load(infile);
  printf("Loading fom xml: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
}

void load_as_binary(ORB_SLAM3::ORBVocabulary* voc, const std::string infile) {
  clock_t tStart = clock();
  voc->loadFromBinaryFile(infile);
  printf("Loading fom binary: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
}

void save_as_xml(ORB_SLAM3::ORBVocabulary* voc, const std::string outfile) {
  clock_t tStart = clock();
  voc->save(outfile);
  printf("Saving as xml: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
}

void save_as_text(ORB_SLAM3::ORBVocabulary* voc, const std::string outfile) {
  clock_t tStart = clock();
  voc->saveToTextFile(outfile);
  printf("Saving as text: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
}

void save_as_binary(ORB_SLAM3::ORBVocabulary* voc, const std::string outfile) {
  clock_t tStart = clock();
  voc->saveToBinaryFile(outfile);
  printf("Saving as binary: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
}


int main(int argc, char **argv) {
  cout << "BoW load/save benchmark" << endl;
  ORB_SLAM3::ORBVocabulary* voc = new ORB_SLAM3::ORBVocabulary();
  std::string path_work = ros::package::getPath("realm_ros");

  std::cout << "Opening file: " << path_work + std::string("/tools/ORBvoc.txt") << std::endl;
  std::cout << "Saving to file: " << path_work + std::string("/tools/ORBvoc.bin") << std::endl;

  load_as_text(voc, path_work + std::string("/tools/ORBvoc.txt"));
  save_as_binary(voc, path_work + std::string("/tools/ORBvoc.bin"));

  return 0;
}
