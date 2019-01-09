#ifndef HEADPOSEDETECTOR_H
#define HEADPOSEDETECTOR_H

#include "ofxHeadPoseEstimator.h"

using namespace std;
using namespace cv;

class headPoseDetector
{
public:
    headPoseDetector();
    int getHeadPoses(const Mat &Image3D , std::vector<cv::Vec<float, POSE_SIZE>> &destVector);
    bool setup();

private:
    //-------------- Estimator parameters
    // Path to trees
    string g_treepath;
    // Number of trees
    int g_ntrees;
    // Patch width
    int g_p_width;
    // Patch height
    int g_p_height;
    //head threshold - to classify a cluster of votes as a head
    int g_th;
    //threshold for the probability of a patch to belong to a head
    float g_prob_th;
    //threshold on the variance of the leaves
    float g_maxv;
    //stride (how densely to sample test patches - increase for higher speed)
    int g_stride;
    //radius used for clustering votes into possible heads
    float g_larger_radius_ratio;
    //radius used for mean shift
    float g_smaller_radius_ratio;
    //pointer to the actual estimator
    CRForestEstimator* g_Estimate;
    // estimator trees properly loaded
    bool bTreesLoaded = false;
    //-------------------------------------------------------
    // I copied this code from the library demo code
    // this kind of nomenclature is unknown to me
    std::vector< cv::Vec<float,POSE_SIZE> > g_means; //outputs
    std::vector< std::vector< Vote > > g_clusters; //full clusters of votes
    std::vector< Vote > g_votes; //all votes returned by the forest

};

#endif // HEADPOSEDETECTOR_H
