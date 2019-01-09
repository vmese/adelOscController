#include "headposedetector.h"

using namespace std;
using namespace cv;

headPoseDetector::headPoseDetector()
{

}

bool headPoseDetector::setup()
{
    // Number of trees
    g_ntrees = 10;
    //head threshold - to classify a cluster of votes as a head
    g_th = 500;
    //threshold for the probability of a patch to belong to a head
    g_prob_th = 1.0f;
    //threshold on the variance of the leaves
    g_maxv = 800.f;
    //stride (how densely to sample test patches - increase for higher speed)
    g_stride = 5;
    //radius used for clustering votes into possible heads
    g_larger_radius_ratio = 1.6f;
    //radius used for mean shift
    g_smaller_radius_ratio = 5.f;

    g_Estimate =  new CRForestEstimator();
    g_treepath = "./data/trees/tree";

    if( !g_Estimate->loadForest(g_treepath.c_str(), g_ntrees) ){
                //ofLog(OF_LOG_ERROR, "could not read forest!");
        bTreesLoaded = false;
        } else bTreesLoaded = true;

    return bTreesLoaded;
}

int headPoseDetector::getHeadPoses(const Mat &Image3D, std::vector< cv::Vec<float,POSE_SIZE>> &destVector)
{
    g_means.clear();
    g_votes.clear();
    g_clusters.clear();

    //do the actual estimation
    g_Estimate->estimate(   Image3D,
                            g_means,
                            g_clusters,
                            g_votes,
                            g_stride,
                            g_maxv,
                            g_prob_th,
                            g_larger_radius_ratio,
                            g_smaller_radius_ratio,
                            false,
                            g_th
                        );
    for (int i=0;i<g_means.size();i++)
    {
        destVector.push_back(g_means[i]);
    }
    return g_means.size();
}
