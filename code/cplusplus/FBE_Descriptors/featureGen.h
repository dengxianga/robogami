#ifndef __FeatureGen__
#define __FeatureGen__

#include "TriMesh.h"
#include <Eigen/Dense>
#include "params_features.h"

class EmbreeIntersector;



class MLFFeatureGen{
public:
	//void genImageFeatures(std::ofstream &outfile, bool isVol);	
	void getImageFeatures(Eigen::VectorXd & center, bool isVol);
	void getFullFeatures(Eigen::VectorXd & vec, const  Eigen::MatrixXd & PCA_TRANS);
	//void genLightField(std::ofstream &outfile);
	MLFFeatureGen(TriMesh * mesh);
	~MLFFeatureGen();
private:
	TriMesh * mesh;
	//std::string filename;
	EmbreeIntersector* ei;
	void lf_findCenter(unsigned char *srcBuff, int width, int height, double *CenX, double *CenY);
	void getSrcBuff(unsigned char **srcBuff, int dim);
};

#endif