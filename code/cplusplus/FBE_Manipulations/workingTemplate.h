#pragma once
#include "NewPatch.h"
#include "FunctionEvalStability.h"
#include "geometryOptimizer.h"

namespace FabByExample{
class Geometry;
class Template;
class KinChain;
class ElectronicsGraph;
class MetricsInfo;
class Metrics;
class SymmetryChoices;


struct WorkingTemplate {
	
	

	WorkingTemplate(Template* tmpl);
	~WorkingTemplate();
	Template* getTemplate() { return tmpl; }

	// TODO(adriana): This is not being used.
	Geometry* getAnimation(double t);

	// Computes the animation for the current model returns a vector of geometry and stability costa and take sin the number of loops it should do.
	double getNewAnimation(bool useSteadyState, vector<Geometry*> & anim, bool doSequence, vector<double> &stabilityCost,   bool computeCollision=false, vector<bool> &collisionInfo=std::vector<bool>());
	double getViewGait(vector<Geometry*> & anim);

	// For a given direction of scaling change, determines if the model becomes more stable (negative value) less stable (positive) or stays the same (zero)
	double getDirStability(TemplateElement* templateElement, int subElementId, int axis, int gaitId, int objectiveId);


	// stabilizes the shape according to some criteria
	GeometryResults callOptimization(GeometryOptimizationWeights weights, bool& success);
	bool stabilize(GeometryOptimizationWeights weights);
	bool stabilize(FunctionEvalStability::StabilizationObjective objectiveType);
	bool stabilize_old(FunctionEvalStability::StabilizationObjective objectiveType);

	// runs all stabilizations algorithms and outputs resutls to file and stls - for testing and generating paper figures
	void evalStabilization();

	// Computes the gradient direction for stability using finite differences (esp is the finite diff parameter)
	void getDiniteDiffStability(double eps, Eigen::VectorXd & grad);

	// Recompute the metrics values.
	void computeMetrics(int Nrounds, double delta, bool soSequence);
	void writeMeshToFile(std::string filename);

	void contrainAllContactPoints();
	void makeUniformSpacing();
	void equalizeLegWidth();
	void equalizeLinkLength();

	//symmetry stuff
	void updateSymmetryChoices(bool symm_ground, bool symm_legW, bool symm_legL, bool symm_spacing);
	void replaceShoulderJoints();
	std::vector<std::pair<double,double>> getTopView(); 

	drawing::Drawing getTopViewDrawing();	
		
		
	Metrics * metrics;
	MetricsInfo* metricsInfo;
	KinChain* kinchain;
	ElectronicsGraph* elecgraph;
	Template* tmpl;

	DISALLOW_COPY_AND_ASSIGN(WorkingTemplate);
};
}