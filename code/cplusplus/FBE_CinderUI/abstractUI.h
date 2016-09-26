#include "design.h"
#include "template.h"
#include "ConstraintsEval.h"
#include "WorkingTemplate.h"
#include "drawing.h"
#include "mesh.h"
#include "ConnectingInfo.h"

using namespace std;
using namespace FabByExample;

class abstractUI {
public:
	abstractUI();
	// Get a map from each template element to Mesh.
	unordered_map<Template*, ::TriMesh*> getMeshes();
	drawing::Drawing abstractUI::getTopViewDrawing();


	// Get a map from each template element to Drawing.
	unordered_map<Template*, const drawing::Drawing*> getDrawings();

	WorkingTemplate* addTemplateByFilename(std::string filename, int id, int x, int y);
	
	WorkingTemplate* loadAdditionalProtoTemplateByFilename(std::string filename, int id);
	
	// WorkingTemplate, Template, TemplateElement
	vector<WorkingTemplate*> workingTemplates;

	ConnectingInfo connectingInfo;

	void handleClear();
	PatchPair handleSnap(PatchPair const& current, double maxDistance);
	void handleMultiSnap(double maxDistance);
	void handleConnect();
	void snapToGround() ;
	std::vector<NewPatch *> getClossestPatch(); 

	void generateFoldableSTL();

	void rotate(Template* tmpl, double x, double y, double z, double w, double centerX, double centerY, double centerZ);
	void translate(Template* tmpl, double x, double y, double z);
	void scale(TemplateElement* tmpl, int subElementID, int axis, double amount, PatchPair const& snappingConstraint, bool preventCollisions);
	void translatePart(TemplateElement* tmpl, int subElementID, int axis, double amount);
	double getStabilityDir(TemplateElement* tmpl, int subElementID, int axis, int gaitId, int objective);
	void generateStationary3DStl(); 

	void ensureAboveGround();
	bool SHOW_COMPOSITION;
	std::vector<string> loadedFileNames;
	void saveProto(std::string filename);
	DISALLOW_COPY_AND_ASSIGN(abstractUI);
};