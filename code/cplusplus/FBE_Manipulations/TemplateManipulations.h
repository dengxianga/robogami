#pragma once
#include "symbolic.h"
#include <NewPatch.h>
#include "newSnapping.h"

namespace FabByExample{
struct OverlapPreventionHack {
	// Filled in by CppCsBridge, which will eventually call a C# function to detect collision.
	static int(*countOverlapness)(drawing::Drawing const* drawing);

	// If true, unconditionally reflects all additional template's drawings in order to remove
	// overlap. This is a hack before a more robust way exists to decide whether reflecting
	// would remove overlaps.
	static bool reflectAllDrawings;
};

class TemplateManipulations {
public:
	// Snap the given additional template to the given main template.
	// Parameters:
	//  main - The main template that will not be affected.
	//  add - The additional template that we want to manipulate so that it snaps to the main.
	//  current - The current patch pair, from the previous snapping call. If there wasn't one,
	//            an empty patch pair can be passed in. See PatchPair.
	//  maxDistance - Maximum distance threshold allowed for snapping.
	//  connectingInfo - A structure that can later be passed to connect() to connect the two
	//                   templates.
	// Returns:
	//   The pair of patches that were snapped.
	static PatchPair Snap(Template* main, Template* add, PatchPair const& current,
		double maxDistance, ConnectingInfo& connectingInfo);

	// Snap the given additional template to the given main template, allowing multiple edges
	// to snap at the same time.
	// 
	// Parameters:
	//  main - The main template that will not be affected.
	//  add - The additional template that we want to manipulate so that it snaps to the main.
	//  maxDistance - Maximum distance threshold allowed for snapping.
	//  connectingInfo - A structure that can later be passed to connect() to connect the two
	//                   templates.
	static void MultiSnap(Template* main, Template* add, double maxDistance, ConnectingInfo* connectingInfo);

	// Connect the given two templates using the given connection info, and return the newly
	// composed root template after connecting.
	static Template* Connect(Template* main, Template* add, const ConnectingInfo& connectingInfo);

	// Given an additional template, and a list of patch pairs we want to connect, return a
	// maximal sublist of patch pairs such that no two patch pairs are connected through a
	// path of connections.
	static vector<PatchPair> ElectNonConnectingPatchPairs(vector<PatchPair> const& pairs, Template* addTemplate);
	static Template* ConnectWithGrammar(Template* main, Template* add); 
	static std::vector<Template*> SnappingWithGrammar(Template* main, Template* add); 
	static std::vector<NewPatch*> findClossestElement(Template* main, Template* add); 

	DISALLOW_COPY_AND_ASSIGN(TemplateManipulations);
};
}
