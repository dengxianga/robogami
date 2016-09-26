#include "element_openscad.h"
#include "geometry.h"
#include "template.h"
#include "templateElement.h"
#include <Windows.h>
#include <fstream>
#include <iostream>

namespace FabByExample {

	// Return the path to a temporary file given by Windows.
	string MakeTempFileName() {
		// Call the Windows API to get a temporary file.
		CHAR szTempFileName[MAX_PATH];
		CHAR lpTempPathBuffer[MAX_PATH];
		GetTempPathA(MAX_PATH, lpTempPathBuffer);
		GetTempFileNameA(lpTempPathBuffer, "fbe_openscad", 0, szTempFileName);
		return string(szTempFileName);
	}

	TriMesh* parseAsciiStl(istream& in) {
		TriMesh* tri = new TriMesh();
		string s;
		getline(in, s);
		int i = 0;
		while (true) {
			in >> s;
			if (s == "endsolid") {
				break;
			}
			if (s == "facet") {
				in >> s;
				vec norm;
				in >> norm[0] >> norm[1] >> norm[2];
				in >> s >> s;
				point vert[3];
				in >> s >> vert[0][0] >> vert[0][1] >> vert[0][2];
				in >> s >> vert[1][0] >> vert[1][1] >> vert[1][2];
				in >> s >> vert[2][0] >> vert[2][1] >> vert[2][2];
				in >> s >> s;
				tri->vertices.push_back(vert[0]);
				tri->vertices.push_back(vert[1]);
				tri->vertices.push_back(vert[2]);
				tri->normals.push_back(norm);
				tri->normals.push_back(norm);
				tri->normals.push_back(norm);
				tri->faces.push_back(TriMesh::Face(i, i + 1, i + 2));
				i += 3;
			}
		}
		return tri;
	}


	void Element_Openscad::computeNewGeo() {
		// Update the param values (kind of hacky here... don't really need to keep two copies)
		Template* tmpl = getRefTemplateElement();
		if (tmpl == nullptr) return;
		for (int i = 0; i < tmpl->numSymbols(); i++) {
			params[i].initValue = tmpl->getCurrentValue(i);
		}

		// Input file name (OpenSCAD file)
		string tempIn = MakeTempFileName();
		string tempOut = MakeTempFileName() + ".stl";

		// Write the input file.
		ofstream in(tempIn);
		for each (const auto& param in params) {
			// Add a line into the code that assigns each parameter.
			in << param.name << " = " << param.initValue << ";" << endl;
		}
		in << code;
		in.close();

		// Call the Windows API to invoke openscad.exe without spawning up a black window.
		STARTUPINFOA si = { 0 };
		si.cb = sizeof(si);
		si.dwFlags = STARTF_USESTDHANDLES | STARTF_USESHOWWINDOW;
		si.hStdInput = GetStdHandle(STD_INPUT_HANDLE);
		si.hStdOutput = GetStdHandle(STD_OUTPUT_HANDLE);
		si.hStdError = GetStdHandle(STD_ERROR_HANDLE);
		si.wShowWindow = SW_HIDE;  // No black window
		PROCESS_INFORMATION pi;

		CHAR cmd[MAX_PATH * 3];
		sprintf(cmd, "..\\bin\\openscad\\openscad.exe -o %s %s", tempOut.c_str(), tempIn.c_str());

		if (!CreateProcessA(
			NULL, cmd,
			NULL, NULL, TRUE, 0,
			NULL, NULL, &si, &pi)) {
			LOG(ERROR) << "Failed to call openscad.";
		}

		// Wait till the process exits, for a maximum of 20 seconds.
		DWORD waitResult = WaitForSingleObject(pi.hProcess, 20000);
		if (waitResult != WAIT_OBJECT_0) {
			if (waitResult == WAIT_TIMEOUT) {
				LOG(ERROR) << "Openscad took too long. Consider making the code simpler.";
			}
			else {
				LOG(ERROR) << "Failed to wait for openscad result: " << waitResult;
			}
		}

		// Read back the output of the OpenSCAD process.
		if (tempGeometry != nullptr) {
			delete tempGeometry;
		}
		tempGeometry = new Geometry();
		ifstream stlIn(tempOut);
		TriMesh* tri = parseAsciiStl(stlIn);
		stlIn.close();
		if (tri == nullptr) {
			return;
		}
		tempGeometry->addMesh(tri);
		tempGeometry->getMesh()->need_bbox();
	}


	::TriMesh::BBox Element_Openscad::evalBoundingBox() {
		return tempGeometry->getMesh()->bbox;
	}

	void Element_Openscad::modify(Element_Openscad* as) {
		// Change the code and params.
		this->code = as->code;
		this->params = as->params;

		// Make sure that the template also defines the same parameters. We
		// need to do this because the C# side could add or remove parameters.
		// Hacky.
		Template* tmpl = getRefTemplateElement();
		vector<string> paramNames;
		for each (const auto& param in params) {
			paramNames.push_back(param.name);
		}
		Eigen::VectorXd paramVals(params.size());
		for (int i = 0; i < params.size(); i++) {
			paramVals[i] = params[i].initValue;
		}
		tmpl->forceRedefineParams(paramNames, paramVals);
	}
}