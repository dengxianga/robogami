#pragma once

#ifndef _PRINTABLE_H
#define _PRINTABLE_H

#include <stdio.h>
#include <list>
#include <Eigen/Dense>
#include "symbolic.h"
#include "simple_svg_1.0.0.h"

namespace FabByExample
{
	struct MotorMountLocations; //TODO: forward declaration okay?
	class FoldableNode;
	class KinChain;

	class Edge2dp{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Eigen::Vector2d v1;
		Eigen::Vector2d v2; 
		double offset;
		Edge2dp(){};
		
	};

	class PrintablePart {
	public:
		enum PrintablePartType{FOLD, FACE,TOUCH, TEETH, HINGE, BALLJOINT, PRISMATIC, FILL, MOTORMOUNT};
		PrintablePart(int _partid, FoldableNode* fn){partid = _partid; foldnode = fn;}
		virtual void print() =0;
		virtual void generateSVG(svg::Document& doc) =0;
		virtual void convertToMesh(std::string & hardStls, std::string & softStls, PROCESS_INFORMATION &pi, bool isAssembled = false) = 0;
		int getId() {return partid;}
		virtual PrintablePartType getType() = 0;
		virtual void shrinkAllVertices(double delta){}; 
		virtual void simplify() {};
		Eigen::Matrix4d get2Dto3Dtransform();
		FoldableNode* getFoldNode() {return foldnode;}

	protected:
		int partid;
		FoldableNode* foldnode;
	};	

	class PrintableFace : public PrintablePart {
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		std::vector<MotorMountLocations *> mountPatches;
		std::vector<Eigen::Vector2d *> servoPatches;
		PrintableFace(drawing::Face face, int id, TriMesh mesh, std::vector<MotorMountLocations *> mountPatches, std::vector<Eigen::Vector2d *> servoPatches, FoldableNode* fn);
		~PrintableFace();
		void convertToMesh(std::string & hardStls, std::string & softStls,PROCESS_INFORMATION &pi, bool isAssembled = false);
		void shrinkFace(int edgeID, double amount);
		Edge2dp * getUnshrinkFace(int edgeID, double amount);
		Edge2dp* getEdge(int edgeID);
		void generateSVG(svg::Document& doc);
		void print();
		PrintablePartType getType() { return PrintablePart::PrintablePartType::FACE; }
		void shrinkAllVertices(double delta);
		void simplify();

		void printEdgeInfo(int edgeID1) {
			std::cout<< face.edges[edgeID1].trimFront << " " << face.edges[edgeID1].trimBack << std::endl;
		}

	private:
		drawing::Face face;
		TriMesh triMesh;
		double thickness;
	};

	class PrintableMesh : public PrintablePart {
	public:
		PrintableMesh(int id, TriMesh mesh, FoldableNode* te);
		~PrintableMesh();
		void convertToMesh(std::string & hardStls, std::string & softStls,PROCESS_INFORMATION &pi, bool isAssembled = false);
		void generateSVG(svg::Document& doc);
		void print();
		PrintablePartType getType() { return PrintablePart::PrintablePartType::FACE; }
		void shrinkAllVertices(double delta);
		void simplify();


	private:
		TriMesh triMesh;
	};


	class PrintableFill : public PrintablePart {
	public:
		void print();
		void generateSVG(svg::Document& doc);
		PrintableFill(int id, Edge2dp* _edge1, Edge2dp* _auxedge1, FoldableNode* fn);
		~PrintableFill();
		void convertToMesh(std::string & hardStls, std::string & softStls,PROCESS_INFORMATION &pi, bool isAssembled = false);
		PrintablePartType getType() {return PrintablePart::PrintablePartType::FILL;}
		Edge2dp* edge1;
		Edge2dp* auxedge1;
		

	private:
		//drawing::Face auxFoldFace;
		double angle;

	};

	class MotorMount : public PrintablePart {
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		void print();
		void generateSVG(svg::Document& doc);
		MotorMount(int id, Eigen::Vector2d * dir, Eigen::Vector2d * point, double displacement);
		~MotorMount();
		void convertToMesh(std::string & hardStls, std::string & softStls,PROCESS_INFORMATION &pi, bool isAssembled = false);
		PrintablePartType getType() {return PrintablePart::PrintablePartType::MOTORMOUNT;}
		Eigen::Vector2d dir;
		Eigen::Vector2d point;
		double displacement;
	};


	class PrintableFold : public PrintablePart {
	public:
		void print();
		void generateSVG(svg::Document& doc);
		PrintableFold(int id, double _angle, Edge2dp* _edge1, Edge2dp* _edge2, Edge2dp* _auxedge1, Edge2dp* _auxedge2, FoldableNode* fn);
		~PrintableFold();
		void convertToMesh(std::string & hardStls, std::string & softStls,PROCESS_INFORMATION &pi, bool isAssembled = false);
		PrintablePartType getType() {return PrintablePart::PrintablePartType::FOLD;}
		//drawing::Face getFace(){ return auxFoldFace; }
		double getAngle(){ return angle; }
		Edge2dp* edge1;
		Edge2dp* edge2;
		Edge2dp* auxedge1;
		Edge2dp* auxedge2;
		

	private:
		//drawing::Face auxFoldFace;
		double angle;

	};


	class PrintableTeeth : public PrintablePart {
	public:
		PrintableTeeth(int id, double _anglel, Edge2dp* _edge1, Edge2dp* _edge2, Edge2dp* _auxedge1, Edge2dp* _auxedge2, FoldableNode* fn);
		~PrintableTeeth();
		void convertToMesh(std::string & hardStls, std::string & softStls,PROCESS_INFORMATION &pi, bool isAssembled = false);
		PrintablePartType getType() { return PrintablePart::PrintablePartType::TEETH; };
		Edge2dp* edge1;
		Edge2dp* edge2;
		Edge2dp* auxedge1;
		Edge2dp* auxedge2;
		void print();
		void generateSVG(svg::Document& doc);

	private:
		//drawing::Face auxFoldFace;
		double angle;
	};


	class PrintableTouch : public PrintablePart{
	public:
		PrintableTouch(int id, double _angle, Edge2dp* _edge1, Edge2dp* _edge2, Edge2dp* _auxedge1, Edge2dp* _auxedge2, FoldableNode* fn);
		~PrintableTouch();
		void generateSVG(svg::Document& doc);
		void convertToMesh(std::string & hardStls, std::string & softStls,PROCESS_INFORMATION &pi, bool isAssembled = false);
		PrintablePartType getType() { return PrintablePart::PrintablePartType::TOUCH; };
		Edge2dp* edge1;
		Edge2dp* edge2;
		Edge2dp* auxedge1;
		Edge2dp* auxedge2;
		void print();

	private:
		//drawing::Face auxFoldFace;
		double angle;

	};


	class PrintableHinge: public PrintablePart {
	public:
		void print();
		void generateSVG(svg::Document& doc);
		PrintableHinge(int id, double _angle, Edge2dp* _edge1, Edge2dp* _edge2, Edge2dp* _auxedge1, Edge2dp* _auxedge2, FoldableNode* fn);
		~PrintableHinge();
		void convertToMesh(std::string & hardStls, std::string & softStls,PROCESS_INFORMATION &pi, bool isAssembled = false);
		PrintablePartType getType() { return PrintablePart::PrintablePartType::HINGE; }
		//drawing::Face getFace(){ return auxFoldFace; }
		double getAngle(){ return angle; }
		Edge2dp* edge1;
		Edge2dp* edge2;
		Edge2dp* auxedge1;
		Edge2dp* auxedge2;
		

	private:
		//drawing::Face auxFoldFace;
		double angle;

	};

	class PrintableBallJoint: public PrintablePart {
	public:
		void print();
		void generateSVG(svg::Document& doc);
		PrintableBallJoint(int id, double _angle, Edge2dp* _edge1, Edge2dp* _edge2, Edge2dp* _auxedge1, Edge2dp* _auxedge2, FoldableNode* fn);
		~PrintableBallJoint();
		void convertToMesh(std::string & hardStls, std::string & softStls,PROCESS_INFORMATION &pi, bool isAssembled = false);
		PrintablePartType getType() { return PrintablePart::PrintablePartType::BALLJOINT; }
		//drawing::Face getFace(){ return auxFoldFace; }
		double getAngle(){ return angle; }
		Edge2dp* edge1;
		Edge2dp* edge2;
		Edge2dp* auxedge1;
		Edge2dp* auxedge2;

	private:
		double angle;

	};

	class PrintablePrismatic: public PrintablePart {
	public:
		void print();
		void generateSVG(svg::Document& doc);
		PrintablePrismatic(int id, double _angle, Edge2dp* _edge1, Edge2dp* _edge2, Edge2dp* _auxedge1, Edge2dp* _auxedge2, FoldableNode* fn);
		~PrintablePrismatic();
		void convertToMesh(std::string & hardStls, std::string & softStls,PROCESS_INFORMATION &pi, bool isAssembled = false);
		PrintablePartType getType() { return PrintablePart::PrintablePartType::PRISMATIC; }
		double getAngle(){ return angle; }
		Edge2dp* edge1;
		Edge2dp* edge2;
		Edge2dp* auxedge1;
		Edge2dp* auxedge2;

	private:
		double angle;

	};

	class PrintableDesign {
	private:
		std::vector<PrintablePart*> printableParts;
		
	public:
		std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> points;
		~PrintableDesign();
		void generateSVG(int width, int height, std::string path);
		void addPrintablePart(PrintablePart *pf);
		PrintableFace* getPrintableFaceByIndex(int index);
		void generatePrint(std::string destfilename, bool is3D, KinChain* kinchain, std::vector<double> times);
		void generatePrint(std::string destfilename = "result", bool is3D = false, KinChain* kinchain = NULL) {std::vector<double> times; times.push_back(0); this->generatePrint(destfilename, is3D, kinchain, times);}
		void shrinkDesign(double delta);
		void simplifyFaces();
	};

};
#endif
