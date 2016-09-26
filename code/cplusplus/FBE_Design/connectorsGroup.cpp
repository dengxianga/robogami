//#include "connectorsGroup.h"
//
//void ConnectorsGroup::updateScaleAndTranslate(){
//	Component * connectivecomp;
//	//int connectiveaxis;
//	bool isPlaneMin, isPlaneMin1, isPlaneMin2, ipMin1, ipMin2;
//
//	//if there are two parts: find the side that is more closely connected.
//	std::list<Component*>::iterator it,  it2;
//	for (it = parts.begin(); it != parts.end(); it++){
//		it2 = it;
//		it2++;
//		for (; it2 != parts.end(); it2++){
//			//std::cout << "component1: " << (*it)->getName() << std::endl; 
//			//std::cout << "component2: " << (*it2)->getName() << std::endl; 
//			TriMesh *m1  = (*it)->getGeometryInGlobalCoord()->getMesh();
//			m1->need_bbox();
//			TriMesh *m2  = (*it2)->getGeometryInGlobalCoord()->getMesh();
//			m2->need_bbox();
//			double mindist = 9999999;
//			int connectiveaxis =0;
//			for (int i=0; i<3; i++){
//				//std::cout << "i: " << i << std::endl; 
//				double m1min = m1->bbox.min[i];
//				double m1max = m1->bbox.max[i];
//				double m2min = m2->bbox.min[i];
//				double m2max = m2->bbox.max[i];
//				//std::cout << "m1min: " << m1min << std::endl; 
//				//std::cout << "m1max: " << m1max << std::endl; 
//				//std::cout << "m2min: " << m2min << std::endl; 
//				//std::cout << "m2max: " << m2max << std::endl; 
//				//std::cout << "abs(m1min - m2min): " << abs(m1min - m2min) << std::endl; 
//				double mindisti = abs(m1min - m2max);
//				isPlaneMin1 = true;
//				isPlaneMin2 = false;
//				if (abs(m1max - m2min) < mindisti) {
//					mindisti = abs(m1max - m2min);
//					isPlaneMin1 = false;
//					isPlaneMin2 = true;
//				}
//				//std::cout << "mindisti: " << mindisti << std::endl; 
//				if (mindisti < mindist){
//					connectiveaxis = i;
//					mindist = mindisti; 
//					ipMin1 = isPlaneMin1;
//					ipMin2 = isPlaneMin2;
//				//std::cout << "mindist: " << mindist << std::endl; 
//
//				}
//			}
//			double area1 = 1;
//			double area2 = 1;
//			for (int i=0; i<3; i++){
//				//std::cout << "area1[" << i << "] = " << (m1->bbox.max[i] - m1->bbox.min[i]) << std::endl; 
//				//std::cout << "area2[" << i << "] = " << (m2->bbox.max[i] - m2->bbox.min[i]) << std::endl; 
//				if (i != connectiveaxis){
//					area1 *= (m1->bbox.max[i] - m1->bbox.min[i]);
//					area2 *= (m2->bbox.max[i] - m2->bbox.min[i]);
//				}
//			}
//			//std::cout << "area1: " << area1 << std::endl; 
//			//std::cout << "area2: " << area2 << std::endl; 
//			//std::cout << "active axis: " << connectiveaxis << std::endl; 
//			connectivecomp = (*it);
//			isPlaneMin = ipMin1;
//			if (area1 > area2){
//				connectivecomp = (*it2);
//				isPlaneMin = ipMin2;
//			}
//			//std::cout << "active comp: " << connectivecomp->getName() << std::endl; 
//			//std::cout << "isPlan eMin: " << isPlaneMin << std::endl; 	
//			groupScale = connectivecomp->getTempScale();
//			groupTrans = connectivecomp->getTempTrans();
//			double nminX = connectivecomp->getGeometryInGlobalCoord()->getMesh()->bbox.min[0];
//			double nminY = connectivecomp->getGeometryInGlobalCoord()->getMesh()->bbox.min[1];
//			double nminZ = connectivecomp->getGeometryInGlobalCoord()->getMesh()->bbox.min[2];
//			orgPart = vector3f(nminX, nminY, nminZ); 
//			//std::cout << "orgPart: " << orgPart.vertex[0] << " | " << orgPart.vertex[1] <<  " | " << orgPart.vertex[2] << std::endl; 
//			//std::cout << "groupTrans: " << groupTrans.vertex[0] << " | " << groupTrans.vertex[1] <<  " | " << groupTrans.vertex[2] << std::endl; 
//			//std::cout << "groupScale: " << groupScale.vertex[0] << " | " << groupScale.vertex[1] <<  " | " << groupScale.vertex[2] << std::endl; 
//			groupScale.vertex[connectiveaxis] =1;
//			//groupTrans.vertex[connectiveaxis] = connectivecomp->getTempTrans().vertex[connectiveaxis];
//			if (!isPlaneMin){
//				double imax = connectivecomp->getGeometryInGlobalCoord()->getMesh()->bbox.max[connectiveaxis];
//				double imin = connectivecomp->getGeometryInGlobalCoord()->getMesh()->bbox.min[connectiveaxis];
//				double iscale = connectivecomp->getTempScale().vertex[connectiveaxis];
//				groupTrans.vertex[connectiveaxis] += (imax - imin)*(iscale -1);
//			}
//			//std::cout << "groupTrans: " << groupTrans.vertex[0] << " | " << groupTrans.vertex[1] <<  " | " << groupTrans.vertex[2] << std::endl; 
//			//std::cout << "groupScale: " << groupScale.vertex[0] << " | " << groupScale.vertex[1] <<  " | " << groupScale.vertex[2] << std::endl; 
//
//			
//		}
//	}
//
//	//if there are three parts: do it later
//}
//void ConnectorsGroup::resetTempConnectors(){
//	std::list<GNode*>::iterator it;
//	std::list<Component*>::iterator itc;
//	for (it = connectors.begin(); it != connectors.end(); it++){
//		std::list<Component*> comps = (*it)->getComponents();
//		for (itc = comps.begin(); itc != comps.end(); itc++){
//			Component * comp = (*itc);
//			std::cout << "-> " << comp->getId() << " - " << comp->getName()  << std::endl;
//			Geometry *m = new Geometry;
//			TriMesh *mp  = comp->getGeometryInGlobalCoord()->getMesh();
//			m->addMesh(mp);
//			m->mesh->need_bbox();
//
//			vector3f ctrans = groupTrans;
//			for (int i=0; i<3; i++){
//				if (hasCoplanarity(i, comps, parts) == false){
//					double di = m->mesh->bbox.min[i] - orgPart.vertex[i];
//					ctrans.vertex[i] += (groupScale.vertex[i] -1)*di;
//				}
//				//std::cout << "--> trans[" << i << "] = "<< ctrans.vertex[i] << std::endl;
//			}
//			//std::cout << "ctrans: " << ctrans.vertex[0] << " | " << ctrans.vertex[1] <<  " | " << ctrans.vertex[2] << std::endl; 
//			xform trans(1, 0, 0, 0,
//					0, 1, 0, 0,
//					0, 0, 1, 0,
//					ctrans.vertex[0] ,  ctrans.vertex[1] , ctrans.vertex[2] , 1);
//			apply_xform(m->mesh, trans);
//
//			//double tempBBox[6];
//			//tempBBox[0] = m->mesh->bbox.min[0] + ctrans.vertex[0];
//			//tempBBox[1] = m->mesh->bbox.min[1] + ctrans.vertex[1];
//			//tempBBox[2] = m->mesh->bbox.min[2] + ctrans.vertex[2];
//			//tempBBox[3] = m->mesh->bbox.max[0] - m->mesh->bbox.min[0];
//			//tempBBox[4] = m->mesh->bbox.max[1] - m->mesh->bbox.min[1];
//			//tempBBox[5] = m->mesh->bbox.max[2] - m->mesh->bbox.min[2];
//
//			//comp->setTemplBBBox(tempBBox);
//
//			//m->getMesh()->need_normals();
//			//m->getMesh()->need_tstrips();
//			//m->getMesh()->need_bsphere();
//
//			std::cout << "updating component " << comp->getName() << std::endl;
//
//			comp->setTempGeoGC(m);
//		}
//	}
//	
//}
//
//
//bool ConnectorsGroup::hasCoplanarity(int i, std::list<Component*> clist, std::list<Component*> plist){
//		std::list<Component*>::iterator itc, itp;
//		for (itp = plist.begin(); itp != plist.end(); itp++){
//			for (itc = clist.begin(); itc != clist.end(); itc++){
//				if ((*itp)->checkCoplanarity((*itc), i) == true){
//					return true;
//				}
//			}
//		}
//		return false;
//}