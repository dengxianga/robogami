extern "C" {
#include "../3dalignment_v1.8/ds.h"
#include "../3dalignment_v1.8/RegionShape.h"
#include "../3dalignment_v1.8/Circularity.h"
#include "../3dalignment_v1.8/FourierDescriptor.h"
#include "../3dalignment_v1.8/Eccentricity.h"
#include "../3dalignment_v1.8/edge.h"
}

#include <stdio.h>
#include <malloc.h>
#include <memory.h>
#include <string.h>
#include <limits.h>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include "featureGen.h"
#include "EmbreeIntersector.h"

#define NUMBER_TEMP_SAMPLES 1 
#define	QUANT8				256	
#define FD_SCALE			2



MLFFeatureGen::MLFFeatureGen(TriMesh * _mesh){
	//filename = _filename;
	mesh = _mesh;
	ei = new EmbreeIntersector(_mesh);
}

MLFFeatureGen::~MLFFeatureGen(){
	delete ei;
}



void MLFFeatureGen::lf_findCenter(unsigned char *srcBuff, int width, int height, double *CenX, double *CenY)
{
	int					x, y, count;
	unsigned char		*pImage;
	int					maxX, minX, maxY, minY;
	int					MeanX, MeanY; 

	count = 0;
	pImage = srcBuff;

	// ***********************************************************************************************
	// if use "mean" to each 2D shape independnetly, the origin will be moved a lot in 3D
	// if ues "center" to each 2D shape independnetly, the origin will be moved only a little in 3D
	// if center can be defined in 3D, the origin will not be moved any more.
	// But this will not very robust in 3D similarity transformation
	// In addition, to make center of each 2D shape more close to user drawn 2D shapes,
	// it's better to define center for each 2D shape independently

	// uee center of max and min to be center
	maxX = maxY = -1;
	minX = minY = INT_MAX;
	for (y=0 ; y<height ; y++)
	for (x=0 ; x<width; x++)
	{
		if( *pImage < 255 )
		{
			if( x > maxX ) maxX = x;
			if( x < minX ) minX = x;
			if( y > maxY ) maxY = y;
			if( y < minY ) minY = y;
		}
		pImage++;
	}

	if( maxX > 0 )
	{
		*CenX = (maxX+minX) / 2.0;
		*CenY = (maxY+minY) / 2.0;
	}
	else
		*CenX = *CenY = -1;		// nothing to be rendered

	// use me
//	an to be center
/*	count = 0;
	MeanX = MeanY = 0;
	for (y=0 ; y<height ; y++)
	for (x=0 ; x<width; x++)
	{
		if( *pImage < 255 )
		{
			MeanX += x;
			MeanY += y;
			count ++;
		}
		pImage++;
	}

	if( count > 0 )
	{
		*CenX = (double)MeanX / (double)count;
		*CenY = (double)MeanY / (double)count;
	}
	else
		*CenX = *CenY = -1;		// nothing to be rendered
*/
}


void MLFFeatureGen::getSrcBuff(unsigned char **srcBuff, int dim){
	//bool saveFile = false;
	int winw = WIDTH, winh = HEIGHT;
	int	*hitsBuff;
	hitsBuff = (int *) malloc (winw * winh * sizeof(int));
	ei->getHitsForDim(hitsBuff, dim, winw, winh);

	for(int level=0; level<CAMNUM; level++){
		//sprintf(filename, "F:\\projects\\FabByEx\\data\\RayTracedLevels\\%s_%d_%d.txt", fname, dim+1, level +1);
		//if( (fpt = fopen(filename, "rb")) == NULL )	{ printf("%s does not exist.\n", filename);}
		//fread(srcBuff[level], winw * winh, sizeof(unsigned char), fpt);
		//fclose(fpt);



		for (int iy = 0; iy < winh; iy++){
			#pragma omp parallel for
			for (int ix = 0; ix < winw; ix++){
				int index = iy*winw + ix; 
				unsigned char val;
				if(hitsBuff[index]/2 >= level+1){
					val = 0;
				}
				else{
					val = 255;
				}
				srcBuff[level][index]= val;
			}
		}

		/*
		if(saveFile){
			std::stringstream fullfilename;
			fullfilename << "F:\\projects\\FabByEx\\data\\levels\\"<< filename.c_str() << "_" << dim << "_" << level << ".ppm";
			std::string finalfilename = fullfilename.str();
			std::ofstream file(finalfilename);	
			if (!file.is_open()) {std::cout << "Unable to open file" << std::endl;}
			file << "P3" << std::endl;
			file << "# The P3 means colors are in ASCII, then 3 columns and 2 rows," << std::endl;
			file << "# then 255 for max color, then RGB triplets" << std::endl;
			file <<	winw << " " <<  winh << std::endl;
			file << 255 << std::endl;
			for (int iy = 0; iy < winh; iy++){
				for (int ix = 0; ix < winw; ix++){
					int index = iy*winw + ix; 
					int valInt;
					if(hitsBuff[index]/2 >= level+1){
						valInt =  0;
					}
					else{
						valInt = 255;
					}
					file << valInt << " " << valInt << " " << 255 << " ";
				}
				file << std::endl;
			}

			file.close();
		}*/
	}
	

	free(hitsBuff);

}


void MLFFeatureGen::getImageFeatures(Eigen::VectorXd & vec, bool isVol){
	int npixels =  32;
	vec = Eigen::VectorXd::Zero(npixels*npixels*3);
	for (int dim =0; dim <=2; dim++){
		Eigen::VectorXd auxvec (npixels*npixels);
		ei->saveMeadianHitsForDim(dim, npixels, npixels, auxvec, isVol);
		vec.segment(dim*(npixels*npixels), (npixels*npixels)) = auxvec;
	}
}

void MLFFeatureGen::getFullFeatures(Eigen::VectorXd & vec, const  Eigen::MatrixXd & PCA_TRANS){

	Eigen::VectorXd vec1;
	getImageFeatures(vec1, false);
	//std::cout << "vec1 = " <<   vec1.segment(500,10).transpose()<< std::endl;
	//std::cout << "vec1 size = [" <<  vec1.rows() << ", " << vec1.cols() << std::endl;	
	Eigen::VectorXd vec2;
	getImageFeatures(vec2, true);
	//std::cout << "vec2 = " <<  vec2.segment(500,10).transpose() << std::endl;
	//std::cout << "vec2 size = [" <<  vec2.rows() << ", " << vec2.cols() << std::endl;	
	vec = PCA_TRANS.leftCols(vec1.rows()) * vec1 + PCA_TRANS_NORM_FACTOR * PCA_TRANS.rightCols(vec2.rows()) * vec2;
	//std::cout << "vec size = [" <<  vec.rows() << ", " << vec.cols() << std::endl;	

}

/*
void MLFFeatureGen::genImageFeatures(std::ofstream &outfile, bool isVol){
	int npixels =  32;
	for (int dim =0; dim <=2; dim++){
		Eigen::VectorXd auxvec (npixels*npixels);
		ei->saveMeadianHitsForDim(dim, npixels, npixels, auxvec, isVol);

		// save to big file
		bool saveImagesFlag = true; 
		if(saveImagesFlag){
			std::stringstream fullfilename;
			if(isVol){
				fullfilename << "F:\\projects\\FabByEx\\data\\\imagesForPaper\\descriptor\\"<< filename.c_str() << "_dim_" << dim << "_volumeFeature_final.ppm";
			}else{
				fullfilename << "F:\\projects\\FabByEx\\data\\\imagesForPaper\\descriptor\\"<< filename.c_str() << "_dim_" << dim << "_imageFeature_final.ppm";
			}
			std::ofstream file(fullfilename.str());
			if (!file.is_open()) { std::cout << "Unable to open file2" << std::endl; }
			file << "P3" << std::endl;
			file << "# The P3 means colors are in ASCII, then 3 columns and 2 rows," << std::endl;
			file << "# then 255 for max color, then RGB triplets" << std::endl;
			file <<	npixels << " " <<  npixels << std::endl;
			int Nmax = 10;
			file <<	Nmax << std::endl;
			for (int i =0; i < npixels ; i++){
				for (int j =0; j < npixels ; j++){
					int val = round(auxvec( i* npixels + j ));
					if(val > Nmax)
						file << 0 << " " << 0 << " " << 0 << " ";
					else
 						file << Nmax -val << " " << Nmax -val << " " << Nmax -val << " ";
				}
				file << std::endl;
			}
		
		}

	//	//save little files
	//	for (int i =0; i < npixels ; i++){
	//		for (int j =0; j < npixels ; j++){
	//			outfile<< auxvec( i* npixels + j ) << " ";
	//		}
	//	}

	}
	//outfile << std::endl;
}



void MLFFeatureGen::genLightField(std::ofstream &outfile)
{

	char			fullfilename[400];
	double			CenX[CAMNUM], CenY[CAMNUM];
	unsigned char	*srcBuff[CAMNUM], *EdgeBuff;
	int			winw = WIDTH, winh = HEIGHT;
	// coefficients
	double			src_ArtCoeff[ANGLE][CAMNUM][ART_ANGULAR][ART_RADIAL];
	double			cir_Coeff[ANGLE][CAMNUM];
	unsigned char	dest_cirCoeff[ANGLE][CAMNUM];
	double			ecc_Coeff[ANGLE][CAMNUM];
	double			src_FdCoeff[ANGLE][CAMNUM][FD_COEFF_NO], dest_FdCoeff[ANGLE][CAMNUM][FD_COEFF_NO];



	FILE *fpt;

	int i, itmp, Count, sindex, srcCam, j, k, p, a, r, high, low, middle;
	int total;
	sPOINT * Contour;
	unsigned char	*ContourMask;



	// initialize ART
	GenerateBasisLUT();

	for(i=0; i<CAMNUM; i++)
	{
		srcBuff[i] = (unsigned char *) malloc (winw * winh * sizeof(unsigned char));
	}
	
	// add edge to test retrieval
	EdgeBuff = (unsigned char *) malloc (winw * winh * sizeof(unsigned char));

	// for Fourier Descriptor
	total = winw * winh;
	Contour = (sPOINT *) malloc( total * sizeof(sPOINT));
	ContourMask = (unsigned char *) malloc( total * sizeof(unsigned char));


	Count = 0;
			

	// get all INFO
	for( srcCam=0; srcCam<ANGLE; srcCam++)
	{

		getSrcBuff(srcBuff,  srcCam);	
			
		//std::cout<< "i = " << i << std::endl;
		//std::cout<< "..........." << std::endl;
		//// writing the buffer to check accurary
		//std::stringstream ppmfilename;
		//for(i=0; i<CAMNUM; i++){
		//	std::cout<< "i = " << i << std::endl;
		//	ppmfilename.str(std::string());
		//	ppmfilename << "F:\\projects\\FabByEx\\data\\results\\"<< fname << "_V2_" << srcCam << "_" << i << ".ppm";
		//	std::ofstream file(ppmfilename.str());	
		//	if (!file.is_open()) {std::cout << "Unable to open file" << std::endl;}
		//	file << "P3" << std::endl;
		//	file << "# The P3 means colors are in ASCII, then 3 columns and 2 rows," << std::endl;
		//	file << "# then 255 for max color, then RGB triplets" << std::endl;
		//	file <<	winw << " " <<  winh << std::endl;
		//	file << 255 << std::endl;
		//	for (int iy = 0; iy < winh; iy++){
		//		for (int ix = 0; ix < winw; ix++){
		//			int index = iy*winw + ix; 
		//			unsigned char val = srcBuff[i][index];
		//			int valInt;
		//			if(val < 255){
		//				valInt =  0;
		//			}
		//			else{
		//				valInt = 255;
		//			}
		//			file << valInt << " " << valInt << " " << 255 << " ";
		//		}
		//		file << std::endl;
		//	}
		//	file.close();
		//}


		for(i=0; i<CAMNUM; i++)
			lf_findCenter(srcBuff[i], winw, winh, CenX+i, CenY+i);

		// get Zernike moment
		FindRadius(srcBuff, CenX, CenY);
		for(i=0; i<CAMNUM; i++)
		{
			ExtractCoefficients(srcBuff[i], src_ArtCoeff[srcCam][i], EdgeBuff, CenX[i], CenY[i]);
		}

		// get Fourier descriptor
		for(i=0; i<CAMNUM; i++)
			FourierDescriptor(src_FdCoeff[srcCam][i], srcBuff[i], winw, winh, Contour, ContourMask, CenX[i], CenY[i]);

		// get eccentricity
		for(i=0; i<CAMNUM; i++)
			ecc_Coeff[srcCam][i] = Eccentricity(srcBuff[i], winw, winh, CenX[i], CenY[i]);

		// get circularity
		for(i=0; i<CAMNUM; i++)
		{
			EdgeDetectSil(EdgeBuff, srcBuff[i], winw, winh);
			cir_Coeff[srcCam][i] = Circularity(srcBuff[i], winw, winh, EdgeBuff);
		}
	}
		
	//Save all info
	//sprintf(fullfilename, "F:\\projects\\FabByEx\\data\\features\\%s_full_descriptorsF6.txt", filename.c_str());

	//fpt = fopen(fullfilename, "w" );

	// ECC Coeff
	for(srcCam=0; srcCam<ANGLE; srcCam++){
		for(i=0; i<CAMNUM; i++){
			//fprintf( fpt, "%f\n", 1*ecc_Coeff[srcCam][i]);
			outfile << ecc_Coeff[srcCam][i]<< " ";
		}
	}
	//  Cir coeff
	for(srcCam=0; srcCam<ANGLE; srcCam++){
		for(i=0; i<CAMNUM; i++){
			//fprintf( fpt, "%f\n", 1*cir_Coeff[srcCam][i]);
			outfile << cir_Coeff[srcCam][i] << " ";
		}
	}

	// ArtCoeff
	for(i=0; i<ANGLE; i++)
		for(j=0; j<CAMNUM; j++) {
			p = 0;
			for(r=1 ; r<ART_RADIAL ; r++){
				//fprintf( fpt, "%f\n", 1*src_ArtCoeff[i][j][p][r]);
				outfile << src_ArtCoeff[i][j][p][r] << " ";				
			}
			for(p=1; p<ART_ANGULAR ; p++){
				for(r=0 ; r<ART_RADIAL ; r++){
					//fprintf( fpt, "%f\n", 1*src_ArtCoeff[i][j][p][r]);
					outfile << src_ArtCoeff[i][j][p][r] << " ";				
				}
			}
		}
	for(i=0; i<ANGLE; i++){
		for(j=0; j<CAMNUM; j++){
			for(k=0; k<FD_COEFF_NO; k++){
				//fprintf( fpt, "%f\n", 1*FD_SCALE* src_FdCoeff[i][j][k]);
				outfile << FD_SCALE* src_FdCoeff[i][j][k] << " ";
			}
		}
	}
	//fclose(fpt);
	outfile << std::endl;	
	//printf(" Done. \n");
	

	for(i=0; i<CAMNUM; i++)
	{
		free(srcBuff[i]);
	}
	free(EdgeBuff);
	free(Contour);
	free(ContourMask);
}

*/