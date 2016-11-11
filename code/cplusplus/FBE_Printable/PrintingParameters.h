#ifndef PrintingParameters_h
#define PrintingParameters_h


namespace FabByExample{

	class PrintingParameters
	{
	public:
		static bool isPrint(){return true;}
		static double getResolution(){
			if (isPrint())
				return 20;
			else
				return 6;
		}
		static bool useMultiMaterialFold(){return false;}
		static double getWallThickness(){return 1.0;}
		static double getTeethMulti(){return 2;}
		PrintingParameters(){} 
		static double getHingeR1(){ return 2.5; }
		static double getHingeShrinkAmount(){ return getHingeR1()+0.3; };
		static double getFoldHingeShrinkAmount(){ return 1.5+0.3; };
		static int getOrientInd(){ return 1; };
		static double getMinGap(){return 0.4;}
		static double getTeethShrinkAmount(double angle){
			std::cout << "Comptuing teeth shirk for angle = " << angle << std::endl;
			angle = angle*M_PI/180;
			if(angle == 0){
				//std::cout << "angle = " << angle << std::endl;
				//system("pause"); 
				angle = M_PI;
			}
			double amount;
			if(abs(angle -3.14) < 0.1)
				amount = PrintingParameters::getWallThickness();
			else
				amount = 2*PrintingParameters::getWallThickness()/tan(angle/2);
			return amount;
		}
		static double getJointShrinkAmount() {
			return 4.0;
		}
		static double getPrismaticShrinkAmount() {
			return 2.0; //TODO: update this value based on the prismatic joint model chosen
		}

	};

};



#endif