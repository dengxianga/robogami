# Robogami

## ORGANIZATION:
- code\cplusplus: C++ code for template manipulations and composition
- code\csharp: C# code for UI
- data: protos, stls, etc. used and produced by the system


## SETUP INSTRUCTIONS (for Windows):

### STEP 1: GIT
- Clone the repository: https://github.com/mit-drl/robogami
- Let XXX be the folder where you cloned the git repository

### STEP 2: DOWNLOAD ADDITIONAL SETUP FILES
- Download 7z files from:
  https://www.dropbox.com/sh/e9x7wmjvdh6pcl0/AAC7MNYK521Q_MIdCg3MGSX3a?dl=0

### STEP 3:  CPLEX
- Unzip the setup file CPLEX_Studio124.7z into XXX\code\cplusplus
- Add CPLEX to the path:
  
  Create these two Environment Variables:
    - Name: CPLEX_STUDIO_BINARIES124
      
	  Value: XXX\code\cplusplus\CPLEX_Studio124\cplex\bin\x86_win32

    - Name: CPLEX_STUDIO_DIR124
      
	  Value: XXX\code\cplusplus\CPLEX_Studio124
  
  Add or extend the PATH environment variable:
      
	- Name: PATH
      
	  Value: %CPLEX_STUDIO_DIR124%;%CPLEX_STUDIO_BINARIES124%

      If the PATH environment variable already exists, just extend it to include these two new variables.
  
- Restart Visual Studio and other applications for this change in the operating system to take effect. You might need to restart your computer.

### STEP 4: CINDER
- Unzip the setup file cinder_0.8.5_vc2010.7z and add it to C:

### STEP 5: OPENSCAD
- Install OpenSCAD
- Add OpenSCAD to the path:
  
  Create a new Environment Variable:
    - Name: SCAD_PATH
      
	  Value: path to openscad.exe (e.g., C:\Program Files\OpenSCAD)
  
  Add or extend the PATH environment variable:
    - Name: PATH
      
	  Value: %SCAD_PATH%