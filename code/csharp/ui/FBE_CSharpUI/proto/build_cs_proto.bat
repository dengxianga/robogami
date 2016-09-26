copy ..\..\..\..\cplusplus\FBE_Proto\symbolic.proto . 
copy ..\..\..\..\cplusplus\FBE_Proto\template.proto . 
copy ..\..\..\..\cplusplus\FBE_Proto\openscad.proto .
protogen.exe -i:symbolic.proto -o:../SymbolicProto.cs
protogen.exe -i:template.proto -o:../TemplateProto.cs
protogen.exe -i:openscad.proto -o:../OpenscadDesignProto.cs
del symbolic.proto
del template.proto
del openscad.proto
pause