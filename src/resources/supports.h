#ifndef RTLBS_SUPPORTS
#define RTLBS_SUPPORTS

#include "rtlbs.h"
#include "utility/define.h"
#include <unordered_set>

//系统支持的几何模型扩展形状后缀名
const std::unordered_set<std::string> supportedExtensionsGeometry = {
	".3ds",  // 3D Studio Max
	".obj",  // Wavefront Object
	".fbx",  // Autodesk FBX
	".dae",  // Collada
	".stl",  // Stereolithography
	".ply",  // Polygon File Format or Stanford Triangle Format
	".dxf",  // AutoCAD DXF
	".lwo",  // LightWave
	".lws",  // LightWave Scene
	".ase",  // 3D Studio Max ASE
	".ac",   // AC3D
	".ms3d", // Milkshape3D
	".cob",  // TrueSpace (COB)
	".mdl",  // Quake I
	".md2",  // Quake II
	".md3",  // Quake III
	".pk3",  // Quake III, RTCW
	".mdc",  // Quake III, RTCW
	".md5",  // Doom 3
	".smd",  // Valve Model
	".vta",  // Valve Model
	".ogex", // Open Game Engine Exchange
	".3d",   // UNREAL 3D
	".b3d",  // Blitz3D
	".q3o",  // Quick3D Object
	".q3s",  // Quick3D Scene
	".raw",  // RAW Triangle Data
	".off",  // OFF
	".ter",  // Terragen Terrain
	".hmp",  // 3D GameStudio Terrain
	".ndo",  // 3D GameStudio Model
};

//系统支持地形扩展文件名称
const std::unordered_set<std::string> supportedExtensionsTerrain = {
	".asc", //Arc/Info ASCII Grid
	".adf", //Arc/Info Binary Grid
	".bil", //Band Interleaved by Line
	".bip", //Band Interleaved by Pixel
	".bsq", //Band Sequential
	".dat", //ENVI DAT File
	".dem", //USGS ASCII DEM 
	".dt0", //DTED Level 0 (ASCII)
	".dt1", //DTED Level 1 (ASCII)
	".dt2", //DTED Level 2 (ASCII)
	".dt3", //DTED Level 3 (ASCII)
	".ecw", //ERDAS Compressed Wavelets (ECW)
	".hdr", //ESRI BIL Header
	".hgt", //SRTM HGT Format
	".img", //ERDAS IMAGINE
	".jp2", //JPEG2000 driver based on OpenJPEG library
	".jpg", //JPEG JFIF
	".jpeg", //JPEG JFIF
	".jpe", //JPEG JFIF
	".lan", //Erdas Raw Languange File (.lan)
	".lcp", //FARSITE v.4 Landscape File (.lcp)
	".mem", //In Memory Raster
	".nc", //NetCDF
	".ntf", //National Transfer Format (NTF)
	".pix", //PCI Geomatics Database File
	".png", //Portable Network Graphics
	".raw", //ERDAS Raw
	".rda", //DigitalGlobe Raster Data Access driver (RDA)
	".rpf", //RPF - Raster Product Format
	".rst", //Idrisi Raster A.1 Format 
	".tif", //Tagged Image File Format
	".tiff" //Tagged Image File Format
};


#endif
