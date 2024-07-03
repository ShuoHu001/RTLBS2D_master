#include "terrain.h"
#include "tree/raypath3d.h"


TerrainCell::TerrainCell()
	: m_mat(nullptr)
{
}

TerrainCell::TerrainCell(const TerrainCell& cell)
	: m_mat(cell.m_mat)
	, m_cornerPoint(cell.m_cornerPoint)
{
	int segmentSize = static_cast<int>(cell.m_segments.size());
	int facetSize = static_cast<int>(cell.m_facets.size());
	if (segmentSize != 0) 
		m_segments.resize(segmentSize);
	if (facetSize != 0) 
		m_facets.resize(facetSize);
	for (int i = 0; i < segmentSize; ++i)		//复制地形线段
		m_segments[i] = cell.m_segments[i];		
	for (int i = 0; i < facetSize; ++i)			//复制地形面元
		m_facets[i] = cell.m_facets[i];
}

TerrainCell::~TerrainCell()
{
}

bool TerrainCell::GetIntersectEdgeFacet(Ray2D*& ray2d, Ray3DLite* ray3d, TerrainFacet* curFacet, TerrainFacet*& outFacet) const
{
	RtLbsType tmin = FLT_MAX;											/** @brief	存储距离最大值	*/
	RtLbsType cur_t = FLT_MAX;											/** @brief	当前存储的距离值	*/
	TerrainFacet* nearestFacet = nullptr;								/** @brief	距离最近的面元	*/
	outFacet = nullptr;													//输出面元初值定义
	for (auto it = m_facets.begin(); it != m_facets.end(); ++it) {
		TerrainFacet* facet = *it;
		if (facet->m_isEdgeFacet == false)
			continue;
		if (facet->m_facetId == curFacet->m_facetId)
			continue;
		//计算三角面元与二维射线相交（寻找最近的面元）
		if (facet->HasEdgeSegmentIntersect(ray2d, cur_t)) {
			if (cur_t < tmin) {
				tmin = cur_t;
				nearestFacet = facet;
			}
		}
	}
	if (tmin < FLT_MAX) {
		outFacet = nearestFacet;										//最近地形面元赋值
		ray2d->m_Ori = ray2d->GetRayCoordinate(tmin);					//更新二维射线坐标
		return true;
	}
	return false;
}



Terrain::Terrain()
{
	_init();
}

Terrain::Terrain(const Terrain& terrain)
	: m_terrainId(terrain.m_terrainId)
	, m_category(terrain.m_category)
	, m_bbox3d(terrain.m_bbox3d)
	, m_propagationProperty(terrain.m_propagationProperty)
	, m_meshes(terrain.m_meshes)
	, m_voxelCount(terrain.m_voxelCount)
	, m_calcMode(terrain.m_calcMode)
	, m_simplifyFlag(terrain.m_simplifyFlag)
	, m_simplifyRate(terrain.m_simplifyRate)
	, m_bbox2d(terrain.m_bbox2d)
	, m_centerPosition(terrain.m_centerPosition)
	, m_averageRidgeGap(terrain.m_averageRidgeGap)
{
	//进行初始参数的拷贝
	for (int i = 0; i < 2; ++i) {
		m_voxelNum[i] = terrain.m_voxelNum[i];
		m_voxelExtent[i] = terrain.m_voxelExtent[i];
		m_voxelInvExtent[i] = terrain.m_voxelInvExtent[i];
	}

	//进行点集的拷贝
	int pSize = static_cast<int>(terrain.m_pointBuf.size());
	m_pointBuf.resize(pSize);
	for (int i = 0; i < pSize; ++i) {
		m_pointBuf[i] = new Point3D(*terrain.m_pointBuf[i]);
	}

	//进行线段的拷贝
	int sSize = static_cast<int>(terrain.m_segmentBuf.size());
	m_segmentBuf.resize(sSize);	
	unsigned psId, peId;														/** @brief 线段起始点和终止点的ID	*/
	for (int i = 0; i < sSize; ++i) {
		m_segmentBuf[i] = new TerrainSegment(*terrain.m_segmentBuf[i]);
		psId = terrain.m_segmentBuf[i]->m_ps->m_pId;							/** @brief	线段起始点ID	*/
		peId = terrain.m_segmentBuf[i]->m_pe->m_pId;							/** @brief	线段终止点ID	*/
		m_segmentBuf[i]->m_ps = m_pointBuf[psId];								/** @brief	起始点坐标地址赋值	*/
		m_segmentBuf[i]->m_pe = m_pointBuf[peId];								/** @brief	终止点坐标地址赋值	*/
	}

	//进行面元的拷贝
	int fSize = static_cast<int>(terrain.m_facetBuf.size());
	m_facetBuf.resize(fSize);
	unsigned p1Id, p2Id, p3Id;													/** @brief	三角形三个顶点的ID	*/
	for (int i = 0; i < fSize; ++i) {
		m_facetBuf[i] = new TerrainFacet(*terrain.m_facetBuf[i]);
		p1Id = terrain.m_facetBuf[i]->m_p1->m_pId;								/** @brief	三角形顶点1 ID	*/
		p2Id = terrain.m_facetBuf[i]->m_p2->m_pId;								/** @brief	三角形顶点2 ID	*/
		p3Id = terrain.m_facetBuf[i]->m_p3->m_pId;								/** @brief	三角形顶点3 ID	*/
		m_facetBuf[i]->m_p1 = m_pointBuf[p1Id];									/** @brief	三角形顶点1地址赋值	*/
		m_facetBuf[i]->m_p2 = m_pointBuf[p2Id];									/** @brief	三角形顶点2地址赋值	*/
		m_facetBuf[i]->m_p3 = m_pointBuf[p3Id];									/** @brief	三角形顶点3地址赋值	*/
	}

	//映射线段与面元关系
	unsigned facetId;															/** @brief	面元ID	*/
	for (int i = 0; i < sSize; ++i) {
		if (terrain.m_segmentBuf[i]->m_facet1 != nullptr) {
			facetId = terrain.m_segmentBuf[i]->m_facet1->m_facetId;				/** @brief	邻接面元1 ID	*/
			m_segmentBuf[i]->m_facet1 = m_facetBuf[facetId];					/** @brief	邻接面元1地址赋值	*/
		}
		if (terrain.m_segmentBuf[i]->m_facet2 != nullptr) {
			facetId = terrain.m_segmentBuf[i]->m_facet2->m_facetId;				/** @brief	邻接面元2 ID	*/
			m_segmentBuf[i]->m_facet2 = m_facetBuf[facetId];					/** @brief	邻接面元2地址赋值	*/
		}
	}

	//映射面元与线段间关系
	unsigned segmentId;															/** @brief	线段ID	*/
	for (int i = 0; i < fSize; ++i) {
		if (terrain.m_facetBuf[i]->m_segment1 != nullptr) {
			segmentId = terrain.m_facetBuf[i]->m_segment1->m_segmentId;			/** @brief	邻接线段1 ID	*/
			m_facetBuf[i]->m_segment1 = m_segmentBuf[segmentId];				/** @brief	邻接线段1地址赋值	*/
		}
		if (terrain.m_facetBuf[i]->m_segment2 != nullptr) {
			segmentId = terrain.m_facetBuf[i]->m_segment2->m_segmentId;			/** @brief	邻接线段2 ID	*/
			m_facetBuf[i]->m_segment2 = m_segmentBuf[segmentId];				/** @brief	邻接线段2地址赋值	*/
		}
		if (terrain.m_facetBuf[i]->m_segment3 != nullptr) {
			segmentId = terrain.m_facetBuf[i]->m_segment3->m_segmentId;			/** @brief	邻接线段3 ID	*/
			m_facetBuf[i]->m_segment3 = m_segmentBuf[segmentId];				/** @brief	邻接线段3地址赋值	*/
		}
	}

	//进行gridcell的拷贝
	unsigned gSize = static_cast<unsigned>(terrain.m_gridCells.size());
	unsigned gFacetId = 0, gSegmentId = 0;							/** @brief	cell中的面元ID、cell中的线段ID	*/
	m_gridCells.resize(gSize);
	for (unsigned i = 0; i < gSize; ++i) {
		m_gridCells[i] = new TerrainCell(*terrain.m_gridCells[i]);
		for (unsigned j = 0; j < terrain.m_gridCells[i]->m_facets.size(); ++j) {
			gFacetId = terrain.m_gridCells[i]->m_facets[j]->m_facetId;
			m_gridCells[i]->m_facets[j] = m_facetBuf[gFacetId];
		}
		for (unsigned j = 0; j < terrain.m_gridCells[i]->m_segments.size(); ++j) {
			gSegmentId = terrain.m_gridCells[i]->m_segments[j]->m_segmentId;
			m_gridCells[i]->m_segments[j] = m_segmentBuf[gSegmentId];
		}
	}
}

Terrain::~Terrain()
{
	_release();
}

bool Terrain::Init(const TerrainConfig& config, const MaterialLibrary& matLibrary)
{
	m_category = config.m_category;																					//地形类型赋值
	m_simplifyFlag = config.m_simplifyFlag;																			//地形简化标志赋值
	m_simplifyRate = config.m_simplifyRate;																			//地形简化率赋值
	m_averageRidgeGap = config.m_averageRidgeGap;																	//地形平均峰峦间距赋值
	m_propagationProperty = config.m_propagationProperty;															//地形传播属性赋值
	if (config.m_loadingMode == TERRAIN_GRID) {				//栅格模式读取
		//基于GDAL进行数据读取
		const std::string& fileName = config.m_gridConfig.m_heightMatrixFile;
		if (fileName.empty())
			return false;
		std::string extension = GetFileExtension(fileName);
		if (supportedExtensionsTerrain.find(extension) != supportedExtensionsTerrain.end()) {						//检查文件扩展名是否满足既定的格式要求
			GDALDataset* dataset = static_cast<GDALDataset*>(GDALOpen(fileName.c_str(), GA_ReadOnly));				//打开地形文件
			if (!dataset)
				return false;																						//数据打开失败
			GDALRasterBand* band = dataset->GetRasterBand(1);														//默认高程数据在波段1中
			RtLbsType minmaxValue[2];																				/** @brief	高程最大最小值	*/
			band->ComputeRasterMinMax(0, minmaxValue);
			int rows = band->GetYSize();
			int cols = band->GetXSize();
			float* elevation = new float[rows * cols];
			band->RasterIO(GF_Read, 0, 0, cols, rows, elevation, cols, rows, GDT_Float32, 0, 0);					//获取高程值
			double geoTransform[6];
			dataset->GetGeoTransform(geoTransform);																	//获取地理变换信息
			RtLbsType colGap = abs(geoTransform[1]);																/** @brief	X 方向间隔	*/
			RtLbsType rowGap = abs(geoTransform[5]);																/** @brief	Y 方向间隔	*/
			//读取地形对应的材质信息，这里给出默认值
			std::vector<Material*> matMatrix(rows * cols);
			for (auto& mat : matMatrix) {
				mat = matLibrary.GetDefaultMaterial();																//这里给定材质的默认值
			}
			//基于数据初始化terrain
			this->_initData(elevation, matMatrix, rows, cols, rowGap, colGap, minmaxValue[0], minmaxValue[1], 1.0);
			GDALClose(dataset);																						//关闭GDAL数据流
			//释放无关内存
			delete[] elevation;
			matMatrix.clear();
			std::vector<Material*>().swap(matMatrix);
			return true;
		}
		return false;
	}
	else if (config.m_loadingMode == TERRAIN_OBJECT) {	//物体模式读取
		//基于assimp进行数据读取
		const TerrainObjectConfig& objectConfig = config.m_objectConfig;
		const std::string& fileName = objectConfig.m_fileName;
		if (fileName.empty())
			return false;
		std::string extension = GetFileExtension(fileName);
		if (supportedExtensionsGeometry.find(extension) != supportedExtensionsGeometry.end()) {						//检查文件名是否满足物体扩展名要求
			Assimp::Importer importer;
			const aiScene* scene = importer.ReadFile(fileName, aiProcess_Triangulate | aiProcess_CalcTangentSpace);
			if (scene == nullptr)
				return false;
			//在文件中读取mesh对应的材质信息
			const std::vector<std::string>& matNames = objectConfig.m_matNames;
			std::vector<Material*> terrainMaterials(matNames.size());
			for (unsigned i = 0; i < matNames.size(); ++i) {
				terrainMaterials[i] = matLibrary.GetMaterial(matNames[i]);
			}
			this->_initData(scene, 1.0, terrainMaterials);
			return true;
		}
		return false;
	}
	return false;
}

void Terrain::Update(const Vector3D& offset)
{
	m_centerPosition += offset;

	//点集更新
	for (auto it = m_pointBuf.begin(); it != m_pointBuf.end(); ++it) {
		Point3D* p = *it;
		*p += offset;				//增加位移变换
	}

	//线段更新
	for (auto it = m_segmentBuf.begin(); it != m_segmentBuf.end(); ++it) {
		TerrainSegment* segment = *it;
		segment->Update();
	}

	//面元更新
	for (auto it = m_facetBuf.begin(); it != m_facetBuf.end(); ++it) {
		TerrainFacet* facet = *it;
		facet->Update();
	}
}

void Terrain::Update(const Euler& posture)
{
	//按照物体几何中心作为旋转中心
	
	//点集信息
	for (auto it = m_pointBuf.begin(); it != m_pointBuf.end(); ++it) {
		Point3D* p = *it;
		*p -= m_centerPosition;												//位移至“中心原点”
		*p *= posture;														//进行旋转操作
		*p -= m_centerPosition;												//反位移至目标点处
	}

	//线段更新
	for (auto it = m_segmentBuf.begin(); it != m_segmentBuf.end(); ++it) {
		TerrainSegment* segment = *it;
		segment->Update();
	}

	//面元更新
	for (auto it = m_facetBuf.begin(); it != m_facetBuf.end(); ++it) {
		TerrainFacet* facet = *it;
		facet->Update();
	}

}

void Terrain::Update(const Vector3D& offset, const Euler& posture)
{
	Vector3D extraOffset = offset + m_centerPosition;
	//坐标更新
	for (auto it = m_pointBuf.begin(); it != m_pointBuf.end(); ++it) {
		Point3D* p = *it;
		*p -= m_centerPosition;												//位移至物体中心坐标原点
		*p *= posture;														//进行旋转操作
		*p += extraOffset;													//进行累积位移
	}

	m_centerPosition += offset;

	//线段更新
	for (auto it = m_segmentBuf.begin(); it != m_segmentBuf.end(); ++it) {
		TerrainSegment* segment = *it;
		segment->Update();
	}

	//面元更新
	for (auto it = m_facetBuf.begin(); it != m_facetBuf.end(); ++it) {
		TerrainFacet* facet = *it;
		facet->Update();
	}
}

RtLbsType Terrain::GetObjectFoundationHeight(const Object2D* object) const
{
	//1-获取二维坐标点所在的三角形
	RtLbsType foundationHeight = FLT_MAX;
	for (int i = 0; i > object->m_segments.size(); ++i) {
		const Point2D& edgePoint = object->m_segments[i]->m_ps;
		TerrainFacet* facet = _getTerrainFacetViaPoint(edgePoint);
		if (facet == nullptr) {
			continue;
		}
		//获得三角形上的高度值
		RtLbsType facetHeight = facet->GetFacetHeightViaPoint(edgePoint);
		if (facetHeight < foundationHeight) {								//求解最小值
			foundationHeight = facetHeight;
		}
	}
	
	return foundationHeight;

}

bool Terrain::_simplify(double ratio)
{
	if (!CGAL::is_triangle_mesh(m_meshes)) {
		LOG_ERROR << "Terrain: failed to simplify, not a triangle mesh" << ENDL;
		return false;
	}
	if (ratio <= 0.0 || ratio >= 1.0) {
		LOG_ERROR << "Terrain: failed to simplify, out of range." << ENDL;
		return false;
	}
	SMS::Count_ratio_stop_predicate<SurfaceMesh> stop(ratio);
	int num_removed_edges = SMS::edge_collapse(m_meshes, stop);
	LOG_INFO << "Terrain: simplify success." << ENDL;
	return true;
}

void Terrain::write_OBJ(std::string& filename)
{
	std::ofstream outFile(filename);
	if (!outFile.is_open()) {
		LOG_WARNING << "Terrain: " <<"Error opening file "<< filename << " for writing." << ENDL;
		return;
	}
	bool isSuccess = CGAL::IO::write_OBJ(outFile, m_meshes);
	if (!isSuccess) {
		LOG_WARNING << "Terrain: " << filename << " : failed to write mesh to file." << ENDL;
	}
	LOG_INFO << "Terrain: write OBJ file complete." << ENDL;
	outFile.close();
}

void Terrain::write_STL(std::string& filename)
{
	std::ofstream outFile(filename);
	if (!outFile.is_open()) {
		LOG_WARNING << "Terrain: " << "Error opening file " << filename << " for writing." << ENDL;
		return;
	}
	bool isSuccess = CGAL::IO::write_STL(outFile, m_meshes);
	if (!isSuccess) {
		LOG_WARNING << "Terrain: " << filename << " : failed to write mesh to file." << ENDL;
		return;
	}
	LOG_INFO << "Terrain: write STL file complete." << ENDL;
	outFile.close();
}

void Terrain::write_GOCAD(std::string& filename)
{
	std::ofstream outFile(filename);
	if (!outFile.is_open()) {
		LOG_WARNING << "Terrain: " << "Error opening file " << filename << " for writing." << ENDL;
		return;
	}
	bool isSuccess = CGAL::IO::write_GOCAD(outFile, m_meshes);
	if (!isSuccess) {
		LOG_WARNING << "Terrain: " << filename << " : failed to write mesh to file." << ENDL;
		return;
	}
	LOG_INFO << "Terrain: write GOCAD file complete." << ENDL;
	outFile.close();
}

void Terrain::write_PLY(std::string& filename)
{
	std::ofstream outFile(filename);
	if (!outFile.is_open()) {
		LOG_WARNING << "Terrain: " << "Error opening file " << filename << " for writing." << ENDL;
		return;
	}
	bool isSuccess = CGAL::IO::write_PLY(outFile, m_meshes);
	if (!isSuccess) {
		LOG_WARNING << "Terrain: " << filename << " : failed to write mesh to file." << ENDL;
		return;
	}
	LOG_INFO << "Terrain: write PLY file complete." << ENDL;
	outFile.close();
}

bool Terrain::GetIntersect(Ray3DLite* rayInit, Point3D* intersectPoint) const
{
	Ray2D* ray2d = new Ray2D();
	ray2d->m_Ori.x = rayInit->m_ori.x;
	ray2d->m_Ori.y = rayInit->m_ori.y;
	ray2d->m_Dir.x = rayInit->m_dir.x;
	ray2d->m_Dir.y = rayInit->m_dir.y;
	ray2d->m_Dir.Normalize(); //赋值后需要归一化数值
	//射线与地形相交目前为三种相交模式 grid模式、mesh模式和孔洞模式
	if (m_category == GRIDCELL) {						//计算模式为均匀网格模式
		RtLbsType maxt;
		RtLbsType curt = m_bbox2d.Intersect(*ray2d, &maxt);
		if (curt < 0.0)
			return false;																		//射线起点在包围盒外部
		if (curt == maxt)
			curt = 0.0;																			//射线在包围盒内部，距离置零

		int curGrid[2], dir[2];																	//确定射线起点在哪个网格内
		RtLbsType delta[2], next[2];
		for (int i = 0; i < 2; ++i) {
			curGrid[i] = _point2VoxelId((*ray2d)(curt), i);
			dir[i] = (ray2d->m_Dir[i] > 0.0) ? 1 : -1;
			if (abs(ray2d->m_Dir[i]) >= EPSILON)
				delta[i] = abs(m_voxelExtent[i] / ray2d->m_Dir[i]);
			else
				delta[i] = FLT_MAX;
		}

		Point2D gridCorner = _voxelId2Point(curGrid);
		for (int i = 0; i < 2; ++i) {
			//获得下一个t值
			RtLbsType target = gridCorner[i] + ((dir[i] + 1) >> 1) * m_voxelExtent[i];
			next[i] = (target - ray2d->m_Ori[i]) / ray2d->m_Dir[i];
		}

		//遍历网格
		const unsigned array[] = { 0, 0, 1, 1 };												//[0],[3]不可能相交
		while ( curt < maxt ) {

			//得到体素ID
			unsigned voxelId = _offset(curGrid[0], curGrid[1]);

			//计算下一个t值
			unsigned nextAxis = (next[0] <= next[1]) + ((unsigned)(next[1] <= next[0])) * 2;
			nextAxis = array[nextAxis];

			if (_getIntersect(rayInit, voxelId)) {
				delete ray2d;
				return true;
			}

			// get to the next voxel
			curGrid[nextAxis] += dir[nextAxis];

			if (curGrid[nextAxis] < 0 || (unsigned)curGrid[nextAxis] >= m_voxelNum[nextAxis]) { //数组越界后依然没有交点
				delete ray2d;
				return false;
			}

			// update next
			curt = next[nextAxis];
			next[nextAxis] += delta[nextAxis];
		}
		delete ray2d;
		return false;
	}
	else if (m_category == MESHNONHOLE) {					//计算模式为常规半边模式
		TerrainFacet* curFacet;																	/** @brief	当前运算到的面元	*/
		TerrainSegment* curSegment = nullptr;													/** @brief	当前运算到的线段	*/
		TerrainSegment* prevSegment = nullptr;													/** @brief	上一个运算到的线段	*/
		RtLbsType t_cur;																		/** @brief	当前射线与线段相交的距离二维	*/
		RtLbsType h_cur;																		/** @brief	当前射线与线段相交的交点高度（用于比较是否与射线相交）	*/
		curFacet = _getTerrainFacetViaPoint(ray2d->m_Ori);										//初始化面元
		if (ray2d->m_Dir.x == 0.0 && ray2d->m_Dir.y == 0.0) {
			delete ray2d;
			return curFacet->GetIntersect(rayInit);
		}

		while (true) {
			prevSegment = curSegment;
			curSegment = curFacet->GetIntersectSegment(ray2d, prevSegment, &t_cur, &h_cur);		//求解射线与面元内部的相交线段
			if (curSegment == nullptr)
				return false;		//迭代到无法解决的极限误差问题，放弃追踪该条射线
			RtLbsType rayHeight = rayInit->GetRayHeight(t_cur);
			if (rayHeight <= h_cur || (rayHeight - h_cur) <= 10) {								//射线和线段的高度差在10m以内，采用射线与三角面元求交模式求解精确解
				if (curFacet->GetIntersect(rayInit)) {								//计算射线与当前面元的相交结果
					delete ray2d;
					return true;
				}
			}
			curFacet = curSegment->GetAdjacentFacet(curFacet);
			if (curFacet == nullptr)															//当遍历到面元为空时（表明运算到了目标边界）
				break;
		}
		delete ray2d;
		return false;
	}
	else if (m_category == MESHHOLE) {					//计算模式为网格+半边混合模式
		//确定起点所在的网格单元
		//迭代至具有面元的网格，并确定射线起点在哪个三角形中
		//开始进行半边迭代，计算是否相交
		//检测到是否到达半边边界和网格边界（不相交）
		RtLbsType maxt;
		RtLbsType curt = m_bbox2d.Intersect(*ray2d, &maxt);										//射线与地形包围盒相交的距离
		if (curt < 0.0) {																		//射线起点在包围盒外部，需要求解出射线与包围盒的交点所在的点值
			ray2d->m_Ori = ray2d->GetRayCoordinate(curt);										//更新射线在网格边界上的起点坐标
			curt = 0.0;																			//距离置零
		}
		if (curt == maxt)
			curt = 0.0;																			//射线在包围盒内部，距离置零
		//求解起点所在的网格单元
		int curGrid[2];																			/** @brief	起点所在的单元	*/
		int dir[2];																				/** @brief	射线起点的方向	*/
		RtLbsType delta[2];																		/** @brief	下一体素所需要的增量	*/
		RtLbsType next[2];																		/** @brief	确定下一个体素网格的所需要的变量	*/
		for (int i = 0; i < 2; ++i) {
			curGrid[i] = _point2VoxelId((*ray2d)(curt), i);
			dir[i] = (ray2d->m_Dir[i] > 0.0) ? 1 : -1;
			if (abs(ray2d->m_Dir[i]) >= EPSILON)
				delta[i] = abs(m_voxelExtent[i] / ray2d->m_Dir[i]);
			else
				delta[i] = FLT_MAX;
		}
		Point2D gridCorner = _voxelId2Point(curGrid);
		for (int i = 0; i < 2; ++i) {
			//获得下一个t值
			RtLbsType target = gridCorner[i] + ((dir[i] + 1) >> 1) * m_voxelExtent[i];
			next[i] = (target - ray2d->m_Ori[i]) / ray2d->m_Dir[i];
		}

		const unsigned array[] = { 0, 0, 1, 1 };												//[0],[3]不可能相交
		while (curt < maxt) {

			//得到体素ID
			unsigned voxelId = _offset(curGrid[0], curGrid[1]);

			//计算下一个t值
			unsigned nextAxis = (next[0] <= next[1]) + ((unsigned)(next[1] <= next[0])) * 2;
			nextAxis = array[nextAxis];
			unsigned targetVoxelId = 0;
			//先判定是否求交，并返回半边迭代结束后的索引
			if (_getIntersect(rayInit,ray2d,curt,voxelId,targetVoxelId)) {																			//基于半边法求解是否与地形面元相交，若相交则返回真
				delete ray2d;
				return true;
			}
			//半边法不相交，返回边界网格ID
			_offset_reverse(targetVoxelId, curGrid[0], curGrid[1]);


			// get to the next voxel
			curGrid[nextAxis] += dir[nextAxis];

			if (curGrid[nextAxis] < 0 || (unsigned)curGrid[nextAxis] >= m_voxelNum[nextAxis]) { //数组越界后依然没有交点
				delete ray2d;
				return false;
			}

			// update next
			curt = next[nextAxis];
			next[nextAxis] += delta[nextAxis];
		}
		delete ray2d;
		return false;

	}
	delete ray2d;
	return false;
}

bool Terrain::IsBlock(const Point3D& ps, const Point3D& pe) const
{
	//构造三维射线，判定是否相交
	Ray3DLite* ray3d = new Ray3DLite(ps, pe);					//构造三维射线
	Point3D intersect;											//交点
	if (GetIntersect(ray3d, &intersect)) {						
		Vector3D se = pe - ps;
		Vector3D si = intersect - ps;
		if (si.Length()<se.Length())							//若交点长度小于起始点长度，则交点有效，被环境遮挡
			return true;
	}
	return false;
}

Material* Terrain::GetMaterial(const Point3D& p) const
{
	//计算出p点在cell中的栅格编号
	//进行相关计算
	unsigned voxelId[2];						//体素Id
	for (unsigned i = 0; i < 2; ++i) {
		voxelId[i] = _point2VoxelId(p, i);
	}
	unsigned offset = _offset(voxelId[0], voxelId[1]);		//体素Id在一维数据中的编号
	return m_gridCells[offset]->m_mat;						//返回体素中的材质类型
}

bool Terrain::IsValidPoint(const Point3D& p) const
{
	//判定点是否处于包围盒之外
	if (!m_bbox3d.IsContainPoint(p))
		return false;
	//求解点所在的三角形面元
	TerrainFacet* facet = _getTerrainFacetViaPoint(p);
	if (!facet)
		return false;
	if (facet->GetVerticleDistanceToPoint(p) < 0)
		return false;
	return true;
}

bool Terrain::GetTerrainDiffractionPath(Point3D tx, Point3D rx, TerrainDiffractionPath*& outPath) const
{
	//修正1-无论是否被遮挡，均返回地面投影路径

	TerrainFacet* txFacet = _getTerrainFacetViaPoint(tx);
	TerrainFacet* rxFacet = _getTerrainFacetViaPoint(rx);
	if (txFacet == rxFacet)//0-确定收发坐标点所在的面元，若相同，则返回false
		return false;

	std::vector<Point3D> routeAboveRay;//高于射线高度的路径点集合
	std::vector<Point3D> route;		//收发机间的地形点集合
	//求解发射点在对应面元上的投影坐标
	Point3D txOnFacet = txFacet->GetPointOnPlane(tx); //发射机在面元上的坐标   
	Point3D rxOnFacet = rxFacet->GetPointOnPlane(rx); //接收机在面元上的坐标
	route.push_back(txOnFacet); //先将发射坐标点纳入路径中
	//基于半边搜索方法确定射线所经过的地形表面起伏信息
	//构建基础三维射线
	Vector3D rt = rx - tx;
	Vector3D rt_dir = Normalize(rt);
	Ray3DLite* ray3d = new Ray3DLite();
	ray3d->m_ori = tx;
	ray3d->m_dir = rt_dir;
	//构建基础二维射线
	Ray2D* ray2d = new Ray2D();
	ray2d->m_Ori.x = ray3d->m_ori.x;
	ray2d->m_Ori.y = ray3d->m_ori.y;
	ray2d->m_Dir.x = ray3d->m_dir.x;
	ray2d->m_Dir.y = ray3d->m_dir.y;
	ray2d->m_Dir.Normalize(); //赋值后需要归一化数值

	TerrainFacet* curFacet = txFacet; /** @brief	当前运算到的面元	*/
	TerrainSegment* curSegment = nullptr; /** @brief	当前运算到的线段	*/
	TerrainSegment* prevSegment = nullptr; /** @brief	上一个运算到的线段	*/
	RtLbsType t_cur; /** @brief	当前射线与线段相交的距离二维	*/
	RtLbsType h_cur; /** @brief	当前射线与线段相交的交点高度（用于比较是否与射线相交）	*/
	RtLbsType t_max = rt.LengthXY();
	bool hasIntersect = false; //射线被环境遮挡状态
	while (true) { //遍历到相等的位置
		prevSegment = curSegment;
		curSegment = curFacet->GetIntersectSegment(ray2d, prevSegment, &t_cur, &h_cur);//求解射线与面元内部的相交线段
		if (curSegment == nullptr)	//迭代到无法解决的误差问题，终止路径循迹
			break;
		RtLbsType rayHeight = ray3d->GetRayHeight(t_cur);
		//计算二维射线与线段的交点并根据h_cur转换为三维交点
		Point2D pInTerrain2D = (*ray2d)(t_cur);
		Point3D pInTerrain3D(pInTerrain2D.x, pInTerrain2D.y, h_cur);
		route.push_back(pInTerrain3D);
		if (rayHeight <= h_cur) {//将路径添加至高于接收天线的路径中
			routeAboveRay.push_back(pInTerrain3D);
		}
		if (rayHeight <= h_cur || (rayHeight - h_cur) <= 1) { //射线和线段的高度差在1m以内，采用射线与三角面元求交模式求解精确解
			if (curFacet->GetIntersect(ray3d))//计算射线与当前面元的相交结果
				hasIntersect = true; //与环境相交
		}
		curFacet = curSegment->GetAdjacentFacet(curFacet); //计算下一个面元
		if (curFacet == rxFacet || t_cur >= t_max) //遍历到相等面元或距离超过最大距离限制则进行break
			break;
	}
	route.push_back(rxOnFacet); //添加接收机表面路径

	TerrainProfile* profile = new TerrainProfile();
	//获得路径上每个节点的电磁材质信息
	std::vector<Material*> mats(route.size());															/** @brief	地形点材质数组	*/
	for (int i = 0; i < route.size(); ++i)
		mats[i] = GetMaterial(route[i]);														//获得路径上每个点的材质ID
	profile->InitParameters(route, mats, tx, rx, m_averageRidgeGap);
	profile->GetDiffractPathOverRidges(outPath);	//由profile生成绕射路径
	if (outPath != nullptr) {								//当且仅当outPath不为nullptr时使用
		outPath->m_terrainDiffractionMode = m_propagationProperty.m_terrainDiffractionMode;			//地形绕射计算模式判别
	}
	//profile.WriteRidgesToFile("ridges.txt");		//将profile中的峰峦写入到文件中-调试用
	profile->WriteProfileToFile("profile.txt");		//将profile中的地形剖面写入到文件中-调试用
	delete profile;
	delete ray2d;
	delete ray3d;

	return true;
}

bool Terrain::GetTerrainReflectionPaths(const Point3D& tx, const Point3D& rx, std::vector<RayPath3D*>& outPath) const
{
	std::vector<TerrainFacet*> attachGroundFacets;																	/** @brief	贴近地表的面元集合	*/
	if (!_getTerrainProfileFacets(tx, rx, attachGroundFacets))														//若产生不了地表面元集合，则返回false
		return false;

	std::vector<Point3D> intersectPoints;																			/** @brief	有效的交点，即地面反射点	*/
	//遍历地表面元集合，计算镜像点并求解射线与面元的交点
	for (auto it = attachGroundFacets.begin(); it != attachGroundFacets.end(); ++it) {
		const TerrainFacet* curFacet = *it;
		Point3D mirrorPoint = curFacet->GetMirrorPoint(tx);															/** @brief	求解关于tx的镜像点	*/
		Ray3DLite ray3d(mirrorPoint, rx);																			/** @brief	构造基础三维射线	*/
		Point3D intersectPoint;																						/** @brief	射线与面元的交点	*/
		if (curFacet->GetIntersect(&ray3d, &intersectPoint)) {														//当前面元与地形有交点，则表明反射存在
			RayPath3D* newRayPath = new RayPath3D();																/** @brief	新路径	*/
			PathNode3D* txNode = new PathNode3D(tx, NODE_ROOT);														/** @brief	构造发射节点	*/
			PathNode3D* reflNode = new PathNode3D(intersectPoint, NODE_REFL, curFacet);								/** @brief	构造中间反射节点	*/
			PathNode3D * rxNode = new PathNode3D(rx, NODE_STOP);													/** @brief	构造接收节点	*/
			newRayPath->Union(txNode);
			newRayPath->Union(reflNode);
			newRayPath->Union(rxNode);
			newRayPath->m_type = RAYPATH_TERRAIN_REFLECTION;
			outPath.push_back(newRayPath);
		}
	}
	if (outPath.size() == 0)																						//若out的路径数量为0, 则返回false, 寻找地形反射路径无效
		return false;
	return true;
}

void Terrain::_init()
{
	for (int i = 0; i < 2; i++) {
		m_voxelNum[i] = 0;
		m_voxelExtent[i] = 0.0;
		m_voxelInvExtent[i] = 0.0;
	}
	m_voxelCount = 0;
	m_category = MESHNONHOLE;					//默认模式为mesh无孔洞模式
	m_terrainId = -1;							//默认ID为-1，代表无效
	m_simplifyFlag = false;						//默认不简化
	m_simplifyRate = 1.0;						//默认简化率为1,不简化
	m_averageRidgeGap = 100;					//默认地形中峰峦间距为100m
}

bool Terrain::_initData(float* elevation, std::vector<Material*>& matMatrix, int rows, int cols, RtLbsType rowGap, RtLbsType colGap, RtLbsType minValue, RtLbsType maxValue, RtLbsType ratio)
{
	//栅格模式初始化地形参数
	m_voxelNum[0] = cols - 1;
	m_voxelNum[1] = rows - 1;
	m_voxelExtent[0] = colGap;
	m_voxelExtent[1] = rowGap;
	m_voxelInvExtent[0] = 1.0 / colGap;
	m_voxelInvExtent[1] = 1.0 / rowGap;
	m_voxelCount = (cols - 1) * (rows - 1);
	m_bbox2d.m_min.x = 0.0;
	m_bbox2d.m_min.y = 0.0;
	m_bbox2d.m_max.x = (cols - 1) * colGap;
	m_bbox2d.m_max.y = (rows - 1) * rowGap;
	m_bbox3d.m_min.x = 0.0;
	m_bbox3d.m_min.y = 0.0;
	m_bbox3d.m_max.x = (cols - 1) * colGap;
	m_bbox3d.m_max.y = (rows - 1) * rowGap;
	m_bbox3d.m_min.z = minValue - 20;
	m_bbox3d.m_max.z = maxValue + 20;



	//创建网格顶点
	std::vector<SurfaceMesh::Vertex_index> vertices(rows * cols);
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			RtLbsType y = i * rowGap; //行-y方向
			RtLbsType x = j * colGap; //列-x方向
			RtLbsType z = elevation[(rows - 1 - i) * cols + j];							//修复地形读取时镜像问题
			vertices[i * cols + j] = m_meshes.add_vertex(Kernel::Point_3(x, y, z));
		}
	}
	//创建三角形网格面元
	for (int i = 0; i < rows - 1; ++i) {
		for (int j = 0; j < cols - 1; ++j) {
			//m_mesh.add_face(vertices[i * cols + j], vertices[i * cols + j + 1], vertices[(i + 1) * cols + j + 1]);
			//m_mesh.add_face(vertices[i * cols + j], vertices[(i + 1) * cols + j + 1], vertices[(i + 1) * cols + j]);
			m_meshes.add_face(vertices[(i + 1) * cols + j + 1], vertices[i * cols + j + 1], vertices[i * cols + j]);
			m_meshes.add_face(vertices[(i + 1) * cols + j], vertices[(i + 1) * cols + j + 1], vertices[i * cols + j]);
		}
	}

	if (m_simplifyFlag == true) {
		if (!_simplify(m_simplifyRate))
			return false;

	}
	if (!_transform(m_meshes))
		return false;

	_build(matMatrix); //将线段和面元转换完成后开始执行build程序
	LOG_INFO << "Terrain: geometry loading success." << ENDL;
	return true;
}

bool Terrain::_initData(const aiScene* scene, RtLbsType ratio, std::vector<Material*>& materials)
{
	//问题-在面元简化过程中，需要考虑简化后的面元与简化前面元的材质属性的对应关系-已解决
	//问题2-在地形场景面元为非栅格类型面元时，需要进行构建对应的网格,暂时不做考虑
	m_calcMode = TERRAIN_HALFFACEMODE;																							//加载assimp数据,为半边计算模式
	if (materials.size() != scene->mNumMeshes) {
		LOG_INFO << "Terrain: " << "can't match material num and scene meshes." << ENDL;
		return false;
	}
	for (unsigned int i = 0; i < scene->mNumMeshes; ++i) {																		//针对object中的每一个mesh进行几何属性和材质信息的赋值
		SurfaceMesh mesh;
		const aiMesh* aimesh = scene->mMeshes[i];
		Material* curMeshMaterial = materials[i];																				//当前mesh的材质属性
		//提取物体顶点信息
		std::vector< SurfaceMesh::Vertex_index> vertices(aimesh->mNumVertices);
		Point3D pmin(FLT_MAX, FLT_MAX, FLT_MAX), pmax(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		for (unsigned int j = 0; j < aimesh->mNumVertices; ++j) {
			aiVector3D aivertex = aimesh->mVertices[j];
			pmin.x = std::min(static_cast<RtLbsType>(aivertex.x), pmin.x);
			pmin.y = std::min(static_cast<RtLbsType>(aivertex.y), pmin.y);
			pmin.z = std::min(static_cast<RtLbsType>(aivertex.z), pmin.z);
			pmax.x = std::max(static_cast<RtLbsType>(aivertex.x), pmax.x);
			pmax.y = std::max(static_cast<RtLbsType>(aivertex.y), pmax.y);
			pmax.z = std::max(static_cast<RtLbsType>(aivertex.z), pmax.z);
			SurfaceMesh::Vertex_index vertex = mesh.add_vertex(SurfaceMesh::Point(aivertex.x, aivertex.y, aivertex.z));
			vertices[j] = vertex;
		}
		//提取面元信息
		for (unsigned int j = 0; j < aimesh->mNumFaces; ++j) {
			aiFace aiface = aimesh->mFaces[j];
			std::vector<SurfaceMesh::Vertex_index> face(aiface.mNumIndices);
			for (unsigned int k = 0; k < aiface.mNumIndices; ++k) {
				face[k] = vertices[aiface.mIndices[k]];
			}
			mesh.add_face(face);
		}

		//计算包围盒
		m_bbox3d.Union(pmin);
		m_bbox3d.Union(pmax);
		m_bbox2d.Union(Point2D(pmin.x, pmin.y));
		m_bbox2d.Union(Point2D(pmax.x, pmax.y));

		if (m_simplifyFlag == true) {								//进行简化
			if (!_simplify(m_simplifyRate))
				return false;										//地形简化失败
		}
		if (!_transform(mesh)) {
			return false;
		}
	}

	unsigned squaredVoxelNum = static_cast<int>(trunc(sqrt(m_facetBuf.size())));			//采用平方根法构建均匀网格加速结构
	m_voxelNum[0] = squaredVoxelNum;
	m_voxelNum[1] = squaredVoxelNum;
	m_voxelExtent[0] = (m_bbox2d.m_max.x - m_bbox2d.m_min.x) / m_voxelNum[0];
	m_voxelExtent[1] = (m_bbox2d.m_max.y - m_bbox2d.m_min.y) / m_voxelNum[1];
	m_voxelInvExtent[0] = 1.0 / m_voxelExtent[0];
	m_voxelInvExtent[1] = 1.0 / m_voxelExtent[1];
	m_voxelCount = m_voxelNum[0] * m_voxelNum[1];

	_build();																			//将线段和面元转换完成后开始执行build程序
	LOG_INFO << "Terrain: geometry loading success." << ENDL;
	return true;
}

void Terrain::_release()
{
	//主要释放pointBuf,facetBuf和segmentBuf

	for (auto it = m_pointBuf.begin(); it != m_pointBuf.end(); ++it) {
		delete* it;
	}
	m_pointBuf.clear();

	for (auto it = m_facetBuf.begin(); it != m_facetBuf.end(); ++it) {
		delete* it;
	}
	m_facetBuf.clear();

	for (auto it = m_segmentBuf.begin(); it != m_segmentBuf.end(); ++it) {
		delete* it;
	}
	m_segmentBuf.clear();

	for (auto it = m_gridCells.begin(); it != m_gridCells.end(); ++it) {
		delete* it;
	}
	m_gridCells.clear();
}

bool Terrain::_transform(const SurfaceMesh& mesh, Material* mat)
{
	if (!CGAL::is_triangle_mesh(mesh)) {
		LOG_ERROR << "Terrain: terrain mesh failed to transform, not a triangle mesh." << ENDL;
		return false;
	}

	//从cgal中读取点集并进行赋值
	m_pointBuf.resize(mesh.num_vertices());
	int pid = 0;																				//点坐标ID
	for (auto it = mesh.points().begin(); it != mesh.points().end(); ++it) {
		const Kernel::Point_3& p = *it;
		m_pointBuf[pid] = new Point3D(p.x(), p.y(), p.z(), pid);
		pid++;
	}


	//求解点集的平均值，即模型几何中心
	for (auto it = m_pointBuf.begin(); it != m_pointBuf.end(); ++it) {
		Point3D* p = *it;
		m_centerPosition += *p;
	}
	m_centerPosition /= static_cast<RtLbsType>(m_pointBuf.size());								//求解几何中心

	//从cgal中读取三角形面元并进行初始化
	unsigned facetOffset = static_cast<unsigned>(m_facetBuf.size());
	m_facetBuf.resize(facetOffset + mesh.faces().size());
	Point3D pointTemp[3]; //临时用于接受坐标点
	std::unordered_map<int, int> facetIdMap; //面元搜索映射表
	int pointId = 0;
	int facetId = facetOffset;
	int p_offset[3];																			//三角形中三个点坐标在点集中的偏离值
	for (auto itFacet = mesh.faces_begin(); itFacet != mesh.faces_end(); ++itFacet) {
		facetIdMap[(*itFacet).idx()] = facetId;
		auto itRangeVertice = mesh.vertices_around_face(mesh.halfedge(*itFacet));
		if (itRangeVertice.size() == 3) { //保证数据为三角形面元
			pointId = 0;
			//计算面元坐标点的在点坐标数组中的偏离值
			for (const auto& vertex_index : itRangeVertice) {
				p_offset[pointId] = vertex_index.idx();
				pointId++;
			}
			//开始赋值三角形
			m_facetBuf[facetId++] = new TerrainFacet(facetId, m_pointBuf[p_offset[2]], m_pointBuf[p_offset[1]], m_pointBuf[p_offset[0]], mat); //由于CGAL中多边形顶点为顺序，这里改为逆序条件
		}
	}

	//创建全边，并进行哈希映射然后创建半边
	std::vector<TerrainSegment*> segments((m_facetBuf.size()-facetOffset) * 3);
	int edgeId = 0;
	for (auto it = m_facetBuf.begin() + facetOffset; it != m_facetBuf.end(); ++it) {
		TerrainFacet* facet = *it;
		Point3D* p1 = facet->m_p1;
		Point3D* p2 = facet->m_p2;
		Point3D* p3 = facet->m_p3;
		TerrainSegment* edge1 = new TerrainSegment(edgeId, p1, p2, facet);
		segments[edgeId++] = edge1;
		TerrainSegment* edge2 = new TerrainSegment(edgeId, p2, p3, facet);
		segments[edgeId++] = edge2;
		TerrainSegment* edge3 = new TerrainSegment(edgeId, p3, p1, facet);
		segments[edgeId++] = edge3;
	}

	unsigned edgeOffset = static_cast<unsigned>(m_segmentBuf.size());

	//寻找双面边
	int numValidEdges = 0;															//有效的边数量，包含双面边和单面边
	std::vector<TerrainSegment*> validSegments; //有效的边
	std::unordered_map<size_t, TerrainSegment*> segmentMap; //寻找相同边用到结构
	for (auto it = segments.begin(); it != segments.end(); ++it) {
		TerrainSegment* segment = *it;
		size_t hash = segment->GetHash();
		if (segmentMap.find(hash) == segmentMap.end()) { //找不到当前hash值,则将当前数据放入hash值中
			segmentMap[hash] = segment;
		}
		else { //寻找到重复的hash值，构建双面边结构
			segmentMap[hash]->m_facet2 = segment->m_facet1;//赋值线段的另一个面元
			TerrainSegment* newSegment = new TerrainSegment(*segmentMap[hash]);
			newSegment->m_isShared = true;									//共享边赋值
			newSegment->m_segmentId = numValidEdges + edgeOffset; //赋值线段的id
			validSegments.push_back(newSegment);
			numValidEdges++;
		}
	}

	//寻找单面边
	for (auto it = segmentMap.begin(); it != segmentMap.end(); ++it) {
		TerrainSegment* segment = (*it).second;
		if (segment->m_facet2 == nullptr) {
			TerrainSegment* newSegment = new TerrainSegment(*segment);
			newSegment->m_segmentId = numValidEdges + edgeOffset;
			newSegment->m_isShared = false;									//非共享边赋值
			validSegments.push_back(newSegment);
			numValidEdges++;
		}
	}
	
	//销毁原始边数据
	for (auto it = segments.begin(); it != segments.end();++it) {
		delete* it;
	}
	segments.clear();

	//更新面元中的segment数据
	m_segmentBuf.insert(m_segmentBuf.begin() + edgeOffset, validSegments.begin(), validSegments.end());
	for (auto it = m_segmentBuf.begin(); it != m_segmentBuf.end(); ++it) {
		TerrainSegment* segment = *it;
		if (segment->m_facet1 != nullptr) {
			TerrainFacet* facet = segment->m_facet1;
			facet->AssignEdge(segment);
		}
		if (segment->m_facet2 != nullptr) {
			TerrainFacet* facet = segment->m_facet2;
			facet->AssignEdge(segment);
		}
	}


	//循环所有面元，检查面元中是否包含单面边判定是否是边界面元
	for (auto it = m_facetBuf.begin(); it != m_facetBuf.end(); ++it) {
		TerrainFacet* facet = *it;
		if (facet->m_segment1->m_isShared == true ||
			facet->m_segment2->m_isShared == true ||
			facet->m_segment3->m_isShared == true) {			//当且仅当面元中至少有一个边界面元时，该面元为边界面元
			facet->m_isEdgeFacet = true;
		}
		else {
			facet->m_isEdgeFacet = false;
		}
	}

	LOG_INFO << "Terrain: terrain mesh has been transform successfully." << ENDL;
	return true;
}

int Terrain::_offset(int x, int y) const
{
	return y * m_voxelNum[0] + x;
}

void Terrain::_offset_reverse(unsigned voxelId, int& x, int& y) const
{
	y = static_cast<unsigned>(floor(voxelId / m_voxelNum[0]));
	x = voxelId - y * m_voxelNum[0];
}

int Terrain::_point2VoxelId(const Point2D& p, unsigned axis) const
{
	return std::min(m_voxelNum[axis] - 1, (unsigned)((p[axis] - m_bbox2d.m_min[axis]) * m_voxelInvExtent[axis]));
}

int Terrain::_point2VoxelId(const Point3D& p, unsigned axis) const
{
	return std::min(m_voxelNum[axis] - 1, (unsigned)((p[axis] - m_bbox2d.m_min[axis]) * m_voxelInvExtent[axis]));
}

Point2D Terrain::_voxelId2Point(int voxel[2]) const
{
	Point2D p;
	p.x = m_bbox2d.m_min.x + voxel[0] * m_voxelExtent[0];
	p.y = m_bbox2d.m_min.y + voxel[1] * m_voxelExtent[1];
	return p;
}

void Terrain::_build(std::vector<Material*>& matMatrix)
{
	m_gridCells.resize(m_voxelCount);//初始化网格数据
	int id = 0;
	Point2D cornerPoint = m_bbox2d.m_min;
	if (!m_simplifyFlag) {				//不进行简化
		//从m_facets中读取数据并进行加载，方向为先row(y)再col(x)
		for (unsigned y = 0; y < m_voxelNum[1]; ++y) {		//先遍历y方向
			for (unsigned x = 0; x < m_voxelNum[0]; ++x) {	//再遍历x方向
				int cellId = _offset(x, y);
				TerrainFacet* facet1 = m_facetBuf[2 * cellId];
				TerrainFacet* facet2 = m_facetBuf[2 * cellId + 1];
				m_gridCells[cellId] = new TerrainCell();
				m_gridCells[cellId]->m_cornerPoint[0] = cornerPoint[0] + x * m_voxelExtent[0];
				m_gridCells[cellId]->m_cornerPoint[1] = cornerPoint[1] + y * m_voxelExtent[1];
				m_gridCells[cellId]->m_facets.push_back(facet1);
				m_gridCells[cellId]->m_facets.push_back(facet2);
				m_gridCells[cellId]->m_mat = matMatrix[cellId];//执行材质赋值

			}
		}
	}
	else {			//简化后采用半边模式加载面元
		//先进行cell初始化
		for (unsigned y = 0; y < m_voxelNum[1]; ++y) {//先遍历y方向
			for (unsigned x = 0; x < m_voxelNum[0]; ++x) {//再遍历x方向
				int cellId = _offset(x, y);
				m_gridCells[cellId] = new TerrainCell();
				m_gridCells[cellId]->m_cornerPoint[0] = cornerPoint[0] + x * m_voxelExtent[0];
				m_gridCells[cellId]->m_cornerPoint[1] = cornerPoint[1] + y * m_voxelExtent[1];
				m_gridCells[cellId]->m_mat = matMatrix[cellId];//执行材质赋值
				id++;
			}
		}

		//针对不规则面元进行赋值，主要赋值栅格内容
		for (auto it = m_facetBuf.begin(); it != m_facetBuf.end(); ++it) {
			TerrainFacet* facet = *it;
			unsigned maxGridId[2];
			unsigned minGridId[2];
			for (int i = 0; i < 2; ++i) {
				minGridId[i] = _point2VoxelId(facet->m_bbox.m_min, i);
				maxGridId[i] = _point2VoxelId(facet->m_bbox.m_max, i);
			}

			//执行包围盒赋值
			for (unsigned y = minGridId[1]; y <= maxGridId[1]; ++y) {
				for (unsigned x = minGridId[0]; x <= maxGridId[0]; ++x) {
					unsigned offset = _offset(x, y);
					m_gridCells[offset]->m_facets.push_back(facet);
				}
			}
		}

	}
	

	LOG_INFO << "terrain grid construct success!" << ENDL;

	

}

void Terrain::_build()
{
	m_gridCells.resize(m_voxelCount);//初始化网格数据
	int id = 0;
	Point2D cornerPoint = m_bbox2d.m_min;
	for (unsigned y = 0; y < m_voxelNum[1]; ++y) {//先遍历y方向
		for (unsigned x = 0; x < m_voxelNum[0]; ++x) {//再遍历x方向
			m_gridCells[id] = new TerrainCell();
			m_gridCells[id]->m_cornerPoint[0] = cornerPoint[0] + x * m_voxelExtent[0];
			m_gridCells[id]->m_cornerPoint[1] = cornerPoint[1] + y * m_voxelExtent[1];
			id++;
		}
	}

	//2、半边版本,需要赋值面元和边，面元提供查询,边提供相交判定
	if (m_category == MESHNONHOLE || m_category == MESHHOLE) {						//含孔洞和不含孔洞版本
		//针对不规则面元进行赋值，主要赋值栅格内容
		for (auto it = m_facetBuf.begin(); it != m_facetBuf.end(); ++it) {
			TerrainFacet* facet = *it;
			unsigned maxGridId[2];
			unsigned minGridId[2];
			for (int i = 0; i < 2; ++i) {
				minGridId[i] = _point2VoxelId(facet->m_bbox.m_min, i);
				maxGridId[i] = _point2VoxelId(facet->m_bbox.m_max, i);
			}

			//执行包围盒赋值
			for (unsigned y = minGridId[1]; y <= maxGridId[1]; ++y) {
				for (unsigned x = minGridId[0]; x <= maxGridId[0]; ++x) {
					unsigned offset = _offset(x, y);
					m_gridCells[offset]->m_facets.push_back(facet);
					m_gridCells[offset]->m_mat = facet->m_mat;			//材质信息赋值
				}
			}
		}

		LOG_INFO << "terrain grid construct success!" << ENDL;
		return;
	}
	LOG_ERROR << "Terrain: build data error, wrong region." << ENDL;
}

std::vector<unsigned> Terrain::_getGridCoordAlongSegment(TerrainSegment* segment)
{
	std::vector<unsigned> ids;
	//基于网格迭代算法进行精确赋值
	int curGrid[2], endGrid[2], dir[2]; //网格当前编号、网格终止编号、网格行进方向
	RtLbsType delta[2], next[2];
	RtLbsType maxt = segment->GetLengthXY();
	RtLbsType curt = 0.0;
	Vector2D s_dir = segment->GetDirXY();
	const Point2D& sp = segment->GetStartPoint2D();
	const Point2D& ep = segment->GetEndPoint2D();

	//计算单位增量值
	for (int i = 0; i < 2; ++i) {
		curGrid[i] = _point2VoxelId(sp, i);
		endGrid[i] = _point2VoxelId(ep, i);
		dir[i] = (s_dir[i] > 0.0) ? 1 : -1;
		if (s_dir[i] != 0.0)
			delta[i] = abs(m_voxelExtent[i] / s_dir[i]);
		else
			delta[i] = FLT_MAX;
	}
	
	int voxelId = _offset(curGrid[0], curGrid[1]);
	//计算目标值
	Point2D& curGridCorner = m_gridCells[voxelId]->m_cornerPoint;
	for (int i = 0; i < 2; ++i) {
		RtLbsType target = curGridCorner[i] + ((dir[i] + 1) >> 1) * m_voxelExtent[i];
		next[i] = (target - sp[i]) / s_dir[i];
		if (delta[i] == FLT_MAX)
			next[i] = FLT_MAX;
	}

	//遍历网格求解体素编号
	const unsigned array[] = { 0,0,1,1 };//[0]、[3]不可能相交
	while (curt < maxt) {
		voxelId = _offset(curGrid[0], curGrid[1]);
		ids.push_back(voxelId); //将采集到的网格编号记录在vector中
		//计算下一个t值
		unsigned nextAxis = (next[0] <= next[1]) + ((unsigned)(next[1] <= next[0])) * 2;
		nextAxis = array[nextAxis];
		if (curt < 0) {
			LOG_ERROR << "_getGridCoordAlongSegment:" << "curt < 0" << CRASH;
		}
		//推演至下一个voxel
		curGrid[nextAxis] += dir[nextAxis];
		curt = next[nextAxis];
		next[nextAxis] += delta[nextAxis];
	}
	return ids;
}

bool Terrain::_getIntersect(Ray3DLite* ray, unsigned voxelId, Point3D* intersectPoint) const
{
	if (voxelId >= m_voxelCount)
		LOG_ERROR << "Voxel id is out of range.(" << voxelId << "/" << m_voxelCount << ")" << CRASH;
	for (auto it = m_gridCells[voxelId]->m_facets.begin(); it != m_gridCells[voxelId]->m_facets.end(); ++it) {
		TerrainFacet* facet = *it;
		if (facet->GetIntersect(ray, intersectPoint)) {
			return true;
		}
	}
	return false;
}

bool Terrain::_getIntersect(Ray3DLite* ray3d, Ray2D* ray2d, RtLbsType t, unsigned voxelId, unsigned& targetVoxelId) const
{
	//判定voxel中面元是否满足规定的要求
	TerrainCell* curCell = m_gridCells[voxelId];								//获取当前的网格单元
	if (curCell->m_facets.size() == 0) {										//当前网格单元中的面元数量为0，不具备相交条件
		return false;
	}
	else {
		//求解起点所在的面元
		Point2D targetPoint = ray2d->GetRayCoordinate(t);						/** @brief	获取迭代长度t处的射线上的坐标	*/
		TerrainFacet* curFacet = _getTerrainFacetViaPoint(targetPoint);		/** @brief	获取目标坐标上的面元	*/
		if (curFacet == nullptr) {					//若面元为空，则求解射线与边界面元的求交情况
			//计算射线与cell中的边界面元相交情况，然后使用半边迭代方法继续求解
			TerrainFacet* nextFacet = nullptr;
			if (!curCell->GetIntersectEdgeFacet(ray2d, ray3d, curFacet, nextFacet)) {
				return false;				//当前面元中无其他可供半边方法的边界面元，返回false
			}
			curFacet = nextFacet;
		}
		//保证面元不为空，采用半边迭代法求交
		TerrainSegment* curSegment = nullptr;							/** @brief	当前运算到的线段	*/
		TerrainSegment* prevSegment = nullptr;							/** @brief	上一个运算到的线段	*/
		RtLbsType t_cur;												/** @brief	当前射线与线段相交的距离二维	*/
		RtLbsType h_cur;												/** @brief	当前射线与线段相交的交点高度（用于比较是否与射线相交）	*/
		while (curFacet != nullptr) {
			prevSegment = curSegment;
			curSegment = curFacet->GetIntersectSegment(ray2d, prevSegment, &t_cur, &h_cur);		//求解射线与面元内部的相交线段
			if (curSegment == nullptr)
				return false;		//迭代到无法解决的误差问题，放弃追踪该条射线
			RtLbsType rayHeight = ray3d->GetRayHeight(t_cur);									/** @brief	三维射线上的高度值	*/
			if (rayHeight <= h_cur || (rayHeight - h_cur) <= 0.5) {								//射线和线段的高度差在10m以内，采用射线与三角面元求交模式求解精确解
				if (curFacet->GetIntersect(ray3d)) {								//计算射线与当前面元的相交结果
					return true;
				}
			}
			curFacet = curSegment->GetAdjacentFacet(curFacet);									//迭代下一个面元
			if (curFacet == nullptr) {															//迭代至边界面元，需要判别cell内还有其他边界面元与之相交
				//更新二维射线起始坐标-用以计算射线与voxel内下一个边界面元的交点
				TerrainFacet* nextFacet = nullptr;
				if (curCell->GetIntersectEdgeFacet(ray2d, ray3d, curFacet, nextFacet)) {
					curFacet = nextFacet;														//在当前cell中还存在其他边界面元，继续进行面元迭代；
					continue;
				}
				//若在当前cell中不存在边界面元，则直接返回false，不相交
				return false;
			}
		}
		return false;				//当前面元为空面元，返回false
	}

}

TerrainFacet* Terrain::_getTerrainFacetViaPoint(const Point2D& point) const
{
	//1-计算点在哪个栅格内
	int gridId[2];
	for (unsigned i = 0; i < 2; ++i) {
		gridId[i] = _point2VoxelId(point, i);
	}
	int voxelId = _offset(gridId[0], gridId[1]);
	//2-计算栅格内面元是否包含对应坐标点
	TerrainCell* cell = m_gridCells[voxelId];
	for (TerrainFacet* facet : cell->m_facets) {
		if (facet->CheckInside(point))
			return facet;
	}
	return nullptr; //若检查到面元不包含任何面元，则返回nullptr
}

TerrainFacet* Terrain::_getTerrainFacetViaPoint(const Point3D& point) const
{
	//1-计算点在哪个栅格内
	int gridId[2];
	for (unsigned i = 0; i < 2; ++i) {
		gridId[i] = _point2VoxelId(point, i);
	}
	int voxelId = _offset(gridId[0], gridId[1]);
	//2-计算栅格内面元是否包含对应坐标点
	TerrainCell* cell = m_gridCells[voxelId];
	for (TerrainFacet* facet : cell->m_facets) {
		if (facet->CheckInside(point))
			return facet;
	}
	return nullptr; //若检查到面元不包含任何面元，则返回nullptr
}

bool Terrain::_getTerrainProfileFacets(const Point3D& txPosition, const Point3D& rxPosition, std::vector<TerrainFacet*>& outFacets) const
{
	TerrainFacet* txFacet = _getTerrainFacetViaPoint(txPosition);
	TerrainFacet* rxFacet = _getTerrainFacetViaPoint(rxPosition);
	if (txFacet == rxFacet) {																				//若收发点坐标在同一个面元上，则直接定义该面元，并返回真
		outFacets.resize(1);
		outFacets[0] = txFacet;
		return true;
	}

	Point3D txOnFacet = txFacet->GetPointOnPlane(txPosition);												//发射机在面元上的坐标 
	Point3D rxOnFacet = rxFacet->GetPointOnPlane(rxPosition);												//接收机在面元上的坐标

	//-----------------------------------基于半边方法确定射线所经历过的地形表面起伏信息-----------------------------------------------------------
	
	Vector3D rt = rxPosition - txPosition;
	Vector3D rt_dir = Normalize(rt);																		//构建基础三维射线
	Ray3DLite* ray3d = new Ray3DLite();
	ray3d->m_ori = txPosition;
	ray3d->m_dir = rt_dir;

	Ray2D* ray2d = new Ray2D();																				//构建基础二维射线
	ray2d->m_Ori.x = ray3d->m_ori.x;
	ray2d->m_Ori.y = ray3d->m_ori.y;
	ray2d->m_Dir.x = ray3d->m_dir.x;
	ray2d->m_Dir.y = ray3d->m_dir.y;
	ray2d->m_Dir.Normalize();																				//赋值后需要归一化数值

	TerrainFacet* curFacet = txFacet;																		/** @brief	当前运算到的面元	*/
	TerrainSegment* curSegment = nullptr;																	/** @brief	当前运算到的线段	*/
	TerrainSegment* prevSegment = nullptr;																	/** @brief	上一个运算到的线段	*/
	RtLbsType t_cur;																						/** @brief	当前射线与线段相交的距离二维	*/
	RtLbsType h_cur;																						/** @brief	当前射线与线段相交的交点高度（用于比较是否与射线相交）	*/
	RtLbsType t_max = rt.LengthXY();
	outFacets.push_back(curFacet);																			//将当前的地形面元纳入输出面元集合中

	while (true) {																							//面元迭代器
		prevSegment = curSegment;																			//线段迭代器
		curSegment = curFacet->GetIntersectSegment(ray2d, prevSegment, &t_cur, &h_cur);						//求解射线与面元内部的相交线段
		if (curSegment == nullptr)																			//迭代到无法解决的误差问题，终止路径循迹
			break;
		curFacet = curSegment->GetAdjacentFacet(curFacet);													//计算下一个面元
		outFacets.push_back(curFacet);																		//纳入面元
		if (curFacet == rxFacet || t_cur >= t_max)															//遍历到相等面元或距离超过最大距离限制则进行break
			break;
	}
	if (outFacets.size() != 0)
		return false;
	return true;
}

