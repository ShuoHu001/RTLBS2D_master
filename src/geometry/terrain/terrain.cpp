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
	for (int i = 0; i < segmentSize; ++i)		//���Ƶ����߶�
		m_segments[i] = cell.m_segments[i];		
	for (int i = 0; i < facetSize; ++i)			//���Ƶ�����Ԫ
		m_facets[i] = cell.m_facets[i];
}

TerrainCell::~TerrainCell()
{
}

bool TerrainCell::GetIntersectEdgeFacet(Ray2D*& ray2d, Ray3DLite* ray3d, TerrainFacet* curFacet, TerrainFacet*& outFacet) const
{
	RtLbsType tmin = FLT_MAX;											/** @brief	�洢�������ֵ	*/
	RtLbsType cur_t = FLT_MAX;											/** @brief	��ǰ�洢�ľ���ֵ	*/
	TerrainFacet* nearestFacet = nullptr;								/** @brief	�����������Ԫ	*/
	outFacet = nullptr;													//�����Ԫ��ֵ����
	for (auto it = m_facets.begin(); it != m_facets.end(); ++it) {
		TerrainFacet* facet = *it;
		if (facet->m_isEdgeFacet == false)
			continue;
		if (facet->m_facetId == curFacet->m_facetId)
			continue;
		//����������Ԫ���ά�����ཻ��Ѱ���������Ԫ��
		if (facet->HasEdgeSegmentIntersect(ray2d, cur_t)) {
			if (cur_t < tmin) {
				tmin = cur_t;
				nearestFacet = facet;
			}
		}
	}
	if (tmin < FLT_MAX) {
		outFacet = nearestFacet;										//���������Ԫ��ֵ
		ray2d->m_Ori = ray2d->GetRayCoordinate(tmin);					//���¶�ά��������
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
	//���г�ʼ�����Ŀ���
	for (int i = 0; i < 2; ++i) {
		m_voxelNum[i] = terrain.m_voxelNum[i];
		m_voxelExtent[i] = terrain.m_voxelExtent[i];
		m_voxelInvExtent[i] = terrain.m_voxelInvExtent[i];
	}

	//���е㼯�Ŀ���
	int pSize = static_cast<int>(terrain.m_pointBuf.size());
	m_pointBuf.resize(pSize);
	for (int i = 0; i < pSize; ++i) {
		m_pointBuf[i] = new Point3D(*terrain.m_pointBuf[i]);
	}

	//�����߶εĿ���
	int sSize = static_cast<int>(terrain.m_segmentBuf.size());
	m_segmentBuf.resize(sSize);	
	unsigned psId, peId;														/** @brief �߶���ʼ�����ֹ���ID	*/
	for (int i = 0; i < sSize; ++i) {
		m_segmentBuf[i] = new TerrainSegment(*terrain.m_segmentBuf[i]);
		psId = terrain.m_segmentBuf[i]->m_ps->m_pId;							/** @brief	�߶���ʼ��ID	*/
		peId = terrain.m_segmentBuf[i]->m_pe->m_pId;							/** @brief	�߶���ֹ��ID	*/
		m_segmentBuf[i]->m_ps = m_pointBuf[psId];								/** @brief	��ʼ�������ַ��ֵ	*/
		m_segmentBuf[i]->m_pe = m_pointBuf[peId];								/** @brief	��ֹ�������ַ��ֵ	*/
	}

	//������Ԫ�Ŀ���
	int fSize = static_cast<int>(terrain.m_facetBuf.size());
	m_facetBuf.resize(fSize);
	unsigned p1Id, p2Id, p3Id;													/** @brief	���������������ID	*/
	for (int i = 0; i < fSize; ++i) {
		m_facetBuf[i] = new TerrainFacet(*terrain.m_facetBuf[i]);
		p1Id = terrain.m_facetBuf[i]->m_p1->m_pId;								/** @brief	�����ζ���1 ID	*/
		p2Id = terrain.m_facetBuf[i]->m_p2->m_pId;								/** @brief	�����ζ���2 ID	*/
		p3Id = terrain.m_facetBuf[i]->m_p3->m_pId;								/** @brief	�����ζ���3 ID	*/
		m_facetBuf[i]->m_p1 = m_pointBuf[p1Id];									/** @brief	�����ζ���1��ַ��ֵ	*/
		m_facetBuf[i]->m_p2 = m_pointBuf[p2Id];									/** @brief	�����ζ���2��ַ��ֵ	*/
		m_facetBuf[i]->m_p3 = m_pointBuf[p3Id];									/** @brief	�����ζ���3��ַ��ֵ	*/
	}

	//ӳ���߶�����Ԫ��ϵ
	unsigned facetId;															/** @brief	��ԪID	*/
	for (int i = 0; i < sSize; ++i) {
		if (terrain.m_segmentBuf[i]->m_facet1 != nullptr) {
			facetId = terrain.m_segmentBuf[i]->m_facet1->m_facetId;				/** @brief	�ڽ���Ԫ1 ID	*/
			m_segmentBuf[i]->m_facet1 = m_facetBuf[facetId];					/** @brief	�ڽ���Ԫ1��ַ��ֵ	*/
		}
		if (terrain.m_segmentBuf[i]->m_facet2 != nullptr) {
			facetId = terrain.m_segmentBuf[i]->m_facet2->m_facetId;				/** @brief	�ڽ���Ԫ2 ID	*/
			m_segmentBuf[i]->m_facet2 = m_facetBuf[facetId];					/** @brief	�ڽ���Ԫ2��ַ��ֵ	*/
		}
	}

	//ӳ����Ԫ���߶μ��ϵ
	unsigned segmentId;															/** @brief	�߶�ID	*/
	for (int i = 0; i < fSize; ++i) {
		if (terrain.m_facetBuf[i]->m_segment1 != nullptr) {
			segmentId = terrain.m_facetBuf[i]->m_segment1->m_segmentId;			/** @brief	�ڽ��߶�1 ID	*/
			m_facetBuf[i]->m_segment1 = m_segmentBuf[segmentId];				/** @brief	�ڽ��߶�1��ַ��ֵ	*/
		}
		if (terrain.m_facetBuf[i]->m_segment2 != nullptr) {
			segmentId = terrain.m_facetBuf[i]->m_segment2->m_segmentId;			/** @brief	�ڽ��߶�2 ID	*/
			m_facetBuf[i]->m_segment2 = m_segmentBuf[segmentId];				/** @brief	�ڽ��߶�2��ַ��ֵ	*/
		}
		if (terrain.m_facetBuf[i]->m_segment3 != nullptr) {
			segmentId = terrain.m_facetBuf[i]->m_segment3->m_segmentId;			/** @brief	�ڽ��߶�3 ID	*/
			m_facetBuf[i]->m_segment3 = m_segmentBuf[segmentId];				/** @brief	�ڽ��߶�3��ַ��ֵ	*/
		}
	}

	//����gridcell�Ŀ���
	unsigned gSize = static_cast<unsigned>(terrain.m_gridCells.size());
	unsigned gFacetId = 0, gSegmentId = 0;							/** @brief	cell�е���ԪID��cell�е��߶�ID	*/
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
	m_category = config.m_category;																					//�������͸�ֵ
	m_simplifyFlag = config.m_simplifyFlag;																			//���μ򻯱�־��ֵ
	m_simplifyRate = config.m_simplifyRate;																			//���μ��ʸ�ֵ
	m_averageRidgeGap = config.m_averageRidgeGap;																	//����ƽ�����ͼ�ำֵ
	m_propagationProperty = config.m_propagationProperty;															//���δ������Ը�ֵ
	if (config.m_loadingMode == TERRAIN_GRID) {				//դ��ģʽ��ȡ
		//����GDAL�������ݶ�ȡ
		const std::string& fileName = config.m_gridConfig.m_heightMatrixFile;
		if (fileName.empty())
			return false;
		std::string extension = GetFileExtension(fileName);
		if (supportedExtensionsTerrain.find(extension) != supportedExtensionsTerrain.end()) {						//����ļ���չ���Ƿ�����ȶ��ĸ�ʽҪ��
			GDALDataset* dataset = static_cast<GDALDataset*>(GDALOpen(fileName.c_str(), GA_ReadOnly));				//�򿪵����ļ�
			if (!dataset)
				return false;																						//���ݴ�ʧ��
			GDALRasterBand* band = dataset->GetRasterBand(1);														//Ĭ�ϸ߳������ڲ���1��
			RtLbsType minmaxValue[2];																				/** @brief	�߳������Сֵ	*/
			band->ComputeRasterMinMax(0, minmaxValue);
			int rows = band->GetYSize();
			int cols = band->GetXSize();
			float* elevation = new float[rows * cols];
			band->RasterIO(GF_Read, 0, 0, cols, rows, elevation, cols, rows, GDT_Float32, 0, 0);					//��ȡ�߳�ֵ
			double geoTransform[6];
			dataset->GetGeoTransform(geoTransform);																	//��ȡ����任��Ϣ
			RtLbsType colGap = abs(geoTransform[1]);																/** @brief	X ������	*/
			RtLbsType rowGap = abs(geoTransform[5]);																/** @brief	Y ������	*/
			//��ȡ���ζ�Ӧ�Ĳ�����Ϣ���������Ĭ��ֵ
			std::vector<Material*> matMatrix(rows * cols);
			for (auto& mat : matMatrix) {
				mat = matLibrary.GetDefaultMaterial();																//����������ʵ�Ĭ��ֵ
			}
			//�������ݳ�ʼ��terrain
			this->_initData(elevation, matMatrix, rows, cols, rowGap, colGap, minmaxValue[0], minmaxValue[1], 1.0);
			GDALClose(dataset);																						//�ر�GDAL������
			//�ͷ��޹��ڴ�
			delete[] elevation;
			matMatrix.clear();
			std::vector<Material*>().swap(matMatrix);
			return true;
		}
		return false;
	}
	else if (config.m_loadingMode == TERRAIN_OBJECT) {	//����ģʽ��ȡ
		//����assimp�������ݶ�ȡ
		const TerrainObjectConfig& objectConfig = config.m_objectConfig;
		const std::string& fileName = objectConfig.m_fileName;
		if (fileName.empty())
			return false;
		std::string extension = GetFileExtension(fileName);
		if (supportedExtensionsGeometry.find(extension) != supportedExtensionsGeometry.end()) {						//����ļ����Ƿ�����������չ��Ҫ��
			Assimp::Importer importer;
			const aiScene* scene = importer.ReadFile(fileName, aiProcess_Triangulate | aiProcess_CalcTangentSpace);
			if (scene == nullptr)
				return false;
			//���ļ��ж�ȡmesh��Ӧ�Ĳ�����Ϣ
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

	//�㼯����
	for (auto it = m_pointBuf.begin(); it != m_pointBuf.end(); ++it) {
		Point3D* p = *it;
		*p += offset;				//����λ�Ʊ任
	}

	//�߶θ���
	for (auto it = m_segmentBuf.begin(); it != m_segmentBuf.end(); ++it) {
		TerrainSegment* segment = *it;
		segment->Update();
	}

	//��Ԫ����
	for (auto it = m_facetBuf.begin(); it != m_facetBuf.end(); ++it) {
		TerrainFacet* facet = *it;
		facet->Update();
	}
}

void Terrain::Update(const Euler& posture)
{
	//�������弸��������Ϊ��ת����
	
	//�㼯��Ϣ
	for (auto it = m_pointBuf.begin(); it != m_pointBuf.end(); ++it) {
		Point3D* p = *it;
		*p -= m_centerPosition;												//λ����������ԭ�㡱
		*p *= posture;														//������ת����
		*p -= m_centerPosition;												//��λ����Ŀ��㴦
	}

	//�߶θ���
	for (auto it = m_segmentBuf.begin(); it != m_segmentBuf.end(); ++it) {
		TerrainSegment* segment = *it;
		segment->Update();
	}

	//��Ԫ����
	for (auto it = m_facetBuf.begin(); it != m_facetBuf.end(); ++it) {
		TerrainFacet* facet = *it;
		facet->Update();
	}

}

void Terrain::Update(const Vector3D& offset, const Euler& posture)
{
	Vector3D extraOffset = offset + m_centerPosition;
	//�������
	for (auto it = m_pointBuf.begin(); it != m_pointBuf.end(); ++it) {
		Point3D* p = *it;
		*p -= m_centerPosition;												//λ����������������ԭ��
		*p *= posture;														//������ת����
		*p += extraOffset;													//�����ۻ�λ��
	}

	m_centerPosition += offset;

	//�߶θ���
	for (auto it = m_segmentBuf.begin(); it != m_segmentBuf.end(); ++it) {
		TerrainSegment* segment = *it;
		segment->Update();
	}

	//��Ԫ����
	for (auto it = m_facetBuf.begin(); it != m_facetBuf.end(); ++it) {
		TerrainFacet* facet = *it;
		facet->Update();
	}
}

RtLbsType Terrain::GetObjectFoundationHeight(const Object2D* object) const
{
	//1-��ȡ��ά��������ڵ�������
	RtLbsType foundationHeight = FLT_MAX;
	for (int i = 0; i > object->m_segments.size(); ++i) {
		const Point2D& edgePoint = object->m_segments[i]->m_ps;
		TerrainFacet* facet = _getTerrainFacetViaPoint(edgePoint);
		if (facet == nullptr) {
			continue;
		}
		//����������ϵĸ߶�ֵ
		RtLbsType facetHeight = facet->GetFacetHeightViaPoint(edgePoint);
		if (facetHeight < foundationHeight) {								//�����Сֵ
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
	ray2d->m_Dir.Normalize(); //��ֵ����Ҫ��һ����ֵ
	//����������ཻĿǰΪ�����ཻģʽ gridģʽ��meshģʽ�Ϳ׶�ģʽ
	if (m_category == GRIDCELL) {						//����ģʽΪ��������ģʽ
		RtLbsType maxt;
		RtLbsType curt = m_bbox2d.Intersect(*ray2d, &maxt);
		if (curt < 0.0)
			return false;																		//��������ڰ�Χ���ⲿ
		if (curt == maxt)
			curt = 0.0;																			//�����ڰ�Χ���ڲ�����������

		int curGrid[2], dir[2];																	//ȷ������������ĸ�������
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
			//�����һ��tֵ
			RtLbsType target = gridCorner[i] + ((dir[i] + 1) >> 1) * m_voxelExtent[i];
			next[i] = (target - ray2d->m_Ori[i]) / ray2d->m_Dir[i];
		}

		//��������
		const unsigned array[] = { 0, 0, 1, 1 };												//[0],[3]�������ཻ
		while ( curt < maxt ) {

			//�õ�����ID
			unsigned voxelId = _offset(curGrid[0], curGrid[1]);

			//������һ��tֵ
			unsigned nextAxis = (next[0] <= next[1]) + ((unsigned)(next[1] <= next[0])) * 2;
			nextAxis = array[nextAxis];

			if (_getIntersect(rayInit, voxelId)) {
				delete ray2d;
				return true;
			}

			// get to the next voxel
			curGrid[nextAxis] += dir[nextAxis];

			if (curGrid[nextAxis] < 0 || (unsigned)curGrid[nextAxis] >= m_voxelNum[nextAxis]) { //����Խ�����Ȼû�н���
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
	else if (m_category == MESHNONHOLE) {					//����ģʽΪ������ģʽ
		TerrainFacet* curFacet;																	/** @brief	��ǰ���㵽����Ԫ	*/
		TerrainSegment* curSegment = nullptr;													/** @brief	��ǰ���㵽���߶�	*/
		TerrainSegment* prevSegment = nullptr;													/** @brief	��һ�����㵽���߶�	*/
		RtLbsType t_cur;																		/** @brief	��ǰ�������߶��ཻ�ľ����ά	*/
		RtLbsType h_cur;																		/** @brief	��ǰ�������߶��ཻ�Ľ���߶ȣ����ڱȽ��Ƿ��������ཻ��	*/
		curFacet = _getTerrainFacetViaPoint(ray2d->m_Ori);										//��ʼ����Ԫ
		if (ray2d->m_Dir.x == 0.0 && ray2d->m_Dir.y == 0.0) {
			delete ray2d;
			return curFacet->GetIntersect(rayInit);
		}

		while (true) {
			prevSegment = curSegment;
			curSegment = curFacet->GetIntersectSegment(ray2d, prevSegment, &t_cur, &h_cur);		//�����������Ԫ�ڲ����ཻ�߶�
			if (curSegment == nullptr)
				return false;		//�������޷�����ļ���������⣬����׷�ٸ�������
			RtLbsType rayHeight = rayInit->GetRayHeight(t_cur);
			if (rayHeight <= h_cur || (rayHeight - h_cur) <= 10) {								//���ߺ��߶εĸ߶Ȳ���10m���ڣ�����������������Ԫ��ģʽ��⾫ȷ��
				if (curFacet->GetIntersect(rayInit)) {								//���������뵱ǰ��Ԫ���ཻ���
					delete ray2d;
					return true;
				}
			}
			curFacet = curSegment->GetAdjacentFacet(curFacet);
			if (curFacet == nullptr)															//����������ԪΪ��ʱ���������㵽��Ŀ��߽磩
				break;
		}
		delete ray2d;
		return false;
	}
	else if (m_category == MESHHOLE) {					//����ģʽΪ����+��߻��ģʽ
		//ȷ��������ڵ�����Ԫ
		//������������Ԫ�����񣬲�ȷ������������ĸ���������
		//��ʼ���а�ߵ����������Ƿ��ཻ
		//��⵽�Ƿ񵽴��߽߱������߽磨���ཻ��
		RtLbsType maxt;
		RtLbsType curt = m_bbox2d.Intersect(*ray2d, &maxt);										//��������ΰ�Χ���ཻ�ľ���
		if (curt < 0.0) {																		//��������ڰ�Χ���ⲿ����Ҫ�����������Χ�еĽ������ڵĵ�ֵ
			ray2d->m_Ori = ray2d->GetRayCoordinate(curt);										//��������������߽��ϵ��������
			curt = 0.0;																			//��������
		}
		if (curt == maxt)
			curt = 0.0;																			//�����ڰ�Χ���ڲ�����������
		//���������ڵ�����Ԫ
		int curGrid[2];																			/** @brief	������ڵĵ�Ԫ	*/
		int dir[2];																				/** @brief	�������ķ���	*/
		RtLbsType delta[2];																		/** @brief	��һ��������Ҫ������	*/
		RtLbsType next[2];																		/** @brief	ȷ����һ���������������Ҫ�ı���	*/
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
			//�����һ��tֵ
			RtLbsType target = gridCorner[i] + ((dir[i] + 1) >> 1) * m_voxelExtent[i];
			next[i] = (target - ray2d->m_Ori[i]) / ray2d->m_Dir[i];
		}

		const unsigned array[] = { 0, 0, 1, 1 };												//[0],[3]�������ཻ
		while (curt < maxt) {

			//�õ�����ID
			unsigned voxelId = _offset(curGrid[0], curGrid[1]);

			//������һ��tֵ
			unsigned nextAxis = (next[0] <= next[1]) + ((unsigned)(next[1] <= next[0])) * 2;
			nextAxis = array[nextAxis];
			unsigned targetVoxelId = 0;
			//���ж��Ƿ��󽻣������ذ�ߵ��������������
			if (_getIntersect(rayInit,ray2d,curt,voxelId,targetVoxelId)) {																			//���ڰ�߷�����Ƿ��������Ԫ�ཻ�����ཻ�򷵻���
				delete ray2d;
				return true;
			}
			//��߷����ཻ�����ر߽�����ID
			_offset_reverse(targetVoxelId, curGrid[0], curGrid[1]);


			// get to the next voxel
			curGrid[nextAxis] += dir[nextAxis];

			if (curGrid[nextAxis] < 0 || (unsigned)curGrid[nextAxis] >= m_voxelNum[nextAxis]) { //����Խ�����Ȼû�н���
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
	//������ά���ߣ��ж��Ƿ��ཻ
	Ray3DLite* ray3d = new Ray3DLite(ps, pe);					//������ά����
	Point3D intersect;											//����
	if (GetIntersect(ray3d, &intersect)) {						
		Vector3D se = pe - ps;
		Vector3D si = intersect - ps;
		if (si.Length()<se.Length())							//�����㳤��С����ʼ�㳤�ȣ��򽻵���Ч���������ڵ�
			return true;
	}
	return false;
}

Material* Terrain::GetMaterial(const Point3D& p) const
{
	//�����p����cell�е�դ����
	//������ؼ���
	unsigned voxelId[2];						//����Id
	for (unsigned i = 0; i < 2; ++i) {
		voxelId[i] = _point2VoxelId(p, i);
	}
	unsigned offset = _offset(voxelId[0], voxelId[1]);		//����Id��һά�����еı��
	return m_gridCells[offset]->m_mat;						//���������еĲ�������
}

bool Terrain::IsValidPoint(const Point3D& p) const
{
	//�ж����Ƿ��ڰ�Χ��֮��
	if (!m_bbox3d.IsContainPoint(p))
		return false;
	//�������ڵ���������Ԫ
	TerrainFacet* facet = _getTerrainFacetViaPoint(p);
	if (!facet)
		return false;
	if (facet->GetVerticleDistanceToPoint(p) < 0)
		return false;
	return true;
}

bool Terrain::GetTerrainDiffractionPath(Point3D tx, Point3D rx, TerrainDiffractionPath*& outPath) const
{
	//����1-�����Ƿ��ڵ��������ص���ͶӰ·��

	TerrainFacet* txFacet = _getTerrainFacetViaPoint(tx);
	TerrainFacet* rxFacet = _getTerrainFacetViaPoint(rx);
	if (txFacet == rxFacet)//0-ȷ���շ���������ڵ���Ԫ������ͬ���򷵻�false
		return false;

	std::vector<Point3D> routeAboveRay;//�������߸߶ȵ�·���㼯��
	std::vector<Point3D> route;		//�շ�����ĵ��ε㼯��
	//��ⷢ����ڶ�Ӧ��Ԫ�ϵ�ͶӰ����
	Point3D txOnFacet = txFacet->GetPointOnPlane(tx); //���������Ԫ�ϵ�����   
	Point3D rxOnFacet = rxFacet->GetPointOnPlane(rx); //���ջ�����Ԫ�ϵ�����
	route.push_back(txOnFacet); //�Ƚ��������������·����
	//���ڰ����������ȷ�������������ĵ��α��������Ϣ
	//����������ά����
	Vector3D rt = rx - tx;
	Vector3D rt_dir = Normalize(rt);
	Ray3DLite* ray3d = new Ray3DLite();
	ray3d->m_ori = tx;
	ray3d->m_dir = rt_dir;
	//����������ά����
	Ray2D* ray2d = new Ray2D();
	ray2d->m_Ori.x = ray3d->m_ori.x;
	ray2d->m_Ori.y = ray3d->m_ori.y;
	ray2d->m_Dir.x = ray3d->m_dir.x;
	ray2d->m_Dir.y = ray3d->m_dir.y;
	ray2d->m_Dir.Normalize(); //��ֵ����Ҫ��һ����ֵ

	TerrainFacet* curFacet = txFacet; /** @brief	��ǰ���㵽����Ԫ	*/
	TerrainSegment* curSegment = nullptr; /** @brief	��ǰ���㵽���߶�	*/
	TerrainSegment* prevSegment = nullptr; /** @brief	��һ�����㵽���߶�	*/
	RtLbsType t_cur; /** @brief	��ǰ�������߶��ཻ�ľ����ά	*/
	RtLbsType h_cur; /** @brief	��ǰ�������߶��ཻ�Ľ���߶ȣ����ڱȽ��Ƿ��������ཻ��	*/
	RtLbsType t_max = rt.LengthXY();
	bool hasIntersect = false; //���߱������ڵ�״̬
	while (true) { //��������ȵ�λ��
		prevSegment = curSegment;
		curSegment = curFacet->GetIntersectSegment(ray2d, prevSegment, &t_cur, &h_cur);//�����������Ԫ�ڲ����ཻ�߶�
		if (curSegment == nullptr)	//�������޷������������⣬��ֹ·��ѭ��
			break;
		RtLbsType rayHeight = ray3d->GetRayHeight(t_cur);
		//�����ά�������߶εĽ��㲢����h_curת��Ϊ��ά����
		Point2D pInTerrain2D = (*ray2d)(t_cur);
		Point3D pInTerrain3D(pInTerrain2D.x, pInTerrain2D.y, h_cur);
		route.push_back(pInTerrain3D);
		if (rayHeight <= h_cur) {//��·����������ڽ������ߵ�·����
			routeAboveRay.push_back(pInTerrain3D);
		}
		if (rayHeight <= h_cur || (rayHeight - h_cur) <= 1) { //���ߺ��߶εĸ߶Ȳ���1m���ڣ�����������������Ԫ��ģʽ��⾫ȷ��
			if (curFacet->GetIntersect(ray3d))//���������뵱ǰ��Ԫ���ཻ���
				hasIntersect = true; //�뻷���ཻ
		}
		curFacet = curSegment->GetAdjacentFacet(curFacet); //������һ����Ԫ
		if (curFacet == rxFacet || t_cur >= t_max) //�����������Ԫ����볬�����������������break
			break;
	}
	route.push_back(rxOnFacet); //��ӽ��ջ�����·��

	TerrainProfile* profile = new TerrainProfile();
	//���·����ÿ���ڵ�ĵ�Ų�����Ϣ
	std::vector<Material*> mats(route.size());															/** @brief	���ε��������	*/
	for (int i = 0; i < route.size(); ++i)
		mats[i] = GetMaterial(route[i]);														//���·����ÿ����Ĳ���ID
	profile->InitParameters(route, mats, tx, rx, m_averageRidgeGap);
	profile->GetDiffractPathOverRidges(outPath);	//��profile��������·��
	if (outPath != nullptr) {								//���ҽ���outPath��Ϊnullptrʱʹ��
		outPath->m_terrainDiffractionMode = m_propagationProperty.m_terrainDiffractionMode;			//�����������ģʽ�б�
	}
	//profile.WriteRidgesToFile("ridges.txt");		//��profile�еķ���д�뵽�ļ���-������
	profile->WriteProfileToFile("profile.txt");		//��profile�еĵ�������д�뵽�ļ���-������
	delete profile;
	delete ray2d;
	delete ray3d;

	return true;
}

bool Terrain::GetTerrainReflectionPaths(const Point3D& tx, const Point3D& rx, std::vector<RayPath3D*>& outPath) const
{
	std::vector<TerrainFacet*> attachGroundFacets;																	/** @brief	�����ر����Ԫ����	*/
	if (!_getTerrainProfileFacets(tx, rx, attachGroundFacets))														//���������˵ر���Ԫ���ϣ��򷵻�false
		return false;

	std::vector<Point3D> intersectPoints;																			/** @brief	��Ч�Ľ��㣬�����淴���	*/
	//�����ر���Ԫ���ϣ����㾵��㲢�����������Ԫ�Ľ���
	for (auto it = attachGroundFacets.begin(); it != attachGroundFacets.end(); ++it) {
		const TerrainFacet* curFacet = *it;
		Point3D mirrorPoint = curFacet->GetMirrorPoint(tx);															/** @brief	������tx�ľ����	*/
		Ray3DLite ray3d(mirrorPoint, rx);																			/** @brief	���������ά����	*/
		Point3D intersectPoint;																						/** @brief	��������Ԫ�Ľ���	*/
		if (curFacet->GetIntersect(&ray3d, &intersectPoint)) {														//��ǰ��Ԫ������н��㣬������������
			RayPath3D* newRayPath = new RayPath3D();																/** @brief	��·��	*/
			PathNode3D* txNode = new PathNode3D(tx, NODE_ROOT);														/** @brief	���췢��ڵ�	*/
			PathNode3D* reflNode = new PathNode3D(intersectPoint, NODE_REFL, curFacet);								/** @brief	�����м䷴��ڵ�	*/
			PathNode3D * rxNode = new PathNode3D(rx, NODE_STOP);													/** @brief	������սڵ�	*/
			newRayPath->Union(txNode);
			newRayPath->Union(reflNode);
			newRayPath->Union(rxNode);
			newRayPath->m_type = RAYPATH_TERRAIN_REFLECTION;
			outPath.push_back(newRayPath);
		}
	}
	if (outPath.size() == 0)																						//��out��·������Ϊ0, �򷵻�false, Ѱ�ҵ��η���·����Ч
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
	m_category = MESHNONHOLE;					//Ĭ��ģʽΪmesh�޿׶�ģʽ
	m_terrainId = -1;							//Ĭ��IDΪ-1��������Ч
	m_simplifyFlag = false;						//Ĭ�ϲ���
	m_simplifyRate = 1.0;						//Ĭ�ϼ���Ϊ1,����
	m_averageRidgeGap = 100;					//Ĭ�ϵ����з��ͼ��Ϊ100m
}

bool Terrain::_initData(float* elevation, std::vector<Material*>& matMatrix, int rows, int cols, RtLbsType rowGap, RtLbsType colGap, RtLbsType minValue, RtLbsType maxValue, RtLbsType ratio)
{
	//դ��ģʽ��ʼ�����β���
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



	//�������񶥵�
	std::vector<SurfaceMesh::Vertex_index> vertices(rows * cols);
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			RtLbsType y = i * rowGap; //��-y����
			RtLbsType x = j * colGap; //��-x����
			RtLbsType z = elevation[(rows - 1 - i) * cols + j];							//�޸����ζ�ȡʱ��������
			vertices[i * cols + j] = m_meshes.add_vertex(Kernel::Point_3(x, y, z));
		}
	}
	//����������������Ԫ
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

	_build(matMatrix); //���߶κ���Ԫת����ɺ�ʼִ��build����
	LOG_INFO << "Terrain: geometry loading success." << ENDL;
	return true;
}

bool Terrain::_initData(const aiScene* scene, RtLbsType ratio, std::vector<Material*>& materials)
{
	//����-����Ԫ�򻯹����У���Ҫ���Ǽ򻯺����Ԫ���ǰ��Ԫ�Ĳ������ԵĶ�Ӧ��ϵ-�ѽ��
	//����2-�ڵ��γ�����ԪΪ��դ��������Ԫʱ����Ҫ���й�����Ӧ������,��ʱ��������
	m_calcMode = TERRAIN_HALFFACEMODE;																							//����assimp����,Ϊ��߼���ģʽ
	if (materials.size() != scene->mNumMeshes) {
		LOG_INFO << "Terrain: " << "can't match material num and scene meshes." << ENDL;
		return false;
	}
	for (unsigned int i = 0; i < scene->mNumMeshes; ++i) {																		//���object�е�ÿһ��mesh���м������ԺͲ�����Ϣ�ĸ�ֵ
		SurfaceMesh mesh;
		const aiMesh* aimesh = scene->mMeshes[i];
		Material* curMeshMaterial = materials[i];																				//��ǰmesh�Ĳ�������
		//��ȡ���嶥����Ϣ
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
		//��ȡ��Ԫ��Ϣ
		for (unsigned int j = 0; j < aimesh->mNumFaces; ++j) {
			aiFace aiface = aimesh->mFaces[j];
			std::vector<SurfaceMesh::Vertex_index> face(aiface.mNumIndices);
			for (unsigned int k = 0; k < aiface.mNumIndices; ++k) {
				face[k] = vertices[aiface.mIndices[k]];
			}
			mesh.add_face(face);
		}

		//�����Χ��
		m_bbox3d.Union(pmin);
		m_bbox3d.Union(pmax);
		m_bbox2d.Union(Point2D(pmin.x, pmin.y));
		m_bbox2d.Union(Point2D(pmax.x, pmax.y));

		if (m_simplifyFlag == true) {								//���м�
			if (!_simplify(m_simplifyRate))
				return false;										//���μ�ʧ��
		}
		if (!_transform(mesh)) {
			return false;
		}
	}

	unsigned squaredVoxelNum = static_cast<int>(trunc(sqrt(m_facetBuf.size())));			//����ƽ��������������������ٽṹ
	m_voxelNum[0] = squaredVoxelNum;
	m_voxelNum[1] = squaredVoxelNum;
	m_voxelExtent[0] = (m_bbox2d.m_max.x - m_bbox2d.m_min.x) / m_voxelNum[0];
	m_voxelExtent[1] = (m_bbox2d.m_max.y - m_bbox2d.m_min.y) / m_voxelNum[1];
	m_voxelInvExtent[0] = 1.0 / m_voxelExtent[0];
	m_voxelInvExtent[1] = 1.0 / m_voxelExtent[1];
	m_voxelCount = m_voxelNum[0] * m_voxelNum[1];

	_build();																			//���߶κ���Ԫת����ɺ�ʼִ��build����
	LOG_INFO << "Terrain: geometry loading success." << ENDL;
	return true;
}

void Terrain::_release()
{
	//��Ҫ�ͷ�pointBuf,facetBuf��segmentBuf

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

	//��cgal�ж�ȡ�㼯�����и�ֵ
	m_pointBuf.resize(mesh.num_vertices());
	int pid = 0;																				//������ID
	for (auto it = mesh.points().begin(); it != mesh.points().end(); ++it) {
		const Kernel::Point_3& p = *it;
		m_pointBuf[pid] = new Point3D(p.x(), p.y(), p.z(), pid);
		pid++;
	}


	//���㼯��ƽ��ֵ����ģ�ͼ�������
	for (auto it = m_pointBuf.begin(); it != m_pointBuf.end(); ++it) {
		Point3D* p = *it;
		m_centerPosition += *p;
	}
	m_centerPosition /= static_cast<RtLbsType>(m_pointBuf.size());								//��⼸������

	//��cgal�ж�ȡ��������Ԫ�����г�ʼ��
	unsigned facetOffset = static_cast<unsigned>(m_facetBuf.size());
	m_facetBuf.resize(facetOffset + mesh.faces().size());
	Point3D pointTemp[3]; //��ʱ���ڽ��������
	std::unordered_map<int, int> facetIdMap; //��Ԫ����ӳ���
	int pointId = 0;
	int facetId = facetOffset;
	int p_offset[3];																			//�������������������ڵ㼯�е�ƫ��ֵ
	for (auto itFacet = mesh.faces_begin(); itFacet != mesh.faces_end(); ++itFacet) {
		facetIdMap[(*itFacet).idx()] = facetId;
		auto itRangeVertice = mesh.vertices_around_face(mesh.halfedge(*itFacet));
		if (itRangeVertice.size() == 3) { //��֤����Ϊ��������Ԫ
			pointId = 0;
			//������Ԫ�������ڵ����������е�ƫ��ֵ
			for (const auto& vertex_index : itRangeVertice) {
				p_offset[pointId] = vertex_index.idx();
				pointId++;
			}
			//��ʼ��ֵ������
			m_facetBuf[facetId++] = new TerrainFacet(facetId, m_pointBuf[p_offset[2]], m_pointBuf[p_offset[1]], m_pointBuf[p_offset[0]], mat); //����CGAL�ж���ζ���Ϊ˳�������Ϊ��������
		}
	}

	//����ȫ�ߣ������й�ϣӳ��Ȼ�󴴽����
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

	//Ѱ��˫���
	int numValidEdges = 0;															//��Ч�ı�����������˫��ߺ͵����
	std::vector<TerrainSegment*> validSegments; //��Ч�ı�
	std::unordered_map<size_t, TerrainSegment*> segmentMap; //Ѱ����ͬ���õ��ṹ
	for (auto it = segments.begin(); it != segments.end(); ++it) {
		TerrainSegment* segment = *it;
		size_t hash = segment->GetHash();
		if (segmentMap.find(hash) == segmentMap.end()) { //�Ҳ�����ǰhashֵ,�򽫵�ǰ���ݷ���hashֵ��
			segmentMap[hash] = segment;
		}
		else { //Ѱ�ҵ��ظ���hashֵ������˫��߽ṹ
			segmentMap[hash]->m_facet2 = segment->m_facet1;//��ֵ�߶ε���һ����Ԫ
			TerrainSegment* newSegment = new TerrainSegment(*segmentMap[hash]);
			newSegment->m_isShared = true;									//����߸�ֵ
			newSegment->m_segmentId = numValidEdges + edgeOffset; //��ֵ�߶ε�id
			validSegments.push_back(newSegment);
			numValidEdges++;
		}
	}

	//Ѱ�ҵ����
	for (auto it = segmentMap.begin(); it != segmentMap.end(); ++it) {
		TerrainSegment* segment = (*it).second;
		if (segment->m_facet2 == nullptr) {
			TerrainSegment* newSegment = new TerrainSegment(*segment);
			newSegment->m_segmentId = numValidEdges + edgeOffset;
			newSegment->m_isShared = false;									//�ǹ���߸�ֵ
			validSegments.push_back(newSegment);
			numValidEdges++;
		}
	}
	
	//����ԭʼ������
	for (auto it = segments.begin(); it != segments.end();++it) {
		delete* it;
	}
	segments.clear();

	//������Ԫ�е�segment����
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


	//ѭ��������Ԫ�������Ԫ���Ƿ����������ж��Ƿ��Ǳ߽���Ԫ
	for (auto it = m_facetBuf.begin(); it != m_facetBuf.end(); ++it) {
		TerrainFacet* facet = *it;
		if (facet->m_segment1->m_isShared == true ||
			facet->m_segment2->m_isShared == true ||
			facet->m_segment3->m_isShared == true) {			//���ҽ�����Ԫ��������һ���߽���Ԫʱ������ԪΪ�߽���Ԫ
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
	m_gridCells.resize(m_voxelCount);//��ʼ����������
	int id = 0;
	Point2D cornerPoint = m_bbox2d.m_min;
	if (!m_simplifyFlag) {				//�����м�
		//��m_facets�ж�ȡ���ݲ����м��أ�����Ϊ��row(y)��col(x)
		for (unsigned y = 0; y < m_voxelNum[1]; ++y) {		//�ȱ���y����
			for (unsigned x = 0; x < m_voxelNum[0]; ++x) {	//�ٱ���x����
				int cellId = _offset(x, y);
				TerrainFacet* facet1 = m_facetBuf[2 * cellId];
				TerrainFacet* facet2 = m_facetBuf[2 * cellId + 1];
				m_gridCells[cellId] = new TerrainCell();
				m_gridCells[cellId]->m_cornerPoint[0] = cornerPoint[0] + x * m_voxelExtent[0];
				m_gridCells[cellId]->m_cornerPoint[1] = cornerPoint[1] + y * m_voxelExtent[1];
				m_gridCells[cellId]->m_facets.push_back(facet1);
				m_gridCells[cellId]->m_facets.push_back(facet2);
				m_gridCells[cellId]->m_mat = matMatrix[cellId];//ִ�в��ʸ�ֵ

			}
		}
	}
	else {			//�򻯺���ð��ģʽ������Ԫ
		//�Ƚ���cell��ʼ��
		for (unsigned y = 0; y < m_voxelNum[1]; ++y) {//�ȱ���y����
			for (unsigned x = 0; x < m_voxelNum[0]; ++x) {//�ٱ���x����
				int cellId = _offset(x, y);
				m_gridCells[cellId] = new TerrainCell();
				m_gridCells[cellId]->m_cornerPoint[0] = cornerPoint[0] + x * m_voxelExtent[0];
				m_gridCells[cellId]->m_cornerPoint[1] = cornerPoint[1] + y * m_voxelExtent[1];
				m_gridCells[cellId]->m_mat = matMatrix[cellId];//ִ�в��ʸ�ֵ
				id++;
			}
		}

		//��Բ�������Ԫ���и�ֵ����Ҫ��ֵդ������
		for (auto it = m_facetBuf.begin(); it != m_facetBuf.end(); ++it) {
			TerrainFacet* facet = *it;
			unsigned maxGridId[2];
			unsigned minGridId[2];
			for (int i = 0; i < 2; ++i) {
				minGridId[i] = _point2VoxelId(facet->m_bbox.m_min, i);
				maxGridId[i] = _point2VoxelId(facet->m_bbox.m_max, i);
			}

			//ִ�а�Χ�и�ֵ
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
	m_gridCells.resize(m_voxelCount);//��ʼ����������
	int id = 0;
	Point2D cornerPoint = m_bbox2d.m_min;
	for (unsigned y = 0; y < m_voxelNum[1]; ++y) {//�ȱ���y����
		for (unsigned x = 0; x < m_voxelNum[0]; ++x) {//�ٱ���x����
			m_gridCells[id] = new TerrainCell();
			m_gridCells[id]->m_cornerPoint[0] = cornerPoint[0] + x * m_voxelExtent[0];
			m_gridCells[id]->m_cornerPoint[1] = cornerPoint[1] + y * m_voxelExtent[1];
			id++;
		}
	}

	//2����߰汾,��Ҫ��ֵ��Ԫ�ͱߣ���Ԫ�ṩ��ѯ,���ṩ�ཻ�ж�
	if (m_category == MESHNONHOLE || m_category == MESHHOLE) {						//���׶��Ͳ����׶��汾
		//��Բ�������Ԫ���и�ֵ����Ҫ��ֵդ������
		for (auto it = m_facetBuf.begin(); it != m_facetBuf.end(); ++it) {
			TerrainFacet* facet = *it;
			unsigned maxGridId[2];
			unsigned minGridId[2];
			for (int i = 0; i < 2; ++i) {
				minGridId[i] = _point2VoxelId(facet->m_bbox.m_min, i);
				maxGridId[i] = _point2VoxelId(facet->m_bbox.m_max, i);
			}

			//ִ�а�Χ�и�ֵ
			for (unsigned y = minGridId[1]; y <= maxGridId[1]; ++y) {
				for (unsigned x = minGridId[0]; x <= maxGridId[0]; ++x) {
					unsigned offset = _offset(x, y);
					m_gridCells[offset]->m_facets.push_back(facet);
					m_gridCells[offset]->m_mat = facet->m_mat;			//������Ϣ��ֵ
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
	//������������㷨���о�ȷ��ֵ
	int curGrid[2], endGrid[2], dir[2]; //����ǰ��š�������ֹ��š������н�����
	RtLbsType delta[2], next[2];
	RtLbsType maxt = segment->GetLengthXY();
	RtLbsType curt = 0.0;
	Vector2D s_dir = segment->GetDirXY();
	const Point2D& sp = segment->GetStartPoint2D();
	const Point2D& ep = segment->GetEndPoint2D();

	//���㵥λ����ֵ
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
	//����Ŀ��ֵ
	Point2D& curGridCorner = m_gridCells[voxelId]->m_cornerPoint;
	for (int i = 0; i < 2; ++i) {
		RtLbsType target = curGridCorner[i] + ((dir[i] + 1) >> 1) * m_voxelExtent[i];
		next[i] = (target - sp[i]) / s_dir[i];
		if (delta[i] == FLT_MAX)
			next[i] = FLT_MAX;
	}

	//��������������ر��
	const unsigned array[] = { 0,0,1,1 };//[0]��[3]�������ཻ
	while (curt < maxt) {
		voxelId = _offset(curGrid[0], curGrid[1]);
		ids.push_back(voxelId); //���ɼ����������ż�¼��vector��
		//������һ��tֵ
		unsigned nextAxis = (next[0] <= next[1]) + ((unsigned)(next[1] <= next[0])) * 2;
		nextAxis = array[nextAxis];
		if (curt < 0) {
			LOG_ERROR << "_getGridCoordAlongSegment:" << "curt < 0" << CRASH;
		}
		//��������һ��voxel
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
	//�ж�voxel����Ԫ�Ƿ�����涨��Ҫ��
	TerrainCell* curCell = m_gridCells[voxelId];								//��ȡ��ǰ������Ԫ
	if (curCell->m_facets.size() == 0) {										//��ǰ����Ԫ�е���Ԫ����Ϊ0�����߱��ཻ����
		return false;
	}
	else {
		//���������ڵ���Ԫ
		Point2D targetPoint = ray2d->GetRayCoordinate(t);						/** @brief	��ȡ��������t���������ϵ�����	*/
		TerrainFacet* curFacet = _getTerrainFacetViaPoint(targetPoint);		/** @brief	��ȡĿ�������ϵ���Ԫ	*/
		if (curFacet == nullptr) {					//����ԪΪ�գ������������߽���Ԫ�������
			//����������cell�еı߽���Ԫ�ཻ�����Ȼ��ʹ�ð�ߵ��������������
			TerrainFacet* nextFacet = nullptr;
			if (!curCell->GetIntersectEdgeFacet(ray2d, ray3d, curFacet, nextFacet)) {
				return false;				//��ǰ��Ԫ���������ɹ���߷����ı߽���Ԫ������false
			}
			curFacet = nextFacet;
		}
		//��֤��Ԫ��Ϊ�գ����ð�ߵ�������
		TerrainSegment* curSegment = nullptr;							/** @brief	��ǰ���㵽���߶�	*/
		TerrainSegment* prevSegment = nullptr;							/** @brief	��һ�����㵽���߶�	*/
		RtLbsType t_cur;												/** @brief	��ǰ�������߶��ཻ�ľ����ά	*/
		RtLbsType h_cur;												/** @brief	��ǰ�������߶��ཻ�Ľ���߶ȣ����ڱȽ��Ƿ��������ཻ��	*/
		while (curFacet != nullptr) {
			prevSegment = curSegment;
			curSegment = curFacet->GetIntersectSegment(ray2d, prevSegment, &t_cur, &h_cur);		//�����������Ԫ�ڲ����ཻ�߶�
			if (curSegment == nullptr)
				return false;		//�������޷������������⣬����׷�ٸ�������
			RtLbsType rayHeight = ray3d->GetRayHeight(t_cur);									/** @brief	��ά�����ϵĸ߶�ֵ	*/
			if (rayHeight <= h_cur || (rayHeight - h_cur) <= 0.5) {								//���ߺ��߶εĸ߶Ȳ���10m���ڣ�����������������Ԫ��ģʽ��⾫ȷ��
				if (curFacet->GetIntersect(ray3d)) {								//���������뵱ǰ��Ԫ���ཻ���
					return true;
				}
			}
			curFacet = curSegment->GetAdjacentFacet(curFacet);									//������һ����Ԫ
			if (curFacet == nullptr) {															//�������߽���Ԫ����Ҫ�б�cell�ڻ��������߽���Ԫ��֮�ཻ
				//���¶�ά������ʼ����-���Լ���������voxel����һ���߽���Ԫ�Ľ���
				TerrainFacet* nextFacet = nullptr;
				if (curCell->GetIntersectEdgeFacet(ray2d, ray3d, curFacet, nextFacet)) {
					curFacet = nextFacet;														//�ڵ�ǰcell�л����������߽���Ԫ������������Ԫ������
					continue;
				}
				//���ڵ�ǰcell�в����ڱ߽���Ԫ����ֱ�ӷ���false�����ཻ
				return false;
			}
		}
		return false;				//��ǰ��ԪΪ����Ԫ������false
	}

}

TerrainFacet* Terrain::_getTerrainFacetViaPoint(const Point2D& point) const
{
	//1-��������ĸ�դ����
	int gridId[2];
	for (unsigned i = 0; i < 2; ++i) {
		gridId[i] = _point2VoxelId(point, i);
	}
	int voxelId = _offset(gridId[0], gridId[1]);
	//2-����դ������Ԫ�Ƿ������Ӧ�����
	TerrainCell* cell = m_gridCells[voxelId];
	for (TerrainFacet* facet : cell->m_facets) {
		if (facet->CheckInside(point))
			return facet;
	}
	return nullptr; //����鵽��Ԫ�������κ���Ԫ���򷵻�nullptr
}

TerrainFacet* Terrain::_getTerrainFacetViaPoint(const Point3D& point) const
{
	//1-��������ĸ�դ����
	int gridId[2];
	for (unsigned i = 0; i < 2; ++i) {
		gridId[i] = _point2VoxelId(point, i);
	}
	int voxelId = _offset(gridId[0], gridId[1]);
	//2-����դ������Ԫ�Ƿ������Ӧ�����
	TerrainCell* cell = m_gridCells[voxelId];
	for (TerrainFacet* facet : cell->m_facets) {
		if (facet->CheckInside(point))
			return facet;
	}
	return nullptr; //����鵽��Ԫ�������κ���Ԫ���򷵻�nullptr
}

bool Terrain::_getTerrainProfileFacets(const Point3D& txPosition, const Point3D& rxPosition, std::vector<TerrainFacet*>& outFacets) const
{
	TerrainFacet* txFacet = _getTerrainFacetViaPoint(txPosition);
	TerrainFacet* rxFacet = _getTerrainFacetViaPoint(rxPosition);
	if (txFacet == rxFacet) {																				//���շ���������ͬһ����Ԫ�ϣ���ֱ�Ӷ������Ԫ����������
		outFacets.resize(1);
		outFacets[0] = txFacet;
		return true;
	}

	Point3D txOnFacet = txFacet->GetPointOnPlane(txPosition);												//���������Ԫ�ϵ����� 
	Point3D rxOnFacet = rxFacet->GetPointOnPlane(rxPosition);												//���ջ�����Ԫ�ϵ�����

	//-----------------------------------���ڰ�߷���ȷ���������������ĵ��α��������Ϣ-----------------------------------------------------------
	
	Vector3D rt = rxPosition - txPosition;
	Vector3D rt_dir = Normalize(rt);																		//����������ά����
	Ray3DLite* ray3d = new Ray3DLite();
	ray3d->m_ori = txPosition;
	ray3d->m_dir = rt_dir;

	Ray2D* ray2d = new Ray2D();																				//����������ά����
	ray2d->m_Ori.x = ray3d->m_ori.x;
	ray2d->m_Ori.y = ray3d->m_ori.y;
	ray2d->m_Dir.x = ray3d->m_dir.x;
	ray2d->m_Dir.y = ray3d->m_dir.y;
	ray2d->m_Dir.Normalize();																				//��ֵ����Ҫ��һ����ֵ

	TerrainFacet* curFacet = txFacet;																		/** @brief	��ǰ���㵽����Ԫ	*/
	TerrainSegment* curSegment = nullptr;																	/** @brief	��ǰ���㵽���߶�	*/
	TerrainSegment* prevSegment = nullptr;																	/** @brief	��һ�����㵽���߶�	*/
	RtLbsType t_cur;																						/** @brief	��ǰ�������߶��ཻ�ľ����ά	*/
	RtLbsType h_cur;																						/** @brief	��ǰ�������߶��ཻ�Ľ���߶ȣ����ڱȽ��Ƿ��������ཻ��	*/
	RtLbsType t_max = rt.LengthXY();
	outFacets.push_back(curFacet);																			//����ǰ�ĵ�����Ԫ���������Ԫ������

	while (true) {																							//��Ԫ������
		prevSegment = curSegment;																			//�߶ε�����
		curSegment = curFacet->GetIntersectSegment(ray2d, prevSegment, &t_cur, &h_cur);						//�����������Ԫ�ڲ����ཻ�߶�
		if (curSegment == nullptr)																			//�������޷������������⣬��ֹ·��ѭ��
			break;
		curFacet = curSegment->GetAdjacentFacet(curFacet);													//������һ����Ԫ
		outFacets.push_back(curFacet);																		//������Ԫ
		if (curFacet == rxFacet || t_cur >= t_max)															//�����������Ԫ����볬�����������������break
			break;
	}
	if (outFacets.size() != 0)
		return false;
	return true;
}

