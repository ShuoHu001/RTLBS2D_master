#ifndef RTLBS_PATHNODE
#define RTLBS_PATHNODE


#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "geometry/Intersection2D.h"
#include "physical/limitinfo.h"

class Point2D;
class Segment2D;
class PathNode3D;

class PathNode {

public:
	
	LimitInfo m_limitInfo;/** @brief	�ڵ�������Ϣ	*/
	PATHNODETYPE m_type;/** @brief	�ڵ�����	*/
	Point2D m_point;/** @brief	�ڵ�����	*/
	Segment2D* m_segment;/** @brief	�ڵ�������Ԫ��ָ��	*/
	Wedge2D* m_wedge;/** @brief	�ڵ�����wedge��ָ��	*/
	Ray2D m_prevRay; /** @brief	�ڵ�֮ǰ������(���ñ���)	*/
	Ray2D m_nextRay; /** @brief	�ڵ�֮�������(���ñ���)	*/
	bool m_bTraceBack; /** @brief	�Ƿ�Ϊ�ɻ��ݽڵ㣬�����ù���Դ��������·����������͸��ڵ�Ľ�ֹͨ������Դ����	*/
	
	Point2D m_source;	/** @brief	����Դ�������	*/
	RtLbsType m_ft;		/** @brief	��ǰ�ڵ����rootԴ�ľ���	*/  

public:
	PathNode();//ͳһ���������е�pathnode�洢Ϊǰһ���ڵ��ray
	PathNode(const LimitInfo& limitInfo, PATHNODETYPE type, Point2D point);															//���ڵ�ĳ�ʼ��
	PathNode(const LimitInfo& limitInfo, PATHNODETYPE type, Point2D point, const Ray2D& prevRay);											//�Ӿ�ڵ�ĳ�ʼ��/��ֹ�ڵ�ĳ�ʼ��
	PathNode(const LimitInfo& limitInfo, PATHNODETYPE type, Point2D point, Segment2D* primitive, const Ray2D& prevRay);					//������ֹ�ڵ�ĳ�ʼ��
	PathNode(const LimitInfo& limitInfo, PATHNODETYPE type, Point2D point, Segment2D* primitive, const Ray2D& prevRay, const Ray2D& nextRay); //����ڵ�ĳ�ʼ��
	PathNode(const LimitInfo& limitInfo, PATHNODETYPE type, Point2D point, Wedge2D* wedge, const Ray2D& prevRay, const Ray2D& nextRay);		//����ڵ�ĳ�ʼ��
	PathNode(const LimitInfo& limitInfo, PATHNODETYPE type, Point2D point, Wedge2D* wedge, const Ray2D& prevRay);							//������ֹ�ڵ�ĳ�ʼ��
	PathNode(const PathNode& pathnode);         //���ƽڵ�
	~PathNode() {};
	PathNode& operator = (PathNode& node);//��ֵ���������
	bool IsContainPointByAngle(Point2D p); //��ǰ�ڵ��Ƿ��ڽǶ����ڲ���ĳ������
};

#endif
