#ifndef RTLBS_PATHINFOCLUSTER
#define RTLBS_PATHINFOCLUSTER

#include "rtlbs.h"
#include "utility/define.h"
#include "pathinfo.h"


//·����Ϣ�����ھ���ʹ��
class PathInfoCluster {
public:
	std::vector<PathInfo> m_infos;								/** @brief	��Ҫ���е���Ϣ����	*/
	PathInfo m_mergedInfo;										/** @brief	�ϲ�����Ϣ	*/

public:
	PathInfoCluster();
	PathInfoCluster(const PathInfoCluster& cluster);
	~PathInfoCluster();
	PathInfoCluster& operator = (const PathInfoCluster& cluster);
	bool CanAddToClusterByAOA2D(const PathInfo& info, RtLbsType threshold) const;			//���ն�ά�Ƕ��Ƿ��������
	bool CanAddToClusterByAOA3D(const PathInfo& info, RtLbsType threshold) const;			//������ά�Ƕ��Ƿ��������
	bool CanAddToClusterByDelay(const PathInfo& info, RtLbsType threshold) const;			//����ʱ���Ƿ��������
	void CalMergedInfoByAOA();																//����ϳ���Ϣ,���Ȱ��սǶ���Ϣ��
	void CalMergedInfoByDelay();															//����ϳ���Ϣ,���Ȱ���ʱ����Ϣ��
};

//���չ��ʴ�С�������򣬴Ӵ�С
inline bool ComparedByPower_PathInfoCluster(const PathInfoCluster& cluster1, const PathInfoCluster& cluster2) {
	return cluster1.m_mergedInfo.m_power > cluster2.m_mergedInfo.m_power;
}

//����ʱ�ӵĴ�С�������򣬴�С����
inline bool ComparedByDelay_PathInfoCluster(const PathInfoCluster& cluster1, const PathInfoCluster& cluster2) {
	return cluster1.m_mergedInfo.m_timeDelay < cluster2.m_mergedInfo.m_timeDelay;
}

//����AOA��ά�ǶȽ��о���
inline std::vector<PathInfoCluster> ClusterPathInfoByAOA2D(std::vector<PathInfo>& infos, RtLbsType threshold)
{
	//�Ƚ��о���
	std::vector<PathInfoCluster> clusters;								/** @brief	��Ҫ�۳ɵ��༯��	*/

	for (auto& curInfo : infos) {
		bool addFlag = false;
		for (auto& curCluster : clusters) {
			if (curCluster.CanAddToClusterByAOA2D(curInfo, threshold)) {
				curCluster.m_infos.push_back(curInfo);					//���¼��������
				curCluster.CalMergedInfoByAOA();
				addFlag = true;
			}
		}
		if (!addFlag) {													//��û�ڵ�ǰ���ڵ�cluster�о��࣬��Ӧ�ù�������
			PathInfoCluster newCluster;
			newCluster.m_infos.push_back(curInfo);
			newCluster.CalMergedInfoByAOA();							//���¼��������
			clusters.push_back(newCluster);
		}
	}

	return clusters;
}

//����AOA��ά�ǶȽ��о���
inline std::vector<PathInfoCluster> ClusterPathInfoByAOA3D(std::vector<PathInfo>& infos, RtLbsType threshold)
{
	std::vector<PathInfoCluster> clusters;								/** @brief	��Ҫ�۳ɵ��༯��	*/

	for (auto& curInfo : infos) {
		bool addFlag = false;
		for (auto& curCluster : clusters) {
			if (curCluster.CanAddToClusterByAOA3D(curInfo, threshold)) {
				curCluster.m_infos.push_back(curInfo);
				curCluster.CalMergedInfoByAOA();
				addFlag = true;
			}
		}
		if (!addFlag) {													//��û�ڵ�ǰ���ڵ�cluster�о��࣬��Ӧ�ù�������
			PathInfoCluster newCluster;
			newCluster.m_infos.push_back(curInfo);
			newCluster.CalMergedInfoByAOA();
			clusters.push_back(newCluster);
		}
	}

	return clusters;
}

//����ʱ�ӽ��о���
inline std::vector<PathInfoCluster> ClusterPathInfoByDelay(std::vector<PathInfo>& infos, RtLbsType threshold)
{
	std::vector<PathInfoCluster> clusters;								/** @brief	��Ҫ�۳ɵ��༯��	*/

	for (auto& curInfo : infos) {
		bool addFlag = false;
		for (auto& curCluster : clusters) {
			if (curCluster.CanAddToClusterByDelay(curInfo, threshold)) {
				curCluster.m_infos.push_back(curInfo);
				curCluster.CalMergedInfoByDelay();
				addFlag = true;
			}
		}
		if (!addFlag) {													//��û�ڵ�ǰ���ڵ�cluster�о��࣬��Ӧ�ù�������
			PathInfoCluster newCluster;
			newCluster.m_infos.push_back(curInfo);
			newCluster.CalMergedInfoByDelay();
			clusters.push_back(newCluster);
		}
	}

	return clusters;
}
#endif
