#ifndef RTLBS_ENUM
#define RTLBS_ENUM

enum LOGLEVEL {
	LEVEL_INFO,
	LEVEL_WARNING,
	LEVEL_ERROR
};

/** @brief	Ӳ��ģʽ	*/
enum HARDWAREMODE {
	CPU_SINGLETHREAD,			/** @brief	CPU���߳�	*/
	CPU_MULTITHREAD,			/** @brief	CPU���߳�	*/
	GPU_MULTITHREAD,			/** @brief	GPU���߳�	*/
	CPU_GPU_MULTITHREAD			/** @brief	CPU_GPU��϶��߳�	*/
};

/** @brief	��������	*/
enum OBJECT2DCATEGORY {
	BUILDING2D,					/** @brief	��ά������	*/
	VEGETATION2D,				/** @brief	��ά�̻���	*/
	WALL2D						/** @brief	��άǽ��	*/
};

/** @brief	������ѡ��ģʽ	*/
enum ACCEL_TYPE {
	ACCEL_NONE = 0, /** @brief	�������ģʽ	*/
	ACCEL_BVH = 1,	/** @brief	BVH���ٽṹ��	*/
	ACCEL_KD = 2, /** @brief	KDTree���ٽṹ��*/
	ACCEL_UG = 3,	/** @brief	����������ٽṹ��	*/
	ACCEL_FastUG = 4,	/** @brief	�Ż��ľ���������ٽṹ��	*/
	ACCEL_SDF = 5	/** @brief	���볡���ٽṹ��	*/
};

/** @brief	���߷���ģʽ	*/
enum RAYLAUNCHMODE {
	SINGLE_DIRECTION,				/** @brief	��һ����	*/
	UNIFORM_ICOSAHEDRON,			/** @brief	����ʮ������ȷ���	*/
};

/** @brief	����ѡ��ģʽ	*/
enum ANT_MODE {
	MODE_INTERNAL = 0, /** @brief	����ģʽ	*/
	MODE_EXTERNAL = 1  /** @brief	����ģʽ	*/
};

/** @brief	��������	*/
enum ANT_TYPE {
	TYPE_OMNI = 0, /** @brief	ȫ������	*/
	TYPE_DIPO = 1  /** @brief	ż��������	*/
};

enum PATHNODETYPE {
	NODE_ROOT,				/** @brief	���ڵ�	*/
	NODE_LOS,				/** @brief	�Ӿ�ڵ�	*/
	NODE_REFL,				/** @brief	����ڵ�	*/
	NODE_TRANIN,			/** @brief	͸����ڵ�	*/
	NODE_TRANOUT,			/** @brief	͸����ڵ�	*/
	NODE_ETRANIN,			/** @brief	����͸����ڵ�	*/
	NODE_ETRANOUT,			/** @brief	����͸����ڵ�	*/
	NODE_DIFF,				/** @brief	����ڵ�	*/
	NODE_REFLDIFF,			/** @brief	����+����ڵ�	*/
	NODE_SCAT,				/** @brief	ɢ��ڵ�	*/
	NODE_INIT,				/** @brief	��ʼ�ڵ�	*/
	NODE_STOP				/** @brief	��ֹ�ڵ�	*/
};

enum SYSTEM_MODE {
	MODE_RT = 0, /** @brief	���߸����㷨ģʽ	*/
	MODE_LBS = 1 /** @brief	��λģʽ	*/
};

/** @brief	��λ����	*/
enum LOCALIZATION_METHOD {
	LBS_METHOD_RT_AOA = 0,				/** @brief	RT-AOA ��λ	*/
	LBS_METHOD_RT_TDOA = 1,				/** @brief	RT-TDOA ��λ	*/
	LBS_METHOD_RT_TOA = 2,				/** @brief	RT_TOA��λ	*/
	LBS_METHOD_RT_AOA_TOA = 3,			/** @brief	RT-AOA-TOA ��λ	*/
	LBS_METHOD_RT_AOA_TDOA = 4,			/** @brief	RT_AOA_TDOA ��λ	*/
};

/** @brief	��λģʽ	*/
enum LOCALIZATION_MODE {
	LBS_MODE_SPSTMD,				/** @brief	��վ��Դ�����ݽ��ж�λ	*/
	LBS_MODE_MPSTSD,				/** @brief	��վ��Դ�����ݽ��ж�λ	*/
	LBS_MODE_SPMTMD,				/** @brief	��վ��Դ�����ݽ��ж�λ	*/
	LBS_MODE_MPMTMD					/** @brief	��վ��Դ�����ݽ��ж�λ	*/
};

/** @brief	���ն�Ԥ��ģʽ	*/
enum PREDICTION_MODE {
	PREDICTION_FILE = -1,			/** @brief	���ļ��м���Ԥ��	*/
	PREDICTION_SINGLEPOINT = 0,		/** @brief	������Ԥ��	*/
	PREDICTION_LINE = 1,			/** @brief	·��Ԥ��	*/
	PREDICTION_SCARRERPOINT = 2,	/** @brief	��ɢ����Ԥ��	*/
	PREDICTION_PLANE = 3,			/** @brief	ƽ��Ԥ��	*/
	PREDICTION_SOLID = 4,			/** @brief	������Ԥ��	*/
};

/** @brief	���ζ�ȡģʽ	*/
enum TERRAINLOADMODE {
	TERRAIN_OBJECT,						/** @brief	obj ����ģʽ��ȡ	*/
	TERRAIN_GRID,						/** @brief	դ��ģʽ��ȡ	*/
};

enum TERRAINCATEGORY {
	GRIDCELL,							/** @brief	դ������	*/
	MESHHOLE,							/** @brief	���п׶�������	*/
	MESHNONHOLE							/** @brief	�����׶�������	*/
};

/** @brief		*/
enum WINDING_ORDER {
	CLOCKWISE = 0,			/** @brief	˳ʱ�� DirectX����Ⱦ�����ļ�	*/
	COUNTCLOCKWISE = 1		/** @brief	��ʱ�� OpenGL����Ⱦ�����ļ�	*/
};

/** @brief	��������ģʽ	*/
enum TERRAINDIFFRACTIONMODE {
	DIFFRACTIONMODE_PICQUENARD,			/** @brief	Picquenard's model ��������	*/
	DIFFRACTIONMODE_EPSTEIN,			/** @brief	Epstein Peterson ��������	*/
	DIFFRACTIONMODE_HUSHI,				/** @brief	HuShuo's model ��������	*/
	DIFFRACTIONMODE_UTD					/** @brief	�����������䷽������	*/
};


/** @brief	���μ���ģʽ�����ټ��㷽�棩	*/
enum TERRAIN_CALCMODE {
	TERRAIN_HALFFACEMODE,					/** @brief	��߷���������	*/
	TERRAIN_GRIDMODE,						/** @brief	դ����������	*/
};

/** @brief	���߷���ͼ����	*/
enum ANTENNAPATTERN_TYPE {
	PATTERN2D,							/** @brief	��ά����ͼ	*/
	PATTERN3D,							/** @brief	��ά����ͼ	*/
};

/** @brief	·������	*/
enum RAYPATHTYPE {
	RAYPATH_COMMON,								/** @brief	��������	*/
	RAYPATH_TERRAIN_REFLECTION,					/** @brief	���淴��	*/
	RAYPATH_TERRAIN_DIFFRACTION					/** @brief	��������	*/
};

/** @brief	���ģʽ	*/
enum SOLVINGMODE {
	SOLVING_LS,							/** @brief	��С���˷�	*/
	SOLVING_WLS,						/** @brief	��Ȩ��С���˷�	*/
	SOLVING_IRLS,						/** @brief	������С���˷�	*/
	SOLVING_WIRLS,						/** @brief	��Ȩ������С���˷�	*/
};

/** @brief	��ʧ��������	*/
enum LOSSFUNCTIONTYPE {
	LOSS_NONE,							/** @brief	����ʧ����	*/
	LOSS_HUBER,							/** @brief	Huber ��ʧ����	*/
	LOSS_CAUCHY,						/** @brief	Cauchy ��ʧ����	*/
	LOSS_ARCTAN,						/** @brief	Arctan ��ʧ����	*/
	LOSS_TUKEY,							/** @brief	Tukey ��ʧ����	*/
	LOSS_FAIR,							/** @brief	Fair ��ʧ����	*/
	LOSS_GEMANMCCLURE,					/** @brief	Geman-McClure��ʧ����	*/
	LOSS_DCS,							/** @brief	DCS��ʧ����	*/
};


#endif
