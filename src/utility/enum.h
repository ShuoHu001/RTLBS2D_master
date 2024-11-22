#ifndef RTLBS_ENUM
#define RTLBS_ENUM

enum LOGLEVEL {
	LEVEL_INFO,
	LEVEL_WARNING,
	LEVEL_ERROR
};

/** @brief	硬件模式	*/
enum HARDWAREMODE {
	CPU_SINGLETHREAD,			/** @brief	CPU单线程	*/
	CPU_MULTITHREAD,			/** @brief	CPU多线程	*/
	GPU_MULTITHREAD,			/** @brief	GPU多线程	*/
	CPU_GPU_MULTITHREAD			/** @brief	CPU_GPU混合多线程	*/
};

/** @brief	对象种类	*/
enum OBJECT2DCATEGORY {
	BUILDING2D,					/** @brief	二维建筑物	*/
	VEGETATION2D,				/** @brief	二维绿化带	*/
	WALL2D						/** @brief	二维墙体	*/
};

/** @brief	加速体选择模式	*/
enum ACCEL_TYPE {
	ACCEL_NONE = 0, /** @brief	暴力求解模式	*/
	ACCEL_BVH = 1,	/** @brief	BVH加速结构体	*/
	ACCEL_KD = 2, /** @brief	KDTree加速结构体*/
	ACCEL_UG = 3,	/** @brief	均匀网格加速结构体	*/
	ACCEL_FastUG = 4,	/** @brief	优化的均匀网格加速结构体	*/
	ACCEL_SDF = 5	/** @brief	距离场加速结构体	*/
};

/** @brief	射线发射模式	*/
enum RAYLAUNCHMODE {
	SINGLE_DIRECTION,				/** @brief	单一方向	*/
	UNIFORM_ICOSAHEDRON,			/** @brief	正二十面体均匀发射	*/
};

/** @brief	天线选择模式	*/
enum ANT_MODE {
	MODE_INTERNAL = 0, /** @brief	内置模式	*/
	MODE_EXTERNAL = 1  /** @brief	外置模式	*/
};

/** @brief	天线类型	*/
enum ANT_TYPE {
	TYPE_OMNI = 0, /** @brief	全向天线	*/
	TYPE_DIPO = 1  /** @brief	偶极子天线	*/
};

enum PATHNODETYPE {
	NODE_ROOT,				/** @brief	根节点	*/
	NODE_LOS,				/** @brief	视距节点	*/
	NODE_REFL,				/** @brief	反射节点	*/
	NODE_TRANIN,			/** @brief	透射入节点	*/
	NODE_TRANOUT,			/** @brief	透射出节点	*/
	NODE_ETRANIN,			/** @brief	经验透射入节点	*/
	NODE_ETRANOUT,			/** @brief	经验透射出节点	*/
	NODE_DIFF,				/** @brief	绕射节点	*/
	NODE_REFLDIFF,			/** @brief	反射+绕射节点	*/
	NODE_SCAT,				/** @brief	散射节点	*/
	NODE_INIT,				/** @brief	初始节点	*/
	NODE_STOP				/** @brief	终止节点	*/
};

enum SYSTEM_MODE {
	MODE_RT = 0, /** @brief	射线跟踪算法模式	*/
	MODE_LBS = 1 /** @brief	定位模式	*/
};

/** @brief	定位方法	*/
enum LOCALIZATION_METHOD {
	LBS_METHOD_RT_AOA = 0,				/** @brief	RT-AOA 定位	*/
	LBS_METHOD_RT_TDOA = 1,				/** @brief	RT-TDOA 定位	*/
	LBS_METHOD_RT_TOA = 2,				/** @brief	RT_TOA定位	*/
	LBS_METHOD_RT_AOA_TOA = 3,			/** @brief	RT-AOA-TOA 定位	*/
	LBS_METHOD_RT_AOA_TDOA = 4,			/** @brief	RT_AOA_TDOA 定位	*/
};

/** @brief	定位模式	*/
enum LOCALIZATION_MODE {
	LBS_MODE_SPSTMD,				/** @brief	单站单源多数据进行定位	*/
	LBS_MODE_MPSTSD,				/** @brief	多站单源单数据进行定位	*/
	LBS_MODE_SPMTMD,				/** @brief	单站多源多数据进行定位	*/
	LBS_MODE_MPMTMD					/** @brief	多站多源多数据进行定位	*/
};

/** @brief	接收端预测模式	*/
enum PREDICTION_MODE {
	PREDICTION_FILE = -1,			/** @brief	从文件中加载预测	*/
	PREDICTION_SINGLEPOINT = 0,		/** @brief	单点型预测	*/
	PREDICTION_LINE = 1,			/** @brief	路径预测	*/
	PREDICTION_SCARRERPOINT = 2,	/** @brief	离散点型预测	*/
	PREDICTION_PLANE = 3,			/** @brief	平面预测	*/
	PREDICTION_SOLID = 4,			/** @brief	立体型预测	*/
};

/** @brief	地形读取模式	*/
enum TERRAINLOADMODE {
	TERRAIN_OBJECT,						/** @brief	obj 物体模式读取	*/
	TERRAIN_GRID,						/** @brief	栅格模式读取	*/
};

enum TERRAINCATEGORY {
	GRIDCELL,							/** @brief	栅格类型	*/
	MESHHOLE,							/** @brief	含有孔洞的类型	*/
	MESHNONHOLE							/** @brief	不含孔洞的类型	*/
};

/** @brief		*/
enum WINDING_ORDER {
	CLOCKWISE = 0,			/** @brief	顺时针 DirectX等渲染引擎文件	*/
	COUNTCLOCKWISE = 1		/** @brief	逆时针 OpenGL等渲染引擎文件	*/
};

/** @brief	地形绕射模式	*/
enum TERRAINDIFFRACTIONMODE {
	DIFFRACTIONMODE_PICQUENARD,			/** @brief	Picquenard's model 方法计算	*/
	DIFFRACTIONMODE_EPSTEIN,			/** @brief	Epstein Peterson 方法计算	*/
	DIFFRACTIONMODE_HUSHI,				/** @brief	HuShuo's model 方法计算	*/
	DIFFRACTIONMODE_UTD					/** @brief	采用劈尖绕射方法计算	*/
};


/** @brief	地形计算模式（加速计算方面）	*/
enum TERRAIN_CALCMODE {
	TERRAIN_HALFFACEMODE,					/** @brief	半边法搜索计算	*/
	TERRAIN_GRIDMODE,						/** @brief	栅格化搜索计算	*/
};

/** @brief	天线方向图类型	*/
enum ANTENNAPATTERN_TYPE {
	PATTERN2D,							/** @brief	二维方向图	*/
	PATTERN3D,							/** @brief	三维方向图	*/
};

/** @brief	路径类型	*/
enum RAYPATHTYPE {
	RAYPATH_COMMON,								/** @brief	常规类型	*/
	RAYPATH_TERRAIN_REFLECTION,					/** @brief	地面反射	*/
	RAYPATH_TERRAIN_DIFFRACTION					/** @brief	地面绕射	*/
};

/** @brief	求解模式	*/
enum SOLVINGMODE {
	SOLVING_LS,							/** @brief	最小二乘法	*/
	SOLVING_WLS,						/** @brief	加权最小二乘法	*/
	SOLVING_IRLS,						/** @brief	迭代最小二乘法	*/
	SOLVING_WIRLS,						/** @brief	加权迭代最小二乘法	*/
};

/** @brief	损失函数类型	*/
enum LOSSFUNCTIONTYPE {
	LOSS_NONE,							/** @brief	无损失函数	*/
	LOSS_HUBER,							/** @brief	Huber 损失函数	*/
	LOSS_CAUCHY,						/** @brief	Cauchy 损失函数	*/
	LOSS_ARCTAN,						/** @brief	Arctan 损失函数	*/
	LOSS_TUKEY,							/** @brief	Tukey 损失函数	*/
	LOSS_FAIR,							/** @brief	Fair 损失函数	*/
	LOSS_GEMANMCCLURE,					/** @brief	Geman-McClure损失函数	*/
	LOSS_DCS,							/** @brief	DCS损失函数	*/
};


#endif
