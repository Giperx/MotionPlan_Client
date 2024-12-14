
#define PI 3.14159265358979323846
typedef struct{
	INT16 coor_x;
	INT16 coor_y;
	double coor_ori;
}POSE;
struct Point {
	INT16 coor_x;
	INT16 coor_y;

	//// 定义小于运算符，方便 priority_queue 排序
	//bool operator< (const Point& other) const {
	//	// 这里假设优先队列按照坐标的距离排序
	//	return coor_x < other.coor_x || (coor_x == other.coor_x && coor_y < other.coor_y);
	//}
	// 重载 Point 结构体的 < 运算符，确保优先队列正确排序
	bool operator<(const Point& other) const {
		if (coor_x == other.coor_x)
			return coor_y > other.coor_y;  // 优先队列按 y 坐标升序排序
		return coor_x > other.coor_x;     // 优先队列按 x 坐标升序排序
	}

};
typedef struct{
	INT16 Timestamp;           //Timestamp
	INT16 Runstatus;           //Runstatus：0：Shutdown，1：defauted 2：running
	BYTE task_finish;          //task_finish
	BYTE detect_object;        //detect target
	BYTE collision;            //collision label
	INT16 obstacle[360];       //Laser information, 0 degree for forward direction
	POSE initial_rpose;        //initial pose of robot,0 means invalid
	Point initial_dpose;       //initial pose of target,0 means invalid
	double target_angle;       //Target direction in robot coordinates frame，clockwise for [0,PI],anticlockwise[0, -PI];
}S2CINFO;
typedef struct{
	INT16 Timestamp;           //Timestamp 1 for 0.2 seconds
	INT16 Runstatus;
	double tra_vel;            //Linear speed cm/s
	double rot_vel;            //Angular speed Rad/s
	POSE cur_rpose;            //Current pose of robot, 0-invalid
	Point Traj[100];           //Global path from map[][] based pathplanning 
}C2SINFO;