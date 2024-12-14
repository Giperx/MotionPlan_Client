// run
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <atlstr.h>
#include "list"
#include "vector"
#include "winsock2.h"
#include "StructData.h"
#include "queue"
#include "float.h"
#pragma comment(lib,"ws2_32.lib")
using namespace std;
SOCKET sockClient;
S2CINFO S2Cdata;
C2SINFO C2Sdata;
POSE Initial_rPos;
POSE Cur_rPos;
POSE Cur_dPos;
Point goal;
INT16 obstacle[360]; //Laser data, 360 degree
int Runstatus = 1;   //0:Program shutdown 2: running
double cur_tra_vel = 0;
double cur_rot_vel = 0;
// 地图大小定义为1500x500
#define MAP_SIZE_X 1500
#define MAP_SIZE_Y 1000
#define INF 1e9
int map1[MAP_SIZE_X][MAP_SIZE_Y];  // 0: 空地, 1: 障碍物
vector<Point> globalPath;

void Initialition(){
	memset(&S2Cdata, 0, sizeof(S2CINFO));
	memset(&C2Sdata, 0, sizeof(C2SINFO));
	memset(&Initial_rPos, 0, sizeof(POSE));
	memset(&Cur_rPos, 0, sizeof(POSE));
	memset(&Cur_dPos, 0, sizeof(POSE));
	memset(obstacle, 0, 360 * sizeof(INT16));
}
// 判断三点是否共线
bool isCollinear(const Point& p1, const Point& p2, const Point& p3) {
	// 判断 (p1, p2, p3) 是否共线，通过叉积判断
	return (p2.coor_x - p1.coor_x) * (p3.coor_y - p2.coor_y) == (p2.coor_y - p1.coor_y) * (p3.coor_x - p2.coor_x);
}

// 缩短路径
void shortenPath(vector<Point>& path) {
	if (path.size() < 3) return; // 少于3个点时，不能缩短

	vector<Point> shortenedPath;
	shortenedPath.push_back(path[0]); // 添加起点

	for (size_t i = 1; i < path.size() - 1; ++i) {
		// 如果当前点与前一个点和下一个点共线，就跳过当前点
		if (!isCollinear(path[i - 1], path[i], path[i + 1])) {
			shortenedPath.push_back(path[i]); // 否则，保留当前点
		}
	}

	shortenedPath.push_back(path.back()); // 添加终点
	path = shortenedPath; // 更新路径
}
// 更新地图
void UpdateMap(INT16 obstacle[360], POSE Cur_rPos) {
	// 清空地图
	//memset(map1, 0, sizeof(map1));  // 假设每次都重新开始（可以改为只清除障碍物）

	// 膨胀范围
	int expand_radius = 7;  // 假设机器人大小的膨胀半径为 7

	for (int i = 0; i < 360; i++) {
		int angle = i;  // 激光雷达的角度
		int dist = obstacle[i];  // 激光雷达的距离

		if (dist == 0) continue;

		// 计算激光雷达角度相对于机器人朝向的角度
		// 角度加上机器人的朝向（已是弧度），然后转为弧度
		double adjusted_angle = (angle * PI / 180.0) + Cur_rPos.coor_ori;  // 先将角度转换为弧度再加上机器人的朝向
		if (adjusted_angle > PI) {
			adjusted_angle -= 2 * PI;
		}
		if (adjusted_angle < -PI) {
			adjusted_angle += 2 * PI;
		}
		// 将相对距离转换为地图坐标
		int x = Cur_rPos.coor_x + dist * cos(adjusted_angle);  // 计算雷达点的x坐标
		int y = Cur_rPos.coor_y + dist * sin(adjusted_angle);  // 计算雷达点的y坐标

		// 检查坐标是否在地图范围内，并标记为障碍物
		if (x >= 0 && x < MAP_SIZE_X && y >= 0 && y < MAP_SIZE_Y) {
			// 标记当前位置为障碍物
			map1[x][y] = 1;

			// 进行膨胀：标记周围 expand_radius 范围内的点为障碍物
			for (int dx = -expand_radius; dx <= expand_radius; dx++) {
				for (int dy = -expand_radius; dy <= expand_radius; dy++) {
					int nx = x + dx;
					int ny = y + dy;

					// 检查新位置是否在地图范围内，并标记为障碍物
					if (nx >= 0 && nx < MAP_SIZE_X && ny >= 0 && ny < MAP_SIZE_Y) {
						map1[nx][ny] = 1;
					}
				}
			}
		}
	}
}
//// Dijkstra算法
vector<Point> Dijkstra(Point start) {
	// 创建一个优先队列，按照代价（cost）排序
	priority_queue<pair<double, Point>, vector<pair<double, Point>>, greater<pair<double, Point>>> pq;
	vector<vector<double>> dist(MAP_SIZE_X, vector<double>(MAP_SIZE_Y, INF));
	vector<vector<Point>> parent(MAP_SIZE_X, vector<Point>(MAP_SIZE_Y));
		//int dx[] = { 1, -1, 0, 0,1,1,-1,-1};  // 八个方向偏移
		//int dy[] = { 0, 0, 1, -1,1,-1,1,-1 };
	int dx[] = { 1, -1, 0, 0 };  // 上下左右的偏移
	int dy[] = { 0, 0, 1, -1 };
	pq.push({ 0, start });
	dist[start.coor_x][start.coor_y] = 0;

	vector<Point> path;
	if (start.coor_x < 0 || start.coor_y < 0) return path;

	while (!pq.empty()) {
		Point cur = pq.top().second;
		pq.pop();

		// 如果目标已经找到，提前结束
		if (cur.coor_x == goal.coor_x && cur.coor_y == goal.coor_y) {
			break;
		}

		// 遍历四个方向
		for (int i = 0; i < 7; ++i) {
			int nx = cur.coor_x + dx[i];
			int ny = cur.coor_y + dy[i];

			// 检查新位置是否在地图范围内，并且没有障碍物
			if (nx >= 0 && nx < MAP_SIZE_X && ny >= 0 && ny < MAP_SIZE_Y && map1[nx][ny] == 0) {
				double newCost = dist[cur.coor_x][cur.coor_y] + 1.0;

				// 如果新的路径更短，则更新
				if (newCost < dist[nx][ny]) {
					dist[nx][ny] = newCost;
					parent[nx][ny] = cur;
					pq.push(std::make_pair(newCost, Point{ (INT16)nx, (INT16)ny }));
				}
			}
		}
	}

	// 如果目标不可达，返回空路径
	
	if (dist[goal.coor_x][goal.coor_y] == INF) return path;

	// 回溯路径
	Point cur = goal;
	//cout << "goal: " << goal.coor_x << ' ' << goal.coor_y << endl;
	//cout << "cur: " << cur.coor_x << ' ' << cur.coor_y << endl;
	while (!(cur.coor_x == start.coor_x && cur.coor_y == start.coor_y)) {
		path.push_back(cur);
		cur = parent[cur.coor_x][cur.coor_y];
	}
	path.push_back(start);

	// 反转路径
	reverse(path.begin(), path.end());
	//cout << "size:" << path.size() << endl;
	globalPath.clear();
	for (size_t i = 0; i < path.size(); i++)
	{
		globalPath.push_back(path[i]);

	}
	//cout << "size:" << globalPath.size() << endl;
	return path;
}
// 计算机器人需要的偏移角度
double CalculateAngle(POSE current, Point target) {
	double dx = target.coor_x - current.coor_x;
	double dy = target.coor_y - current.coor_y;
	//cout << "antan2(dy, dx) = " << atan2(dy, dx) << endl;
	//double angleTar = atan2(dy, dx) * 180.0 / PI;
	double angleTar = atan2(dy, dx);
	cout << "angleTar = " << angleTar << endl;
	return angleTar - Cur_rPos.coor_ori;
}


CString GetExePath()
{
	CString strExePath;
	CString strPath;
	GetModuleFileName(NULL, strPath.GetBufferSetLength(MAX_PATH + 1), MAX_PATH + 1);
	int nPos = strPath.ReverseFind(_T('\\'));
	strExePath = strPath.Left(nPos + 1);
	return strExePath;
}
bool isObstacleAhead(POSE current, double angle) {
	// 将弧度转换为角度
	//double angleInDegrees = angle * 180 / PI;
	//// 计算雷达数据中对应角度的索引
	int startIdx = (int)(0 - angle); // 左边5度
	int endIdx = (int)(0 + angle); // 右边5度
	// 限制索引范围在0到359之间
	startIdx = (startIdx + 360) % 360;
	endIdx = (endIdx + 360) % 360;

	int distance = 65;

	// 遍历该扇形区域的雷达数据
	for (int i = startIdx; i != endIdx; i = (i + 1) % 360) {
		if (obstacle[i] <= distance) {
			return true; // 有障碍物
		}
	}
	return false; // 没有障碍物
}
bool flag = false;

int cnt_rota = 0;
void rotateToTarget(POSE current, int index) {
	int indextmp = index;
	
	if (S2Cdata.Timestamp >= 135) index = globalPath.size() - 3; // add close
	if (S2Cdata.Timestamp >= 300) index = indextmp; // add close
	if (globalPath.size()- 1 <= index) 
	{
		cout << "globalPath.size()- 1 <= index" << endl;
		index = globalPath.size() - 1; // add modi 1
		
	}
	if (cnt_rota >= 20) index = globalPath.size() / 2;




	//if (S2Cdata.Timestamp >= 300) {
	//	
	//	index = globalPath.size() - 3;
	//	if (globalPath.size() - 1 <= index)
	//	{
	//		cout << "globalPath.size()- 1 <= index" << endl;
	//		index = globalPath.size() - 1;

	//	}
	//	//if (S2Cdata.Timestamp >= 300) flag = true;
	//	if (cnt_rota >= 40) index = globalPath.size() / 2;
	//}



	Point next = globalPath[index];
	double diff_angle = CalculateAngle(current, next);
	if (diff_angle < -PI) diff_angle = 2 * PI + diff_angle;
	if (diff_angle > PI) diff_angle = 2 * PI - diff_angle;

	cout << "diff_angle:" << diff_angle << endl;
	if (flag) {
		if (S2Cdata.target_angle > 0) {
			Cur_rPos.coor_ori = Cur_rPos.coor_ori + cur_rot_vel * 0.2;
			if (Cur_rPos.coor_ori > PI) Cur_rPos.coor_ori = 2 * PI - Cur_rPos.coor_ori;
			if (Cur_rPos.coor_ori < -PI) Cur_rPos.coor_ori = 2 * PI + Cur_rPos.coor_ori;
			Cur_rPos.coor_x = INT16(Cur_rPos.coor_x + cur_tra_vel * 0.2 * cos(Cur_rPos.coor_ori));
			Cur_rPos.coor_y = INT16(Cur_rPos.coor_y + cur_tra_vel * 0.2 * sin(Cur_rPos.coor_ori));
			C2Sdata.cur_rpose.coor_x = Cur_rPos.coor_x;
			C2Sdata.cur_rpose.coor_y = Cur_rPos.coor_y;
			C2Sdata.cur_rpose.coor_ori = Cur_rPos.coor_ori;
			C2Sdata.tra_vel = 0;
			C2Sdata.rot_vel = PI / 8;
			cur_tra_vel = 0;
			cur_rot_vel = PI / 8;
			cnt_rota++;
		}
		else {
			Cur_rPos.coor_ori = Cur_rPos.coor_ori + cur_rot_vel * 0.2;
			if (Cur_rPos.coor_ori > PI) Cur_rPos.coor_ori = 2 * PI - Cur_rPos.coor_ori;
			if (Cur_rPos.coor_ori < -PI) Cur_rPos.coor_ori = 2 * PI + Cur_rPos.coor_ori;
			Cur_rPos.coor_x = INT16(Cur_rPos.coor_x + cur_tra_vel * 0.2 * cos(Cur_rPos.coor_ori));
			Cur_rPos.coor_y = INT16(Cur_rPos.coor_y + cur_tra_vel * 0.2 * sin(Cur_rPos.coor_ori));
			C2Sdata.cur_rpose.coor_x = Cur_rPos.coor_x;
			C2Sdata.cur_rpose.coor_y = Cur_rPos.coor_y;
			C2Sdata.cur_rpose.coor_ori = Cur_rPos.coor_ori;
			C2Sdata.tra_vel = 0;
			C2Sdata.rot_vel = -PI / 8;
			cur_tra_vel = 0;
			cur_rot_vel = -PI / 8;
			cnt_rota++;
		}
		flag = false;
	}
	else {
		//if (fabs(diff_angle) > PI / 20) { // 目标在机器人中轴左侧
			if (true) { // 目标在机器人中轴左侧
			if (diff_angle < 0) { //需要逆时针旋转

				Cur_rPos.coor_ori = Cur_rPos.coor_ori + cur_rot_vel * 0.2;
				if (Cur_rPos.coor_ori > PI) Cur_rPos.coor_ori = 2 * PI - Cur_rPos.coor_ori;
				if (Cur_rPos.coor_ori < -PI) Cur_rPos.coor_ori = 2 * PI + Cur_rPos.coor_ori;
				Cur_rPos.coor_x = INT16(Cur_rPos.coor_x + cur_tra_vel * 0.2 * cos(Cur_rPos.coor_ori));
				Cur_rPos.coor_y = INT16(Cur_rPos.coor_y + cur_tra_vel * 0.2 * sin(Cur_rPos.coor_ori));
				C2Sdata.cur_rpose.coor_x = Cur_rPos.coor_x;
				C2Sdata.cur_rpose.coor_y = Cur_rPos.coor_y;
				C2Sdata.cur_rpose.coor_ori = Cur_rPos.coor_ori;
				C2Sdata.tra_vel = 0;
				C2Sdata.rot_vel = -PI / 8;
				cur_tra_vel = 0;
				cur_rot_vel = -PI / 8;
				cnt_rota++;
			}
			else {

				Cur_rPos.coor_ori = Cur_rPos.coor_ori + cur_rot_vel * 0.2;
				if (Cur_rPos.coor_ori > PI) Cur_rPos.coor_ori = 2 * PI - Cur_rPos.coor_ori;
				if (Cur_rPos.coor_ori < -PI) Cur_rPos.coor_ori = 2 * PI + Cur_rPos.coor_ori;
				Cur_rPos.coor_x = INT16(Cur_rPos.coor_x + cur_tra_vel * 0.2 * cos(Cur_rPos.coor_ori));
				Cur_rPos.coor_y = INT16(Cur_rPos.coor_y + cur_tra_vel * 0.2 * sin(Cur_rPos.coor_ori));
				C2Sdata.cur_rpose.coor_x = Cur_rPos.coor_x;
				C2Sdata.cur_rpose.coor_y = Cur_rPos.coor_y;
				C2Sdata.cur_rpose.coor_ori = Cur_rPos.coor_ori;
				C2Sdata.tra_vel = 0;
				C2Sdata.rot_vel = PI / 8;
				cur_tra_vel = 0;
				cur_rot_vel = PI / 8;
				cnt_rota++;
			}
		}
	}
	
	
}
// 控制机器人前进
void moveToNextPoint(int index) {
	Point next = globalPath[index];
	// 确保障碍物在前方安全距离外
	if (isObstacleAhead(Cur_rPos,15)) {
		// 如果有障碍物，旋转绕过障碍
		rotateToTarget(Cur_rPos, index + 2);  // 顺时针或逆时针旋转
	}
	else {
		// 向目标前进
		
		Cur_rPos.coor_ori = Cur_rPos.coor_ori + cur_rot_vel * 0.2;
		Cur_rPos.coor_x = INT16(Cur_rPos.coor_x + cur_tra_vel * 0.2 * cos(Cur_rPos.coor_ori));
		Cur_rPos.coor_y = INT16(Cur_rPos.coor_y + cur_tra_vel * 0.2 * sin(Cur_rPos.coor_ori));
		C2Sdata.cur_rpose.coor_x = Cur_rPos.coor_x;
		C2Sdata.cur_rpose.coor_y = Cur_rPos.coor_y;
		C2Sdata.cur_rpose.coor_ori = Cur_rPos.coor_ori;
		C2Sdata.tra_vel = 20;
		C2Sdata.rot_vel = 0;
		cur_tra_vel = 25;
		cur_rot_vel = 0;
		cnt_rota = 0;

		//if (obstacle[315] > obstacle[45] && obstacle[45] < 50) { // 前方扇形90°,右前方障碍物
		//	C2Sdata.rot_vel = -PI / 4;
		//	cur_rot_vel = -PI / 4;
		//}
		//if (obstacle[315] < obstacle[45] && obstacle[315] < 50) { // 左前方障碍物
		//	C2Sdata.rot_vel = PI / 4;
		//	cur_rot_vel = PI / 4;

		//}

		/*for (int i = 30; i <= 90; i++) {
			if (obstacle[360 - i] > obstacle[i] && obstacle[i] < 30) {
				C2Sdata.rot_vel = -PI / 8;
				cur_rot_vel = -PI / 8;
				C2Sdata.tra_vel = 0;
				cur_tra_vel = 0;
				cnt_rota++;
				break;

			}
			if (obstacle[360 - i] < obstacle[i] && obstacle[360 - i] < 30) {
				C2Sdata.rot_vel = PI / 8;
				cur_rot_vel = PI / 8;
				C2Sdata.tra_vel = 0;
				cur_tra_vel = 0;
				cnt_rota++;
				break;
			}
		}*/
	}
}

DWORD WINAPI Recv_Thre(LPVOID lpParameter)
{
	char recvBuf[800]; // Data received
	char Sendbuff[600];// Data sent
	while (true){
		memset(recvBuf, 0, sizeof(recvBuf));
		if (recv(sockClient, recvBuf, sizeof(recvBuf), 0) > 0){
			memset(&S2Cdata, 0, sizeof(S2CINFO));
			memset(&C2Sdata, 0, sizeof(C2SINFO));
			memcpy(&S2Cdata, recvBuf, sizeof(S2CINFO));
			{
				if (S2Cdata.Timestamp == 59) {
					cout << 59 << endl;
				}
				if (S2Cdata.Runstatus == 0){
					Runstatus = 0;
					return 0;
				}
				if(S2Cdata.Timestamp == 0){//Get the poses of robot and target at the first frame;
					memcpy(&Initial_rPos, &S2Cdata.initial_rpose, sizeof(POSE));
					memcpy(&Cur_rPos, &S2Cdata.initial_rpose, sizeof(POSE));
					memcpy(&Cur_dPos, &S2Cdata.initial_dpose, sizeof(POSE));
				}
				cout << Cur_rPos.coor_x << ' ' << Cur_rPos.coor_y << ' ' << Cur_rPos.coor_ori << endl;
				C2Sdata.Timestamp = S2Cdata.Timestamp;
				C2Sdata.Runstatus = S2Cdata.Runstatus;
				memcpy(obstacle, S2Cdata.obstacle, 360 * sizeof(INT16));
				//Add global path here
				
				//Add localization of robot here
				if (S2Cdata.detect_object > 0)
					S2Cdata.detect_object = S2Cdata.detect_object;
				if (S2Cdata.Timestamp == 0) {
					//UpdateMap(obstacle, Cur_rPos);
					Cur_rPos.coor_ori = Cur_rPos.coor_ori + cur_rot_vel * 0.2;
					Cur_rPos.coor_x = INT16(Cur_rPos.coor_x + cur_tra_vel * 0.2 * cos(Cur_rPos.coor_ori));
					Cur_rPos.coor_y = INT16(Cur_rPos.coor_y + cur_tra_vel * 0.2 * sin(Cur_rPos.coor_ori));
					C2Sdata.cur_rpose.coor_x = Cur_rPos.coor_x;
					C2Sdata.cur_rpose.coor_y = Cur_rPos.coor_y;
					C2Sdata.cur_rpose.coor_ori = Cur_rPos.coor_ori;
					C2Sdata.tra_vel = 20;
					C2Sdata.rot_vel = 0;
					cur_tra_vel = 25;
					cur_rot_vel = 0;
					//Send data
					cout << "TimeStamp: " << C2Sdata.Timestamp << endl;
					memset(Sendbuff, 0, sizeof(Sendbuff));
					memcpy(Sendbuff, &C2Sdata, sizeof(C2SINFO));
					send(sockClient, Sendbuff, sizeof(Sendbuff), 0);
					Sleep(30);
					continue;
				}
				
				
				Point start = { Cur_rPos.coor_x,Cur_rPos.coor_y };
				goal = { S2Cdata.initial_dpose.coor_x, S2Cdata.initial_dpose.coor_y };
				//cout << start.coor_x << ' ' << start.coor_y << endl;
				//cout << goal.coor_x << ' ' << goal.coor_y << endl;
				if (S2Cdata.detect_object) { // 找到目标
					cout << "find object" << endl;
					cout << "target_angle" << S2Cdata.target_angle << endl;
					if (S2Cdata.target_angle > PI / 10) { // 18°
						cout << "target_angle" << S2Cdata.target_angle << endl;
						Cur_rPos.coor_ori = Cur_rPos.coor_ori + cur_rot_vel * 0.2;
						if (Cur_rPos.coor_ori > PI) Cur_rPos.coor_ori = 2 * PI - Cur_rPos.coor_ori;
						if (Cur_rPos.coor_ori < -PI) Cur_rPos.coor_ori = 2 * PI + Cur_rPos.coor_ori;
						Cur_rPos.coor_x = INT16(Cur_rPos.coor_x + cur_tra_vel * 0.2 * cos(Cur_rPos.coor_ori));
						Cur_rPos.coor_y = INT16(Cur_rPos.coor_y + cur_tra_vel * 0.2 * sin(Cur_rPos.coor_ori));
						C2Sdata.cur_rpose.coor_x = Cur_rPos.coor_x;
						C2Sdata.cur_rpose.coor_y = Cur_rPos.coor_y;
						C2Sdata.cur_rpose.coor_ori = Cur_rPos.coor_ori;
						C2Sdata.tra_vel = 0;
						C2Sdata.rot_vel = PI / 8; // 2.25°
						cur_tra_vel = 0;
						cur_rot_vel = PI / 8;
						cout << "TimeStamp: " << C2Sdata.Timestamp << endl;
						memset(Sendbuff, 0, sizeof(Sendbuff));
						memcpy(Sendbuff, &C2Sdata, sizeof(C2SINFO));
						send(sockClient, Sendbuff, sizeof(Sendbuff), 0);
						Sleep(30);
						continue;
					}
					else if (S2Cdata.target_angle < -PI / 10) {
						cout << "target_angle" << S2Cdata.target_angle << endl;
						Cur_rPos.coor_ori = Cur_rPos.coor_ori + cur_rot_vel * 0.2;
						if (Cur_rPos.coor_ori > PI) Cur_rPos.coor_ori = 2 * PI - Cur_rPos.coor_ori;
						if (Cur_rPos.coor_ori < -PI) Cur_rPos.coor_ori = 2 * PI + Cur_rPos.coor_ori;
						Cur_rPos.coor_x = INT16(Cur_rPos.coor_x + cur_tra_vel * 0.2 * cos(Cur_rPos.coor_ori));
						Cur_rPos.coor_y = INT16(Cur_rPos.coor_y + cur_tra_vel * 0.2 * sin(Cur_rPos.coor_ori));
						C2Sdata.cur_rpose.coor_x = Cur_rPos.coor_x;
						C2Sdata.cur_rpose.coor_y = Cur_rPos.coor_y;
						C2Sdata.cur_rpose.coor_ori = Cur_rPos.coor_ori;
						C2Sdata.tra_vel = 0;
						C2Sdata.rot_vel = -PI / 8;
						cur_tra_vel = 0;
						cur_rot_vel = -PI / 8;
						cout << "TimeStamp: " << C2Sdata.Timestamp << endl;
						memset(Sendbuff, 0, sizeof(Sendbuff));
						memcpy(Sendbuff, &C2Sdata, sizeof(C2SINFO));
						send(sockClient, Sendbuff, sizeof(Sendbuff), 0);
						Sleep(30);
						continue;
					}
					else {

						Cur_rPos.coor_ori = Cur_rPos.coor_ori + cur_rot_vel * 0.2;
						if (Cur_rPos.coor_ori > PI) Cur_rPos.coor_ori = 2 * PI - Cur_rPos.coor_ori;
						if (Cur_rPos.coor_ori < -PI) Cur_rPos.coor_ori = 2 * PI + Cur_rPos.coor_ori;
						Cur_rPos.coor_x = INT16(Cur_rPos.coor_x + cur_tra_vel * 0.2 * cos(Cur_rPos.coor_ori));
						Cur_rPos.coor_y = INT16(Cur_rPos.coor_y + cur_tra_vel * 0.2 * sin(Cur_rPos.coor_ori));
						C2Sdata.cur_rpose.coor_x = Cur_rPos.coor_x;
						C2Sdata.cur_rpose.coor_y = Cur_rPos.coor_y;
						C2Sdata.cur_rpose.coor_ori = Cur_rPos.coor_ori;
						C2Sdata.rot_vel = 0;
						cur_rot_vel = 0;

						cout << endl;
						for (int i = 30; i <= 90; i++) {
							if (obstacle[360 - i] > obstacle[i] && obstacle[i] < 35) {
								C2Sdata.rot_vel = -PI / 4;
								cur_rot_vel = -PI / 4;
								break;

							}
							if (obstacle[360 - i] < obstacle[i] && obstacle[360 - i] < 35) {
								C2Sdata.rot_vel = PI / 4;
								cur_rot_vel = PI / 4;
								break;
							}
						}

						C2Sdata.tra_vel = 20;
						cur_tra_vel = 25;
						cout << "TimeStamp: " << C2Sdata.Timestamp << endl;
						memset(Sendbuff, 0, sizeof(Sendbuff));
						memcpy(Sendbuff, &C2Sdata, sizeof(C2SINFO));
						send(sockClient, Sendbuff, sizeof(Sendbuff), 0);
						Sleep(30);
						continue;
					}
				}
				cout << "cnt_rota:" << cnt_rota << endl;
				// 太久没动,逃脱
				if (cnt_rota == 30) { 
					int tmpMax = 0, maxIndex = 0;
					for (int i = 0; i < 360; i++) {
						if (obstacle[i] > tmpMax) {
							tmpMax = obstacle[i];
							maxIndex = i;
						}
					}
					double maxRad = (maxIndex * PI / 180);
					if (maxRad > PI) maxRad = maxRad - 2 * PI;
					cout << "maxRad:" << maxRad << endl;
					while (fabs(maxRad) > PI / 30) {
						cout << "maxRad:" << maxRad << endl;
						//memset(&S2Cdata, 0, sizeof(S2CINFO));


						memset(&C2Sdata, 0, sizeof(C2SINFO));
						memcpy(&S2Cdata, recvBuf, sizeof(S2CINFO));
						C2Sdata.Timestamp = S2Cdata.Timestamp;
						C2Sdata.Runstatus = S2Cdata.Runstatus;
						memcpy(obstacle, S2Cdata.obstacle, 360 * sizeof(INT16));
						if (maxRad > 0) {
							Cur_rPos.coor_ori = Cur_rPos.coor_ori + cur_rot_vel * 0.2;
							maxRad = maxRad - cur_rot_vel * 0.2;
							if (Cur_rPos.coor_ori > PI) Cur_rPos.coor_ori = 2 * PI - Cur_rPos.coor_ori;
							if (Cur_rPos.coor_ori < -PI) Cur_rPos.coor_ori = 2 * PI + Cur_rPos.coor_ori;
							Cur_rPos.coor_x = INT16(Cur_rPos.coor_x + cur_tra_vel * 0.2 * cos(Cur_rPos.coor_ori));
							Cur_rPos.coor_y = INT16(Cur_rPos.coor_y + cur_tra_vel * 0.2 * sin(Cur_rPos.coor_ori));
							C2Sdata.cur_rpose.coor_x = Cur_rPos.coor_x;
							C2Sdata.cur_rpose.coor_y = Cur_rPos.coor_y;
							C2Sdata.cur_rpose.coor_ori = Cur_rPos.coor_ori;
							C2Sdata.tra_vel = 0;
							C2Sdata.rot_vel = PI / 8; // * 0.2 = 4.5°
							cur_tra_vel = 0;
							cur_rot_vel = PI / 8;
							cnt_rota++;
						}
						else {
							Cur_rPos.coor_ori = Cur_rPos.coor_ori + cur_rot_vel * 0.2;
							maxRad = maxRad - cur_rot_vel * 0.2;
							if (Cur_rPos.coor_ori > PI) Cur_rPos.coor_ori = 2 * PI - Cur_rPos.coor_ori;
							if (Cur_rPos.coor_ori < -PI) Cur_rPos.coor_ori = 2 * PI + Cur_rPos.coor_ori;
							Cur_rPos.coor_x = INT16(Cur_rPos.coor_x + cur_tra_vel * 0.2 * cos(Cur_rPos.coor_ori));
							Cur_rPos.coor_y = INT16(Cur_rPos.coor_y + cur_tra_vel * 0.2 * sin(Cur_rPos.coor_ori));
							C2Sdata.cur_rpose.coor_x = Cur_rPos.coor_x;
							C2Sdata.cur_rpose.coor_y = Cur_rPos.coor_y;
							C2Sdata.cur_rpose.coor_ori = Cur_rPos.coor_ori;
							C2Sdata.tra_vel = 0;
							C2Sdata.rot_vel = -PI / 8;
							cur_tra_vel = 0;
							cur_rot_vel = -PI / 8;
							cnt_rota++;
						}
						cout << "TimeStamp: " << C2Sdata.Timestamp << endl;
						memset(Sendbuff, 0, sizeof(Sendbuff));
						memcpy(Sendbuff, &C2Sdata, sizeof(C2SINFO));
						send(sockClient, Sendbuff, sizeof(Sendbuff), 0);
						Sleep(30);
						recv(sockClient, recvBuf, sizeof(recvBuf), 0);
						continue;
					}
					cout << "ret" << endl;
					int cnttmp1 = 5;
					while (cnttmp1--) {

						memset(&C2Sdata, 0, sizeof(C2SINFO));
						memcpy(&S2Cdata, recvBuf, sizeof(S2CINFO));
						C2Sdata.Timestamp = S2Cdata.Timestamp;
						C2Sdata.Runstatus = S2Cdata.Runstatus;
						memcpy(obstacle, S2Cdata.obstacle, 360 * sizeof(INT16));
						Cur_rPos.coor_ori = Cur_rPos.coor_ori + cur_rot_vel * 0.2;
						if (Cur_rPos.coor_ori > PI) Cur_rPos.coor_ori = 2 * PI - Cur_rPos.coor_ori;
						if (Cur_rPos.coor_ori < -PI) Cur_rPos.coor_ori = 2 * PI + Cur_rPos.coor_ori;

						Cur_rPos.coor_x = INT16(Cur_rPos.coor_x + cur_tra_vel * 0.2 * cos(Cur_rPos.coor_ori));
						Cur_rPos.coor_y = INT16(Cur_rPos.coor_y + cur_tra_vel * 0.2 * sin(Cur_rPos.coor_ori));
						C2Sdata.cur_rpose.coor_x = Cur_rPos.coor_x;
						C2Sdata.cur_rpose.coor_y = Cur_rPos.coor_y;
						C2Sdata.cur_rpose.coor_ori = Cur_rPos.coor_ori;
						C2Sdata.rot_vel = 0;
						cur_rot_vel = 0;
						for (int i = 30; i <= 90; i++) {
							if (obstacle[360 - i] > obstacle[i] && obstacle[i] < 45) {
								C2Sdata.rot_vel = -PI / 4;
								cur_rot_vel = -PI / 4;
								break;

							}
							if (obstacle[360 - i] < obstacle[i] && obstacle[360 - i] < 45) {
								C2Sdata.rot_vel = PI / 4;
								cur_rot_vel = PI / 4;
								break;
							}
						}
						C2Sdata.tra_vel = 20;
						cur_tra_vel = 25;
						//Send data
						cout << "TimeStamp: " << C2Sdata.Timestamp << endl;
						memset(Sendbuff, 0, sizeof(Sendbuff));
						memcpy(Sendbuff, &C2Sdata, sizeof(C2SINFO));
						send(sockClient, Sendbuff, sizeof(Sendbuff), 0);
						Sleep(30);
						if (cnttmp1 != 1) recv(sockClient, recvBuf, sizeof(recvBuf), 0);

					}
					cout << "restart" << endl;
					cnt_rota = 0;
					UpdateMap(obstacle, Cur_rPos);
				}

				if (S2Cdata.Timestamp % 7 == 0 || S2Cdata.Timestamp == 1) {
					UpdateMap(obstacle, Cur_rPos);
					Dijkstra(start);
					shortenPath(globalPath);	
					cout << "size:" << globalPath.size() << endl;
					if (globalPath.empty()) {
						cout << "目标不可达!" << endl;
						continue;
					}

					// 将路径更新到C2Sdata.Traj
					int pathLength = globalPath.size();
					for (int i = 0; i <= 99 && i < pathLength; ++i) {
						// 假设 Traj 是一个数组，可以直接赋值
						C2Sdata.Traj[i] = globalPath[i];
					}
					//cout << "size " << pathLength << ' ' << C2Sdata.Traj[pathLength].coor_x << ' ' << C2Sdata.Traj[pathLength].coor_y << endl;
				}
				
				
				moveToNextPoint(1);
				
				//Add plan code here
				
				//Send data
				cout << "TimeStamp: " << C2Sdata.Timestamp << endl;
				memset(Sendbuff, 0, sizeof(Sendbuff));
				memcpy(Sendbuff, &C2Sdata, sizeof(C2SINFO));
				send(sockClient, Sendbuff, sizeof(Sendbuff), 0);
				Sleep(30);
			}
		}
		else
			Sleep(30);
	}
	closesocket(sockClient);
	return 0;
}
int main()
{
	CString path = GetExePath() + "Plan_Server.exe";
	ShellExecute(NULL, NULL, path, NULL, NULL, SW_SHOW);
	Sleep(1000);
	Initialition();
	WSADATA wsaData;
	if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
	{
		printf("Winsock initialization failure");
		return 0;
	}
	SOCKADDR_IN addrSrv;
	addrSrv.sin_family = AF_INET;
	addrSrv.sin_port = htons(8888);
	addrSrv.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");
	sockClient = socket(AF_INET, SOCK_STREAM, 0);
	if (SOCKET_ERROR == sockClient){
		printf("Socket() error:%d", WSAGetLastError());
		return 0;
	}
	if (connect(sockClient, (struct  sockaddr*)&addrSrv, sizeof(addrSrv)) == INVALID_SOCKET){
		printf("Connection failure:%d", WSAGetLastError());
		return 0;
	}
	HANDLE hThreadRecv = CreateThread(NULL, 0, Recv_Thre, 0, 0, NULL);
	if (NULL == hThreadRecv)
		CloseHandle(hThreadRecv);
	while (Runstatus)
	{
		Sleep(50);
	}
	WSACleanup();
	return 0;
}
