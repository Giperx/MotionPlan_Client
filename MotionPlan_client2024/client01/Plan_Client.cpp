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
// ��ͼ��С����Ϊ1500x500
#define MAP_SIZE_X 1500
#define MAP_SIZE_Y 1000
#define INF 1e9
int map1[MAP_SIZE_X][MAP_SIZE_Y];  // 0: �յ�, 1: �ϰ���
vector<Point> globalPath;

void Initialition(){
	memset(&S2Cdata, 0, sizeof(S2CINFO));
	memset(&C2Sdata, 0, sizeof(C2SINFO));
	memset(&Initial_rPos, 0, sizeof(POSE));
	memset(&Cur_rPos, 0, sizeof(POSE));
	memset(&Cur_dPos, 0, sizeof(POSE));
	memset(obstacle, 0, 360 * sizeof(INT16));
}
// �ж������Ƿ���
bool isCollinear(const Point& p1, const Point& p2, const Point& p3) {
	// �ж� (p1, p2, p3) �Ƿ��ߣ�ͨ������ж�
	return (p2.coor_x - p1.coor_x) * (p3.coor_y - p2.coor_y) == (p2.coor_y - p1.coor_y) * (p3.coor_x - p2.coor_x);
}

// ����·��
void shortenPath(vector<Point>& path) {
	if (path.size() < 3) return; // ����3����ʱ����������

	vector<Point> shortenedPath;
	shortenedPath.push_back(path[0]); // ������

	for (size_t i = 1; i < path.size() - 1; ++i) {
		// �����ǰ����ǰһ�������һ���㹲�ߣ���������ǰ��
		if (!isCollinear(path[i - 1], path[i], path[i + 1])) {
			shortenedPath.push_back(path[i]); // ���򣬱�����ǰ��
		}
	}

	shortenedPath.push_back(path.back()); // ����յ�
	path = shortenedPath; // ����·��
}
// ���µ�ͼ
void UpdateMap(INT16 obstacle[360], POSE Cur_rPos) {
	// ��յ�ͼ
	//memset(map1, 0, sizeof(map1));  // ����ÿ�ζ����¿�ʼ�����Ը�Ϊֻ����ϰ��

	// ���ͷ�Χ
	int expand_radius = 7;  // ��������˴�С�����Ͱ뾶Ϊ 7

	for (int i = 0; i < 360; i++) {
		int angle = i;  // �����״�ĽǶ�
		int dist = obstacle[i];  // �����״�ľ���

		if (dist == 0) continue;

		// ���㼤���״�Ƕ�����ڻ����˳���ĽǶ�
		// �Ƕȼ��ϻ����˵ĳ������ǻ��ȣ���Ȼ��תΪ����
		double adjusted_angle = (angle * PI / 180.0) + Cur_rPos.coor_ori;  // �Ƚ��Ƕ�ת��Ϊ�����ټ��ϻ����˵ĳ���
		if (adjusted_angle > PI) {
			adjusted_angle -= 2 * PI;
		}
		if (adjusted_angle < -PI) {
			adjusted_angle += 2 * PI;
		}
		// ����Ծ���ת��Ϊ��ͼ����
		int x = Cur_rPos.coor_x + dist * cos(adjusted_angle);  // �����״���x����
		int y = Cur_rPos.coor_y + dist * sin(adjusted_angle);  // �����״���y����

		// ��������Ƿ��ڵ�ͼ��Χ�ڣ������Ϊ�ϰ���
		if (x >= 0 && x < MAP_SIZE_X && y >= 0 && y < MAP_SIZE_Y) {
			// ��ǵ�ǰλ��Ϊ�ϰ���
			map1[x][y] = 1;

			// �������ͣ������Χ expand_radius ��Χ�ڵĵ�Ϊ�ϰ���
			for (int dx = -expand_radius; dx <= expand_radius; dx++) {
				for (int dy = -expand_radius; dy <= expand_radius; dy++) {
					int nx = x + dx;
					int ny = y + dy;

					// �����λ���Ƿ��ڵ�ͼ��Χ�ڣ������Ϊ�ϰ���
					if (nx >= 0 && nx < MAP_SIZE_X && ny >= 0 && ny < MAP_SIZE_Y) {
						map1[nx][ny] = 1;
					}
				}
			}
		}
	}
}
//// Dijkstra�㷨
vector<Point> Dijkstra(Point start) {
	// ����һ�����ȶ��У����մ��ۣ�cost������
	priority_queue<pair<double, Point>, vector<pair<double, Point>>, greater<pair<double, Point>>> pq;
	vector<vector<double>> dist(MAP_SIZE_X, vector<double>(MAP_SIZE_Y, INF));
	vector<vector<Point>> parent(MAP_SIZE_X, vector<Point>(MAP_SIZE_Y));
		//int dx[] = { 1, -1, 0, 0,1,1,-1,-1};  // �˸�����ƫ��
		//int dy[] = { 0, 0, 1, -1,1,-1,1,-1 };
	int dx[] = { 1, -1, 0, 0 };  // �������ҵ�ƫ��
	int dy[] = { 0, 0, 1, -1 };
	pq.push({ 0, start });
	dist[start.coor_x][start.coor_y] = 0;

	vector<Point> path;
	if (start.coor_x < 0 || start.coor_y < 0) return path;

	while (!pq.empty()) {
		Point cur = pq.top().second;
		pq.pop();

		// ���Ŀ���Ѿ��ҵ�����ǰ����
		if (cur.coor_x == goal.coor_x && cur.coor_y == goal.coor_y) {
			break;
		}

		// �����ĸ�����
		for (int i = 0; i < 7; ++i) {
			int nx = cur.coor_x + dx[i];
			int ny = cur.coor_y + dy[i];

			// �����λ���Ƿ��ڵ�ͼ��Χ�ڣ�����û���ϰ���
			if (nx >= 0 && nx < MAP_SIZE_X && ny >= 0 && ny < MAP_SIZE_Y && map1[nx][ny] == 0) {
				double newCost = dist[cur.coor_x][cur.coor_y] + 1.0;

				// ����µ�·�����̣������
				if (newCost < dist[nx][ny]) {
					dist[nx][ny] = newCost;
					parent[nx][ny] = cur;
					pq.push(std::make_pair(newCost, Point{ (INT16)nx, (INT16)ny }));
				}
			}
		}
	}

	// ���Ŀ�겻�ɴ���ؿ�·��
	
	if (dist[goal.coor_x][goal.coor_y] == INF) return path;

	// ����·��
	Point cur = goal;
	//cout << "goal: " << goal.coor_x << ' ' << goal.coor_y << endl;
	//cout << "cur: " << cur.coor_x << ' ' << cur.coor_y << endl;
	while (!(cur.coor_x == start.coor_x && cur.coor_y == start.coor_y)) {
		path.push_back(cur);
		cur = parent[cur.coor_x][cur.coor_y];
	}
	path.push_back(start);

	// ��ת·��
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
// �����������Ҫ��ƫ�ƽǶ�
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
	// ������ת��Ϊ�Ƕ�
	//double angleInDegrees = angle * 180 / PI;
	//// �����״������ж�Ӧ�Ƕȵ�����
	int startIdx = (int)(0 - angle); // ���5��
	int endIdx = (int)(0 + angle); // �ұ�5��
	// ����������Χ��0��359֮��
	startIdx = (startIdx + 360) % 360;
	endIdx = (endIdx + 360) % 360;

	int distance = 65;

	// ����������������״�����
	for (int i = startIdx; i != endIdx; i = (i + 1) % 360) {
		if (obstacle[i] <= distance) {
			return true; // ���ϰ���
		}
	}
	return false; // û���ϰ���
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
		//if (fabs(diff_angle) > PI / 20) { // Ŀ���ڻ������������
			if (true) { // Ŀ���ڻ������������
			if (diff_angle < 0) { //��Ҫ��ʱ����ת

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
// ���ƻ�����ǰ��
void moveToNextPoint(int index) {
	Point next = globalPath[index];
	// ȷ���ϰ�����ǰ����ȫ������
	if (isObstacleAhead(Cur_rPos,15)) {
		// ������ϰ����ת�ƹ��ϰ�
		rotateToTarget(Cur_rPos, index + 2);  // ˳ʱ�����ʱ����ת
	}
	else {
		// ��Ŀ��ǰ��
		
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

		//if (obstacle[315] > obstacle[45] && obstacle[45] < 50) { // ǰ������90��,��ǰ���ϰ���
		//	C2Sdata.rot_vel = -PI / 4;
		//	cur_rot_vel = -PI / 4;
		//}
		//if (obstacle[315] < obstacle[45] && obstacle[315] < 50) { // ��ǰ���ϰ���
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
				if (S2Cdata.detect_object) { // �ҵ�Ŀ��
					cout << "find object" << endl;
					cout << "target_angle" << S2Cdata.target_angle << endl;
					if (S2Cdata.target_angle > PI / 10) { // 18��
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
						C2Sdata.rot_vel = PI / 8; // 2.25��
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
				// ̫��û��,����
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
							C2Sdata.rot_vel = PI / 8; // * 0.2 = 4.5��
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
						cout << "Ŀ�겻�ɴ�!" << endl;
						continue;
					}

					// ��·�����µ�C2Sdata.Traj
					int pathLength = globalPath.size();
					for (int i = 0; i <= 99 && i < pathLength; ++i) {
						// ���� Traj ��һ�����飬����ֱ�Ӹ�ֵ
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
