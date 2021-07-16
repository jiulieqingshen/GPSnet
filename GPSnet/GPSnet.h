#pragma once
#include<iostream>
#include<fstream>
#include<string>
#include <map>
#include <Eigen/Dense>

using std::string;
using std::ifstream;
using std::ofstream;
using std::ios;
using std::cout;
using std::map;
using std::endl;
using namespace::Eigen;//����Eigen�����


struct Point
{
	string pointName;//����
	int pointNum = -1;//���
	double x = 0, y = 0, z = 0;//���xyz����
	bool known = false;//�õ��Ƿ���֪������߽�������
};

struct GPSline	
{
	//���߽ṹ�壬������㡢�յ�ͻ��ߵ�xyz���������
	Point begin, end;
	double dx, dy, dz;
};



//GPS����
class GPSnet
{
private:

	GPSline* lines;	//�������飬�洢ÿһ��������Ϣ
	Point* knownPoints;//��֪�����飬�洢��֪����Ϣ
	map<string, int>dic;//�ֵ䣬keyΪ������valueΪ���
	MatrixXd B;//ϵ������
	MatrixXd l;//l
	MatrixXd P;//Ȩ��P
	MatrixXd v;//����������v
	MatrixXd x;//
	int known_point_num, nuknown_point_num, observation_num;//��֪����Ŀ��δ֪����Ŀ��������Ŀ


public:

	GPSnet();//�հ׹���
	void loadData(string filePath);//����GPS������Ҫ���������ļ�·��
	void print();//���������������̨
	void print(string filePath);//������������ļ�
	~GPSnet();//��������

};