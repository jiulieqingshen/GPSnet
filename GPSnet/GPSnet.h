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
using namespace::Eigen;//引入Eigen矩阵库


struct Point
{
	string pointName;//点名
	int pointNum = -1;//点号
	double x = 0, y = 0, z = 0;//点的xyz坐标
	bool known = false;//该点是否已知坐标或者近似坐标
};

struct GPSline	
{
	//基线结构体，包含起点、终点和基线的xyz测量坐标差
	Point begin, end;
	double dx, dy, dz;
};



//GPS网类
class GPSnet
{
private:

	GPSline* lines;	//基线数组，存储每一条基线信息
	Point* knownPoints;//已知点数组，存储已知点信息
	map<string, int>dic;//字典，key为点名，value为点号
	MatrixXd B;//系数矩阵
	MatrixXd l;//l
	MatrixXd P;//权阵P
	MatrixXd v;//改正数矩阵v
	MatrixXd x;//
	int known_point_num, nuknown_point_num, observation_num;//已知点数目、未知点数目、基线数目


public:

	GPSnet();//空白构造
	void loadData(string filePath);//解算GPS网，需要传入数据文件路径
	void print();//输出解算结果到控制台
	void print(string filePath);//输出解算结果到文件
	~GPSnet();//析构函数

};