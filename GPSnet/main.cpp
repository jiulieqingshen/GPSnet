#include<iostream>
#include<fstream>
#include<string>
#include <map>
#include <Eigen/Dense>

using std::string;
using std::ifstream;
using std::cout;
using std::map;
using namespace::Eigen;

struct Point
{
	string pointName;
	int pointNum = -1;
	double x, y, z;
	bool known = false;
};

struct GPSline
{
	Point begin, end;
	double dx, dy, dz;
};

class GPSnet
{
private:

	GPSline* lines;
	Point* knownPoints;
	map<string, int>dic;
	int** B;
	Point* l;
	int known_point_num, nuknown_point_num, observation_num;
	double** P;

public:

	GPSnet(string filePath);
	~GPSnet();

};

int main()
{
	GPSnet network("data.txt");
	return 0;
}

GPSnet::GPSnet(string filePath)
{
	ifstream ifs(filePath);
	ifs >> known_point_num >> nuknown_point_num >> observation_num;


	lines = new GPSline[observation_num];
	knownPoints = new Point[known_point_num];
	B = new int*[observation_num];
	for (int i = 0; i < observation_num; i++)
		B[i] = new int[nuknown_point_num];
	l = new Point[observation_num];
	P = new double*[observation_num * 3];
	for (int i = 0; i < observation_num; i++)
		P[i] = new double[observation_num * 3];
	for (int i = 0; i < observation_num; i++)
		for (int j = 0; j < observation_num; j++)
			P[i][j] = 0;


	for (int i = 0; i < known_point_num; i++)
	{
		ifs >> knownPoints[i].pointName >> knownPoints[i].x >> knownPoints[i].y >> knownPoints[i].z;
		knownPoints[i].known = true;
		knownPoints[i].pointNum = i;
		dic[knownPoints[i].pointName] = i;
	}



	for (int i = 0; i < observation_num; i++)
	{
		ifs >> lines[i].begin.pointName >> lines[i].end.pointName
			>> lines[i].dx >> lines[i].dy >> lines[i].dz;
		lines[i].begin.known = false;
		lines[i].end.known = false;
		ifs >> P[i][0] >> P[i][1] >> P[i][2];
		for (int j = 0; j < observation_num; j++)
		{
			ifs >> P[i * 3 + 0][i * 3 + 0] >> P[i * 3 + 0][i * 3 + 1] >> P[i * 3 + 0][i * 3 + 2]
				>> P[i * 3 + 1][i * 3 + 0] >> P[i * 3 + 1][i * 3 + 1] >> P[i * 3 + 1][i * 3 + 2]
				>> P[i * 3 + 2][i * 3 + 0] >> P[i * 3 + 2][i * 3 + 1] >> P[i * 3 + 3][i * 3 + 2];
		}
	}






	for (int i = 0; i < observation_num; i++)
	{
		if (!(dic.find(lines[i].begin.pointName) == dic.end()))
			lines[i].begin.known = true;
		if (!(dic.find(lines[i].end.pointName) == dic.end()))
			lines[i].end.known = true;
	}



	int sortedNum = known_point_num - 1;
	for (int i = 0; i < observation_num; i++)
	{
		if (!(dic.find(lines[i].begin.pointName) == dic.end()))
			lines[i].begin.pointNum = dic[lines[i].begin.pointName];
		else
		{
			dic[lines[i].begin.pointName] = sortedNum + 1;
			sortedNum++;
			for (int i = 0; i < observation_num; i++)
			{
				if (!(dic.find(lines[i].begin.pointName) == dic.end()))
					lines[i].begin.pointNum = dic[lines[i].begin.pointName];
				if (!(dic.find(lines[i].end.pointName) == dic.end()))
					lines[i].end.pointName = dic[lines[i].end.pointName];
			}
		}
		if (!(dic.find(lines[i].end.pointName) == dic.end()))
			lines[i].end.pointName = dic[lines[i].end.pointName];
		else
		{
			dic[lines[i].end.pointName] = sortedNum + 1;
			sortedNum++;
			for (int i = 0; i < observation_num; i++)
			{
				if (!(dic.find(lines[i].begin.pointName) == dic.end()))
					lines[i].begin.pointNum = dic[lines[i].begin.pointName];
				if (!(dic.find(lines[i].end.pointName) == dic.end()))
					lines[i].end.pointName = dic[lines[i].end.pointName];
			}
		}
	}



	while (true)
	{
		bool success = true;
		for (int i = 0; i < observation_num; i++)
		{
			if (lines[i].begin.known == true && lines[i].end.known == false)
			{
				lines[i].end.x = lines[i].begin.x + lines[i].dx;
				lines[i].end.y = lines[i].begin.y + lines[i].dy;
				lines[i].end.z = lines[i].begin.z + lines[i].dz;
				success = false;
			}
			if (lines[i].begin.known == false && lines[i].end.known == true)
			{
				lines[i].begin.x = lines[i].end.x - lines[i].dx;
				lines[i].begin.y = lines[i].end.y - lines[i].dy;
				lines[i].begin.z = lines[i].end.z - lines[i].dz;
				success = false;
			}
		}
		if (success == true)
			break;
	}


	for (int i = 0; i < observation_num; i++)
	{
		int beginIndex = lines[i].begin.pointNum - known_point_num - 1;
		int endIndex = lines[i].end.pointNum - known_point_num - 1;
		if (beginIndex > 0)
			B[i][beginIndex] = -1;
		if (endIndex > 0)
			B[i][endIndex] = 1;
		l[i].x = lines[i].end.x - lines[i].begin.x - lines[i].dx;
		l[i].y = lines[i].end.y - lines[i].begin.y - lines[i].dy;
		l[i].z = lines[i].end.z - lines[i].begin.z - lines[i].dz;
	}





}

GPSnet::~GPSnet()
{
	delete[] lines;
	delete[] knownPoints;
	delete[] l;
	delete[] B;
}