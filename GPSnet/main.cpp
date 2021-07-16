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
	MatrixXd B;
	MatrixXd l;
	MatrixXd P;
	int known_point_num, nuknown_point_num, observation_num;




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
	B = MatrixXd::Zero(observation_num * 3, nuknown_point_num * 3);
	l = MatrixXd::Random(observation_num * 3, 1);
	P = MatrixXd::Zero(observation_num * 3, observation_num * 3);



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
		ifs >> P(i * 3 + 0, i * 3 + 0) >> P(i * 3 + 0, i * 3 + 1) >> P(i * 3 + 0, i * 3 + 2)
			>> P(i * 3 + 1, i * 3 + 0) >> P(i * 3 + 1, i * 3 + 1) >> P(i * 3 + 1, i * 3 + 2)
			>> P(i * 3 + 2, i * 3 + 0) >> P(i * 3 + 2, i * 3 + 1) >> P(i * 3 + 2, i * 3 + 2);
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
					lines[i].end.pointNum = dic[lines[i].end.pointName];
			}
		}
		if (!(dic.find(lines[i].end.pointName) == dic.end()))
			lines[i].end.pointNum = dic[lines[i].end.pointName];
		else
		{
			dic[lines[i].end.pointName] = sortedNum + 1;
			sortedNum++;
			for (int i = 0; i < observation_num; i++)
			{
				if (!(dic.find(lines[i].begin.pointName) == dic.end()))
					lines[i].begin.pointNum = dic[lines[i].begin.pointName];
				if (!(dic.find(lines[i].end.pointName) == dic.end()))
					lines[i].end.pointNum = dic[lines[i].end.pointName];
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
				lines[i].end.known = true;
				success = false;
			}
			if (lines[i].begin.known == false && lines[i].end.known == true)
			{
				lines[i].begin.x = lines[i].end.x - lines[i].dx;
				lines[i].begin.y = lines[i].end.y - lines[i].dy;
				lines[i].begin.z = lines[i].end.z - lines[i].dz;
				lines[i].begin.known = true;
				success = false;
			}
		}
		if (success == true)
			break;
	}






	for (int i = 0; i < observation_num; i++)
	{
		cout<<lines[i].begin.known;
	}



	for (int i = 0; i < observation_num ; i++)
	{
		int beginIndex = lines[i].begin.pointNum - (known_point_num - 1)-1;
		int endIndex = lines[i].end.pointNum - (known_point_num-1)-1;
		if (beginIndex >= 0)
		{
			B(3 * i + 0, 3 * beginIndex + 0) = -1;
			B(3 * i + 1, 3 * beginIndex + 1) = -1;
			B(3 * i + 2, 3 * beginIndex + 2) = -1;
		}
		if (endIndex >= 0)
		{
			B(3 * i + 0, 3 * endIndex + 0) = 1;
			B(3 * i + 1, 3 * endIndex + 1) = 1;
			B(3 * i + 2, 3 * endIndex + 2) = 1;
		}
		l(3 * i + 0) = lines[i].end.x - lines[i].begin.x - lines[i].dx;
		l(3 * i + 1) = lines[i].end.y - lines[i].begin.y - lines[i].dy;
		l(3 * i + 2) = lines[i].end.z - lines[i].begin.z - lines[i].dz;
	}

}

GPSnet::~GPSnet()
{
	delete[] lines;
	delete[] knownPoints;
}