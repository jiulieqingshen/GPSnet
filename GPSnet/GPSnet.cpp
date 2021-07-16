#include"GPSnet.h"


//空白构造
GPSnet::GPSnet()
{
	
}


//作用：解算GPS网络数据
//参数filePath：数据的url路径
void GPSnet::loadData(string filePath)
{
	//1.读取数据文件
	ifstream ifs(filePath);
	//读取已知点数目、未知点数目、基线数目
	ifs >> known_point_num >> nuknown_point_num >> observation_num;
	//初始化类的属性
	lines = new GPSline[observation_num];
	knownPoints = new Point[known_point_num];
	B = MatrixXd::Zero(observation_num * 3, nuknown_point_num * 3);
	l = MatrixXd::Random(observation_num * 3, 1);
	P = MatrixXd::Zero(observation_num * 3, observation_num * 3);
	//读取已知点信息到knownPoints数组和dic字典
	for (int i = 0; i < known_point_num; i++)
	{
		ifs >> knownPoints[i].pointName >> knownPoints[i].x >> knownPoints[i].y >> knownPoints[i].z;
		knownPoints[i].known = true;
		knownPoints[i].pointNum = i;
		dic[knownPoints[i].pointName] = i;
	}
	//读取基线数据，并计算出权阵P
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
	P = P.inverse();




	//2.为点编号
	//向基线数组填入已知点数据
	for (int i = 0; i < observation_num; i++)
	{
		if (!(dic.find(lines[i].begin.pointName) == dic.end()))
		{
			lines[i].begin.known = true;
			for (int j = 0; j < known_point_num; j++)
			{
				if (knownPoints[j].pointName == lines[i].begin.pointName)
				{
					lines[i].begin.x = knownPoints[j].x;
					lines[i].begin.y = knownPoints[j].y;
					lines[i].begin.z = knownPoints[j].z;
				}
			}
		}
		if (!(dic.find(lines[i].end.pointName) == dic.end()))
		{
			lines[i].end.known = true;
			for (int j = 0; j < known_point_num; j++)
			{
				if (knownPoints[j].pointName == lines[i].end.pointName)
				{
					lines[i].end.x = knownPoints[j].x;
					lines[i].end.y = knownPoints[j].y;
					lines[i].end.z = knownPoints[j].z;
				}
			}
		}
	}
	//然后为已知点、未知点编号（按数字由小到大，已知点在前，未知点在后）
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



	//3.计算近似坐标
	while (true)
	{
		//循环结束标志，初值为true，若该次循环没有计算近似坐标，循环结束
		bool success = true;
		for (int i = 0; i < observation_num; i++)
		{
			//第一种情况：起点坐标已知，终点未知，则终点坐标=起点坐标+Δ
			if (lines[i].begin.known == true && lines[i].end.known == false)
			{
				lines[i].end.x = lines[i].begin.x + lines[i].dx;
				lines[i].end.y = lines[i].begin.y + lines[i].dy;
				lines[i].end.z = lines[i].begin.z + lines[i].dz;
				//为GPS网络中，所有与该终点名称相同的点赋值
				for (int j = 0; j < observation_num; j++)
				{
					if (lines[j].end.pointName == lines[i].end.pointName)
					{
						lines[j].end.known = true;
						lines[j].end.pointNum = lines[i].end.pointNum;
						lines[j].end.x = lines[i].end.x;
						lines[j].end.y = lines[i].end.y;
						lines[j].end.z = lines[i].end.z;
					}
					if (lines[j].begin.pointName == lines[i].end.pointName)
					{
						lines[j].begin.known = true;
						lines[j].begin.pointNum = lines[i].end.pointNum;
						lines[j].begin.x = lines[i].end.x;
						lines[j].begin.y = lines[i].end.y;
						lines[j].begin.z = lines[i].end.z;
					}

				}
				lines[i].end.known = true;
				success = false;
			}
			//第二种情况：起点坐标未知，终点已知，则起点坐标=终点坐标-Δ
			if (lines[i].begin.known == false && lines[i].end.known == true)
			{
				//为起点坐标赋值
				lines[i].begin.x = lines[i].end.x - lines[i].dx;
				lines[i].begin.y = lines[i].end.y - lines[i].dy;
				lines[i].begin.z = lines[i].end.z - lines[i].dz;
				//为GPS网络中，所有与该起点名称相同的点赋值
				for (int j = 0; j < observation_num; j++)
				{
					if (lines[j].begin.pointName == lines[i].begin.pointName)
					{
						lines[j].begin.known = true;
						lines[j].begin.pointNum = lines[i].begin.pointNum;
						lines[j].begin.x = lines[i].begin.x;
						lines[j].begin.y = lines[i].begin.y;
						lines[j].begin.z = lines[i].begin.z;
					}
					if (lines[j].end.pointName == lines[i].begin.pointName)
					{
						lines[j].end.known = true;
						lines[j].end.pointNum = lines[i].begin.pointNum;
						lines[j].end.x = lines[i].begin.x;
						lines[j].end.y = lines[i].begin.y;
						lines[j].end.z = lines[i].begin.z;
					}
				}
				lines[i].begin.known = true;
				success = false;
			}
		}
		if (success == true)
			break;
	}



	//4.计算B、l矩阵
	for (int i = 0; i < observation_num; i++)
	{
		int beginIndex = lines[i].begin.pointNum - (known_point_num - 1) - 1;
		int endIndex = lines[i].end.pointNum - (known_point_num - 1) - 1;
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
		l(3 * i + 0, 0) = lines[i].dx - (lines[i].end.x - lines[i].begin.x);
		l(3 * i + 1, 0) = lines[i].dy - (lines[i].end.y - lines[i].begin.y);
		l(3 * i + 2, 0) = lines[i].dz - (lines[i].end.z - lines[i].begin.z);
	}
}


//输出解算结果到控制台
void GPSnet::print()
{

}


//输出解算结果到文件
void GPSnet::print(string filePath)
{

}


GPSnet::~GPSnet()
{
	delete[] lines;
	delete[] knownPoints;
}