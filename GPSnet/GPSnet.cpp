#include"GPSnet.h"


//�հ׹���
GPSnet::GPSnet()
{
	
}


//���ã�����GPS��������
//����filePath�����ݵ�url·��
void GPSnet::loadData(string filePath)
{
	//1.��ȡ�����ļ�
	ifstream ifs(filePath);
	//��ȡ��֪����Ŀ��δ֪����Ŀ��������Ŀ
	ifs >> known_point_num >> nuknown_point_num >> observation_num;
	//��ʼ���������
	lines = new GPSline[observation_num];
	knownPoints = new Point[known_point_num];
	B = MatrixXd::Zero(observation_num * 3, nuknown_point_num * 3);
	l = MatrixXd::Random(observation_num * 3, 1);
	P = MatrixXd::Zero(observation_num * 3, observation_num * 3);
	//��ȡ��֪����Ϣ��knownPoints�����dic�ֵ�
	for (int i = 0; i < known_point_num; i++)
	{
		ifs >> knownPoints[i].pointName >> knownPoints[i].x >> knownPoints[i].y >> knownPoints[i].z;
		knownPoints[i].known = true;
		knownPoints[i].pointNum = i;
		dic[knownPoints[i].pointName] = i;
	}
	//��ȡ�������ݣ��������Ȩ��P
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




	//2.Ϊ����
	//���������������֪������
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
	//Ȼ��Ϊ��֪�㡢δ֪���ţ���������С������֪����ǰ��δ֪���ں�
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



	//3.�����������
	while (true)
	{
		//ѭ��������־����ֵΪtrue�����ô�ѭ��û�м���������꣬ѭ������
		bool success = true;
		for (int i = 0; i < observation_num; i++)
		{
			//��һ����������������֪���յ�δ֪�����յ�����=�������+��
			if (lines[i].begin.known == true && lines[i].end.known == false)
			{
				lines[i].end.x = lines[i].begin.x + lines[i].dx;
				lines[i].end.y = lines[i].begin.y + lines[i].dy;
				lines[i].end.z = lines[i].begin.z + lines[i].dz;
				//ΪGPS�����У���������յ�������ͬ�ĵ㸳ֵ
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
			//�ڶ���������������δ֪���յ���֪�����������=�յ�����-��
			if (lines[i].begin.known == false && lines[i].end.known == true)
			{
				//Ϊ������긳ֵ
				lines[i].begin.x = lines[i].end.x - lines[i].dx;
				lines[i].begin.y = lines[i].end.y - lines[i].dy;
				lines[i].begin.z = lines[i].end.z - lines[i].dz;
				//ΪGPS�����У�����������������ͬ�ĵ㸳ֵ
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



	//4.����B��l����
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
	x = (B.transpose()*P*B).inverse()*B.transpose()*P*l;
	v = B * x - l;
}

//���������������̨
void GPSnet::print()
{
	cout << "*******************ƽ����*******************" << endl;
	cout << "P:" << endl << P << endl;
	cout << "B:" << endl << B << endl;
	cout << "l:" << endl << l << endl;
	cout << "x:" << endl << x << endl;
	cout << "v:" << endl << v << endl;
	for (int i = 0; i < observation_num; i++)
	{	
		cout << lines[i].begin.pointName << "  " << lines[i].end.pointName << "  ";
		cout << "dx:" << lines[i].dx + v(i * 3 + 0,0) << "  dy:" << lines[i].dy + v(i * 3 + 1,0) << "  dz:" << lines[i].dz + v(i * 3 + 2,0) << endl;
	}
	map<string, int>temp;
	int index = 0;
	for (int i = 0; i < known_point_num; i++)
	{
		temp[knownPoints[i].pointName] = 0;
	}
	for (int i = 0; i < observation_num; i++)
	{
		if ((temp.find(lines[i].begin.pointName) == temp.end()))
		{
			cout << lines[i].begin.pointName << ",   x:" << lines[i].begin.x + x(index,0)
				<< ",   y:" << lines[i].begin.y + x(index + 1,0) << ",   z:" << lines[i].begin.z + x(index + 2,0) << endl;
			index += 3;
			temp[lines[i].begin.pointName] = 1;
		}
		if ((temp.find(lines[i].end.pointName) == temp.end()))
		{
			cout << lines[i].end.pointName << ",   x:" << lines[i].end.x + x(index,0)
				<< ",   y:" << lines[i].end.y + x(index + 1,0) << ",   z:" << lines[i].end.z + x(index + 2,0) << endl;
			index += 3;
			temp[lines[i].end.pointName] = 1;
		}
	}
	cout << "\n\n\n*******************��������*******************" << endl;
	cout << "Qxx:" << endl << (B.transpose()*P*B).inverse() << endl;
	cout << "Qll:" << endl<<B* (B.transpose()*P*B).inverse() *B.transpose()<<endl;
}


//������������ļ�
void GPSnet::print(string filePath)
{
	ofstream ofs(filePath, ios::trunc);
	ofs << "*******************ƽ����*******************" << endl;
	ofs << "P:" << endl << P << endl;
	ofs << "B:" << endl << B << endl;
	ofs << "l:" << endl << l << endl;
	ofs << "x:" << endl << x << endl;
	ofs << "v:" << endl << v << endl;
	for (int i = 0; i < observation_num; i++)
	{
		ofs << lines[i].begin.pointName << "  " << lines[i].end.pointName << "  ";
		ofs << "dx:" << lines[i].dx + v(i * 3 + 0, 0) << "  dy:" << lines[i].dy + v(i * 3 + 1, 0) << "  dz:" << lines[i].dz + v(i * 3 + 2, 0) << endl;
	}
	map<string, int>temp;
	int index = 0;
	for (int i = 0; i < known_point_num; i++)
	{
		temp[knownPoints[i].pointName] = 0;
	}
	for (int i = 0; i < observation_num; i++)
	{
		if ((temp.find(lines[i].begin.pointName) == temp.end()))
		{
			ofs << lines[i].begin.pointName << ",   x:" << lines[i].begin.x + x(index, 0)
				<< ",   y:" << lines[i].begin.y + x(index + 1, 0) << ",   z:" << lines[i].begin.z + x(index + 2, 0) << endl;
			index += 3;
			temp[lines[i].begin.pointName] = 1;
		}
		if ((temp.find(lines[i].end.pointName) == temp.end()))
		{
			ofs << lines[i].end.pointName << ",   x:" << lines[i].end.x + x(index, 0)
				<< ",   y:" << lines[i].end.y + x(index + 1, 0) << ",   z:" << lines[i].end.z + x(index + 2, 0) << endl;
			index += 3;
			temp[lines[i].end.pointName] = 1;
		}
	}
	ofs << "\n\n\n\n*******************��������*******************" << endl;
	ofs << "Qxx:" << endl << (B.transpose()*P*B).inverse() << endl<<endl;
	ofs << "Qll:" << endl << B * (B.transpose()*P*B).inverse() *B.transpose() << endl;
	ofs.close();
}


GPSnet::~GPSnet()
{
	delete[] lines;
	delete[] knownPoints;
}