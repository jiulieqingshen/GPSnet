#include"GPSnet.h"

int main()
{
	GPSnet network;//����GPS������
	network.loadData("../data/data.txt");//���������ļ�
	network.print();//����̨���������
	network.print("../results/result.txt");//������������ļ���
	return 0;
}