#include"GPSnet.h"

int main()
{
	GPSnet network;//创建GPS网对象
	network.loadData("../data/data.txt");//导入数据文件
	network.print();//控制台输出解算结果
	network.print("../results/result.txt");//保存结算结果到文件中
	return 0;
}