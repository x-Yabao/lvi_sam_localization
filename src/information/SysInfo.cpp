#include "SysInfo.h"

//调用linux系统命令获取cpu信息
#define CMD_VMSTAT "vmstat 2 2|tail -1|awk '{print $1,$2,$3,$4,$5,$6,$7,$8,$9,$10,$11,$12,$13,$14,$15,$16,$17}'"

SysInfo::SysInfo()
{

}


int SysInfo::updateSysInfo()
{
	FILE *fp = popen(CMD_VMSTAT, "r");
	if(fp == NULL) {
		return -1;
	}
	fscanf(fp, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d", 
		&r, &b,
		&swpd, &free, &buff, &cache,
		&si, &so,
		&bi, &bo,
		&in, &cs,
		&us, &sy, &id, &wa, &st);
	pclose(fp);

	return 0;
}

void SysInfo::outputSysInfo()
{	
	std::cout << "***************memory info***************" << std::endl;
	std::cout << "swpd:" << swpd << std::endl;
	std::cout << "free:" << free << std::endl;
	std::cout << "buff:" << buff << std::endl;
	std::cout << "cache:" << cache << std::endl;

	std::cout << "***************cpu info***************" << std::endl;
	std::cout << "us:" << us << std::endl;
	std::cout << "sy:" << sy << std::endl;
	std::cout << "id:" << id << std::endl;
	std::cout << "wa:" << wa << std::endl;
	std::cout << "st:" << st << std::endl;
}
