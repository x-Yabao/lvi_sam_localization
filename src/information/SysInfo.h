#pragma once
 
#include <stdio.h>
#include <vector>
#include <iostream>

#include "parameters.h"

class SysInfo
{
public:
	SysInfo();

	int updateSysInfo();
	void outputSysInfo();

	int getMemoryUsage() { return 0; }
	int getCpuUsage() { return us; }

private:
	//17 paras
	//procs
	int r, b;
	//memory
	int swpd;		//已经使用的交换内存（kb）
	int free;		//空闲的物理内存（kb）
	int buff;		//用做缓冲区的内存数（kb）
	int cache;		//用做高速缓存的内存数（kb）
	//swap
	int si, so;
	//io
	int bi, bo;
	//system
	int in, cs;
	//cpu
	int us;			//用户进程使用的cpu时间（%）
	int sy;			//系统进程使用的cpu时间（%）
	int id;			//CPU空闲时间（%）
	int wa;			//等待IO所消耗的cpu时间（%）
	int st;

};


 
