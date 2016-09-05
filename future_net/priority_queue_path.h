#ifndef __PRIORITY_QUEUE_PAHT_H__
#define __PRIORITY_QUEUE_PAHT_H__
#include <string.h>
#include <stdlib.h>
#include "Path.h"

/**
 * 自定义的priority_queue类，用大顶堆实现，只能存放Path*
 * 将buf[]改成Path指针的指针数组，所有的Path都是在外部new出来的，这里只存Path*
 * 不负责Path的申请和释放
 * v1.2
 */
class priority_queue_path{
private:
	Path** buf;
	int buf_size;
	int e_size;
	/*从s指定的位置开始做一次筛选*/
	void shuffle(int s){
		Path* path_s = buf[s];
		int p = s;
		int j;
		for(j=(p<<1)+1; j<e_size; j=(j<<1)+1){
			if(j+1<e_size && *buf[j]<*buf[j+1]) j++;//找出左右孩子中较大的那个
			if(*buf[j]<*path_s) break;
			buf[p] = buf[j];
			p = j;
		}
		buf[p] = path_s;
	}
public:
	priority_queue_path(){
		buf_size = 32;
		buf = new Path*[buf_size];
		e_size = 0;
	}
	/*复制构造函数，形如priority_queue_path queue2 = queue;*/
	priority_queue_path(const priority_queue_path& other){
//		printf("in priority_queue_path(const priority_queue_path& other)\n");
		buf = new Path*[other.buf_size];
		memcpy(buf, other.buf, sizeof(Path*)*other.e_size);
		buf_size = other.buf_size;
		e_size = other.e_size;
	}
	/**
	 * 赋值构造函数，形如
	 * priority_queue_path queue2;
	 * queue2 = queue;
	 */
	priority_queue_path& operator= (const priority_queue_path& other){
//		printf("---in = method---\n");
		if(buf == NULL || e_size < other.e_size){
			delete [] buf;
			buf = new Path*[other.buf_size];
		}
		memcpy(buf, other.buf, sizeof(Path*)*other.e_size);
		buf_size = other.buf_size;
		e_size = other.e_size;
		return *this;
	}
	~priority_queue_path(){
		delete[] buf;
	}
	/*向堆中插入一个元素，如果空间不够先进性扩容*/
	void push(Path* path){
		if(e_size+1 > buf_size){//resize
			Path** buf_n = new Path*[buf_size<<1];
			memcpy(buf_n, buf, sizeof(Path*)*buf_size);
			delete[] buf;
			buf = buf_n;
			buf_size <<= 1;
		}
		int s = e_size;
		for(int p=(s-1)>>1; p>=0; p=(p-1)>>1){
			if(*buf[p]<*path)
				buf[s] = buf[p];
			else
				break;
			s = p;
		}
		buf[s] = path;
		e_size++;
	}
	/*返回权值最大的Path的指针，不进行empty检查*/
	Path* top(){return buf[0];}
	/*删除堆顶元素，不进行empty检查*/
	void pop(){
		buf[0] = buf[e_size-1];
		e_size--;
		shuffle(0);
	}
	int size(){return e_size;}
	bool empty(){return e_size == 0;}
//	void out(){
//		printf("buf_size=%d, size=%d, elememts:", buf_size, e_size);
//		for(int i = 0; i < e_size; i++)
//			printf(" %d", *buf[i]);
//		printf("\n");
//	}
};

#endif
