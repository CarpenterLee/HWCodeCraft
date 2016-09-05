#ifndef __VECTOR_INT_H__
#define __VECTOR_INT_H__
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

//#define __MAX_VECTOR_SIZE 2048 //设成可能需要的最大值
#define __MAX_VECTOR_SIZE 40000

/**
 * 自实现的vector，只能放int, 实现了push_back(), insert(), size() == []等方法
 * 将buf改成动态申请的数组，加入insert(position, n)方法
 * v2.0
 */
class vector_int{
private:
	int* buf;
	int buf_size;
	int length;
public:
	vector_int(){
		buf_size = 8;
		buf = (int*)malloc(sizeof(int)*buf_size);
		length = 0;
	}
	vector_int(int init_size){
		buf_size = 1;
		while(buf_size < init_size)
			buf_size <<= 1;
		buf = (int*)malloc(sizeof(int)*buf_size);
		length = 0;
	}
	vector_int(const vector_int& other){
		buf = (int*)malloc(sizeof(int)*other.buf_size);
		memcpy(buf, other.buf, sizeof(int)*other.length);
		buf_size = other.buf_size;
		length = other.length;
	}
	vector_int& operator= (const vector_int& other){
		if(length<other.length || buf==NULL){
//			if(buf != NULL)
				free(buf);
			buf = (int*)malloc(sizeof(int)*other.buf_size);
		}
		memcpy(buf, other.buf, sizeof(int)*other.length);
		buf_size = other.buf_size;
		length = other.length;
		return *this;
	}
	~vector_int(){
//		if(buf != NULL)
			free(buf);
	}
	/*在最后插入元素*/
	inline void push_back(int n){
		if(length+1 > buf_size){//resize
			buf_size <<= 1;
			if(buf_size > __MAX_VECTOR_SIZE)
				buf_size = __MAX_VECTOR_SIZE;
			buf = (int*)realloc(buf, sizeof(int)*buf_size);
		}
		buf[length++] = n;
	}
	/*将other中的所有元素插依次入到this最后一个元素之后*/
	inline void insert(const vector_int & other){
		if(buf_size < length+other.length){
			while(buf_size < length+other.length)
				buf_size <<= 1;
			if(buf_size > __MAX_VECTOR_SIZE)
				buf_size = __MAX_VECTOR_SIZE;
			buf = (int*)realloc(buf, sizeof(int)*buf_size);
		}
		memcpy(&(buf[length]), other.buf, other.length*sizeof(int));
		length += other.length;
	}
	inline void insert(int position, int n){
		if(length+1 > buf_size){//resize
			buf_size <<= 1;
			if(buf_size > __MAX_VECTOR_SIZE)
				buf_size = __MAX_VECTOR_SIZE;
			buf = (int*)realloc(buf, sizeof(int)*buf_size);
		}
		for(int i = length; i > 0; --i)
			buf[i] = buf[i-1];
		buf[0] = n;
		length++;
	}
	inline int size(){
		return length;
	}
	inline bool operator== (const vector_int & other) const{
		if(length != other.length)
			return false;
//		int i = -1;
//		while(++i<length && buf[i]==other.buf[i]);
//		return i == length;
		for(int i = 0; i < length; ++i)
			if(buf[i]^other.buf[i])
				return false;
		return true;
	}
	inline int& operator[] (const int position){
		return buf[position];
	}
};

#endif
