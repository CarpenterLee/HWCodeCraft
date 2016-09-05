#ifndef __BITSET_H__
#define __BITSET_H__
#include <string.h>
#include <stdio.h>

#define __BITSET_SIZE 2048 //必须是64的整数倍
typedef unsigned int ATOMTYPE;

/**
 * 自定义的bitset类，实现了& | == 操作。不进行下标越界检查。
 * 用int数组做buf
 * v2.0
 */
class bitset{
private:
	ATOMTYPE buf[(__BITSET_SIZE>>3)/sizeof(ATOMTYPE)];
	int buf_size;
	int atom_sizeB;//1, 4 or 8 byte
	int move;// 3, 5 or 6
	int mod;// 8, 32 or 64
	int mod_minus_1;
public:
	bitset(){
		atom_sizeB = sizeof(ATOMTYPE);
		buf_size = (__BITSET_SIZE>>3)/atom_sizeB;
		memset(buf, 0, __BITSET_SIZE>>3);
		if(atom_sizeB == 1){
			move = 3;
			mod = 8;
		}else if(atom_sizeB == 4){
			move = 5;
			mod = 32;
		}else{//atom_size == 8
			move = 6;
			mod = 64;
		}
		mod_minus_1 = mod-1;
//		printf("atom_sizeB=%d, move=%d, mod=%d\n", atom_sizeB, move, mod);
	}
	/*返回1的个数，测试时使用，效率很低*/
	int count(){
		int count = 0;
		for(int i = 0; i < __BITSET_SIZE; ++i)
			if(get(i))
				++count;
		return count;
	}
	/*得到position这一位是0还是1*/
	inline int get(int position){
//		return (buf[position>>move]&(1<<(position%mod))) ? 1 : 0;
		return (buf[position>>move]&(1<<(position&mod_minus_1)));
	}
	/*将position指定的位置1*/
	inline void set(int position){
//		buf[position>>move] |= (1<<(position%mod));
		buf[position>>move] |= (1<<(position&mod_minus_1));
	}
	/*将position指定的位置0*/
	void reset(int position){
//		buf[position>>move] &= ~(1<<(position%mod));
		buf[position>>move] &= ~(1<<(position&mod_minus_1));
	}
	/*bitset按位与操作，返回与之后的结果，不修改原来的值*/
	inline bitset operator& (const bitset& other){
		bitset rs = other;
		for(int i=0; i<buf_size; ++i)
			rs.buf[i] &= this->buf[i];
		return rs;
	}
	/*判断other跟this是否有重复置1的位，如果有就返回false*/
	inline bool isLoopless(int &link_from, bitset& other){
		register int i;
		if(other[link_from]) {
            return false;
		}
		for(i = 0; i < buf_size; ++i){
			if(buf[i]&other.buf[i])
				return false;
		}
		return true;
	}
	/*bitset按位或操作，返回或之后的结果，不修改原来的值*/
	inline bitset operator| (const bitset& other){
		bitset rs = other;
		for(int i=0; i<buf_size; ++i)
			rs.buf[i] |= this->buf[i];
		return rs;
	}
	bool operator== (const bitset& other){
		for(int i=0; i<buf_size; ++i)
			if(this->buf[i] != other.buf[i])
				return false;
		return true;
	}
	inline int operator[] (const int position){
//		return (buf[position>>move]&(1<<(position%mod))) ? 1 : 0;
		return (buf[position>>move]&(1<<(position&mod_minus_1)));
	}
};

#endif
