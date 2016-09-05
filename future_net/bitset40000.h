#ifndef __BITSET40000_H__
#define __BITSET40000_H__
#include <string.h>
#include <stdio.h>

#define __BITSET40000_SIZE 40000 //必须是64的整数倍
typedef unsigned int __ATOMTYPE2;

/**
 * 自定义的bitset，用于route.get_conflict()
 * v1.0
 */
class bitset40000{
private:
	__ATOMTYPE2 buf[(__BITSET40000_SIZE>>3)/sizeof(__ATOMTYPE2)];
	int buf_size;
	int atom_sizeB;//1, 4 or 8 byte
	int move;// 3, 5 or 6
	int mod;// 8, 32 or 64
	int mod_minus_1;
public:
	bitset40000(){
		atom_sizeB = sizeof(__ATOMTYPE2);
		buf_size = (__BITSET40000_SIZE>>3)/atom_sizeB;
		memset(buf, 0, __BITSET40000_SIZE>>3);
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
//		printf("bitset40000 atom_sizeB=%d, move=%d, mod=%d\n", atom_sizeB, move, mod);
	}
	/*全部清零*/
	inline void clear(){
		memset(buf, 0, __BITSET40000_SIZE>>3);
	}
	inline void set(int position){
		buf[position>>move] |= (1<<(position&mod_minus_1));
	}
	/*相与之后求1的个数*/
	inline int get_conflict_count(const bitset40000& other){
		int count = 0;
		for(int i = 0; i < buf_size; ++i){
			__ATOMTYPE2 t = buf[i]&other.buf[i];
			while(t){
				count++;
				t &= t-1;
			}
		}
		return count;
	}
	inline int operator[] (const int position){
		return buf[position>>move]&(1<<(position&mod_minus_1));
	}
};

#endif
