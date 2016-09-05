#ifndef __BITSET_H__
#define __BITSET_H__
#include <string.h>
#include <stdio.h>
#include <immintrin.h>

#define __BITSET_SIZE 2048 //必须是64的整数倍
typedef unsigned int ATOMTYPE;

/**
 * 自定义的bitset类，实现了& | == 操作。不进行下标越界检查。
 * 加入AVX指令加速
 * v2.2
 */
//__m256i a256, b256;
int isLoopless_count = 0;
int naive_count = 0;
int avx_count = 0;
unsigned char sub_table[__BITSET_SIZE];
ATOMTYPE mask_table[__BITSET_SIZE];
bool is_table_init = false;
//#pragma pack(32)
struct bitset{
//private:
	int buf_size; 
	int atom_sizeB;//1, 4 or 8 byte
	int move;// 3, 5 or 6
	int mod;// 8, 32 or 64
	int mod_minus_1;
	ATOMTYPE buf[(__BITSET_SIZE>>3)/sizeof(ATOMTYPE)] __attribute__ ((aligned (32)));
//public:
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
		if(!is_table_init){
			is_table_init = true;
			for(int i = 0; i < __BITSET_SIZE; i++){
				sub_table[i] = (unsigned char)(i>>move);
				mask_table[i] = 1<<(i&mod_minus_1);
			}
		}
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
//		return (buf[position>>move]&(1<<(position&mod_minus_1)));
		return buf[sub_table[position]]&mask_table[position];
	}
	/*将position指定的位置1*/
	inline void set(int position){
//		buf[position>>move] |= (1<<(position%mod));
//		buf[position>>move] |= (1<<(position&mod_minus_1));
		buf[sub_table[position]] |= mask_table[position];
	}
	/*将position指定的位置0*/
	void reset(int position){
//		buf[position>>move] &= ~(1<<(position%mod));
//		buf[position>>move] &= ~(1<<(position&mod_minus_1));
		buf[sub_table[position]] &= ~mask_table[position];
	}
	/*bitset按位与操作，返回与之后的结果，不修改原来的值*/
	inline bitset operator& (const bitset& other){
		bitset rs = other;
		for(int i=0; i<buf_size; ++i)
			rs.buf[i] &= this->buf[i];
		return rs;
	}
	/*判断other跟this是否有重复置1的位，如果有就返回false*/
	bool isLoopless(bitset& other){
//		isLoopless_count++;
		//printf("in_isLoopless-----\n");
/*		if(((unsigned long)buf)&31 || ((unsigned long)other.buf)&31){
//			naive_count++;
//			printf("not aligned, use niave &\n");
			for(int i = 0; i < buf_size; i++)
				if(buf[i]&other.buf[i])
					return false;
			return true;
		}*/
		unsigned char* a = (unsigned char*)buf;
		unsigned char* b = (unsigned char*)(other.buf);
/*		unsigned long lower_bit_a = (unsigned long)buf&31;
		unsigned long lower_bit_b = (unsigned long)other.buf&31;
		if(lower_bit_a && lower_bit_b){//neither a or b is aligned
			for(int i = 0; i < 256; i += 32)
				if(!_mm256_testz_si256(
						_mm256_loadu_si256((__m256i*)&a[i]),
						_mm256_loadu_si256((__m256i*)&b[i])))
					return false;
		}else if(!lower_bit_a){//only a is aligned
			for(int i = 0; i < 256; i += 32)
				if(!_mm256_testz_si256(
						_mm256_load_si256((__m256i*)&a[i]),
						_mm256_loadu_si256((__m256i*)&b[i])))
					return false;
		}else if(!lower_bit_b){//only b is aligned
			for(int i = 0; i < 256; i += 32)
				if(!_mm256_testz_si256(
						_mm256_loadu_si256((__m256i*)&a[i]),
						_mm256_load_si256((__m256i*)&b[i])))
					return false;
		}else{//both a and b are aligned
			for(int i = 0; i < 256; i += 32)
				if(!_mm256_testz_si256(
						_mm256_load_si256((__m256i*)&a[i]),
						_mm256_load_si256((__m256i*)&b[i])))
					return false;
		}
		return true;*/
		for(int i = 0; i < 256; i += 32){
//			if(!_mm256_testz_si256(
//						_mm256_load_si256((__m256i*)&a[i]),
//						_mm256_load_si256((__m256i*)&b[i])))
//				return false;
			if(!_mm256_testz_si256(
						_mm256_loadu_si256((__m256i*)&a[i]),
						_mm256_loadu_si256((__m256i*)&b[i])))
				return false;
		}
		return true;
	}
/*	inline bool isLoopless(const bitset& other){
		for(
				register int i = 0;
				i < buf_size;
				++i){
			if(buf[i]
				   &other.buf[i])
				return false;
		}
		return true;
	}*/
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
//		return (buf[position>>move]&(1<<(position&mod_minus_1)));
		return buf[sub_table[position]]&mask_table[position];
	}
};
//#pragma pack()
#endif
