#ifndef __PATH_H__
#define __PATH_H__
#include "bitset.h"
#include <vector>

typedef struct Path {                                   // 路径信息
    int pathCost;                                       // 路径开销
    int passed;                                         // 路径经过的必经点个数, 不包括起点和终点
    int endPoint;                                       // 路径的终点
    bitset bitPath;                                     // 用顶点编号表示的路径，不包括终点
    std::vector<int> nodePath;                          // 用顶点编号表示的路径，不包括终点
    std::vector<int> edgePath;                          // 用边编号表示的路径
    bool needCheck;                                     // 标记是否需要做减枝操作
    Path(void):pathCost(0), passed(0), endPoint(-1), needCheck(false){}   // 路径的无参构造函数
    bool operator< (const Path & other) const {         // 重载<操作符，路径权值越大优先级越低
        return this->pathCost > other.pathCost;
    }
    bool operator== (const Path & other) const {        //重载==操作符，当两条路径边的编号完全相同时，判定为相等
        if(this->edgePath == other.edgePath) {
            return true;
        } else {
            return false;
        }
    }
    void reverse(int src, int dest) {
        this->endPoint = src;
        this->bitPath.reset(src);
        this->bitPath.set(dest);
        for(unsigned int i = 0, j = edgePath.size() - 1; i < j; i++, j--) {
            int temp = edgePath[i];
            edgePath[i] = edgePath[j];
            edgePath[j] = temp;
        }
        this->nodePath[0] = dest;
        for(unsigned int i = 1, j = nodePath.size() - 1; i < j; i++, j--) {
            int temp = nodePath[i];
            nodePath[i] = nodePath[j];
            nodePath[j] = temp;
        }
    }
} Path;

/*struct path_cmp{//Path* 的比较函数，用于指定priority_queue的比较顺序
	bool operator()(Path *a, Path *b){
//		return *a < *b;
		return a->pathCost > b->pathCost;
	}
};*/

bool path_cmp(Path *a, Path *b) {
    return a->pathCost < b->pathCost;
}


#endif
