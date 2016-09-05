/**
 * 功能: 解决link-disjoint问题, 自实现GLG算法
 * 作者: GuoJiaLiang, LiHao, GuoZhenXing
 * 时间: 2016-5-13
 * 版本: v2.0
 */
#include "route.h"
#include "lib_record.h"
#include "bitset.h"
#include "Path.h"
#include <stdio.h>
#include <stdlib.h>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <vector>
#include <queue>
#include <algorithm>
#include <bitset>

#define MAX_VERTEX 2000
#define INT_MAX 0xffffff

//----------------------------------------------------------------------------------------------------------------------------------------类型定义
struct pair_hash {                                                              // pair类型的hash函数
public:
    template <typename T, typename U>
    std::size_t operator()(const std::pair<T, U> &x) const {
        return (x.first << 11) + x.second;
    }
};
struct pair_cmp{                                                                // pair类型的()函数
	bool operator()(std::pair<int, int> a, std::pair<int, int> b){
		return a.second > b.second;
	}
};
typedef std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int> >, pair_cmp> PairQueue;    // pair类型的优先队列
typedef std::pair<int, int>             Edge;                                   // 有向边的定义, first代表边的起点， second代表边的终点
typedef std::pair<int, int>             EdgeInfo;                               // 有向边的附加信息的定义, first代表边的编号, second代表边的权重
typedef std::unordered_map<Edge, PairQueue, pair_hash> EdgeInfoDict;            // 有向边附加信息字典的定义
typedef std::vector<std::set<int> >   Graph;                                    // 图的定义
typedef std::vector<std::pair<int, int> > Previous;                             // 保存路径前向信息, <父节点编号，边的编号>
typedef std::set<int>                   Conditions;                             // 必须经过的结点集合
typedef std::vector<Path*> PathQueue;                                           // 保存访问路径的数组
typedef std::unordered_map<int, std::unordered_map<int, PathQueue> > DpTable;   // DP表
typedef std::unordered_map<int, PathQueue> DistanceTable;                       // 距离表
typedef std::vector<std::set<int> > DirectArrive;                               // 直接到达矩阵
typedef std::unordered_map<int, std::vector<int> > ObstructTable;

typedef struct CSR {                                                            // CSR数据结构保存稀疏图
    std::vector<PairQueue> edgeinfos;
    std::vector<int> columns;
    std::vector<int> rowIndex;
    CSR(void):rowIndex(MAX_VERTEX + 1) {}
} CSR;

typedef struct OrderConstraint {                                                // OrderConstraint数据结构存储阻塞点
    std::vector<int> columns;
    std::vector<int> rowIndex;
    std::vector<int> obstructNodes;
    OrderConstraint() : rowIndex(MAX_VERTEX + 1) {}
} OrderConstraint;
//-----------------------------------------------------------------------------------------------------------------------------------------全局变量定义
bool forward = true;                                                            // 正向搜索为true, 反向搜索为false
int search_index;                                                               // 搜索编号
int source, dest;                                                               // 源和目的
Conditions conditions[2];                                                       // 两个必经点集合
bitset conditions_bits[2];                                                      // 两个必经点的bitset表示
Graph graph(MAX_VERTEX), tr_graph(MAX_VERTEX);                                  // 图的正向和反向表示
EdgeInfoDict edgeInfoDict, tr_edgeInfoDict;                                     // 边信息字典的正向和反向表示
CSR csr[2], tr_csr[2];                                                          // CSR结构表示的正向和反向图
std::vector<Path *> routes[2];                                                  // 满足约束条件的路径集合
std::vector<Path *> result(2);                                                  // 最终结果, result[0]为work_path, result[1]为back_path

unsigned int MAX_TABLE = 8000;                                                  // 动态规划表中包含的最大路径数
unsigned int F_K_PATH = 9;                                                      // 填动态规划表的路径数
unsigned int DELTA = 100;                                                       // 更新边的权值
const bitset zero_bitset;                                                       // 用于清空bitset
bitset available;                                                               // 有效点(从源点可以到，且可以到目的点)的bitset表示
//-----------------------------------------------------------------------------------------------------------------------------------------输入输出函数
int ReadANumberFromStr(char *, unsigned int &);                                 // 从字符流中读取一个数字
void ReadGraphData(char **, int);                                               // 读取图信息
void ReadConditionsData(char *);                                                // 读取约束条件信息
void PrintPath(void);                                                           // 打印work path, back path
//-----------------------------------------------------------------------------------------------------------------------------------------复赛策略函数
void get_disjoint_paths(char *[MAX_EDGE_NUM], int, char *[MAX_DEMAND_NUM], int);     // 获取link-disjoint path
void CleanGraphData(void);                                                      // 清理图数据, 获得源可达且可达目的的顶点, 更新全局变量available
void DFS(CSR &, int, bitset &);                                                 // 深度优先搜索函数, 辅助清理图数据
void generate_first_paths(int);                                                 // 得到第一组路径, 参数表示受哪组必经点约束, -1时表示不受约束
void generate_second_paths(void);                                               // 得到第二组路径
void generate_csr_with_constraint(Graph &, EdgeInfoDict &, CSR &, bitset &);    // 根据图和边信息, 产生受给定bitset约束的CSR
void generate_csr(Graph &, EdgeInfoDict &, CSR &);                              // 根据图和边信息, 产生CSR
void update_edgeinfo(std::vector<Path *> &);                                    // 更新图的边字典信息
int get_optimum_paths(void);                                                    // 获取最优解
int get_conflict(Path *, Path *);                                               // 得到两条路将重合的边个数
//-------------------------------------------------------------------------------------------------------------------------------------------剪枝函数
void get_arrive_info(int, DirectArrive &, DirectArrive &);                      // 构造直接到达矩阵的一行, 返回一行中1的个数
void generateOrderConstraint(DirectArrive &, OrderConstraint &);                // 将阻塞点信息转换成OrderConstraint数据结构
bool isValid(Path *, OrderConstraint &, OrderConstraint &);                     // 判断路径合法性
//-------------------------------------------------------------------------------------------------------------------------------------------算法函数
void get_shortest_path(int, DistanceTable &, OrderConstraint &, OrderConstraint &); // 使用Dijkstra求权值最短路径
void Dijkstra(int, Previous &, std::vector<int> &, bitset &);                   // Dijkstra算法
Path* search_path(int, int, Previous &, std::vector<int> &);                    // 辅助Dijkstra生成反向路径
void get_second_path(int, int, Previous &, std::vector<int> &, DistanceTable &, OrderConstraint &, OrderConstraint &);      // 求近似第二短路径
void get_BFS_path(int, DistanceTable &, DistanceTable &, OrderConstraint &, OrderConstraint &);                             // 使用广度优先搜索求长度最短路径
void fill_dp(int, int, DpTable &, DistanceTable &, DirectArrive &,              // 填动态规划表
             OrderConstraint &, OrderConstraint &);
void get_result(DpTable & dp);                                                  // 获取满足约束条件的路径
Path* connect_path(Path *, Path *);                                             // 连接两条路径
bool isDuplicate(Path *, std::vector<Path *> &);                                // 判断路径是否重复
//bool isLoopless(Path*, Path* );                                               // 判断两条路径是否构成环
void clear_memory(DpTable &, DistanceTable &);                                  //释放内存空间

void GLG(void);                                                                 // GLG算法
//------------------------------------------------------------------------------------------------------------------------------------赛题入口
void search_route(char *topo[MAX_EDGE_NUM], int edge_num, char *demand[MAX_DEMAND_NUM], int demand_num) {
    get_disjoint_paths(topo, edge_num, demand, demand_num);
    PrintPath();
}
//------------------------------------------------------------------------------------------------------------------------------------输入输出函数实现
int ReadANumberFromStr(char *str, unsigned int &index) {
    int res = str[index] - '0';
    while(str[++index] >= '0' && str[index] <= '9') {
        res *= 10;
        res += str[index] - '0';
    }
    while(index < strlen(str) && !(str[index] >= '0' && str[index] <= '9')){index++;}
    return res;
}

void ReadGraphData(char *graphStream[MAX_EDGE_NUM], int edge_num) {
    int maxCost = -1;
    for(int i = 0; i < edge_num; ++i) {
        unsigned int j = 0;
        int edgeNo = ReadANumberFromStr(graphStream[i], j);
        int edgeFrom = ReadANumberFromStr(graphStream[i], j);
        int edgeTo = ReadANumberFromStr(graphStream[i], j);
        int edgeCost = ReadANumberFromStr(graphStream[i], j);

        graph[edgeFrom].insert(edgeTo);
        tr_graph[edgeTo].insert(edgeFrom);

        Edge edge(edgeFrom, edgeTo);
        Edge tr_edge(edgeTo, edgeFrom);
        EdgeInfo edgeInfo(edgeNo, edgeCost);

        edgeInfoDict[edge].push(edgeInfo);
        tr_edgeInfoDict[tr_edge].push(edgeInfo);
        maxCost = maxCost > edgeCost ? maxCost : edgeCost;
    }
    DELTA = 2 * maxCost;
}

void ReadConditionsData(char *conditionsStream) {
    unsigned int i = 0;
    ReadANumberFromStr(conditionsStream, i);
    source = ReadANumberFromStr(conditionsStream, i);
    dest = ReadANumberFromStr(conditionsStream, i);
    while(i < strlen(conditionsStream)) {
        int number = ReadANumberFromStr(conditionsStream, i);
        conditions[search_index].insert(number);
        conditions_bits[search_index].set(number);
    }
}

void PrintPath() {
    if(result[0] == NULL || result[1] == NULL) {
        return;
    }
    if(forward) {
        for (int i = 0; i < result[0]->edgePath.size(); i++) {
            record_result(WORK_PATH, result[0]->edgePath[i]);
        }
        for (int i = 0; i < result[1]->edgePath.size(); i++) {
            record_result(BACK_PATH, result[1]->edgePath[i]);
        }
    } else {
        for (int i = result[0]->edgePath.size() - 1; i >= 0; i--) {
            record_result(WORK_PATH, result[0]->edgePath[i]);
        }
        for (int i = result[1]->edgePath.size() - 1; i >= 0; i--) {
            record_result(BACK_PATH, result[1]->edgePath[i]);
        }
    }
}
//----------------------------------------------------------------------------------------------------------------------------------------复赛策略函数实现
void get_disjoint_paths(char *topo[MAX_EDGE_NUM], int edge_num, char *demand[MAX_DEMAND_NUM], int demand_num) {
    ReadGraphData(topo, edge_num);
    for(int i = 0; i < demand_num; i++) {
        search_index = i;
        ReadConditionsData(demand[i]);
    }

    forward = false;
    std::swap(source, dest);
    swap(graph, tr_graph);
    swap(edgeInfoDict, tr_edgeInfoDict);

    int first_search, second_search;
    if(conditions[0].size() <= conditions[1].size()) {
        first_search = 0;
        second_search = 1;
    } else {
        first_search = 1;
        second_search = 0;
    }
    if(conditions[first_search].size() || conditions[second_search].size()) {
        // 两个必经点集合都不为空 两个必经点集合中有且仅有一个为空, 先计算约束个数小的
        search_index = first_search;
        generate_first_paths(second_search);
        if(routes[search_index].size()) {
            update_edgeinfo(routes[search_index]);
        } else {    //如果第一条路没求出来，说明无解
            return;
        }
        search_index = second_search;
        generate_second_paths();
        // 得到最优结果
        get_optimum_paths();
    } else {     // 两个必经点集合都为空
        EdgeInfoDict temp_edgeInfoDict = edgeInfoDict, temp_tr_edgeInfoDict = tr_edgeInfoDict;
        search_index = 0;
        generate_first_paths(-1);
        update_edgeinfo(routes[0]);
        search_index = 1;
        generate_second_paths();
        //得到最优结果
        if(get_optimum_paths()) {
            edgeInfoDict = temp_edgeInfoDict;
            tr_edgeInfoDict = temp_tr_edgeInfoDict;
            update_edgeinfo(routes[1]);
            search_index = 0;
            CSR empty;
            csr[0] = tr_csr[0] = empty;
            generate_second_paths();
            get_optimum_paths();
        }
    }
}

void CleanGraphData() {
    bitset source_can_arrive;
    bitset dest_can_arrive;
    DFS(csr[search_index], source, source_can_arrive);
    DFS(tr_csr[search_index], dest, dest_can_arrive);
    available = source_can_arrive & dest_can_arrive;
}

void DFS(CSR &csr, int link_from, bitset &arrive) {
    arrive.set(link_from);
    if(csr.rowIndex[link_from] == csr.rowIndex[link_from + 1]) {
        return;
    }
    for(int i = csr.rowIndex[link_from]; i < csr.rowIndex[link_from + 1]; i++) {
        if(!arrive[csr.columns[i]]) {
            DFS(csr, csr.columns[i], arrive);
        }
    }
}

void generate_first_paths(int constraint_index) {
    if(constraint_index == -1) {
        generate_csr(graph, edgeInfoDict, csr[search_index]);
        generate_csr(tr_graph, tr_edgeInfoDict, tr_csr[search_index]);
    } else {
        generate_csr_with_constraint(graph, edgeInfoDict, csr[search_index], conditions_bits[constraint_index]);
        generate_csr_with_constraint(tr_graph, tr_edgeInfoDict, tr_csr[search_index], conditions_bits[constraint_index]);
    }
    CleanGraphData();
    GLG();
    //printf("%d\n", routes[search_index].size());
    stable_sort(routes[search_index].begin(), routes[search_index].end(), path_cmp);
}

void generate_second_paths() {
    generate_csr(graph, edgeInfoDict, csr[search_index]);
    generate_csr(tr_graph, tr_edgeInfoDict, tr_csr[search_index]);
    GLG();
}

void generate_csr_with_constraint(Graph &graph, EdgeInfoDict &edgeInfoDict, CSR &csr, bitset &conditions_bits) {
    csr.rowIndex[0] = 0;
    for(int i = 0; i < MAX_VERTEX; i++) {
        if(graph[i].size()) {
            csr.rowIndex[i + 1] = csr.rowIndex[i] + graph[i].size();
            for(std::set<int>::const_iterator iter = graph[i].begin(); iter != graph[i].end(); ++iter) {
                if(conditions_bits[i] || conditions_bits[*iter]) {
                    PairQueue temp = edgeInfoDict[Edge(i, *iter)];
                    PairQueue change;
                    while(!temp.empty()) {
                        EdgeInfo edgeinfo = temp.top();
                        temp.pop();
                        edgeinfo.second += DELTA;
                        change.push(edgeinfo);
                    }
                    csr.edgeinfos.push_back(change);
                    csr.columns.push_back(*iter);
                } else {
                    csr.edgeinfos.push_back(edgeInfoDict[Edge(i, *iter)]);
                    csr.columns.push_back(*iter);
                }
            }
        } else {
            csr.rowIndex[i + 1] = csr.rowIndex[i];
        }
    }
}

void generate_csr(Graph &graph, EdgeInfoDict &edgeInfoDict, CSR &csr) {
    csr.rowIndex[0] = 0;
    for(int i = 0; i < MAX_VERTEX; i++) {
        if(graph[i].size()) {
            csr.rowIndex[i + 1] = csr.rowIndex[i] + graph[i].size();
            for(std::set<int>::const_iterator iter = graph[i].begin(); iter != graph[i].end(); ++iter) {
                csr.edgeinfos.push_back(edgeInfoDict[Edge(i, *iter)]);
                csr.columns.push_back(*iter);
            }
        } else {
            csr.rowIndex[i + 1] = csr.rowIndex[i];
        }
    }
}

void update_edgeinfo(std::vector<Path *> &paths) {
    std::set<int> visit;
    for(unsigned int i = 0; i < 1; i++) {
        Path *path = paths[i];
        unsigned int nodePath_size = path->nodePath.size();
        for(unsigned j = 0; j < nodePath_size; j++) {
            Edge edge, tr_edge;
            if(j == nodePath_size - 1) {
                edge = Edge(path->nodePath[j], path->endPoint);
                tr_edge = Edge(path->endPoint, path->nodePath[j]);
            } else {
                edge = Edge(path->nodePath[j], path->nodePath[j + 1]);
                tr_edge = Edge(path->nodePath[j + 1], path->nodePath[j]);
            }
            EdgeInfo edgeinfo = edgeInfoDict[edge].top();
            if(visit.find(edgeinfo.first) == visit.end()) {
                visit.insert(edgeinfo.first);
                edgeinfo.second += DELTA;
                edgeInfoDict[edge].pop();
                edgeInfoDict[edge].push(edgeinfo);
                tr_edgeInfoDict[tr_edge].pop();
                tr_edgeInfoDict[tr_edge].push(edgeinfo);
            }
        }
    }
}

int get_optimum_paths() {
    Path *work_path = NULL, *back_path = NULL;
    int min_conflict = INT_MAX;
    int min_cost = INT_MAX;
    for(unsigned int i = 0; i < routes[0].size(); i++) {
        for(unsigned int j = 0; j < routes[1].size(); j++) {
            int conflict = get_conflict(routes[0][i], routes[1][j]);
            int cost = routes[0][i]->pathCost + routes[1][j]->pathCost;
            if(conflict < min_conflict) {
                work_path = routes[0][i];
                back_path = routes[1][j];
                min_conflict = conflict;
                min_cost = cost;
            } else if(conflict == min_conflict && cost < min_cost) {
                work_path = routes[0][i];
                back_path = routes[1][j];
                min_cost = cost;
            }
        }
    }
    result[0] = work_path;
    result[1] = back_path;
    //printf("min_conflict = %d, min_cost = %d\n", min_conflict, min_cost);
    return min_conflict;
}

int get_conflict(Path *path1, Path *path2) {
    int sum = 0;
    std::bitset<40000> visit;
    for(unsigned int i = 0; i < path1->edgePath.size(); i++) {
        visit.set(path1->edgePath[i]);
    }
    for(unsigned int j = 0; j < path2->edgePath.size(); j++) {
        if(visit[path2->edgePath[j]]) {
            sum++;
        }
    }
    return sum;
}
//----------------------------------------------------------------------------------------------------------------------------------剪枝函数实现
void get_arrive_info(int link_from, DirectArrive &successors, DirectArrive &predecessors) {
    bitset visit;
    visit.set(link_from);
    visit.set(source);
    std::deque<int> paths_dequeue;
    paths_dequeue.push_back(link_from);

    while (!paths_dequeue.empty()) {
        int cur_point = paths_dequeue.front();
        paths_dequeue.pop_front();
        if (cur_point != link_from && (conditions_bits[search_index][cur_point] || cur_point == dest)) {
            successors[link_from].insert(cur_point);
            predecessors[cur_point].insert(link_from);
            continue;
        }
        for(int i = csr[search_index].rowIndex[cur_point]; i < csr[search_index].rowIndex[cur_point + 1]; i++) {
            int link_to = csr[search_index].columns[i];
            if (visit[link_to] || !available[link_to]) {
                continue;
            }
            visit.set(link_to);
            paths_dequeue.push_back(link_to);
        }
    }
}

void generateOrderConstraint(DirectArrive &direct_arrive, OrderConstraint &constraint) {
    constraint.rowIndex[0] = 0;
    for (int i = 0; i < MAX_VERTEX; i++) {
        if (direct_arrive[i].size() != 0 && direct_arrive[i].size() * 1.0 / conditions[search_index].size() < 0.2) {
            constraint.obstructNodes.push_back(i);
            constraint.rowIndex[i + 1] = constraint.rowIndex[i] + direct_arrive[i].size();
            for (std::set<int>::iterator it = direct_arrive[i].begin(); it != direct_arrive[i].end(); it++) {
                constraint.columns.push_back(*it);
            }
        } else {
            constraint.rowIndex[i + 1] = constraint.rowIndex[i];
        }
    }
}

bool isValid(Path *path, OrderConstraint &successors, OrderConstraint &predecessors) {
    if (path->needCheck == true) {
        return true;
    }
    unsigned int true_positive = 0;
    for (unsigned int i = 0; i < successors.obstructNodes.size(); i++) {
        int obNode = successors.obstructNodes[i];
        bool obNode_is_in_path = false;
        bool obNode_is_path_end = false;
        bool first_is_in_successor = false;
        bool successors_all_appear = true;
        if (path->bitPath[obNode] != 0 || path->endPoint == obNode) {
            obNode_is_in_path = true;
        }
        if (path->endPoint == obNode) {
            obNode_is_path_end = true;
        }

        for (int j = successors.rowIndex[obNode]; j < successors.rowIndex[obNode + 1]; j++) {
            if (path->bitPath[successors.columns[j]] == 0 && path->endPoint != successors.columns[j]) {
                successors_all_appear = false;
            }
            if (path->nodePath[0] == successors.columns[j]) {
                first_is_in_successor = true;
            }
        }
        if(obNode_is_in_path == true && obNode_is_path_end == false){
            true_positive++;
        }
        if (successors_all_appear == false) {
            continue;
        } else if (obNode_is_in_path == true) {
            if (obNode_is_path_end == true) {
                return false;
            } else {
                continue;
            }
        } else if (first_is_in_successor == true) {
            continue;
        } else {
            return false;
        }
    }

    for (unsigned int i = 0; i < predecessors.obstructNodes.size(); i++) {
        int obNode = predecessors.obstructNodes[i];
        bool obNode_appears_in_path = false;
        bool obNode_is_path_first = false;
        bool end_is_in_predecessor=false;
        bool predecessors_all_appear = true;
        if (path->bitPath[obNode] != 0 || path->endPoint == obNode) {
            obNode_appears_in_path = true;
        }
        if (path->nodePath[0] == obNode) {
            obNode_is_path_first = true;
        }
        for (int j = predecessors.rowIndex[obNode]; j < predecessors.rowIndex[obNode + 1]; j++) {
            if (path->bitPath[predecessors.columns[j]] == 0 && path->endPoint != predecessors.columns[j]) {
                predecessors_all_appear = false;
            }
            if(path->endPoint == predecessors.columns[j]){
                end_is_in_predecessor=true;
            }
        }
        if(obNode_appears_in_path == true && obNode_is_path_first == false){
            true_positive++;
        }
        if (predecessors_all_appear == false) {
            continue;
        }else if(obNode_appears_in_path == true){
            if(obNode_is_path_first == true){
                return false;
            }else{
                continue;
            }
        }else if(end_is_in_predecessor == true){
            continue;
        }else{
            return false;
        }
    }
    if(true_positive == successors.obstructNodes.size() + predecessors.obstructNodes.size()){
        path->needCheck = true;
    }
    return true;
}
//----------------------------------------------------------------------------------------------------------------------------------算法函数实现
void get_shortest_path(int link_from, DistanceTable &distance, OrderConstraint &successor, OrderConstraint &predecessor)
{
    Previous prev(MAX_VERTEX, std::pair<int, int>(-1, -1));
    std::vector<int> dis(MAX_VERTEX, INT_MAX);
	bitset without;
    without.set(source);
    without.set(dest);
    Dijkstra(link_from, prev, dis, without);

    for(Conditions::iterator iter = conditions[search_index].begin(); iter != conditions[search_index].end(); iter++) {
        if(link_from == *iter) {
            continue;
        } else {
            if(dis[*iter] != INT_MAX){
                Path* temp = search_path(link_from, *iter, prev, dis);
                if (isValid(temp, successor, predecessor)) {
                    distance[*iter].push_back(temp);
                }
                get_second_path(link_from, *iter, prev, dis, distance, successor, predecessor);
            }
        }
    }
}

void Dijkstra(int link_from, Previous &prev, std::vector<int> &dis, bitset &without_bits) {
	bitset processed;       // 已处理过的结点
    PairQueue candidates;   // 待处理的结点， 配合上Path的定义， 这便是一个小顶堆
    // 算法初始化， 起点加入processed集合， 起点的邻接点加入candidates集合
    processed.set(link_from);
    dis[link_from] = 0;
    prev[link_from].first = link_from;

    for(int i = tr_csr[search_index].rowIndex[link_from]; i < tr_csr[search_index].rowIndex[link_from + 1]; i++) {
        // 排除必须要排除的点
        int link_to = tr_csr[search_index].columns[i];
        if(without_bits[link_to]) {
            continue;
        }
        dis[link_to] = tr_csr[search_index].edgeinfos[i].top().second;
        prev[link_to].first = link_from;
        prev[link_to].second = tr_csr[search_index].edgeinfos[i].top().first;
        candidates.push(std::pair<int, int> (link_to, dis[link_to]));
    }
    // 算法主体开始
    // 第一步： 从候选区挑一个最佳结点， 加入processed集合中去
    // 第二步： 访问最佳结点的所有邻接点， 刷新或扩充候选人集合
    while(!candidates.empty()) {
        // 取出候选区最近的结点, 加入已处理集合中， 并将该结点当前的路径存储到最短路径字典中
        std::pair<int, int> best = candidates.top();
        candidates.pop();
        if(processed[best.first]) {
            continue;
        }
        processed.set(best.first);

        // 访问最佳候选人的所有邻接点， 以刷新或扩充候选结点
        // 如果最佳候选人没有邻接点， 直接开始下一轮循环
        if(tr_csr[search_index].rowIndex[best.first] == tr_csr[search_index].rowIndex[best.first + 1]) {
            continue;
        }
        for(int i = tr_csr[search_index].rowIndex[best.first]; i < tr_csr[search_index].rowIndex[best.first + 1]; i++) {
            int link_to = tr_csr[search_index].columns[i];
            if(processed[link_to] || without_bits[link_to]) {
                continue;
            }
            int edgeCost = tr_csr[search_index].edgeinfos[i].top().second;

            if(dis[link_to] > dis[best.first] + edgeCost) {
                dis[link_to] = dis[best.first] + edgeCost;
                prev[link_to].first = best.first;
                prev[link_to].second = tr_csr[search_index].edgeinfos[i].top().first;
                candidates.push(std::pair<int, int> (link_to, dis[link_to]));
            }
        }
    }
}

Path* search_path(int link_from, int link_to, Previous &prev, std::vector<int> &dis) {
    Path *path = new Path;
    path->endPoint = link_from;
    path->pathCost = dis[link_to];
    path->bitPath.set(link_to);
    path->nodePath.push_back(link_to);
    path->edgePath.push_back(prev[link_to].second);
    int temp = prev[link_to].first;
    while(temp != link_from) {
        path->bitPath.set(temp);
        path->nodePath.push_back(temp);
        path->edgePath.push_back(prev[temp].second);
        if(conditions_bits[search_index][temp]) {
            path->passed++;
        }
        temp = prev[temp].first;
    }
    return path;
}

void get_second_path(int link_from, int link_to, Previous &prev, std::vector<int> &dis, DistanceTable &distance,
                     OrderConstraint &successor, OrderConstraint &predecessor) {
    int minCost = 0xfffff, pre_vertex = -1, pre_edge = -1;
	int cannot_pass = prev[link_to].first;
    for(int i = csr[search_index].rowIndex[link_to]; i < csr[search_index].rowIndex[link_to + 1]; i++) {
        int adjoint = csr[search_index].columns[i];
        if(adjoint == cannot_pass || adjoint == link_from) {
            continue;
        }
        if(dis[adjoint] != INT_MAX) {
            int temp = adjoint;
            bool loopless = true;
            while(prev[temp].first != temp) {
                if(temp == link_to) {
                    loopless = false;
                }
                temp = prev[temp].first;
            }
            if(!loopless) {
                continue;
            }
            int cur_cost = dis[adjoint] + csr[search_index].edgeinfos[i].top().second;
            if(cur_cost < minCost) {
                minCost = cur_cost;
                pre_vertex = adjoint;
                pre_edge = csr[search_index].edgeinfos[i].top().first;
            }
        }
    }
    if(pre_vertex != -1) {
        Path* second_path = search_path(link_from, pre_vertex, prev, dis);
        second_path->pathCost = minCost;
        second_path->bitPath.set(link_to);
        second_path->nodePath.insert(second_path->nodePath.begin(), link_to);
        second_path->edgePath.insert(second_path->edgePath.begin(), pre_edge);
        if(conditions_bits[search_index][pre_vertex]) {
            second_path->passed++;
        }
        if (isValid(second_path, successor, predecessor)) {
            distance[link_to].push_back(second_path);
        }
    }
}

void get_BFS_path(int link_from, DistanceTable &distance, DistanceTable &ToDest, OrderConstraint &successor, OrderConstraint &predecessor) {
    Previous prev(MAX_VERTEX, std::pair<int, int>(-1, -1));
    std::vector<int> dis(MAX_VERTEX, INT_MAX);
    dis[link_from] = 0;
    prev[link_from].first = link_from;
    bitset visit;
    visit.set(link_from);
    visit.set(source);
    std::deque<int> paths_dequeue;
    paths_dequeue.push_back(link_from);

    while(!paths_dequeue.empty()) {
        int cur_point = paths_dequeue.front();
        paths_dequeue.pop_front();
        for(int i = csr[search_index].rowIndex[cur_point]; i < csr[search_index].rowIndex[cur_point + 1]; i++) {
            int link_to = csr[search_index].columns[i];
            if(visit[link_to] || !available[link_to]) {
                continue;
            }
            dis[link_to] = dis[cur_point] + csr[search_index].edgeinfos[i].top().second;
            prev[link_to].first = cur_point;
            prev[link_to].second = csr[search_index].edgeinfos[i].top().first;
            if(conditions_bits[search_index][link_to] || link_to == dest) {
                Path *temp = search_path(link_from, link_to, prev, dis);
                temp->reverse(link_to, link_from);
                PathQueue &paths = (link_to == dest) ? ToDest[link_from] : distance[link_from];
                unsigned int index = 0;
                while(index < paths.size() && !(*paths[index] == *temp)){index++;}
                if(index < paths.size()) {
                    dis[link_to] = -1;
                    prev[link_to] = std::pair<int, int> (-1, -1);
                    delete temp;
                    continue;
                }
                if(link_to == dest) {
                    if (isValid(temp, successor, predecessor)) {
                        ToDest[link_from].push_back(temp);
                    }
                } else {
                    if (isValid(temp, successor, predecessor)) {
                        distance[link_from].push_back(temp);
                    }
                }
            }
            visit.set(link_to);
            paths_dequeue.push_back(link_to);
        }
    }
}

void fill_dp(int iter_count, int link_from, DpTable &dp, DistanceTable &distance,
             DirectArrive &direct_arrive, OrderConstraint &successors, OrderConstraint &predecessors) {
    std::unordered_map<int, std::vector<Path *> > selected_paths;
    //将dp[iter_count - 1][link_from]中经过节个数大于等于iter_count的路径加入dp[iter_count][link_from]中
    PathQueue &upper_queue = dp[iter_count - 1][link_from];
    for(unsigned int i = 0; i < upper_queue.size(); i++) {
        Path *upper = upper_queue[i];
        if(upper->passed >= iter_count) {
            Path *temp = new Path();
            *temp = *upper;
            dp[iter_count][link_from].push_back(temp);
            selected_paths[temp->pathCost].push_back(temp);
        }
    }

    // 通过链接D(vi, vj)中的K条路和fn-1(vj)中的路生成备选路
    PathQueue &shortest_paths = distance[link_from];
    for(unsigned int shortest_index = 0; shortest_index < shortest_paths.size() && dp[iter_count][link_from].size() < MAX_TABLE / conditions[search_index].size(); shortest_index++) {
        Path* shortest_path = shortest_paths[shortest_index];
        if (direct_arrive[link_from].find(shortest_path->endPoint) == direct_arrive[link_from].end()) {
            continue;
        }
        int shortest_path_first = shortest_path->nodePath[0];
        PathQueue &candidate_paths = dp[iter_count - 1][shortest_path->endPoint];
        unsigned int has_connect = 0;
        for(unsigned int candidate_index = 0; candidate_index < candidate_paths.size() && has_connect < F_K_PATH; candidate_index++) {
            Path *candidate_path = candidate_paths[candidate_index];
            if(shortest_path->bitPath.isLoopless(shortest_path_first, candidate_path->bitPath)) {
                Path *temp = connect_path(shortest_path, candidate_path);
                if(isDuplicate(temp, selected_paths[temp->pathCost])) {
                    delete temp;
                    continue;
                }
                // 剪枝
                if (!isValid(temp, successors, predecessors)) {
                    delete temp;
                    continue;
                }
                dp[iter_count][link_from].push_back(temp);
                selected_paths[temp->pathCost].push_back(temp);
                has_connect++;
            }
        }
    }
}

void get_result(DpTable &dp) {
    if(conditions[search_index].size()) {
        for(Conditions::iterator iter = conditions[search_index].begin(); iter != conditions[search_index].end(); iter++) {
            PathQueue & shortest_paths = dp[conditions[search_index].size() - 1][*iter];
            for(unsigned int shortest_index = 0; shortest_index < shortest_paths.size(); shortest_index++) {
                Path* shortest_path = shortest_paths[shortest_index];
                bitset without = shortest_path->bitPath;
                without.set(dest);

                Previous prev(MAX_VERTEX, std::pair<int, int>(-1, -1));
                std::vector<int> dis(MAX_VERTEX, INT_MAX);
                Dijkstra(*iter, prev, dis, without);
                if(dis[source] != INT_MAX) {
                    Path *left_path = search_path(*iter, source, prev, dis);
                    Path *temp = connect_path(left_path, shortest_path);
                    routes[search_index].push_back(temp);
                }
            }
        }
    } else {
        bitset without;
        Previous prev(MAX_VERTEX, std::pair<int, int>(-1, -1));
        std::vector<int> dis(MAX_VERTEX, INT_MAX);
        Dijkstra(dest, prev, dis, without);
        if(dis[source] != INT_MAX) {
            Path *shortest_path = search_path(dest, source, prev, dis);
            routes[search_index].push_back(shortest_path);
        }
    }
}

Path* connect_path(Path *left_path, Path *right_path) {
    Path *temp = new Path;
    temp->endPoint = right_path->endPoint;
    temp->passed = left_path->passed + right_path->passed + 1;
    temp->pathCost = left_path->pathCost + right_path->pathCost;
    temp->bitPath = left_path->bitPath | right_path->bitPath;
    temp->nodePath.insert(temp->nodePath.end(), left_path->nodePath.begin(), left_path->nodePath.end());
    temp->nodePath.insert(temp->nodePath.end(), right_path->nodePath.begin(), right_path->nodePath.end());
    temp->edgePath.insert(temp->edgePath.end(), left_path->edgePath.begin(), left_path->edgePath.end());
    temp->edgePath.insert(temp->edgePath.end(), right_path->edgePath.begin(), right_path->edgePath.end());
    if(left_path->needCheck==true || right_path->needCheck==true){
        temp->needCheck=true;
    }
    return temp;
}

bool isDuplicate(Path *path, std::vector<Path*> &paths) {
    unsigned int paths_size = paths.size();
    for(unsigned int path_index = 0; path_index < paths_size; path_index++) {
        if(path->pathCost != paths[path_index]->pathCost) {
            continue;
        }
    	if(*path == *paths[path_index]) {
    		return true;
    	}
    }
    return false;
}

/*bool isLoopless(Path* path_1, Path* path_2) {
    if((path_1->bitPath & path_2->bitPath) == zero_bitset)
    	return true;
    return false;
}*/

void clear_memory(DpTable &dp, DistanceTable &distance) {
    for(Conditions::iterator iter_src = conditions[search_index].begin(); iter_src != conditions[search_index].end(); iter_src++) {
        PathQueue &path_queue = dp[conditions[search_index].size() - 1][*iter_src];
        for(unsigned int i = 0; i < path_queue.size(); i++) {
            delete path_queue[i];
        }
    }
    for(Conditions::iterator iter_src = conditions[search_index].begin(); iter_src != conditions[search_index].end(); iter_src++) {
        PathQueue & path_queue = distance[*iter_src];
        for(unsigned int i = 0; i < path_queue.size(); i++) {
            delete path_queue[i];
        }
    }
}

void GLG() {
    DpTable dp;                                      // 动态规划表   <迭代次数，<起始点，路径的优先队列（权重从小到大）>>
    DistanceTable distance, ToDest;                  // s到V‘中所有点，V‘中所有点间，V‘中所有点到t的K条路(权重从小到大)
    DirectArrive direct_successors(MAX_VERTEX);      // 后继信息
    DirectArrive direct_predecessors(MAX_VERTEX);    // 前驱信息
    OrderConstraint successors;
    OrderConstraint predecessors;

    //必经节点为NA
    if(!conditions[search_index].size()) {
        get_result(dp);
        return;
    }

    // 根据必经节点个数设置动态规划表中总路径个数以及F_K_PATH
    int max_table_ratio = ((100 - conditions[search_index].size()) / 11);
    MAX_TABLE = 8000 + max_table_ratio * 1000;
    MAX_TABLE = MAX_TABLE <= 15000 ? MAX_TABLE : 15000;
    int F_K_PATH_ratio = ((100 - conditions[search_index].size()) / 20);
    F_K_PATH = 9 + F_K_PATH_ratio;

    // 求必经节点中的阻塞点
    get_arrive_info(source, direct_successors, direct_predecessors);
    for(Conditions::iterator iter = conditions[search_index].begin(); iter != conditions[search_index].end(); iter++) {
        get_arrive_info(*iter, direct_successors, direct_predecessors);
    }
    generateOrderConstraint(direct_successors, successors);
    generateOrderConstraint(direct_predecessors, predecessors);

    // 求每个点间的三条路径
    get_shortest_path(dest, ToDest, successors, predecessors);
    for(Conditions::iterator iter = conditions[search_index].begin(); iter != conditions[search_index].end(); iter++) {
        get_shortest_path(*iter, distance, successors, predecessors);
    }
    for(Conditions::iterator iter = conditions[search_index].begin(); iter != conditions[search_index].end(); iter++) {
        get_BFS_path(*iter, distance, ToDest, successors, predecessors);
        stable_sort(distance[*iter].begin(), distance[*iter].end(), path_cmp);
    }

    // 填动态规划表
    for(unsigned int iter_count = 0; iter_count < conditions[search_index].size(); iter_count++) {
        for(Conditions::iterator iter_src = conditions[search_index].begin(); iter_src != conditions[search_index].end(); iter_src++) {
            if(!iter_count && direct_successors[*iter_src].find(dest) != direct_successors[*iter_src].end()) {
                dp[0][*iter_src] = ToDest[*iter_src];
                stable_sort(dp[0][*iter_src].begin(), dp[0][*iter_src].end(), path_cmp);
            }
            if(iter_count > 0) {
                fill_dp(iter_count, *iter_src, dp, distance, direct_successors, successors, predecessors);
                stable_sort(dp[iter_count][*iter_src].begin(), dp[iter_count][*iter_src].end(), path_cmp);
            }
        }
        // 清空上一个动态规划表
        if(iter_count > 0) {
            for(Conditions::iterator iter_src = conditions[search_index].begin(); iter_src != conditions[search_index].end(); iter_src++) {
                PathQueue &path_queue = dp[iter_count - 1][*iter_src];
                for(unsigned int i = 0; i < path_queue.size(); i++) {
                    delete path_queue[i];
                }
            }
            dp.erase(iter_count - 1);
        }
    }

    // 求取满足约束条件的路径
    get_result(dp);
    // 释放DP表占用的内存空间
    clear_memory(dp, distance);
}
