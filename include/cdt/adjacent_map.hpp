#ifndef _ADJACENT_MAP_H_
#define _ADJACENT_MAP_H_

#include <iostream>
#include <vector>
#include <list>
#include <map>
#include "cdt/faststack.hpp" // take place of std::stack.

using namespace std;
typedef unsigned char byte;
class AdjacentMatrix {
public:
    AdjacentMatrix(int n_pts);
    ~AdjacentMatrix();
    void connectTwoNodes(int i, int j);
    void connectThreeNodes(int i, int j, int k);
    void disconnectTwoNodes(int i, int j);
    void disconnectThreeNodes(int i, int j, int k);
    void setFalseAll(); // all false;
    void setTrueAll();
    void showAllMatrix();

    bool isEdgeExist(int i, int j);

    void resizeAdjacentMatrix(int n_pts_updated);

private:
    int n_nodes_;
    byte** adj_mat_; // {n_nodes_ X n_nodes_} matrix.
};

#endif