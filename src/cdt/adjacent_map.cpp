#include "cdt/adjacent_map.hpp"

AdjacentMatrix::AdjacentMatrix(int n_pts) {
    n_nodes_ = n_pts;
    cout << " adjacent matrix ... n_node : " << n_nodes_ << "\n";

    adj_mat_ = new byte*[n_pts];
    for (int i = 0; i < n_pts; i++) adj_mat_[i] = new byte[n_pts];
    this->setFalseAll(); // default = false;
};

AdjacentMatrix::~AdjacentMatrix() {
    for (int i = 0; i < n_nodes_; i++) delete adj_mat_[i];
};

void AdjacentMatrix::connectTwoNodes(int i, int j) {
    adj_mat_[i][j] = adj_mat_[j][i] = 1;
};

void AdjacentMatrix::connectThreeNodes(int i, int j, int k) {
    adj_mat_[i][j] = adj_mat_[j][i] = 1;
    adj_mat_[i][k] = adj_mat_[k][i] = 1;
    adj_mat_[k][j] = adj_mat_[j][k] = 1;
};

void AdjacentMatrix::disconnectTwoNodes(int i, int j) {
    adj_mat_[i][j] = adj_mat_[j][i] = 0;
};

void AdjacentMatrix::disconnectThreeNodes(int i, int j, int k) {
    adj_mat_[i][j] = adj_mat_[j][i] = 0;
    adj_mat_[i][k] = adj_mat_[k][i] = 0;
    adj_mat_[k][j] = adj_mat_[j][k] = 0;
};

void AdjacentMatrix::setFalseAll() {
    for (int i = 0; i < n_nodes_; i++)
        for (int j = 0; j < n_nodes_; j++)
            adj_mat_[i][j] = 0;
};
void AdjacentMatrix::setTrueAll() {
    for (int i = 0; i < n_nodes_; i++)
        for (int j = 0; j < n_nodes_; j++)
            adj_mat_[i][j] = 1;
};
void AdjacentMatrix::showAllMatrix() {
    cout << " Connectivity matrix: \n\n";
    for (int i = 0; i < n_nodes_; i++) {
        for (int j = 0; j < n_nodes_; j++)
            cout << (int)adj_mat_[i][j] << " ";
        cout << "\n";
    }
};
bool AdjacentMatrix::isEdgeExist(int i, int j) {
    return adj_mat_[i][j];
};

void AdjacentMatrix::resizeAdjacentMatrix(int n_pts_updated) {
    // delete existing adj_mat.
    for (int i = 0; i < n_nodes_; i++) delete adj_mat_[i];

    // resize Adjacent
    n_nodes_ = n_pts_updated;
    cout << " n_node (updated): " << n_pts_updated << "\n";

    adj_mat_ = new byte*[n_nodes_];
    for (int i = 0; i < n_nodes_; i++) adj_mat_[i] = new byte[n_nodes_];
    this->setFalseAll(); // default = false;
};