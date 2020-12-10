#ifndef _CONSTRAINED_DELAUNAY_H_
#define _CONSTRAINED_DELAUNAY_H_

//#define _VERBOSE_

#include <iostream>
#include <vector>
#include <list>
#include <map>
#include <Eigen/Dense>

#include "cdt/faststack.hpp" // take place of std::stack.
#include "cdt/adjacent_map.hpp"
#include "cdt/geometry_struct.hpp" // Vertex, Side, Triangle
#include "point_database.hpp"


using namespace std;
/**
*
* @brief Constrained Delaunay triangulation 
* @details Constrained Delaunay triangulation 
* @author Changhyeon Kim
* @date 2020-10-06
*
*/
class ConstrainedDT {
public:
    /* Public methods */
    ConstrainedDT();
    void initializeDT(const vector<PointDB>& points_input);
    ConstrainedDT(const vector<Vertex>& points_input, const vector<Side>& constraints_input);
    ConstrainedDT(const vector<Eigen::Vector2f>& points_input); // no inputs.
    ConstrainedDT(const vector<PointDB>& points_input); // no inputs.
    ~ConstrainedDT();
    void executeNormalDT();

    // Densification algorithms
    void getCenterPointsOfTriangles(const float& thres_area, vector<Vertex>& points_centers);
    void getCenterPointsOfTriangles(const float& thres_area, vector<Eigen::Vector2f>& points_centers);
    void getCenterPointsOfTriangles(const float& thres_area, vector<PointDB>& points_centers);

    void addPointsIntoDT(const vector<Vertex>& points_addi);
    void addPointsIntoDT(const vector<Eigen::Vector2f>& points_addi);
    void addPointsIntoDT(const vector<PointDB>& points_addi);

    // Constrained DT.
    void executeRefineConstrainedDT();
    void renewConstraints(const vector<Side>& constraint_edges_addi); // After 'addConstraints()', do 'refineConstrainedDT()'.

    void showAllTriangles() {
        cout << " SHOW ALL Triangles! (except for a superTriangle) \n\n";
        cout << " # of triangles : " << tri_map_.size() << "\n";
        for (auto it = tri_map_.begin(); it != tri_map_.end(); it++)
            cout << it->second->idx[0] << "," << it->second->idx[1] << "," << it->second->idx[2] << "\n";
        cout << "\n\n";
    };

    const map<int,Triangle*>& getTriangleMap() const {return tri_map_;};


private:
    /* Private methods */
    void getAndNormalizePoints(const vector<Vertex>& points_input);
    void getAndNormalizePoints(const vector<Eigen::Vector2f>& points_input);
    void getAndNormalizePoints(const vector<PointDB>& points_input);

    bool isInCircum(Triangle* tri, int i, const vector<Vertex>& pts);
    void findLawsonSearch(const Triangle* tri, int& next_dir, int i, const vector<Vertex>& pts);

    bool isInTri(Triangle* tri, int i, const vector<Vertex>& pts);

    bool isCCW(Triangle* tri, const vector<Vertex>& pts);
    void sortVertexOrderCCW(Triangle* tri);

    float calcTriArea(const Vertex& p0, const Vertex& p1, const Vertex& p2);

    void incorporateTriangle(Triangle* tri);
    void disincorporateTriangle(Triangle* tri); // always, create after delete.


    void findSharingTwoTriangles(const int& k, const int& l, Triangle** tris_sharing);

    // for constrained Delaunay Triangle
    bool isStrictlyConvex(Triangle* tri_a, Triangle* tri_b); // two Triangles must be adjacent!!



private:

    bool flag_verbose_; 

    /* Private data */
    // Related to Points
    int n_pts_; // total # of points
    vector<Vertex> points_; // all Vertex points (non-duplicated)

    // Related to coordinate normalization
    float denom_; // denominator for cooridnate normalization
    float x_min_, x_max_; // min & max. values of x-coordinates
    float y_min_, y_max_;
    float y_max_hat_, x_max_hat_; // normalized maximum values

    // Related to a binning process
    int n_slots_; // ceil(sqrt(sqrt(n_pts_));
    int n_bins_;  // n_slots_*n_slots_
    vector<vector<int>> bins_; // n_slots_*n_slots_

    // Related to Triangles
    int n_Triangles_; // total # of Triangles
    int n_tri_counter_; // accumulated # of Triangles.
    Triangle* tri_latest_;   // latest Triangle.

    int id_super_Vertex[3]; // IDs of vertices of a super Triangle.


    // Related to additional densification

    // Related to algorithm executions.
    PointerStack<Triangle> fstack_; // for a fast stack calculation.
    std::map<int, Triangle*> tri_map_; // Map of valid Triangles.
    
    // adjacent map (point connectivity map)
    AdjacentMatrix* adjmat_;

    // Related to constrained Delaunay triangulation.
    int n_constraints_;
    int n_pts_org_;
    vector<Side> constraint_edges_;
    list<Side> intersecting_edges_;
    list<Side> new_edges_;

    // Related to densification
    float thres_area_; // Only for Triangles with smaller area than this value.
};


#endif