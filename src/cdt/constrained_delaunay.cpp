#include "cdt/constrained_delaunay.hpp"
/**
* @brief Constructor
* @details Constrained DT constructor
* @return none.
*/
ConstrainedDT::ConstrainedDT(const vector<Vertex>& points_input, const vector<Side>& constraints_input) {
    flag_verbose_ = false;

    cout << "  Constrained Delaunay triangulation is initialized.\n";

    // insert points (with normalization)
    n_pts_ = points_input.size();
    this->getAndNormalizePoints(points_input);

    // Make bins
    n_slots_ = ceil(sqrtf(sqrtf(n_pts_)));
    n_bins_ = n_slots_*n_slots_;
    bins_.resize(n_bins_);

    // binning all points
    for (int n = 0; n < n_pts_; n++) {
        int i = (int)(0.99f*(float)n_slots_*(points_[n].y / y_max_hat_));
        int j = (int)(0.99f*(float)n_slots_*(points_[n].x / x_max_hat_));
        int n_bin = 0;
        if (i % 2 == 0) { // even number
            n_bin = i*n_slots_ + j + 1;
        }
        else { // odd number
            n_bin = (i + 1)*n_slots_ - j;
        }
        bins_[n_bin - 1].push_back(n);
    }

    // get constraints
    n_constraints_ = constraints_input.size();
    for (int n = 0; n < n_constraints_; n++) {
        // ascending sorted
        int i = constraints_input[n].a;
        int j = constraints_input[n].b;
        if (i > j) {
            int tmp = i;
            i = j;
            j = tmp;
        }
        constraint_edges_.emplace_back(i, j);
    }

    // vertices of 'super Triangle'
    this->points_.emplace_back(10000, -10000); // index: N + 1
    this->points_.emplace_back(0, 10000); // index: N + 2
    this->points_.emplace_back(-10000, -10000); // index: N (left-most)
    id_super_Vertex[0] = n_pts_;
    id_super_Vertex[1] = n_pts_ + 1;
    id_super_Vertex[2] = n_pts_ + 2; // store the ids of vertices of super Triangle

    // initialize Triangles
    // Adjacent map (connectivity matrix after DT. n_pts x n_pts)
    adjmat_ = new AdjacentMatrix(n_pts_ + 3);

    n_tri_counter_ = 0; // accumulated # of triangle. (not related to existing # of triangles.)
    n_Triangles_   = 0; // # of currently existing triangles.
    Triangle* tri_super = new Triangle(n_pts_, n_pts_ + 1, n_pts_ + 2, n_tri_counter_++);
    this->incorporateTriangle(tri_super); // connect 'adjmap' and insert the triangle to 'trimap'.

    // Latest Triangle
    this->tri_latest_ = tri_super;

#ifdef _VERBOSE_
        cout << "   L # of input points : " << n_pts_ << "\n";
        cout << "   L # of slots, bins  : " << n_slots_ << ", " << n_bins_ << "\n";
        cout << "   L # of constraints  : " << n_constraints_ << "\n";
        cout << "   L # of Triangles (initialization step): " << n_Triangles_ << "\n\n";
#endif
};

/**
* @brief Constructor (without constraints)
* @details Constrained DT constructor (overloaded)
* @return none.
*/
ConstrainedDT::ConstrainedDT(const vector<Eigen::Vector2f>& points_input) {
    flag_verbose_ = false;

    cout << "  Constrained Delaunay triangulation is initialized.\n";

    // insert points (with normalization)
    n_pts_ = points_input.size();
    this->getAndNormalizePoints(points_input);

    // Make bins
    n_slots_ = ceil(sqrtf(sqrtf(n_pts_)));
    n_bins_ = n_slots_*n_slots_;
    bins_.resize(n_bins_);

    // binning all points
    for (int n = 0; n < n_pts_; n++) {
        int i = (int)(0.99f*(float)n_slots_*(points_[n].y / y_max_hat_));
        int j = (int)(0.99f*(float)n_slots_*(points_[n].x / x_max_hat_));
        int n_bin = 0;
        if (i % 2 == 0) { // even number
            n_bin = i*n_slots_ + j + 1;
        }
        else { // odd number
            n_bin = (i + 1)*n_slots_ - j;
        }
        bins_[n_bin - 1].push_back(n);
    }

    // vertices of 'super Triangle'
    this->points_.emplace_back(10000, -10000); // index: N + 1
    this->points_.emplace_back(0, 10000); // index: N + 2
    this->points_.emplace_back(-10000, -10000); // index: N (left-most)
    id_super_Vertex[0] = n_pts_;
    id_super_Vertex[1] = n_pts_ + 1;
    id_super_Vertex[2] = n_pts_ + 2; // store the ids of vertices of super Triangle

    // initialize Triangles
    // Adjacent map (connectivity matrix after DT. n_pts x n_pts)
    adjmat_ = new AdjacentMatrix(n_pts_ + 3);

    n_tri_counter_ = 0; // accumulated # of triangle. (not related to existing # of triangles.)
    n_Triangles_   = 0; // # of currently existing triangles.
    Triangle* tri_super = new Triangle(n_pts_, n_pts_ + 1, n_pts_ + 2, n_tri_counter_++);
    this->incorporateTriangle(tri_super); // connect 'adjmap' and insert the triangle to 'trimap'.

    // Latest Triangle
    this->tri_latest_ = tri_super;

#ifdef _VERBOSE_
        cout << "   L # of input points : " << n_pts_ << "\n";
        cout << "   L # of slots, bins  : " << n_slots_ << ", " << n_bins_ << "\n";
        cout << "   L # of constraints  : " << n_constraints_ << "\n";
        cout << "   L # of Triangles (initialization step): " << n_Triangles_ << "\n\n";
#endif
};


/**
* @brief Constructor (without constraints)
* @details Constrained DT constructor (overloaded 2)
* @return none.
*/
ConstrainedDT::ConstrainedDT(const vector<PointDB>& points_input) {
    flag_verbose_ = false;

    cout << "  Constrained Delaunay triangulation is initialized.\n";

    // insert points (with normalization)
    n_pts_ = points_input.size();
    this->getAndNormalizePoints(points_input);

    // Make bins
    n_slots_ = ceil(sqrtf(sqrtf(n_pts_)));
    n_bins_ = n_slots_*n_slots_;
    bins_.resize(n_bins_);

    // binning all points
    for (int n = 0; n < n_pts_; n++) {
        int i = (int)(0.99f*(float)n_slots_*(points_[n].y / y_max_hat_));
        int j = (int)(0.99f*(float)n_slots_*(points_[n].x / x_max_hat_));
        int n_bin = 0;
        if (i % 2 == 0) { // even number
            n_bin = i*n_slots_ + j + 1;
        }
        else { // odd number
            n_bin = (i + 1)*n_slots_ - j;
        }
        bins_[n_bin - 1].push_back(n);
    }

    // vertices of 'super Triangle'
    this->points_.emplace_back(10000, -10000); // index: N + 1
    this->points_.emplace_back(0, 10000); // index: N + 2
    this->points_.emplace_back(-10000, -10000); // index: N (left-most)
    id_super_Vertex[0] = n_pts_;
    id_super_Vertex[1] = n_pts_ + 1;
    id_super_Vertex[2] = n_pts_ + 2; // store the ids of vertices of super Triangle

    // initialize Triangles
    // Adjacent map (connectivity matrix after DT. n_pts x n_pts)
    adjmat_ = new AdjacentMatrix(n_pts_ + 3);

    n_tri_counter_ = 0; // accumulated # of triangle. (not related to existing # of triangles.)
    n_Triangles_   = 0; // # of currently existing triangles.
    Triangle* tri_super = new Triangle(n_pts_, n_pts_ + 1, n_pts_ + 2, n_tri_counter_++);
    this->incorporateTriangle(tri_super); // connect 'adjmap' and insert the triangle to 'trimap'.

    // Latest Triangle
    this->tri_latest_ = tri_super;

#ifdef _VERBOSE_
        cout << "   L # of input points : " << n_pts_ << "\n";
        cout << "   L # of slots, bins  : " << n_slots_ << ", " << n_bins_ << "\n";
        cout << "   L # of constraints  : " << n_constraints_ << "\n";
        cout << "   L # of Triangles (initialization step): " << n_Triangles_ << "\n\n";
#endif
};



void ConstrainedDT::sortVertexOrderCCW(Triangle* tri) {
    Triangle* tri1_temp = tri->adj[1]; // regardless of nullptr.
    tri->adj[1] = tri->adj[2];
    tri->adj[2] = tri1_temp;

    int idx_temp = tri->idx[1];
    tri->idx[1] = tri->idx[2];
    tri->idx[2] = idx_temp;

#ifdef _VERBOSE_
    cout << "    ------------Change order of new Triangle [" << tri->id << "]\n";
#endif
};


/**
* @brief insertPoint.
* @details insertPoint
* @return none.
*/
void ConstrainedDT::getAndNormalizePoints(const vector<Vertex>& points_input) {
    this->points_.reserve(n_pts_); // pre-allocation
    x_max_ = -1e9; x_min_ = 1e9;
    y_max_ = -1e9; y_min_ = 1e9;

    for (int i = 0; i < this->n_pts_; i++) {
        this->points_.emplace_back(points_input[i]);
        if (x_max_ < points_input[i].x) x_max_ = points_input[i].x;
        if (x_min_ > points_input[i].x) x_min_ = points_input[i].x;
        if (y_max_ < points_input[i].y) y_max_ = points_input[i].y;
        if (y_min_ > points_input[i].y) y_min_ = points_input[i].y;
    }

    // normalization of points' coordinates
    denom_ = std::fmaxf(x_max_ - x_min_, y_max_ - y_min_);
    float invdenom = 1.0f / denom_;
    x_max_hat_ = -1;
    y_max_hat_ = -1;
    for (int i = 0; i < this->n_pts_; i++) {
        this->points_[i].x = (this->points_[i].x - x_min_)*invdenom;
        this->points_[i].y = (this->points_[i].y - y_min_)*invdenom;
        if (x_max_hat_ < this->points_[i].x) x_max_hat_ = this->points_[i].x;
        if (y_max_hat_ < this->points_[i].y) y_max_hat_ = this->points_[i].y;
    }

};

/**
* @brief insertPoint. (overloaded)
* @details insertPoint (overloaded)
* @return none.
*/
void ConstrainedDT::getAndNormalizePoints(const vector<Eigen::Vector2f>& points_input) {
    this->points_.reserve(n_pts_); // pre-allocation
    x_max_ = -1e9; x_min_ = 1e9;
    y_max_ = -1e9; y_min_ = 1e9;

    for (int i = 0; i < this->n_pts_; i++) {
        this->points_.emplace_back(points_input[i](0),points_input[i](1));
        if (x_max_ < points_input[i](0)) x_max_ = points_input[i](0);
        if (x_min_ > points_input[i](0)) x_min_ = points_input[i](0);
        if (y_max_ < points_input[i](1)) y_max_ = points_input[i](1);
        if (y_min_ > points_input[i](1)) y_min_ = points_input[i](1);
    }

    // normalization of points' coordinates
    denom_ = std::fmaxf(x_max_ - x_min_, y_max_ - y_min_);
    float invdenom = 1.0f / denom_;
    x_max_hat_ = -1;
    y_max_hat_ = -1;
    for (int i = 0; i < this->n_pts_; i++) {
        this->points_[i].x = (this->points_[i].x - x_min_)*invdenom;
        this->points_[i].y = (this->points_[i].y - y_min_)*invdenom;
        if (x_max_hat_ < this->points_[i].x) x_max_hat_ = this->points_[i].x;
        if (y_max_hat_ < this->points_[i].y) y_max_hat_ = this->points_[i].y;
    }

};


/**
* @brief insertPoint. (overloaded 2)
* @details insertPoint (overloaded 2)
* @return none.
*/
void ConstrainedDT::getAndNormalizePoints(const vector<PointDB>& points_input) {
    this->points_.reserve(n_pts_); // pre-allocation
    x_max_ = -1e9; x_min_ = 1e9;
    y_max_ = -1e9; y_min_ = 1e9;

    for (int i = 0; i < this->n_pts_; i++) {
        this->points_.emplace_back(points_input[i].pts_(0),points_input[i].pts_(1));
        if (x_max_ < points_[i].x) x_max_ = points_[i].x;
        if (x_min_ > points_[i].x) x_min_ = points_[i].x;
        if (y_max_ < points_[i].y) y_max_ = points_[i].y;
        if (y_min_ > points_[i].y) y_min_ = points_[i].y;
    }

    // normalization of points' coordinates
    denom_ = std::fmaxf(x_max_ - x_min_, y_max_ - y_min_);
    float invdenom = 1.0f / denom_;
    x_max_hat_ = -1;
    y_max_hat_ = -1;
    for (int i = 0; i < this->n_pts_; i++) {
        this->points_[i].x = (this->points_[i].x - x_min_)*invdenom;
        this->points_[i].y = (this->points_[i].y - y_min_)*invdenom;
        if (x_max_hat_ < this->points_[i].x) x_max_hat_ = this->points_[i].x;
        if (y_max_hat_ < this->points_[i].y) y_max_hat_ = this->points_[i].y;
    }

};

/**
* @brief Destructor
* @details Constrained DT Destructor
* @return none.
*/
ConstrainedDT::~ConstrainedDT() {

    // adjmat_->showAllMatrix();

    // delete all Triangles
    for (auto it = tri_map_.begin(); it != tri_map_.end(); it++)
        delete it->second;

    delete adjmat_;
    cout << "  Constrained Delaunay triangulation is deleted.\n";
};


/**
* @brief A function to test whether the current Triangle encloses point 'p'
* @details A function to test whether the current Triangle encloses point 'p'
* @return true: this circle encloses 'p', false: this circle does not enclose the 'p'.
*/
bool ConstrainedDT::isInCircum(Triangle* tri, int i, const vector<Vertex>& pts) {
    float x1 = pts[tri->idx[0]].x, y1 = pts[tri->idx[0]].y;
    float x2 = pts[tri->idx[1]].x, y2 = pts[tri->idx[1]].y;
    float x3 = pts[tri->idx[2]].x, y3 = pts[tri->idx[2]].y;

    float xp = pts[i].x, yp = pts[i].y;
    float x13 = x1 - x3, y13 = y1 - y3, x23 = x2 - x3, y23 = y2 - y3;
    float x1p = x1 - xp, y1p = y1 - yp, x2p = x2 - xp, y2p = y2 - yp;

    float det = x1*(y2 - y3) - y1*(x2 - x3) + x2*y3 - x3*y2;

    if (det > 0)
        return ((x13*x23 + y13*y23)*(x2p*y1p - x1p*y2p) < (x23*y13 - x13*y23)*(x2p*x1p + y1p*y2p));
    else {
#ifdef _VERBOSE_
        cout << "                     NOT ALIGNED Triangle!!!\n";
#endif
        return ((x13*x23 + y13*y23)*(x2p*y1p - x1p*y2p) > (x23*y13 - x13*y23)*(x2p*x1p + y1p*y2p));
    }
};


bool ConstrainedDT::isCCW(Triangle* tri, const vector<Vertex>& points) {
    const Vertex& p1 = points[tri->idx[0]];
    const Vertex& p2 = points[tri->idx[1]];
    const Vertex& p3 = points[tri->idx[2]];

    float x1 = p1.x, y1 = p1.y;
    float x2 = p2.x, y2 = p2.y;
    float x3 = p3.x, y3 = p3.y;

    float det = x1*(y2 - y3) - y1*(x2 - x3) + x2*y3 - x3*y2;

    return det > 0; // det > 0: CCW, det < 0: CW
                    // If CW, change the order 1 2 -> 2 1 .
};


/**
* @brief findLawsonSearch
* @details findLawsonSearch
* @return none.
*/

float ccw(const Vertex& a, const Vertex& b) {
    return a.x*b.y - a.y*b.x;
};
float ccw(const Vertex& p, const Vertex& a, const Vertex& b) {
    return ccw(a - p, b - p);
}
bool segmentIntersects(const Vertex& a, const Vertex& b, const Vertex& c, const Vertex& d) {
    float ab = ccw(a, b, c)*ccw(a, b, d);
    float cd = ccw(c, d, a)*ccw(c, d, b);

    return ab <= 0 && cd <= 0;
}

bool isSegmentsIntersect(const Vertex& a0, const Vertex& a1, const Vertex& b0, const Vertex& b1) {
    Vertex a1a0 = a1 - a0;
    Vertex b1b0 = b1 - b0;
    Vertex b0a0 = b0 - a0;
    Vertex b1a0 = b1 - a0;
    Vertex a0b0 = a0 - b0;
    Vertex a1b0 = a1 - b0;

#define CROSS2D(a,b) (a.x*b.y-a.y*b.x)
    bool test0 = CROSS2D(a1a0, b0a0)*CROSS2D(a1a0, b1a0) < 0;
    bool test1 = CROSS2D(b1b0, a0b0)*CROSS2D(b1b0, a1b0) < 0;
    return (test0 && test1); // true: crossing each other, false: not crossing.
};

void ConstrainedDT::findLawsonSearch(const Triangle* tri, int& next_dir, int i, const vector<Vertex>& pts) {
    const Vertex& p = pts[i];
    Vertex p0 = pts[tri->idx[0]];
    Vertex p1 = pts[tri->idx[1]];
    Vertex p2 = pts[tri->idx[2]];

    // center of this Triangle
    Vertex pc((p0.x + p1.x + p2.x) * 0.33333333, (p0.y + p1.y + p2.y) * 0.33333333);

    if      (segmentIntersects(pc, p, p0, p2)) next_dir = 1;
    else if (segmentIntersects(pc, p, p1, p2)) next_dir = 0;
    else if (segmentIntersects(pc, p, p0, p1)) next_dir = 2;
    else throw std::runtime_error("Exceptional case in 'findLawsonSearch'\n");

    if (tri->adj[next_dir] != nullptr) return;
    else throw std::runtime_error("Exceptional case in 'findLawsonSearch': Not defined direction.\n");
};

/**
* @brief calcTriArea
* @details calcTriArea
* @return Triangle area (float, absolute value).
*/
float ConstrainedDT::calcTriArea(const Vertex& p0, const Vertex& p1, const Vertex& p2) {
    return fabs(0.5f * (p0.x*(p1.y - p2.y) + p1.x*(p2.y - p0.y) + p2.x*(p0.y - p1.y)));
};


/**
* @brief isInTri
* @details isInTri
* @return boolean.
*/
bool ConstrainedDT::isInTri(Triangle* tri, int i, const vector<Vertex>& pts) {
    const Vertex& p  = pts[i];
    const Vertex& p1 = pts[tri->idx[0]];
    const Vertex& p2 = pts[tri->idx[1]];
    const Vertex& p3 = pts[tri->idx[2]];

    bool b1 = ((p.x - p1.x)*(p2.y - p1.y) - (p.y - p1.y)*(p2.x - p1.x)) > 0;
    bool b2 = ((p.x - p2.x)*(p3.y - p2.y) - (p.y - p2.y)*(p3.x - p2.x)) > 0;
    bool b3 = ((p.x - p3.x)*(p1.y - p3.y) - (p.y - p3.y)*(p1.x - p3.x)) > 0;

    return ((b1 == b2) && (b2 == b3));
};


/**
* @brief Normal Delaunay triangulation execution
* @details Normal Delaunay triangulation execution
* @return none.
*/
void ConstrainedDT::executeNormalDT() {
    cout << " Execute Normal Delaunay Triangulation.\n";
    int pts_cnt = 0;
    // STEP 4. Loop over each point.
    for (int ii = 0; ii < bins_.size(); ii++) {
        for (int jj = 0; jj < bins_[ii].size(); jj++) {
            // STEP 5. Insert new point in triangulation.
            int i = bins_[ii][jj];

#ifdef _VERBOSE_
            cout << "\n[" << pts_cnt++ << "]-th point... index [" << i << "] point is proceeding...";
            cout << "  points: [" << points_[i].x << ", " << points_[i].y << "]\n";
#endif
            // STEP 5-1. Lawson's searching to find enclosing Triangle (NOT CIRCUMCIRCLE!!!)
            Triangle* tri_cur = this->tri_latest_;
            int cnt = 0;
            while (!isInTri(tri_cur, i, points_)) {// 'tri_temp' encloses the point 'p'
                                                   // 'tri_temp' does not enclose the points_[i]
                                                   // Not enclosing? Do Lawson's searching until finding an enclosing Triangle.
                int next_dir = -1;
                findLawsonSearch(tri_cur, next_dir, i, points_);
                tri_cur = tri_cur->adj[next_dir];
                if ((++cnt) > 100000) throw std::runtime_error("ERROR: Infinite while loop for circum test.\n");
            } // end 

#ifdef _VERBOSE_
            cout << " enclosing Triangle for [" << i << "]: " << tri_cur->id << endl;
#endif
            // disconnect tri_cur's points
            disincorporateTriangle(tri_cur);

            // STEP 5-2. Delete this Triangle (tri_cur) and 'make' new three Triangles by connecting p to its each Vertex.
            Triangle* new0 = new Triangle(-1, -1, -1, n_tri_counter_++);
            Triangle* new1 = new Triangle(-1, -1, -1, n_tri_counter_++);
            Triangle* new2 = new Triangle(-1, -1, -1, n_tri_counter_++);

            // new0 (p-1-2) : opposite Triangle : 0
            new0->idx[0] = i; // opposite p
            new0->idx[1] = tri_cur->idx[1]; // opposite tri_temp 1
            new0->idx[2] = tri_cur->idx[2]; // opposite tri_temp 2
            new0->adj[0] = tri_cur->adj[0];
            new0->adj[1] = new1;
            new0->adj[2] = new2;
            if (new0->adj[0] != nullptr) // find non-sharing point from the tri_temp->adj[0]->idx
                for (int kk = 0; kk < 3; kk++)
                    if (new0->adj[0]->adj[kk] == tri_cur) new0->adj[0]->adj[kk] = new0;
            this->incorporateTriangle(new0);

            // new1 (p-0-2) : opposite Triangle : 1            
            new1->idx[0] = i; // opposite p
            new1->idx[1] = tri_cur->idx[0]; // opposite tri_temp 0
            new1->idx[2] = tri_cur->idx[2]; // opposite tri_temp 2
            new1->adj[0] = tri_cur->adj[1];
            new1->adj[1] = new0;
            new1->adj[2] = new2;
            if (new1->adj[0] != nullptr) // find non-sharing point from the tri_temp->adj[0]->idx
                for (int kk = 0; kk < 3; kk++)
                    if (new1->adj[0]->adj[kk] == tri_cur) new1->adj[0]->adj[kk] = new1;
            this->incorporateTriangle(new1);

            // new2 (p-0-1) : opposite Triangle : 2
            new2->idx[0] = i; // opposite p
            new2->idx[1] = tri_cur->idx[0]; // opposite tri_temp 0
            new2->idx[2] = tri_cur->idx[1]; // opposite tri_temp 1
            new2->adj[0] = tri_cur->adj[2];
            new2->adj[1] = new0;
            new2->adj[2] = new1;
            if (new2->adj[0] != nullptr) // find non-sharing point from the tri_temp->adj[0]->idx
                for (int kk = 0; kk < 3; kk++)
                    if (new2->adj[0]->adj[kk] == tri_cur) new2->adj[0]->adj[kk] = new2;
            this->incorporateTriangle(new2);

#ifdef _VERBOSE_
            cout << " neighbours for NEW 0 (" << new0->id << "): ";
            for (int aa = 0; aa < 3; aa++)
                if (new0->adj[aa] != nullptr)
                    cout << new0->adj[aa]->id << ",";
            cout << "\n";

            cout << " neighbours for NEW 1 (" << new1->id << "): ";
            for (int aa = 0; aa < 3; aa++)
                if (new1->adj[aa] != nullptr)
                    cout << new1->adj[aa]->id << ",";
            cout << "\n";

            cout << " neighbours for NEW 2 (" << new2->id << "): ";
            for (int aa = 0; aa < 3; aa++)
                if (new2->adj[aa] != nullptr)
                    cout << new2->adj[aa]->id << ",";
            cout << "\n";
#endif
            // update last list.
            this->tri_latest_ = new2;

            // delete tri_cur and pop from the map
            delete tri_cur;


            vector<int> id_new;
            // STEP 6. (initialize stack) push adjacent three Triangles of the newly generated Triangles.
            fstack_.clear();
            if (new0->adj[0] != nullptr) {
                fstack_.push(new0->adj[0]);
                id_new.push_back(new0->adj[0]->id);
#ifdef _VERBOSE_
                cout << "   adjacent swap candidate stacked! : " << new0->adj[0]->id << ", size: " << fstack_.size << "\n";
#endif
            }

            if (new1->adj[0] != nullptr) {
                fstack_.push(new1->adj[0]);
                id_new.push_back(new1->adj[0]->id);
#ifdef _VERBOSE_
                cout << "   adjacent swap candidate stacked! : " << new1->adj[0]->id << ", size: " << fstack_.size << "\n";
#endif
            }
            if (new2->adj[0] != nullptr) {
                fstack_.push(new2->adj[0]);
                id_new.push_back(new2->adj[0]->id);
#ifdef _VERBOSE_
                cout << "   adjacent swap candidate stacked! : " << new2->adj[0]->id << ", size: " << fstack_.size << "\n";
#endif
            }

#ifdef _VERBOSE_
            cout << "   Total # tri.: " << n_Triangles_ << " .. ";
            cout << " # of valid adjacents: " << fstack_.size;
            if (fstack_.size > 0) cout << " ... Lawson swap test!\n";
            else cout << " ... No point to be tested. Go to next point.\n";
#endif

            // STEP 7. (Restore Delaunay triangulation) While the stack or Triangles is not empty, 
            // execute Lawson's swapping scheme, as defined by steps 7.1~7.3
            Triangle* tri_outer = nullptr;
            Triangle* tri_in = nullptr;
            while (!fstack_.empty()) {
                // STEP 7-1. Remove a Triangle which is opposite p from the top of the stack.
                tri_outer = fstack_.top();
                fstack_.pop();

#ifdef _VERBOSE_
                cout << "   poped:" << tri_outer->id << ", top of while [remained stack size] : " << fstack_.size << " /";
                cout << " idx: " << tri_outer->idx[0] << "," << tri_outer->idx[1] << "," << tri_outer->idx[2] << "\n";
#endif
                // STEP 7-2. If p is outSide (or on) the circumcircle for this Triangle,
                // return to STEP 7-1.
                if (!isInCircum(tri_outer, i, points_)) {
#ifdef _VERBOSE_
                    cout << "    not circum 'p' ... continue...\n";
#endif
                    continue;
                }
                // Else, the Triangle containing p as a Vertex and the unstacked Triangle form 
                // a convex quadrilateral whose diagonal is drawn in the wrong direction.
                else {
                    // Swap this diagonal so that two old Triangles are repalced by two new
                    // Triangles and the structure of the Delaunay triangulation is locally restored.
                    // Gist: swap the diagonal (delete two Triangles and make new two Triangles)
                    // find opposite Triangle index (i.e. one of newly generated three Triangles.)
                    int idx_op = -1;
                    for (int kk = 0; kk < 3; kk++) {
                        if (tri_outer->adj[kk] != nullptr && tri_outer->adj[kk]->idx[0] == i)
                            idx_op = kk; // tri_outerÀÇ adj Áß, idx[0] == i ÀÎ ÁöÁ¡À» Ã£ŽÂŽÙ.
                    }
                    tri_in = tri_outer->adj[idx_op];
#ifdef _VERBOSE_
                    cout << "    circum  'p' ... SWAP CONDITION...\n";
                    cout << "   center: " << i << "\n";
                    cout << "   L tri  in: [" << tri_in->idx[0] << "," << tri_in->idx[1] << "," << tri_in->idx[2] << "]\n";
                    cout << "   L tri out: [" << tri_outer->idx[0] << "," << tri_outer->idx[1] << "," << tri_outer->idx[2] << "]\n";
#endif
                    // oppo-center = new diagonal.
                    // Thus, center-oppo-a / center-oppo-b (two new Triangles)
                    int oppo = tri_outer->idx[idx_op];
                    int a = tri_in->idx[1];
                    int b = tri_in->idx[2];

                    int idx_a_out = -1;
                    int idx_b_out = -1;
                    for (int kk = 0; kk < 3; kk++) {
                        if (tri_outer->idx[kk] == a) idx_a_out = kk;
                        if (tri_outer->idx[kk] == b) idx_b_out = kk;
                    }
#ifdef _VERBOSE_
                    cout << "     L idx a out: " << idx_a_out << ", idx b out: " << idx_b_out << endl;
#endif
                    // disconnect tri_in and tri_outer's points
                    disincorporateTriangle(tri_in);
                    disincorporateTriangle(tri_outer);

                    // make new two Triangles
                    Triangle* new_a = new Triangle(-1, -1, -1, n_tri_counter_++);
                    Triangle* new_b = new Triangle(-1, -1, -1, n_tri_counter_++);
                    new_a->idx[0] = i;
                    new_a->idx[1] = oppo;
                    new_a->idx[2] = b;
                    new_a->adj[0] = tri_outer->adj[idx_a_out];
                    new_a->adj[1] = tri_in->adj[1];
                    new_a->adj[2] = new_b;
                    if (new_a->adj[0] != nullptr) {
                        int idx_op = -1; // opposite
                        for (int kk = 0; kk < 3; kk++)
                            if (new_a->adj[0]->adj[kk] != nullptr && new_a->adj[0]->adj[kk] == tri_outer) idx_op = kk;
                        new_a->adj[0]->adj[idx_op] = new_a;
                    }
                    if (new_a->adj[1] != nullptr) {
                        int idx_op = -1; // opposite
                        for (int kk = 0; kk < 3; kk++)
                            if (new_a->adj[1]->adj[kk] != nullptr && new_a->adj[1]->adj[kk] == tri_in) idx_op = kk;
                        new_a->adj[1]->adj[idx_op] = new_a;
                    }
                    this->incorporateTriangle(new_a);

                    new_b->idx[0] = i;
                    new_b->idx[1] = oppo;
                    new_b->idx[2] = a;
                    new_b->adj[0] = tri_outer->adj[idx_b_out];
                    new_b->adj[1] = tri_in->adj[2];
                    new_b->adj[2] = new_a;
                    if (new_b->adj[0] != nullptr) {
                        int idx_op = -1; // opposite
                        for (int kk = 0; kk < 3; kk++)
                            if (new_b->adj[0]->adj[kk] != nullptr && new_b->adj[0]->adj[kk] == tri_outer) idx_op = kk;
                        new_b->adj[0]->adj[idx_op] = new_b;
                    }
                    if (new_b->adj[1] != nullptr) {
                        int idx_op = -1; // opposite
                        for (int kk = 0; kk < 3; kk++)
                            if (new_b->adj[1]->adj[kk] != nullptr && new_b->adj[1]->adj[kk] == tri_in) idx_op = kk;
                        new_b->adj[1]->adj[idx_op] = new_b;
                    }
                    this->incorporateTriangle(new_b);

#ifdef _VERBOSE_
                    cout << "   L tri   a: [" << new_a->idx[0] << "," << new_a->idx[1] << "," << new_a->idx[2] << "]\n";
                    cout << "   L tri   b: [" << new_b->idx[0] << "," << new_b->idx[1] << "," << new_b->idx[2] << "]\n";
#endif

                    if (new_a->adj[0] != nullptr) {
                        // PASS when adj[0] == new 0 or 1 or 2.
                        bool isexist = false;
                        for (int kk = 0; kk < id_new.size(); kk++) {
                            if (id_new[kk] == new_a->adj[0]->id) {
                                isexist = true;
                                break;
                            }
                        }
                        if (!isexist) {
                            fstack_.push(new_a->adj[0]);
#ifdef _VERBOSE_
                            cout << "     L adj[0] for a-" << new_a->adj[0]->id << ": [" << new_a->adj[0]->idx[0] << "," << new_a->adj[0]->idx[1] << "," << new_a->adj[0]->idx[2] << "]\n";
                            cout << "     [added stack size] : " << fstack_.size << "\n";
                            for (int nn = 0; nn < fstack_.size; nn++) cout << "       " << (*(fstack_.root() + nn))->id << ", idx: "
                                << (*(fstack_.root() + nn))->idx[0] << ", "
                                << (*(fstack_.root() + nn))->idx[1] << ", "
                                << (*(fstack_.root() + nn))->idx[2]
                                << "\n";
#endif
                        }
                    }

                    if (new_b->adj[0] != nullptr) {
                        // PASS when adj[0] == new 0 or 1 or 2.
                        bool isexist = false;
                        for (int kk = 0; kk < id_new.size(); kk++) {
                            if (id_new[kk] == new_b->adj[0]->id) {
                                isexist = true;
                                break;
                            }
                        }
                        if (!isexist) {
                            fstack_.push(new_b->adj[0]);
#ifdef _VERBOSE_
                            cout << "     L adj[0] for b- " << new_b->adj[0]->id << ": [" << new_b->adj[0]->idx[0] << "," << new_b->adj[0]->idx[1] << "," << new_b->adj[0]->idx[2] << "]\n";
                            cout << "     [added stack size] : " << fstack_.size << "\n";
                            for (int nn = 0; nn < fstack_.size; nn++) cout << "       " << (*(fstack_.root() + nn))->id << ", idx: "
                                << (*(fstack_.root() + nn))->idx[0] << ", "
                                << (*(fstack_.root() + nn))->idx[1] << ", "
                                << (*(fstack_.root() + nn))->idx[2]
                                << "\n";
#endif
                        }
                    }

                    this->tri_latest_ = new_b;

                    // delete two Triangles (tri_in, tri_outer)
                    delete tri_in;
                    delete tri_outer;
#ifdef _VERBOSE_
                    cout << "           Remained stack (end of while): " << fstack_.size << "\n";
                    for (int nn = 0; nn < fstack_.size; nn++)
                        cout << "             " << (*(fstack_.root() + nn))->id << "\n";
#endif
                }
            }// end while
        } // end for jj
    } // end for ii
};


bool ConstrainedDT::isStrictlyConvex(Triangle* tri_a, Triangle* tri_b) {
    // opposite index searching for each Triangle
    int idx_op_a = -1; // opposite
    for (int kk = 0; kk < 3; kk++)
        if (tri_a->adj[kk] == tri_b) idx_op_a = kk;
    int idx_op_b = -1; // opposite
    for (int kk = 0; kk < 3; kk++)
        if (tri_b->adj[kk] == tri_a) idx_op_b = kk;

    // find sharing edge
    int cnt = 0;
    int idx_shared[2] = { 0,0 };
    for (int i = 0; i < 3; i++)
        if (i != idx_op_a) idx_shared[cnt++] = i;

    // intersection test
    const Vertex& a0 = this->points_[tri_a->idx[idx_op_a]];
    const Vertex& a1 = this->points_[tri_b->idx[idx_op_b]];
    const Vertex& b0 = this->points_[tri_a->idx[idx_shared[0]]];
    const Vertex& b1 = this->points_[tri_a->idx[idx_shared[1]]];
    return isSegmentsIntersect(a0, a1, b0, b1);
};


bool isHaveVertex(Triangle* tri, int i) {
    bool is_have = false;
    for (int ii = 0; ii < 3; ii++) {
        if (tri->idx[ii] == i) {
            is_have = true;
            break;
        }
    }
    return is_have;
}

void ConstrainedDT::disincorporateTriangle(Triangle* tri) {
    int idxs[3] = { tri->idx[0], tri->idx[1], tri->idx[2] };
    
    // delete adjacent edges
    this->adjmat_->disconnectThreeNodes(idxs[0], idxs[1], idxs[2]);

    // delete Vertex->list
    std::list<Triangle*>::iterator it;
    for (int i = 0; i < 3; i++) {
        for (it = this->points_[idxs[i]].tri.begin(); it != this->points_[idxs[i]].tri.end(); it++)
            if ((*it) == tri) break;
        this->points_[idxs[i]].tri.erase(it);
    }
    // delete tri_map
    this->tri_map_.erase(tri->id);
    --n_Triangles_;
};

void ConstrainedDT::incorporateTriangle(Triangle* tri) {
    // change 1 and 2 (align points by clockwise order.)
    if (!isCCW(tri, this->points_)) sortVertexOrderCCW(tri);
    
    // set adjacent edges
    this->adjmat_->connectThreeNodes(tri->idx[0], tri->idx[1], tri->idx[2]);

    // set Vertex->list
    for (int i = 0; i < 3; i++) this->points_[tri->idx[i]].tri.push_back(tri);

    // set tri_map
    this->tri_map_.insert(make_pair(tri->id, tri));

    // increase # of valid Triangle.
    ++n_Triangles_;
};

void ConstrainedDT::findSharingTwoTriangles(const int& k, const int& l, Triangle** tris_sharing) {
    std::list<Triangle*>::iterator it;
    int cnt = 0;
    for (it = this->points_[k].tri.begin(); it != this->points_[k].tri.end(); it++) {
        int n_contain = 0;
        for (int mm = 0; mm < 3; mm++)
            if (((*it)->idx[mm] == k) || ((*it)->idx[mm] == l)) n_contain++;
        if (n_contain > 1) tris_sharing[cnt++] = (*it);
    }
    if (cnt != 2) throw std::runtime_error("cnt != 2 !!\n");
#ifdef _VERBOSE_
    cout << "    L Triangles 0 [" << tris_sharing[0]->idx[0] << "," << tris_sharing[0]->idx[1] << "," << tris_sharing[0]->idx[2] << "]\n";
    cout << "    L Triangles 1 [" << tris_sharing[1]->idx[0] << "," << tris_sharing[1]->idx[1] << "," << tris_sharing[1]->idx[2] << "]\n";
#endif
};

/**
* @brief refine the original DT to constrained DT.
* @details refine the original DT to constrained DT.
* @return none.
*/
void ConstrainedDT::executeRefineConstrainedDT() {
    cout << " refine Constraints...\n";
    // STEP 1 (Loop over each constrained edge.)
    for (int nn = 0; nn < n_constraints_; nn++) {
        intersecting_edges_.clear();
        new_edges_.clear();

        int i = constraint_edges_[nn].a; // idx of starting point of a constraint edge.
        int j = constraint_edges_[nn].b; // idx of end point of a constraint edge.
        if (i > j) throw std::runtime_error(" NOT ascending order!\n");

#ifdef _VERBOSE_
        cout << "\n [" << nn << "]-th constraint... idx: " << i << ", " << j << "\n";
#endif
        // Constraint edge Vi-Vj.
        Vertex& V_i = points_[i];
        Vertex& V_j = points_[j];

        // STEP 2 (Find intersecting edges.)
        // STEP 2-A. If the constrained edge Vi-Vj already exists, skip this edge. 
        if (adjmat_->isEdgeExist(i, j)) {
#ifdef _VERBOSE_
            cout << "  L This constrained edge exists in the graph!\n";
#endif
            continue;
        }
        // STEP 2-B. Else, search the triangulation and store all of the edges that cross Vi-Vj.
        else {
            // STEP 2-B-A. Detect all intersected edges by this constrained edge [i].
            // Firstly, find a Triangle with an edge intersecting Vi-Vj.
            Triangle* tri_cur = nullptr;
            Triangle* tri_prev = nullptr;

            int idx_op = -1; int idx_others[2] = { -1,-1 };
            for (auto it = V_i.tri.begin(); it != V_i.tri.end(); it++) {
                // if a side of the current triangle intersects a line 'p_b-p_a', stop and re-start from that triangle.
                // i.e., it is sufficient to test whether an opposite side of point a intersects the line 'p_b-p_a'.
                tri_cur = *it;
                int cnt = 0;
                for (int kk = 0; kk < 3; kk++) {
                    if (tri_cur->idx[kk] == i) idx_op = kk;
                    else idx_others[cnt++] = kk;
                }
                Vertex& V_a = points_[tri_cur->idx[idx_others[0]]];
                Vertex& V_b = points_[tri_cur->idx[idx_others[1]]];

                // if intersect, break. 
                // This Triangle is a starting Triangle to search all edges intersecting this constraint edge.
                if ( isSegmentsIntersect(V_a, V_b, V_i, V_j) )  break;
            } // end for

            int aa = tri_cur->idx[idx_others[0]];
            int bb = tri_cur->idx[idx_others[1]];
            if (aa > bb) {
                int tmp = aa;
                aa = bb;
                bb = tmp;
            }
            intersecting_edges_.emplace_back(aa,bb);
            tri_prev = tri_cur;
            tri_cur  = tri_cur->adj[idx_op];

            // Iteratively traverse toward the adj[idx_op] (!=nullptr) 
            // until a Triangle with 'points_[idx_b]' is encountered.
            while (!isHaveVertex(tri_cur, j)) {
                // If V_j belongs to the current triangle, searching intersecting edges is done.
                // If direct 'kk' is not a 'tri_prev', go to direction intersecting the adj[kk].
                // tri_prev = tri_cur;
                // Áï tri_cur = tri_cur->adj[kk];
                int next_dir = -1; // Search the next triangle. Go to the Lawson direction.
                for (int kk = 0; kk < 3; kk++) {
                    int cnt = 0;
                    for (int jj = 0; jj < 3; jj++)
                        if (jj != kk) idx_others[cnt++] = jj;

                    Vertex& V_a = points_[tri_cur->idx[idx_others[0]]];
                    Vertex& V_b = points_[tri_cur->idx[idx_others[1]]];
                    if ( (tri_cur->adj[kk] != tri_prev) && isSegmentsIntersect(V_a, V_b, V_i, V_j)) {
                        next_dir = kk;
                        break;
                    }
                }

                int aa = tri_cur->idx[idx_others[0]];
                int bb = tri_cur->idx[idx_others[1]];
                if (aa > bb) {
                    int tmp = aa;
                    aa = bb;
                    bb = tmp;
                }
                intersecting_edges_.emplace_back(aa, bb);

                // Next triangle...
                if (tri_cur->adj[next_dir] == nullptr) throw std::runtime_error("refine CDT: no neighborhood Triangle!\n");
                tri_prev = tri_cur;
                tri_cur  = tri_cur->adj[next_dir];
            } // end while(1)
#ifdef _VERBOSE_
            cout << " Intersecting edge list for edge: \n";
            if (!intersecting_edges_.empty())
                for (auto it = intersecting_edges_.begin(); it != intersecting_edges_.end(); it++) {
                    cout << "   [" << it->a << ", " << it->b << "] ";
                    cout << " intersecting test: " << isSegmentsIntersect(points_[it->a], points_[it->b], V_i, V_j) << "\n";
                }
            else cout << "   NONE\n";
#endif
            
            // STEP 3. (Remove intersecting edges.)
            while (!intersecting_edges_.empty()) {
                // STEP 3-A. Remove an edge from the list of intersecting edges. 
                // Let this edge be defined by the vertices Vk and Vl.
                Side VkVl = intersecting_edges_.front();
                int k = VkVl.a;
                int l = VkVl.b;
                intersecting_edges_.pop_front();
#ifdef _VERBOSE_
                cout << "  Currently focused intersecting edge- k: " << k << ", l: " << l << "\n";
#endif
                if (k > l) throw std::runtime_error(" not ascending order\n");

                // find two Triangle sharing edge Vk-Vl.
                Triangle* tris_sharing[2] = { nullptr,nullptr };
                this->findSharingTwoTriangles(k, l, tris_sharing);

                
                // STEP 3-B. 
                // If the two Triangles that share the edge Vk-Vl do not form a quadrilateral which is strictly convex,
                // then place Vk-Vl back on the list of intersecting edges and go STEP 3-A.
                if (!this->isStrictlyConvex(tris_sharing[0], tris_sharing[1])) {
#ifdef _VERBOSE_                    
                   cout << "     L Not strictly convex!  skip and push this edge on intersecting edges list.\n";
#endif
                   intersecting_edges_.emplace_back(k, l);
                }
                else {
#ifdef _VERBOSE_
                    cout << "     L Strictly convex!  Change diagonal\n";
#endif
                    // Else, swap the diagonal of this strictly convex quadrilateral 
                    // so that two new Triangles are substituted for two old Triangles.
                    // disconnect tri_in and tri_outer's points
                    this->disincorporateTriangle(tris_sharing[0]);
                    this->disincorporateTriangle(tris_sharing[1]);

                    int idx_k_a = -1, idx_k_b = -1;
                    int idx_l_a = -1, idx_l_b = -1;
                    int idx_op_a = -1, idx_op_b = -1;
                    for (int kk = 0; kk < 3; kk++) {
                        if (tris_sharing[0]->idx[kk] == k) idx_k_a = kk;
                        else if (tris_sharing[0]->idx[kk] == l) idx_l_a = kk;
                        else idx_op_a = kk;
                    }
                    for (int kk = 0; kk < 3; kk++) {
                        if (tris_sharing[1]->idx[kk] == k) idx_k_b = kk;
                        else if (tris_sharing[1]->idx[kk] == l) idx_l_b = kk;
                        else idx_op_b = kk;
                    }

                    // make new two Triangles
                    Triangle* new_l = new Triangle(-1, -1, -1, n_tri_counter_++);
                    Triangle* new_k = new Triangle(-1, -1, -1, n_tri_counter_++);

                    new_l->idx[0] = l;
                    new_l->idx[1] = tris_sharing[0]->idx[idx_op_a];
                    new_l->idx[2] = tris_sharing[1]->idx[idx_op_b];
                    new_l->adj[0] = new_k;
                    new_l->adj[1] = tris_sharing[1]->adj[idx_k_b];
                    new_l->adj[2] = tris_sharing[0]->adj[idx_k_a];
                    if (new_l->adj[1] != nullptr) {
                        int idx_op = -1; // opposite
                        for (int kk = 0; kk < 3; kk++)
                            if (new_l->adj[1]->adj[kk] == tris_sharing[1]) idx_op = kk;
                        new_l->adj[1]->adj[idx_op] = new_l;
                    }
                    if (new_l->adj[2] != nullptr) {
                        int idx_op = -1; // opposite
                        for (int kk = 0; kk < 3; kk++)
                            if (new_l->adj[2]->adj[kk] == tris_sharing[0]) idx_op = kk;
                        new_l->adj[2]->adj[idx_op] = new_l;
                    }
                    this->incorporateTriangle(new_l);

                    new_k->idx[0] = k;
                    new_k->idx[1] = tris_sharing[0]->idx[idx_op_a];
                    new_k->idx[2] = tris_sharing[1]->idx[idx_op_b];
                    new_k->adj[0] = new_l;
                    new_k->adj[1] = tris_sharing[1]->adj[idx_l_b];
                    new_k->adj[2] = tris_sharing[0]->adj[idx_l_a];
                    if (new_k->adj[1] != nullptr) {
                        int idx_op = -1; // opposite
                        for (int kk = 0; kk < 3; kk++)
                            if (new_k->adj[1]->adj[kk] == tris_sharing[1]) idx_op = kk;
                        new_k->adj[1]->adj[idx_op] = new_k;
                    }
                    if (new_k->adj[2] != nullptr) {
                        int idx_op = -1; // opposite
                        for (int kk = 0; kk < 3; kk++)
                            if (new_k->adj[2]->adj[kk] == tris_sharing[0]) idx_op = kk;
                        new_k->adj[2]->adj[idx_op] = new_k;
                    }
                    this->incorporateTriangle(new_k);

                    // Let the new diagonal be defined by the vertices Vm-Vn. 
                    int m = tris_sharing[0]->idx[idx_op_a];
                    int n = tris_sharing[1]->idx[idx_op_b];
                    if (m > n) {
                        int tmp = m;
                        m = n;
                        n = tmp;
                    }

                    // delete two Triangles ('tris_sharing[0]' and 'tris_sharing[1]')
                    delete tris_sharing[0];
                    delete tris_sharing[1];

                    // If Vm-Vn still intersects the constrained edge Vi-Vj, 
                    // then place it on the list of intersecting edges. 
                    if (isSegmentsIntersect(points_[m], points_[n], V_i, V_j)) intersecting_edges_.emplace_back(m, n);
                    // If Vm-Vn does not intersect Vi-Vj, then place Vm-Vn on a list of newly created edges.
                    else new_edges_.emplace_back(m, n);
                }
            } // end while (end of STEP 3.)
#ifdef _VERBOSE_
            cout << " New_edges: \n";
            for (auto it = new_edges_.begin(); it != new_edges_.end(); it++) {
                cout << "  [" << it->a << ", " << it->b << "]\n";
            }
#endif

            
            // STEP 4. (Restore Delaunay triangulation.) Repeat steps 4.A-4.C until no furthre swaps take place.
            // STEP 4-A. Loop over each edge in the list of newly created edges.
            for (auto it = new_edges_.begin(); it != new_edges_.end(); it++) {
                // STEP 4-B. Let the newly created edge be defined by the vertices Vk an d Vl. 
                int k = it->a;
                int l = it->b;
                if (k > l) throw std::runtime_error(" not ascending order!\n");

                // if the edge Vk-Vl is equal to the constrained edge Vi-Vj, then skip to step 4-A.
                if (k == i && l == j) {
#ifdef _VERBOSE_
                    cout << "   Vk-Vl is equal to a constraint edge Vi-Vj ... skip to next new edge.\n";
#endif
                    continue;
                }
                else {
                    // STEP 4-C. If the two Triangles that share the edge Vk-Vl do not satisfy the Delaunay criterion, 
                    // so that a Vertex of one of the Triangles is inSide the circumcirle of the other Triangle, 
#ifdef _VERBOSE_
                    cout << "   Vk-Vl is not equal to a constraint edge Vi-Vj ... PROCEED...\n";
#endif
                    // find two Triangles sharing this edge (Vk-Vl)
                    Triangle* tris_sharing[2] = { nullptr,nullptr };
                    this->findSharingTwoTriangles(k, l, tris_sharing);

                    // find opposite points 
                    int idx_op_a = -1, idx_op_b = -1;
                    for (int kk = 0; kk < 3; kk++) {
                        if (tris_sharing[0]->idx[kk] != it->a && tris_sharing[0]->idx[kk] != it->b) idx_op_a = kk;
                        if (tris_sharing[1]->idx[kk] != it->a && tris_sharing[1]->idx[kk] != it->b) idx_op_b = kk;
                    }
                    if (isInCircum(tris_sharing[0], tris_sharing[1]->idx[idx_op_b], points_) || isInCircum(tris_sharing[1], tris_sharing[0]->idx[idx_op_a], points_)) {
                        // then these Triangles form a quadrilateral with the diagonal drawn in the wrong direction. 
                        // In this case, the edge Vk-Vl is swapped with the other diagonal(say, Vm-Vn), thus substituting two
                        // new Triangles for two new Triangles, and Vk-Vl is replaced by Vm-Vn in the list of newly created edges.

                        int idx_k_a = -1, idx_k_b = -1;
                        int idx_l_a = -1, idx_l_b = -1;
                        int idx_op_a = -1, idx_op_b = -1;
                        for (int kk = 0; kk < 3; kk++) {
                            if (tris_sharing[0]->idx[kk] == k) idx_k_a = kk;
                            else if (tris_sharing[0]->idx[kk] == l) idx_l_a = kk;
                            else idx_op_a = kk;
                        }
                        for (int kk = 0; kk < 3; kk++) {
                            if (tris_sharing[1]->idx[kk] == k) idx_k_b = kk;
                            else if (tris_sharing[1]->idx[kk] == l) idx_l_b = kk;
                            else idx_op_b = kk;
                        }


                        // disconnect tri_in and tri_outer's points
                        this->disincorporateTriangle(tris_sharing[0]);
                        this->disincorporateTriangle(tris_sharing[1]);

                        // make new two Triangles
                        Triangle* new_l = new Triangle(-1, -1, -1, n_tri_counter_++);
                        Triangle* new_k = new Triangle(-1, -1, -1, n_tri_counter_++);

                        new_l->idx[0] = l;
                        new_l->idx[1] = tris_sharing[0]->idx[idx_op_a];
                        new_l->idx[2] = tris_sharing[1]->idx[idx_op_b];
                        new_l->adj[0] = new_k;
                        new_l->adj[1] = tris_sharing[1]->adj[idx_k_b];
                        new_l->adj[2] = tris_sharing[0]->adj[idx_k_a];
                        if (new_l->adj[1] != nullptr) {
                            int idx_op = -1; // opposite
                            for (int kk = 0; kk < 3; kk++)
                                if (new_l->adj[1]->adj[kk] == tris_sharing[1]) idx_op = kk;
                            new_l->adj[1]->adj[idx_op] = new_l;
                        }
                        if (new_l->adj[2] != nullptr) {
                            int idx_op = -1; // opposite
                            for (int kk = 0; kk < 3; kk++)
                                if (new_l->adj[2]->adj[kk] == tris_sharing[0]) idx_op = kk;
                            new_l->adj[2]->adj[idx_op] = new_l;
                        }
                        this->incorporateTriangle(new_l);

                        new_k->idx[0] = k;
                        new_k->idx[1] = tris_sharing[0]->idx[idx_op_a];
                        new_k->idx[2] = tris_sharing[1]->idx[idx_op_b];
                        new_k->adj[0] = new_l;
                        new_k->adj[1] = tris_sharing[1]->adj[idx_l_b];
                        new_k->adj[2] = tris_sharing[0]->adj[idx_l_a];
                        if (new_k->adj[1] != nullptr) {
                            int idx_op = -1; // opposite
                            for (int kk = 0; kk < 3; kk++)
                                if (new_k->adj[1]->adj[kk] == tris_sharing[1]) idx_op = kk;
                            new_k->adj[1]->adj[idx_op] = new_k;
                        }
                        if (new_k->adj[2] != nullptr) {
                            int idx_op = -1; // opposite
                            for (int kk = 0; kk < 3; kk++)
                                if (new_k->adj[2]->adj[kk] == tris_sharing[0]) idx_op = kk;
                            new_k->adj[2]->adj[idx_op] = new_k;
                        }
                        this->incorporateTriangle(new_k);

                        // Let the new diagonal be defined by the vertices Vm-Vn. 

                        int m = tris_sharing[0]->idx[idx_op_a];
                        int n = tris_sharing[1]->idx[idx_op_b];
                        if (m > n) {
                            int tmp = m;
                            m = n;
                            n = tmp;
                        }

                        it->a = m;
                        it->b = n;

                        delete tris_sharing[0];
                        delete tris_sharing[1];

                    }// if isCircum()
                } // end if else
            } // end for iterator  (END OF STEP 4.)
        } // end if else
    }// end for Vi-Vj
};



void ConstrainedDT::addPointsIntoDT(const vector<Vertex>& points_addi) {
    cout << " Add points into Normal Delaunay Triangulation.\n";
    // Initialize the newly inserted points.
    int n_pts_addi = points_addi.size();  // # of inserted new points.
    n_pts_org_ = points_.size(); // # of existing points. (including three super points)

    for (int i = 0; i < n_pts_addi; i++) this->points_.emplace_back(points_addi[i]);

    cout << "# of org points: " << points_.size() << " / # of addi points: " << points_addi.size() << '\n';


    // normalization of points' coordinates
    // points_addižŠ »õ·ÎÀÌ Ãß°¡ÇØÁØŽÙ.
    denom_ = std::fmaxf(x_max_ - x_min_, y_max_ - y_min_);
    float invdenom = 1.0f / denom_;
    for (int i = n_pts_org_; i < n_pts_org_ +n_pts_addi; i++) {
        this->points_[i].x = (this->points_[i].x - x_min_)*invdenom;
        this->points_[i].y = (this->points_[i].y - y_min_)*invdenom;
    }
    this->n_pts_ = points_.size();

    cout << " redfine adjmat_\n";
    delete adjmat_;
    this->adjmat_ = new AdjacentMatrix(n_pts_ + 3);
    this->adjmat_->connectThreeNodes(n_pts_org_, n_pts_org_+1, n_pts_org_+2);

    // STEP 4. Loop over each point.
    for (int i = n_pts_org_; i < n_pts_; ++i) {
        // STEP 5. Insert new point in triangulation.
        // STEP 5-1. Lawson's searching to find enclosing Triangle (NOT CIRCUMCIRCLE!!!)
        Triangle* tri_cur = this->tri_latest_;
        int cnt = 0;
        while (!isInTri(tri_cur, i, points_)) {// 'tri_temp' encloses the point 'p'
                                               // 'tri_temp' does not enclose the points_[i]
                                               // Not enclosing? Do Lawson's searching until finding an enclosing Triangle.
            int next_dir = -1;
            findLawsonSearch(tri_cur, next_dir, i, points_);
            tri_cur = tri_cur->adj[next_dir];
            if ((++cnt) > 100000) throw std::runtime_error("ERROR: Infinite while loop for circum test.\n");
        } // end 

        // disconnect tri_cur's points
        disincorporateTriangle(tri_cur);

        // STEP 5-2. Delete this Triangle (tri_cur) and 'make' new three Triangles by connecting p to its each Vertex.
        Triangle* new0 = new Triangle(-1, -1, -1, n_tri_counter_++);
        Triangle* new1 = new Triangle(-1, -1, -1, n_tri_counter_++);
        Triangle* new2 = new Triangle(-1, -1, -1, n_tri_counter_++);

        // new0 (p-1-2) : opposite Triangle : 0
        new0->idx[0] = i; // opposite p
        new0->idx[1] = tri_cur->idx[1]; // opposite tri_temp 1
        new0->idx[2] = tri_cur->idx[2]; // opposite tri_temp 2
        new0->adj[0] = tri_cur->adj[0];
        new0->adj[1] = new1;
        new0->adj[2] = new2;
        if (new0->adj[0] != nullptr) // find non-sharing point from the tri_temp->adj[0]->idx
            for (int kk = 0; kk < 3; kk++)
                if (new0->adj[0]->adj[kk] == tri_cur) new0->adj[0]->adj[kk] = new0;
        this->incorporateTriangle(new0);

        // new1 (p-0-2) : opposite Triangle : 1            
        new1->idx[0] = i; // opposite p
        new1->idx[1] = tri_cur->idx[0]; // opposite tri_temp 0
        new1->idx[2] = tri_cur->idx[2]; // opposite tri_temp 2
        new1->adj[0] = tri_cur->adj[1];
        new1->adj[1] = new0;
        new1->adj[2] = new2;
        if (new1->adj[0] != nullptr) // find non-sharing point from the tri_temp->adj[0]->idx
            for (int kk = 0; kk < 3; kk++)
                if (new1->adj[0]->adj[kk] == tri_cur) new1->adj[0]->adj[kk] = new1;
        this->incorporateTriangle(new1);

        // new2 (p-0-1) : opposite Triangle : 2
        new2->idx[0] = i; // opposite p
        new2->idx[1] = tri_cur->idx[0]; // opposite tri_temp 0
        new2->idx[2] = tri_cur->idx[1]; // opposite tri_temp 1
        new2->adj[0] = tri_cur->adj[2];
        new2->adj[1] = new0;
        new2->adj[2] = new1;
        if (new2->adj[0] != nullptr) // find non-sharing point from the tri_temp->adj[0]->idx
            for (int kk = 0; kk < 3; kk++)
                if (new2->adj[0]->adj[kk] == tri_cur) new2->adj[0]->adj[kk] = new2;
        this->incorporateTriangle(new2);

        this->tri_latest_ = new2; // update last list.
        delete tri_cur; // delete tri_cur and pop from the map

        vector<int> id_new;
        // STEP 6. (initialize stack) push adjacent three Triangles of the newly generated Triangles.
        fstack_.clear();
        if (new0->adj[0] != nullptr) {
            fstack_.push(new0->adj[0]);
            id_new.push_back(new0->adj[0]->id);
        }
        if (new1->adj[0] != nullptr) {
            fstack_.push(new1->adj[0]);
            id_new.push_back(new1->adj[0]->id);
        }
        if (new2->adj[0] != nullptr) {
            fstack_.push(new2->adj[0]);
            id_new.push_back(new2->adj[0]->id);
        }

        // STEP 7. (Restore Delaunay triangulation) While the stack or Triangles is not empty, 
        // execute Lawson's swapping scheme, as defined by steps 7.1~7.3
        Triangle* tri_outer = nullptr;
        Triangle* tri_in = nullptr;
        while (!fstack_.empty()) {
            // STEP 7-1. Remove a Triangle which is opposite p from the top of the stack.
            tri_outer = fstack_.top();
            fstack_.pop();

            // STEP 7-2. If p is outSide (or on) the circumcircle for this Triangle,
            // return to STEP 7-1.
            if (!isInCircum(tri_outer, i, points_)) continue;
            // Else, the Triangle containing p as a Vertex and the unstacked Triangle form 
            // a convex quadrilateral whose diagonal is drawn in the wrong direction.
            else {
                // Swap this diagonal so that two old Triangles are repalced by two new
                // Triangles and the structure of the Delaunay triangulation is locally restored.
                // Gist: swap the diagonal (delete two Triangles and make new two Triangles)
                // find opposite Triangle index (i.e. one of newly generated three Triangles.)
                int idx_op = -1;
                for (int kk = 0; kk < 3; kk++) {
                    if (tri_outer->adj[kk] != nullptr && tri_outer->adj[kk]->idx[0] == i)
                        idx_op = kk; // tri_outerÀÇ adj Áß, idx[0] == i ÀÎ ÁöÁ¡À» Ã£ŽÂŽÙ.
                }
                tri_in = tri_outer->adj[idx_op]; 
                                                 // oppo-center = new diagonal.
                                                 // Thus, center-oppo-a / center-oppo-b (two new Triangles)
                int oppo = tri_outer->idx[idx_op];
                int a = tri_in->idx[1];
                int b = tri_in->idx[2];

                int idx_a_out = -1;
                int idx_b_out = -1;
                for (int kk = 0; kk < 3; kk++) {
                    if (tri_outer->idx[kk] == a) idx_a_out = kk;
                    if (tri_outer->idx[kk] == b) idx_b_out = kk;
                }

                // disconnect tri_in and tri_outer's points
                disincorporateTriangle(tri_in);
                disincorporateTriangle(tri_outer);

                // make new two Triangles
                Triangle* new_a = new Triangle(-1, -1, -1, n_tri_counter_++);
                Triangle* new_b = new Triangle(-1, -1, -1, n_tri_counter_++);
                new_a->idx[0] = i;
                new_a->idx[1] = oppo;
                new_a->idx[2] = b;
                new_a->adj[0] = tri_outer->adj[idx_a_out];
                new_a->adj[1] = tri_in->adj[1];
                new_a->adj[2] = new_b;
                if (new_a->adj[0] != nullptr) {
                    int idx_op = -1; // opposite
                    for (int kk = 0; kk < 3; kk++)
                        if (new_a->adj[0]->adj[kk] != nullptr && new_a->adj[0]->adj[kk] == tri_outer) idx_op = kk;
                    new_a->adj[0]->adj[idx_op] = new_a;
                }
                if (new_a->adj[1] != nullptr) {
                    int idx_op = -1; // opposite
                    for (int kk = 0; kk < 3; kk++)
                        if (new_a->adj[1]->adj[kk] != nullptr && new_a->adj[1]->adj[kk] == tri_in) idx_op = kk;
                    new_a->adj[1]->adj[idx_op] = new_a;
                }
                this->incorporateTriangle(new_a);

                new_b->idx[0] = i;
                new_b->idx[1] = oppo;
                new_b->idx[2] = a;
                new_b->adj[0] = tri_outer->adj[idx_b_out];
                new_b->adj[1] = tri_in->adj[2];
                new_b->adj[2] = new_a;
                if (new_b->adj[0] != nullptr) {
                    int idx_op = -1; // opposite
                    for (int kk = 0; kk < 3; kk++)
                        if (new_b->adj[0]->adj[kk] != nullptr && new_b->adj[0]->adj[kk] == tri_outer) idx_op = kk;
                    new_b->adj[0]->adj[idx_op] = new_b;
                }
                if (new_b->adj[1] != nullptr) {
                    int idx_op = -1; // opposite
                    for (int kk = 0; kk < 3; kk++)
                        if (new_b->adj[1]->adj[kk] != nullptr && new_b->adj[1]->adj[kk] == tri_in) idx_op = kk;
                    new_b->adj[1]->adj[idx_op] = new_b;
                }
                this->incorporateTriangle(new_b);

                if (new_a->adj[0] != nullptr) {
                    bool isexist = false;
                    for (int kk = 0; kk < id_new.size(); kk++) {
                        if (id_new[kk] == new_a->adj[0]->id) {
                            isexist = true;
                            break;
                        }
                    }
                    if (!isexist) fstack_.push(new_a->adj[0]);
                }

                if (new_b->adj[0] != nullptr) {
                    bool isexist = false;
                    for (int kk = 0; kk < id_new.size(); kk++) {
                        if (id_new[kk] == new_b->adj[0]->id) {
                            isexist = true;
                            break;
                        }
                    }
                    if (!isexist) fstack_.push(new_b->adj[0]);
                }

                this->tri_latest_ = new_b;

                // delete two Triangles (tri_in, tri_outer)
                delete tri_in;
                delete tri_outer;
            }
        }// end while
    }// end for i
};


void ConstrainedDT::addPointsIntoDT(const vector<Eigen::Vector2f>& points_addi) {
    cout << " Add points into Normal Delaunay Triangulation.\n";
    // Initialize the newly inserted points.
    int n_pts_addi = points_addi.size();  // # of inserted new points.
    n_pts_org_ = points_.size(); // # of existing points. (including three super points)

    for (int i = 0; i < n_pts_addi; i++) this->points_.emplace_back(points_addi[i](0),points_addi[i](1));

    cout << "# of org points: " << points_.size() << " / # of addi points: " << points_addi.size() << '\n';


    // normalization of points' coordinates
    // points_addižŠ »õ·ÎÀÌ Ãß°¡ÇØÁØŽÙ.
    denom_ = std::fmaxf(x_max_ - x_min_, y_max_ - y_min_);
    float invdenom = 1.0f / denom_;
    for (int i = n_pts_org_; i < n_pts_org_ +n_pts_addi; i++) {
        this->points_[i].x = (this->points_[i].x - x_min_)*invdenom;
        this->points_[i].y = (this->points_[i].y - y_min_)*invdenom;
    }
    this->n_pts_ = points_.size();

    cout << " redfine adjmat_\n";
    delete adjmat_;
    this->adjmat_ = new AdjacentMatrix(n_pts_ + 3);
    this->adjmat_->connectThreeNodes(n_pts_org_, n_pts_org_+1, n_pts_org_+2);

    // STEP 4. Loop over each point.
    for (int i = n_pts_org_; i < n_pts_; ++i) {
        // STEP 5. Insert new point in triangulation.
        // STEP 5-1. Lawson's searching to find enclosing Triangle (NOT CIRCUMCIRCLE!!!)
        Triangle* tri_cur = this->tri_latest_;
        int cnt = 0;
        while (!isInTri(tri_cur, i, points_)) {// 'tri_temp' encloses the point 'p'
                                               // 'tri_temp' does not enclose the points_[i]
                                               // Not enclosing? Do Lawson's searching until finding an enclosing Triangle.
            int next_dir = -1;
            findLawsonSearch(tri_cur, next_dir, i, points_);
            tri_cur = tri_cur->adj[next_dir];
            if ((++cnt) > 100000) throw std::runtime_error("ERROR: Infinite while loop for circum test.\n");
        } // end 

        // disconnect tri_cur's points
        disincorporateTriangle(tri_cur);

        // STEP 5-2. Delete this Triangle (tri_cur) and 'make' new three Triangles by connecting p to its each Vertex.
        Triangle* new0 = new Triangle(-1, -1, -1, n_tri_counter_++);
        Triangle* new1 = new Triangle(-1, -1, -1, n_tri_counter_++);
        Triangle* new2 = new Triangle(-1, -1, -1, n_tri_counter_++);

        // new0 (p-1-2) : opposite Triangle : 0
        new0->idx[0] = i; // opposite p
        new0->idx[1] = tri_cur->idx[1]; // opposite tri_temp 1
        new0->idx[2] = tri_cur->idx[2]; // opposite tri_temp 2
        new0->adj[0] = tri_cur->adj[0];
        new0->adj[1] = new1;
        new0->adj[2] = new2;
        if (new0->adj[0] != nullptr) // find non-sharing point from the tri_temp->adj[0]->idx
            for (int kk = 0; kk < 3; kk++)
                if (new0->adj[0]->adj[kk] == tri_cur) new0->adj[0]->adj[kk] = new0;
        this->incorporateTriangle(new0);

        // new1 (p-0-2) : opposite Triangle : 1            
        new1->idx[0] = i; // opposite p
        new1->idx[1] = tri_cur->idx[0]; // opposite tri_temp 0
        new1->idx[2] = tri_cur->idx[2]; // opposite tri_temp 2
        new1->adj[0] = tri_cur->adj[1];
        new1->adj[1] = new0;
        new1->adj[2] = new2;
        if (new1->adj[0] != nullptr) // find non-sharing point from the tri_temp->adj[0]->idx
            for (int kk = 0; kk < 3; kk++)
                if (new1->adj[0]->adj[kk] == tri_cur) new1->adj[0]->adj[kk] = new1;
        this->incorporateTriangle(new1);

        // new2 (p-0-1) : opposite Triangle : 2
        new2->idx[0] = i; // opposite p
        new2->idx[1] = tri_cur->idx[0]; // opposite tri_temp 0
        new2->idx[2] = tri_cur->idx[1]; // opposite tri_temp 1
        new2->adj[0] = tri_cur->adj[2];
        new2->adj[1] = new0;
        new2->adj[2] = new1;
        if (new2->adj[0] != nullptr) // find non-sharing point from the tri_temp->adj[0]->idx
            for (int kk = 0; kk < 3; kk++)
                if (new2->adj[0]->adj[kk] == tri_cur) new2->adj[0]->adj[kk] = new2;
        this->incorporateTriangle(new2);

        this->tri_latest_ = new2; // update last list.
        delete tri_cur; // delete tri_cur and pop from the map

        vector<int> id_new;
        // STEP 6. (initialize stack) push adjacent three Triangles of the newly generated Triangles.
        fstack_.clear();
        if (new0->adj[0] != nullptr) {
            fstack_.push(new0->adj[0]);
            id_new.push_back(new0->adj[0]->id);
        }
        if (new1->adj[0] != nullptr) {
            fstack_.push(new1->adj[0]);
            id_new.push_back(new1->adj[0]->id);
        }
        if (new2->adj[0] != nullptr) {
            fstack_.push(new2->adj[0]);
            id_new.push_back(new2->adj[0]->id);
        }

        // STEP 7. (Restore Delaunay triangulation) While the stack or Triangles is not empty, 
        // execute Lawson's swapping scheme, as defined by steps 7.1~7.3
        Triangle* tri_outer = nullptr;
        Triangle* tri_in = nullptr;
        while (!fstack_.empty()) {
            // STEP 7-1. Remove a Triangle which is opposite p from the top of the stack.
            tri_outer = fstack_.top();
            fstack_.pop();

            // STEP 7-2. If p is outSide (or on) the circumcircle for this Triangle,
            // return to STEP 7-1.
            if (!isInCircum(tri_outer, i, points_)) continue;
            // Else, the Triangle containing p as a Vertex and the unstacked Triangle form 
            // a convex quadrilateral whose diagonal is drawn in the wrong direction.
            else {
                // Swap this diagonal so that two old Triangles are repalced by two new
                // Triangles and the structure of the Delaunay triangulation is locally restored.
                // Gist: swap the diagonal (delete two Triangles and make new two Triangles)
                // find opposite Triangle index (i.e. one of newly generated three Triangles.)
                int idx_op = -1;
                for (int kk = 0; kk < 3; kk++) {
                    if (tri_outer->adj[kk] != nullptr && tri_outer->adj[kk]->idx[0] == i)
                        idx_op = kk; // tri_outerÀÇ adj Áß, idx[0] == i ÀÎ ÁöÁ¡À» Ã£ŽÂŽÙ.
                }
                tri_in = tri_outer->adj[idx_op]; 
                                                 // oppo-center = new diagonal.
                                                 // Thus, center-oppo-a / center-oppo-b (two new Triangles)
                int oppo = tri_outer->idx[idx_op];
                int a = tri_in->idx[1];
                int b = tri_in->idx[2];

                int idx_a_out = -1;
                int idx_b_out = -1;
                for (int kk = 0; kk < 3; kk++) {
                    if (tri_outer->idx[kk] == a) idx_a_out = kk;
                    if (tri_outer->idx[kk] == b) idx_b_out = kk;
                }

                // disconnect tri_in and tri_outer's points
                disincorporateTriangle(tri_in);
                disincorporateTriangle(tri_outer);

                // make new two Triangles
                Triangle* new_a = new Triangle(-1, -1, -1, n_tri_counter_++);
                Triangle* new_b = new Triangle(-1, -1, -1, n_tri_counter_++);
                new_a->idx[0] = i;
                new_a->idx[1] = oppo;
                new_a->idx[2] = b;
                new_a->adj[0] = tri_outer->adj[idx_a_out];
                new_a->adj[1] = tri_in->adj[1];
                new_a->adj[2] = new_b;
                if (new_a->adj[0] != nullptr) {
                    int idx_op = -1; // opposite
                    for (int kk = 0; kk < 3; kk++)
                        if (new_a->adj[0]->adj[kk] != nullptr && new_a->adj[0]->adj[kk] == tri_outer) idx_op = kk;
                    new_a->adj[0]->adj[idx_op] = new_a;
                }
                if (new_a->adj[1] != nullptr) {
                    int idx_op = -1; // opposite
                    for (int kk = 0; kk < 3; kk++)
                        if (new_a->adj[1]->adj[kk] != nullptr && new_a->adj[1]->adj[kk] == tri_in) idx_op = kk;
                    new_a->adj[1]->adj[idx_op] = new_a;
                }
                this->incorporateTriangle(new_a);

                new_b->idx[0] = i;
                new_b->idx[1] = oppo;
                new_b->idx[2] = a;
                new_b->adj[0] = tri_outer->adj[idx_b_out];
                new_b->adj[1] = tri_in->adj[2];
                new_b->adj[2] = new_a;
                if (new_b->adj[0] != nullptr) {
                    int idx_op = -1; // opposite
                    for (int kk = 0; kk < 3; kk++)
                        if (new_b->adj[0]->adj[kk] != nullptr && new_b->adj[0]->adj[kk] == tri_outer) idx_op = kk;
                    new_b->adj[0]->adj[idx_op] = new_b;
                }
                if (new_b->adj[1] != nullptr) {
                    int idx_op = -1; // opposite
                    for (int kk = 0; kk < 3; kk++)
                        if (new_b->adj[1]->adj[kk] != nullptr && new_b->adj[1]->adj[kk] == tri_in) idx_op = kk;
                    new_b->adj[1]->adj[idx_op] = new_b;
                }
                this->incorporateTriangle(new_b);

                if (new_a->adj[0] != nullptr) {
                    bool isexist = false;
                    for (int kk = 0; kk < id_new.size(); kk++) {
                        if (id_new[kk] == new_a->adj[0]->id) {
                            isexist = true;
                            break;
                        }
                    }
                    if (!isexist) fstack_.push(new_a->adj[0]);
                }

                if (new_b->adj[0] != nullptr) {
                    bool isexist = false;
                    for (int kk = 0; kk < id_new.size(); kk++) {
                        if (id_new[kk] == new_b->adj[0]->id) {
                            isexist = true;
                            break;
                        }
                    }
                    if (!isexist) fstack_.push(new_b->adj[0]);
                }

                this->tri_latest_ = new_b;

                // delete two Triangles (tri_in, tri_outer)
                delete tri_in;
                delete tri_outer;
            }
        }// end while
    }// end for i
};




void ConstrainedDT::addPointsIntoDT(const vector<PointDB>& points_addi) {
    cout << " Add points into Normal Delaunay Triangulation.\n";
    // Initialize the newly inserted points.
    int n_pts_addi = points_addi.size();  // # of inserted new points.
    n_pts_org_ = points_.size(); // # of existing points. (including three super points)

    for (int i = 0; i < n_pts_addi; i++) this->points_.emplace_back(points_addi[i].pts_(0),points_addi[i].pts_(1));

    cout << "# of org points: " << points_.size() << " / # of addi points: " << points_addi.size() << '\n';


    // normalization of points' coordinates
    // points_addižŠ »õ·ÎÀÌ Ãß°¡ÇØÁØŽÙ.
    denom_ = std::fmaxf(x_max_ - x_min_, y_max_ - y_min_);
    float invdenom = 1.0f / denom_;
    for (int i = n_pts_org_; i < n_pts_org_ +n_pts_addi; i++) {
        this->points_[i].x = (this->points_[i].x - x_min_)*invdenom;
        this->points_[i].y = (this->points_[i].y - y_min_)*invdenom;
    }
    this->n_pts_ = points_.size();

    cout << " redfine adjmat_\n";
    delete adjmat_;
    this->adjmat_ = new AdjacentMatrix(n_pts_ + 3);
    this->adjmat_->connectThreeNodes(n_pts_org_, n_pts_org_+1, n_pts_org_+2);

    // STEP 4. Loop over each point.
    for (int i = n_pts_org_; i < n_pts_; ++i) {
        // STEP 5. Insert new point in triangulation.
        // STEP 5-1. Lawson's searching to find enclosing Triangle (NOT CIRCUMCIRCLE!!!)
        Triangle* tri_cur = this->tri_latest_;
        int cnt = 0;
        while (!isInTri(tri_cur, i, points_)) {// 'tri_temp' encloses the point 'p'
                                               // 'tri_temp' does not enclose the points_[i]
                                               // Not enclosing? Do Lawson's searching until finding an enclosing Triangle.
            int next_dir = -1;
            findLawsonSearch(tri_cur, next_dir, i, points_);
            tri_cur = tri_cur->adj[next_dir];
            if ((++cnt) > 100000) throw std::runtime_error("ERROR: Infinite while loop for circum test.\n");
        } // end 

        // disconnect tri_cur's points
        disincorporateTriangle(tri_cur);

        // STEP 5-2. Delete this Triangle (tri_cur) and 'make' new three Triangles by connecting p to its each Vertex.
        Triangle* new0 = new Triangle(-1, -1, -1, n_tri_counter_++);
        Triangle* new1 = new Triangle(-1, -1, -1, n_tri_counter_++);
        Triangle* new2 = new Triangle(-1, -1, -1, n_tri_counter_++);

        // new0 (p-1-2) : opposite Triangle : 0
        new0->idx[0] = i; // opposite p
        new0->idx[1] = tri_cur->idx[1]; // opposite tri_temp 1
        new0->idx[2] = tri_cur->idx[2]; // opposite tri_temp 2
        new0->adj[0] = tri_cur->adj[0];
        new0->adj[1] = new1;
        new0->adj[2] = new2;
        if (new0->adj[0] != nullptr) // find non-sharing point from the tri_temp->adj[0]->idx
            for (int kk = 0; kk < 3; kk++)
                if (new0->adj[0]->adj[kk] == tri_cur) new0->adj[0]->adj[kk] = new0;
        this->incorporateTriangle(new0);

        // new1 (p-0-2) : opposite Triangle : 1            
        new1->idx[0] = i; // opposite p
        new1->idx[1] = tri_cur->idx[0]; // opposite tri_temp 0
        new1->idx[2] = tri_cur->idx[2]; // opposite tri_temp 2
        new1->adj[0] = tri_cur->adj[1];
        new1->adj[1] = new0;
        new1->adj[2] = new2;
        if (new1->adj[0] != nullptr) // find non-sharing point from the tri_temp->adj[0]->idx
            for (int kk = 0; kk < 3; kk++)
                if (new1->adj[0]->adj[kk] == tri_cur) new1->adj[0]->adj[kk] = new1;
        this->incorporateTriangle(new1);

        // new2 (p-0-1) : opposite Triangle : 2
        new2->idx[0] = i; // opposite p
        new2->idx[1] = tri_cur->idx[0]; // opposite tri_temp 0
        new2->idx[2] = tri_cur->idx[1]; // opposite tri_temp 1
        new2->adj[0] = tri_cur->adj[2];
        new2->adj[1] = new0;
        new2->adj[2] = new1;
        if (new2->adj[0] != nullptr) // find non-sharing point from the tri_temp->adj[0]->idx
            for (int kk = 0; kk < 3; kk++)
                if (new2->adj[0]->adj[kk] == tri_cur) new2->adj[0]->adj[kk] = new2;
        this->incorporateTriangle(new2);

        this->tri_latest_ = new2; // update last list.
        delete tri_cur; // delete tri_cur and pop from the map

        vector<int> id_new;
        // STEP 6. (initialize stack) push adjacent three Triangles of the newly generated Triangles.
        fstack_.clear();
        if (new0->adj[0] != nullptr) {
            fstack_.push(new0->adj[0]);
            id_new.push_back(new0->adj[0]->id);
        }
        if (new1->adj[0] != nullptr) {
            fstack_.push(new1->adj[0]);
            id_new.push_back(new1->adj[0]->id);
        }
        if (new2->adj[0] != nullptr) {
            fstack_.push(new2->adj[0]);
            id_new.push_back(new2->adj[0]->id);
        }

        // STEP 7. (Restore Delaunay triangulation) While the stack or Triangles is not empty, 
        // execute Lawson's swapping scheme, as defined by steps 7.1~7.3
        Triangle* tri_outer = nullptr;
        Triangle* tri_in = nullptr;
        while (!fstack_.empty()) {
            // STEP 7-1. Remove a Triangle which is opposite p from the top of the stack.
            tri_outer = fstack_.top();
            fstack_.pop();

            // STEP 7-2. If p is outSide (or on) the circumcircle for this Triangle,
            // return to STEP 7-1.
            if (!isInCircum(tri_outer, i, points_)) continue;
            // Else, the Triangle containing p as a Vertex and the unstacked Triangle form 
            // a convex quadrilateral whose diagonal is drawn in the wrong direction.
            else {
                // Swap this diagonal so that two old Triangles are repalced by two new
                // Triangles and the structure of the Delaunay triangulation is locally restored.
                // Gist: swap the diagonal (delete two Triangles and make new two Triangles)
                // find opposite Triangle index (i.e. one of newly generated three Triangles.)
                int idx_op = -1;
                for (int kk = 0; kk < 3; kk++) {
                    if (tri_outer->adj[kk] != nullptr && tri_outer->adj[kk]->idx[0] == i)
                        idx_op = kk; // tri_outerÀÇ adj Áß, idx[0] == i ÀÎ ÁöÁ¡À» Ã£ŽÂŽÙ.
                }
                tri_in = tri_outer->adj[idx_op]; 
                                                 // oppo-center = new diagonal.
                                                 // Thus, center-oppo-a / center-oppo-b (two new Triangles)
                int oppo = tri_outer->idx[idx_op];
                int a = tri_in->idx[1];
                int b = tri_in->idx[2];

                int idx_a_out = -1;
                int idx_b_out = -1;
                for (int kk = 0; kk < 3; kk++) {
                    if (tri_outer->idx[kk] == a) idx_a_out = kk;
                    if (tri_outer->idx[kk] == b) idx_b_out = kk;
                }

                // disconnect tri_in and tri_outer's points
                disincorporateTriangle(tri_in);
                disincorporateTriangle(tri_outer);

                // make new two Triangles
                Triangle* new_a = new Triangle(-1, -1, -1, n_tri_counter_++);
                Triangle* new_b = new Triangle(-1, -1, -1, n_tri_counter_++);
                new_a->idx[0] = i;
                new_a->idx[1] = oppo;
                new_a->idx[2] = b;
                new_a->adj[0] = tri_outer->adj[idx_a_out];
                new_a->adj[1] = tri_in->adj[1];
                new_a->adj[2] = new_b;
                if (new_a->adj[0] != nullptr) {
                    int idx_op = -1; // opposite
                    for (int kk = 0; kk < 3; kk++)
                        if (new_a->adj[0]->adj[kk] != nullptr && new_a->adj[0]->adj[kk] == tri_outer) idx_op = kk;
                    new_a->adj[0]->adj[idx_op] = new_a;
                }
                if (new_a->adj[1] != nullptr) {
                    int idx_op = -1; // opposite
                    for (int kk = 0; kk < 3; kk++)
                        if (new_a->adj[1]->adj[kk] != nullptr && new_a->adj[1]->adj[kk] == tri_in) idx_op = kk;
                    new_a->adj[1]->adj[idx_op] = new_a;
                }
                this->incorporateTriangle(new_a);

                new_b->idx[0] = i;
                new_b->idx[1] = oppo;
                new_b->idx[2] = a;
                new_b->adj[0] = tri_outer->adj[idx_b_out];
                new_b->adj[1] = tri_in->adj[2];
                new_b->adj[2] = new_a;
                if (new_b->adj[0] != nullptr) {
                    int idx_op = -1; // opposite
                    for (int kk = 0; kk < 3; kk++)
                        if (new_b->adj[0]->adj[kk] != nullptr && new_b->adj[0]->adj[kk] == tri_outer) idx_op = kk;
                    new_b->adj[0]->adj[idx_op] = new_b;
                }
                if (new_b->adj[1] != nullptr) {
                    int idx_op = -1; // opposite
                    for (int kk = 0; kk < 3; kk++)
                        if (new_b->adj[1]->adj[kk] != nullptr && new_b->adj[1]->adj[kk] == tri_in) idx_op = kk;
                    new_b->adj[1]->adj[idx_op] = new_b;
                }
                this->incorporateTriangle(new_b);

                if (new_a->adj[0] != nullptr) {
                    bool isexist = false;
                    for (int kk = 0; kk < id_new.size(); kk++) {
                        if (id_new[kk] == new_a->adj[0]->id) {
                            isexist = true;
                            break;
                        }
                    }
                    if (!isexist) fstack_.push(new_a->adj[0]);
                }

                if (new_b->adj[0] != nullptr) {
                    bool isexist = false;
                    for (int kk = 0; kk < id_new.size(); kk++) {
                        if (id_new[kk] == new_b->adj[0]->id) {
                            isexist = true;
                            break;
                        }
                    }
                    if (!isexist) fstack_.push(new_b->adj[0]);
                }

                this->tri_latest_ = new_b;

                // delete two Triangles (tri_in, tri_outer)
                delete tri_in;
                delete tri_outer;
            }
        }// end while
    }// end for i
};


void ConstrainedDT::renewConstraints(const vector<Side>& constraint_edges_addi) {
    // constraint_edges_addi is corresponding to the accumulated # of triangle.
    // get new constraints
    cout << " Renew constraints... \n";
    this->constraint_edges_.resize(0);
    int n_constraint_ = constraint_edges_addi.size();
    for (int n = 0; n < n_constraint_; n++) {
        // ascending sorted
        int i = constraint_edges_addi[n].a + n_pts_org_; // considering the existing points.
        int j = constraint_edges_addi[n].b + n_pts_org_;
        if (i > j) {
            int tmp = i;
            i = j;
            j = tmp;
        }
        this->constraint_edges_.emplace_back(i, j);
    }

    n_constraints_ = this->constraint_edges_.size();
};


void ConstrainedDT::getCenterPointsOfTriangles(const float& thres_area, vector<Vertex>& points_centers) {
    points_centers.resize(0);

    this->thres_area_ = thres_area;
    denom_ = std::fmaxf(x_max_ - x_min_, y_max_ - y_min_);

    for (auto iter = tri_map_.begin(); iter != tri_map_.end(); ++iter){
        Triangle* tri_cur = iter->second;
        Vertex& pa = this->points_[tri_cur->idx[0]];
        Vertex& pb = this->points_[tri_cur->idx[1]];
        Vertex& pc = this->points_[tri_cur->idx[2]];

        float area = calcTriArea(pa,pb,pc)*denom_*denom_;

        if (area > this->thres_area_ && area < 10000)
            points_centers.emplace_back(0.33333f*(pa.x + pb.x + pc.x)*denom_+x_min_, 0.33333f*(pa.y + pb.y + pc.y)*denom_ + y_min_) ;
    }
};

void ConstrainedDT::getCenterPointsOfTriangles(const float& thres_area, vector<Eigen::Vector2f>& points_centers) {
    points_centers.resize(0);

    this->thres_area_ = thres_area;
    denom_ = std::fmaxf(x_max_ - x_min_, y_max_ - y_min_);

    for (auto iter = tri_map_.begin(); iter != tri_map_.end(); ++iter){
        Triangle* tri_cur = iter->second;
        Vertex& pa = this->points_[tri_cur->idx[0]];
        Vertex& pb = this->points_[tri_cur->idx[1]];
        Vertex& pc = this->points_[tri_cur->idx[2]];

        float area = calcTriArea(pa,pb,pc)*denom_*denom_;

        if (area > this->thres_area_ && area < 10000)
            points_centers.emplace_back(0.33333f*(pa.x + pb.x + pc.x)*denom_+x_min_, 0.33333f*(pa.y + pb.y + pc.y)*denom_ + y_min_) ;
    }
};