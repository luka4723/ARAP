#include "globals.h"

int mode = 0;

Eigen::MatrixXd V;
Eigen::MatrixXi F;

std::unordered_set<int> handles;
std::unordered_set<int> anchors;
std::unordered_set<int> available;
std::vector<std::vector<int>> adjacency;
std::map<std::pair<int,int>, std::vector<double>> angles;
igl::opengl::glfw::Viewer viewer;
std::vector<Cell> cells;
Eigen::SparseMatrix<double> L;
Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
int selected_vertex = -1;

void load_mesh(){
    viewer.data().point_size = 10;  
    igl::readOFF("C:\\faks\\eth\\ARAP\\ARAP\\res\\armadillo_1k.off", V, F);
    igl::adjacency_list(F, adjacency);
    available.clear();
    anchors.clear();
    handles.clear();
    for (int i = 0; i < V.rows(); i++) available.insert(i);
    L.resize(V.rows(),V.rows());
}

void build_L()
{
    std::vector<Eigen::Triplet<double>> triplets;

    for (int i = 0; i < V.rows(); i++) {
        if (handles.find(i) != handles.end() || anchors.find(i) != anchors.end()) { 
            triplets.emplace_back(i, i, 1.0);
            continue;  
        }

        double val = 0.0;
        int n = 0;
        for (int j : adjacency[i]) {
            double w = cells[i].weights[n];
            val += w; 
            if (handles.find(j) == handles.end() && anchors.find(j) == anchors.end()) triplets.emplace_back(i, j, -w);   
            n++;
        }
        triplets.emplace_back(i, i, val);
    }
    
    L.setFromTriplets(triplets.begin(), triplets.end());
    L.makeCompressed();
    solver.compute(L);
}