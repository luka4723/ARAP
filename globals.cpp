#include "globals.h"

int mode = 0;
Eigen::MatrixXd V;
Eigen::MatrixXi F;
std::vector<int> handles;
std::vector<int> anchors;
std::vector<std::vector<int>> adjacency;
std::map<std::pair<int,int>, std::vector<double>> angles;
igl::opengl::glfw::Viewer viewer;
std::vector<Cell> cells;
Eigen::SparseMatrix<double> L;
Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
std::vector<int8_t> vertex_type;
int selected_vertex = -1;
Eigen::MatrixXd V_new;
int number_of_iterations = 5;
int algorithm = 0;
bool energy_flag = false;

bool load_mesh(){
    viewer.data().point_size = 10;  
    viewer.core().is_animating = true;
    viewer.core().animation_max_fps = 1000;
    if (!igl::readOFF("meshes/armadillo_1k.off", V, F))
    {
        std::cerr << "Error: Cannot load mesh" << std::endl;
        return false;
    }    
    igl::adjacency_list(F, adjacency);
    V_new = V;
    anchors.clear();
    handles.clear();
    vertex_type.resize(V.rows(), 0);
    L.resize(V.rows(),V.rows());

    std::vector<int> a = {330,18,380,300,235,677,818,189,108};
    std::vector<int> h = {853};
    for(int n : a){
        anchors.push_back(n);
        vertex_type[n] = 2;
    }
    handles.push_back(853);
    vertex_type[853] = 1;
    return true;
}

void build_L()
{
    std::vector<Eigen::Triplet<double>> triplets;
    for (int i = 0; i < V.rows(); i++) {
        if (i == selected_vertex || vertex_type[i] == 2) { 
            triplets.emplace_back(i, i, 1.0);
            continue;  
        }

        double val = 0.0;
        int n = 0;
        for (int j : adjacency[i]) {
            double w = cells[i].weights[n];
            val += w; 
            if (j != selected_vertex && vertex_type[j] != 2) triplets.emplace_back(i, j, -w);   
            n++;
        }
        triplets.emplace_back(i, i, val);
    }
    
    L.setFromTriplets(triplets.begin(), triplets.end());
    L.makeCompressed();
    solver.compute(L);
}