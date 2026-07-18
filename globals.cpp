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


void load_mesh(){
    viewer.data().point_size = 13;  
    igl::readOFF("C:\\faks\\eth\\ARAP\\ARAP\\res\\armadillo_1k.off", V, F);
    igl::adjacency_list(F, adjacency);
    available.clear();
    anchors.clear();
    handles.clear();
    for (int i = 0; i < V.rows(); i++) available.insert(i);
}