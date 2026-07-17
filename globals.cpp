#include "globals.h"

int mode = 0;

Eigen::MatrixXd V;
Eigen::MatrixXi F;

std::unordered_set<int> handles;
std::unordered_set<int> anchors;
std::unordered_set<int> available;
void load_mesh(){
    igl::readOFF("C:\\faks\\eth\\ARAP\\ARAP\\res\\armadillo_1k.off", V, F);
    available.clear();
    anchors.clear();
    handles.clear();
    for (int i = 0; i < V.rows(); i++) available.insert(i);
}