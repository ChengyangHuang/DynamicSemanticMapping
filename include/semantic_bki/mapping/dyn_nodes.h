#pragma once

#include <vector>
#include "point3f.h"
#include "bkioctree_node.h"
#include <limits>

#include <iostream>


namespace semantic_bki {

    class DynamicNodes {
        
        private:
            std::vector<int> dyn_classes;
            std::vector<std::vector<SemanticOcTreeNode>> dyn_nodes_prev;
            std::vector<std::vector<point3f>> dyn_points_prev;
            std::vector<std::vector<SemanticOcTreeNode>> dyn_nodes_curr;
            std::vector<std::vector<point3f>> dyn_points_curr;

        
        public:

            bool empty;

            //Constructor
            DynamicNodes(std::vector<int> dyn_classes_in) {
                dyn_classes = std::vector<int>(dyn_classes_in.size());
                std::copy(dyn_classes_in.begin(), dyn_classes_in.end(), dyn_classes.begin());
                dyn_nodes_prev = std::vector<std::vector<SemanticOcTreeNode>>(dyn_classes.size());
                dyn_points_prev = std::vector<std::vector<point3f>>(dyn_classes.size());
                dyn_nodes_curr = std::vector<std::vector<SemanticOcTreeNode>>(dyn_classes.size());
                dyn_points_curr = std::vector<std::vector<point3f>>(dyn_classes.size());
                empty = true;
            }
            //Deconstructor
            ~DynamicNodes() {}

            void update() {
                for (int i = 0; i < dyn_classes.size(); i++) {
                    // copy all current nodes and points to previous
                    dyn_nodes_prev[i].clear();
                    dyn_points_prev[i].clear();
                    for (int j = 0; j < dyn_nodes_curr[i].size(); j++) {
                        dyn_nodes_prev[i].push_back(dyn_nodes_curr[i][j]);
                        dyn_points_prev[i].push_back(dyn_points_curr[i][j]);
                    }
                    assert(dyn_nodes_prev[i].size() == dyn_nodes_curr[i].size());
                    assert(dyn_points_prev[i].size() == dyn_points_curr[i].size());

                    // clear curr nodes and points
                    dyn_nodes_curr[i].clear();
                    dyn_points_curr[i].clear();
                }
                empty = false;
            }

            //Adds the input node to dyn_nodes and dyn_points
            void add_node(int cl, SemanticOcTreeNode &node, point3f point) {
                dyn_nodes_curr[cl].push_back(node);
                dyn_points_curr[cl].push_back(point);
            }

            //Returns the nearest node to the input point
            std::pair<SemanticOcTreeNode, double> find_nearest(std::vector<int> classes, point3f point) {
                int nearest_node = -1;
                int nearest_node_class = -1;
                double nearest_distance = std::numeric_limits<double>::infinity();
                for (int i = 0; i < classes.size(); i++) {
                    for (int j = 0; j < dyn_points_prev[classes[i]].size(); j++) {
                        double distance = dyn_points_prev[classes[i]][j].distance(point);
                        if (distance < nearest_distance) {
                            nearest_node = j;
                            nearest_node_class = classes[i];
                            nearest_distance = distance;
                        }
                    }
                }

                if (nearest_node < 0) {
                    return std::make_pair(SemanticOcTreeNode(), nearest_distance);
                } 
                else {
                    return std::make_pair(dyn_nodes_prev[nearest_node_class][nearest_node], nearest_distance);
                }
            }
    };

}