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
        bool empty;

        
        public:
            //Constructor
            DynamicNodes(std::vector<int> dyn_classes_in) {
                dyn_classes = dyn_classes_in;
                dyn_nodes_prev = std::vector<std::vector<SemanticOcTreeNode>>(dyn_classes.size());
                dyn_points_prev = std::vector<std::vector<point3f>>(dyn_classes.size());
                dyn_nodes_curr = std::vector<std::vector<SemanticOcTreeNode>>(dyn_classes.size());
                dyn_points_curr = std::vector<std::vector<point3f>>(dyn_classes.size());
                empty = true;
            }
            //Deconstructor
            ~DynamicNodes() {}

            void update() {
                dyn_nodes_prev = dyn_nodes_curr;
                dyn_points_prev = dyn_points_curr;
                dyn_nodes_curr.clear();
                dyn_points_curr.clear();
                empty = false;
            }

            //Adds the input node to dyn_nodes and dyn_points
            void add_node(int cl, SemanticOcTreeNode node, point3f point) {
                dyn_nodes_curr[cl].push_back(node);
                dyn_points_curr[cl].push_back(point);
            }

            bool isempty() {
                return empty;
            }

            //Returns the nearest node to the input point
            SemanticOcTreeNode *find_nearest(std::vector<int> classes, point3f point) {
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

                if (nearest_node <= 0) {
                    return nullptr;
                } 
                else {
                    return &dyn_nodes_prev[nearest_node_class][nearest_node];
                }
            }
    };

}