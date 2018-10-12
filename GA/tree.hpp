//
//  tree.hpp
//  MAMCTS
//
//  Created by Nick Zerbel on 5/25/17.
//  Copyright © 2017 Nicholas Zerbel. All rights reserved.
//

#ifndef tree_hpp
#define tree_hpp

#include <cstdlib>
#include <vector>
#include <fstream>
#include <assert.h>

using namespace std;

class node{
public:
    double x; //X Coordinate of state in Gridworld
    double y; //Y Coordinate of state in Gridworld
    int a_number; //Agent Number
    int n_number; //Node Number
    int p_number; //Parent Number
    double UCB1; //UCB1 value of the node
    double q_node; //MCTS action value of the node
    int action; //Designates which action is taken from the parent to access this state
    double visit_count; //Number of times a particular action has been taken from a parent node
};

class level{
    friend class node;
public:
    vector <node> level_vec; //A level is a vector of nodes
    
};

class tree{
    friend class level;
public:
    vector <level> tree_vec; //A tree is a vector of levels

};

class multi_tree{
    friend class tree;
public:
    vector <tree> ag_tree; //Vector of trees (one for each agent)
    void create_tree();
    void create_level(int agn);
    void create_node(int lev, int pos, double agx, double agy, int ag_num, int node_num, int p_num, int a);
};

#endif /* tree_hpp */
