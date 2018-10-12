//
//  mcts.hpp
//  MAMCTS
//
//  Created by Nick Zerbel on 5/25/17.
//  Copyright © 2017 Nicholas Zerbel. All rights reserved.
//

#ifndef mcts_hpp
#define mcts_hpp

#include <cstdlib>
#include <iostream>
#include <vector>
#include <cmath>
#include <assert.h>
#include <algorithm>
#include "tree.hpp"
#include "agent.hpp"


using namespace std;

class monte_carlo{
public:
    vector <int> rollout_policy;
    
    void set_mc_parameters(multi_tree *tp, int a);
    void create_root_nodes(multi_tree *tp, multi_agent *map);
    void mc_search(multi_tree *tp, multi_agent *map);
    
    //SELECTION
    void select(multi_tree *tp); //Choose an action
    int select_move(multi_tree *tp, int agn, int l); //Choose move which the agent will take after MCTS simulation
    
    //EXPANSION
    void expand(multi_tree *tp);
    void move_left(multi_tree *tp);
    void move_right(multi_tree *tp);
    void move_up(multi_tree *tp);
    void move_down(multi_tree *tp);
    void no_move(multi_tree *tp);
    void update_node_numbers(multi_tree *tp);
    void pruning(multi_tree *tp);
    void check_boundaries(double xx, double yy); //Agent cannot move outside Gridworld
    void prune(multi_tree *tp, int l); //Agent cannot re-visit a state when expanding the tree
    void reset_coordinates();
    
    //SIMULATION
    int select_node(multi_tree *tp); //Select an expanded node for rollout
    void rollout(multi_tree *tp, multi_agent *map, int n);
    
    //BACK-PROPAGATION
    void back_propagate(multi_tree *tp);
    void back_propagate_evals(multi_agent *map, multi_tree *tp, double reward, int agn, int l, int nn);
    
    //Parameters
    vector <double> reward_vec;
    vector <int> n_num_vec;
    bool action_check; //Flags possible actions as valid or invalid
    double parent_visit; //Number of times parent has been visited
    double node_visit; //Number of times child has been visited
    int max_lev;
    int lev; //Current level of the tree
    int p_lev; //Current parent level
    int n_nodes; //Number of nodes currently in a level in the tree
    int node_number;
    int parent_number;
    int current_node;
    int a_num; //Designates which agent is currently simulating
    int n_agents; //Number of agents and goals
    int action; //Designates which action should be taken in Gridworld
    
    //Experimental Parameters
    int rollout_steps; //Number of rollout steps
    double obs_dist;
    double epsilon; //Exploration vs Exploitation parameter for UCB1
    int mc_iterations; //Number of level expansions MCTS does before running a credit eval
    double rollout_reward; //Reward given when goal is found in rollout
    
    //Coordinates
    double ax; //Current agent x coordinate
    double ay; //Current agent y coordinate
    double previous_x; //Previous agent x coordinate
    double previous_y; //Previous agent y coordinate
    int x_lim; //Maximum x dimension
    int y_lim; //Maximum y dimension
};

#endif /* mcts_hpp */
