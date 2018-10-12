//
//  agent.hpp
//  MAMCTS
//
//  Created by Nick Zerbel on 5/25/17.
//  Copyright © 2017 Nicholas Zerbel. All rights reserved.
//

#ifndef agent_hpp
#define agent_hpp

#include <cstdlib>
#include <iostream>
#include <vector>
#include <assert.h>
#include <fstream>

using namespace std;

class agent{
public:
    double agent_x;
    double agent_y;
    
};

class goal{
public:
    double goal_x;
    double goal_y;
};

class multi_agent{
    friend class agent;
    friend class goal;
public:
    vector <agent> agent_vec;
    vector <agent> agent_start_pos;
    vector <agent> agent_list;
    vector <goal> goal_vec;
    vector <goal> goal_start_pos;
    vector <goal> goal_list;
    
    void create_config_list(int max_a, int xd, int yd, int nconfigs); //Create new goal and agent coordinates
    
    //Goal Functions
    void assign_goal_coordinates();
    void record_goal_list();
    void check_goal_coordinates(int n, double xc, double yc);
    void check_goal_status(int gn); //Checks if goal is still in play
    
    //Agent Functions
    void assign_agent_coordinates(int n);
    void record_agent_list();
    void check_agent_coordinates(int n, double x, double y);
    void agent_move(int n, int act); //Agent moves based on information collects from MCTS and Reward Evaluation
    void check_agent_status(int an);
    
    //Parameters
    int n_agents; //Number of agents and goals
    int xdim; //X dimension of Gridworld
    int ydim; //Y dimension of Gridworld
    bool unique_pos; //Detects if an agent or a goal has been stacked on another agent or goal
    bool goal_in_play; //Indicates if a goal has been captured or not
    bool agent_in_play; //Indicates if agent has captrueed a goal or not
};

#endif /* agent_hpp */
