//
//  main.cpp
//  MAMCTS
//
//  Created by Nicholas Zerbel on 11/16/17.
//  Copyright © 2017 Nicholas Zerbel. All rights reserved.
//

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <assert.h>
#include "agent.hpp"
#include "sim.hpp"
#include "tree.hpp"
#include "sim.hpp"

using namespace std;

void import_GE_policies(gridworld *gp, monte_carlo *mcp, int asize, int sr){
    double best;
    vector <int> pol; vector < vector <int> > pop;
    vector <double> fit_vec;
    ifstream bfit("GE_best_fitnesses.txt"); ifstream bpol("GE_policies.txt"); //Import txt files from GA
    for(int i = 0; i < asize; i++){
        pol.push_back(-1);
    }
    for(int i = 0; i < sr; i++){
        fit_vec.push_back(0);
        bfit >> fit_vec.at(i);
        for(int j = 0; j < asize; j++){
            bpol >> pol.at(j);
        }
        pop.push_back(pol);
    }
    best = *max_element(fit_vec.begin(), fit_vec.end()); //Only run the policy which had the highest fitness out of those recorded
    cout << "Best GE Fitness: " << best << endl;
    for(int i = 0; i < sr; i++){
        if(best == fit_vec.at(i)){
            mcp->assign_rollout_policy(pop.at(i)); //Assign Rollout Policy
            break;
        }
    }
}

void import_DE_policies(gridworld *gp, monte_carlo *mcp, int asize, int sr){
    double best;
    vector <int> pol; vector < vector <int> > pop;
    vector <double> fit_vec;
    ifstream bfit("DE_best_fitnesses.txt"); ifstream bpol("DE_policies.txt"); //Import txt files from GA
    for(int i = 0; i < asize; i++){
        pol.push_back(-1);
    }
    for(int i = 0; i < sr; i++){
        fit_vec.push_back(0);
        bfit >> fit_vec.at(i);
        for(int j = 0; j < asize; j++){
            bpol >> pol.at(j);
        }
        pop.push_back(pol);
    }
    best = *max_element(fit_vec.begin(), fit_vec.end()); //Only run the policy which had the highest fitness out of those recorded
    cout << "Best DE Fitness: " << best << endl;
    for(int i = 0; i < sr; i++){
        if(best == fit_vec.at(i)){
            mcp->assign_rollout_policy(pop.at(i)); //Assign Rollout Policy
            break;
        }
    }
}

int main() {
    srand( time(NULL) );
    
    gridworld g; monte_carlo mcts; multi_tree t; multi_agent m;
    gridworld *gp = &g; monte_carlo *mcp = &mcts; multi_tree *tp = &t; multi_agent *map = &m;
    
    //Create Txt Files For Data Output
    ofstream DE_Def, DE_GA, DE_def_steps, DE_GA_steps, GE_Def, GE_GA, GE_def_steps, GE_GA_steps;
    DE_Def.open("DE_Default_Data.txt"); DE_GA.open("DE_GA_Data.txt"); //Outputs number of runs it takes MCTS to find a solution
    DE_def_steps.open("DE_Default_Steps.txt"); DE_GA_steps.open("DE_GA_Steps.txt"); //Outputs how many steps agents take to arrive at goals
    GE_Def.open("GE_Default_Data.txt"); GE_GA.open("GE_GA_Data.txt"); //Outputs number of runs it takes MCTS to find a solution
    GE_def_steps.open("GE_Default_Steps.txt"); GE_GA_steps.open("GE_GA_steps.txt"); //Outputs how many steos agents take to arrive at goals
    
    //Testing Parameters
    int stat_runs = 30; //Number of statistical runs
    int max_run = 1000; //Cuts off the simulation if the number of iterations exceed this amount
    int agent_increment = 1; //Increases the number of agents in a simulation by this amount
    int starting_agents = 2; //Initial number of agents being tested
    int max_agents = 5; //Maximum number of agents to be tested
    g.x_dim = 5; //Maximum X Dimension
    g.y_dim = 5; //Maximum Y Dimension
    mcp->epsilon = 7; //UCB1 exploration constant (0 = greedy action selection)
    mcp->obs_dist = 5; //The observable distance
    mcp->rollout_steps = 10;//Number of rollout moves
    gp->max_lev = g.x_dim + g.y_dim + 10;
    mcp->max_lev = g.x_dim + g.y_dim + 10;
    
    //Rewards and Penalties
    g.goal_reward = 100; //Reward for reaching an unclaimed goal
    g.penalty = 100; //Penalty for reaching a claimed goal
    mcp->rollout_reward = 1;
    g.step_penalty = 1;
    
    //Import policies from GA
    int array_size = mcp->rollout_steps*3;
    int ga_sruns = 30; //Number of stat runs the GA was run for to train rollout policies
    
    //Random Default Policy --------------------------------------------------------------------------------------------------------------------------------------
    mcp->rand_rollout = true; //If false, the GA policy is used
    map->create_config_list(max_agents, g.x_dim, g.y_dim, stat_runs);
    for(int c = 2; c < 4; c++){ //1 = local, 2 = global, 3 = difference
        gp->credit_type = c;
        for(gp->n_agents = starting_agents; gp->n_agents <= max_agents;){
            for(int s = 0; s < stat_runs; s++){
                map->create_start_vecs(s, gp->n_agents, max_agents);
                gp->initialize_parameters(map, mcp);
                mcp->create_root_nodes(tp, map);
                while(gp->gridworld_finished == false){
                    gp->learn_its++; //Tracks the number of learning episodes until MCTS find a solution
                    for(int anum = 0; anum < gp->n_agents; anum++){ //anum = agent number
                        mcp->set_mc_parameters(tp, anum);
                        mcp->mc_search(tp, map); //Runs MCTS for defined number of expansions
                        mcp->n_num_vec.at(anum) = mcp->node_number; //Used to track what the current node number is in each tree
                    }
                    gp->cred_evals(map, tp, mcp);
                    
                    //Check to see if agents have arrived at goals
                    gp->system_rollout(map, tp, mcp);
                    gp->check_goal_conditions(map);
                    if(gp->goal_check == true){
                        gp->gridworld_finished = true;
                    }
                    if(gp->gridworld_finished == false){
                        gp->reset_all_agents(map, tp);
                    }
                    if(gp->learn_its >= max_run){ //If the number of learning iterations is greater or equal to the max, the run has failed
                        break;
                    }
                }
                //Record Information
                if(c == 2){
                    GE_Def << g.learn_its << "\t";
                    if(gp->gridworld_finished == false){
                        g.final_lev = g.max_lev;
                    }
                    GE_def_steps << g.final_lev << "\t";
                }
                if(c == 3){
                    DE_Def << g.learn_its << "\t";
                    if(gp->gridworld_finished == false){
                        g.final_lev = g.max_lev;
                    }
                    DE_def_steps << g.final_lev << "\t";
                }
                g.clear_all_vectors(map, mcp, tp);
            }
            g.n_agents += agent_increment;
            if(c == 2){
                GE_Def << endl;
                GE_def_steps << endl;
            }
            if(c ==3){
                DE_Def << endl;
                DE_def_steps << endl;
            }
        }
    }
    GE_Def.close(); DE_Def.close(); GE_def_steps.close(); DE_def_steps.close();
    
    //GA Evolved Policy ----------------------------------------------------------------------------------------------------------------------------------------
    mcp->rand_rollout = false;
    for(int c = 2; c < 4; c++){ //1 = local, 2 = global, 3 = difference
        gp->credit_type = c;
        if(c == 2){
            mcp->rollout_policy.clear();
            import_GE_policies(gp, mcp, array_size, ga_sruns);
        }
        if(c == 3){
            mcp->rollout_policy.clear();
            import_DE_policies(gp, mcp, array_size, ga_sruns);
        }
        
        for(gp->n_agents = starting_agents; gp->n_agents <= max_agents;){
            for(int s = 0; s < stat_runs; s++){
                map->create_start_vecs(s, gp->n_agents, max_agents);
                gp->initialize_parameters(map, mcp);
                mcp->create_root_nodes(tp, map);
                while(gp->gridworld_finished == false){
                    gp->learn_its++; //Tracks the number of learning episodes until MCTS find a solution
                    
                    for(int anum = 0; anum < gp->n_agents; anum++){ //anum = agent number
                        mcp->set_mc_parameters(tp, anum);
                        mcp->mc_search(tp, map); //Runs MCTS for defined number of expansions
                        mcp->n_num_vec.at(anum) = mcp->node_number; //Used to track what the current node number is in each tree
                    }
                    gp->cred_evals(map, tp, mcp);
                    
                    //Check to see if agents have arrived at goals
                    gp->system_rollout(map, tp, mcp);
                    gp->check_goal_conditions(map);
                    if(gp->goal_check == true){
                        gp->gridworld_finished = true;
                    }
                    if(gp->gridworld_finished == false){
                        gp->reset_all_agents(map, tp);
                    }
                    if(gp->learn_its >= max_run){ //If the number of learning iterations is greater or equal to the max, the run has failed
                        break;
                    }
                }
                //Record Information
                if(c == 2){
                    GE_GA << g.learn_its << "\t";
                    if(gp->gridworld_finished == false){
                        g.final_lev = g.max_lev;
                    }
                    GE_GA_steps << g.final_lev << "\t";
                }
                if(c == 3){
                    DE_GA << g.learn_its << "\t";
                    if(gp->gridworld_finished == false){
                        g.final_lev = g.max_lev;
                    }
                    DE_GA_steps << g.final_lev << "\t";
                }
                g.clear_all_vectors(map, mcp, tp);
            }
            g.n_agents += agent_increment;
            if(c == 2){
                GE_GA << endl;
                GE_GA_steps << endl;
            }
            if(c ==3){
                DE_GA << endl;
                DE_GA_steps << endl;
            }
        }
    }
    DE_GA.close(); GE_GA.close(); GE_GA_steps.close(); DE_GA_steps.close();
    
    return 0;
}
