//
//  main.cpp
//  CS_776_Project
//
//  Created by Nick Zerbel on 11/16/17.
//  Copyright © 2017 Nicholas Zerbel. All rights reserved.
//

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <assert.h>
#include "ea.hpp"
#include "agent.hpp"
#include "sim.hpp"
#include "tree.hpp"
#include "sim.hpp"

using namespace std;

void clear_txt_files(){ //Clear all txt files which get appended when running the code
    ofstream best_p, best_f;
    best_p.open("best_policy.txt", ios::out | ios::trunc);
    best_f.open("best_fit.txt", ios::out | ios::trunc);
    best_p.close(); best_f.close();
}

void record_best_policy(ea *eap, gridworld *gp){ //Records best policy and fitness of all time from each stat run
    ofstream best_p, best_f;
    if(gp->cred_eval == 1){
        best_p.open("GE_best_policy.txt", ios::app); best_f.open("GE_best_fit.txt", ios::app);
    }
    if(gp->cred_eval == 2){
        best_p.open("DE_best_policy.txt", ios::app); best_f.open("DE_best_fit.txt", ios::app);
    }
    for(int i = 0; i < eap->a_size; i++){
        best_p << eap->best_policy.at(i) << "\t";
    }
    best_p << "\n";
    best_f << eap->best_fit << "\n";
    best_p.close(); best_f.close();
}

void record_parameters(gridworld *gp, monte_carlo *mcp, ea *eap, int maxr, int na, int mg, int sr){
    ofstream tp; tp.open("test_parameters.txt");
    tp << "X-Dim: " << gp->x_dim << " Y-Dim: " << gp->y_dim << "\n";
    tp << "Max Runs: " << maxr << "\n";
    tp << "Number of Agents: " << na << "\n";
    tp << "Epsilon: " << mcp->epsilon << "\n";
    tp << "Rollout Steps: " << mcp->rollout_steps << "\n";
    tp << "Observational Distance: " << mcp->obs_dist << "\n";
    tp << "Monte Carlo Iterations: " << mcp->mc_iterations << "\n";
    tp << "Max Generation: " << mg << "\n";
    tp << "EA Stat Runs: " << sr << "\n";
    tp << "Population Size: " << eap->pop_size << "\n";
    tp << "Probability of Crossover: " << eap->p_cross << "\n";
    tp << "Probability of Mutation: " << eap->p_mut << "\n";
    tp << "Credit Type: " << gp->cred_eval << "\n";
    tp.close();
}

int main() {
    srand( time(NULL) );
    
    gridworld g; monte_carlo mcts; multi_tree t; multi_agent m; ea e;
    gridworld *gp = &g; monte_carlo *mcp = &mcts; multi_tree *tp = &t; multi_agent *map = &m; ea *eap = &e;
    
    //MCTS Parameters
    gp->cred_eval = 2; //1 = Global Eval and 2 = Diff Eval
    gp->x_dim = 4; //Maximum X Dimension
    gp->y_dim = 4; //Maximum Y Dimension
    int agent_increment = 1; //Increases the number of agents in a simulation by this amount
    int max_agents = 5; //Maximum number of agents to be tested
    int starting_agents = max_agents; //Initial number of agents being tested
    int n_groups = 1; //Number of agent groups being tested
    int stat_runs = 1; //Number of statistical runs for MCTS
    int max_run = 1000; //Cuts off the simulation if the number of iterations exceed this amount
    mcp->obs_dist = 5; //Observational Distance
    mcp->epsilon = 7; //UCB1 exploration constant (0 = greedy action selection)
    mcp->rollout_steps = 10; //Number of rollout steps during random rollout
    mcp->mc_iterations = 1; //Number of iterations of MCTS each agent performs during each episode
    gp->max_lev = gp->x_dim + gp->y_dim + 10;
    mcp->max_lev = gp->max_lev; //Tree cannot expand beyond this level
    
    //Rewards and Penalties
    g.goal_reward = 100; //Reward for reaching an unclaimed goal
    g.penalty = 100; //Penalty for reaching a claimed goal
    mcp->rollout_reward = 1;
    g.step_penalty = 1;
    
    //EA Parameters
    int ea_s_runs = 5; //Number of stat runs for the EA
    int max_gen = 50; //Maximum number of generations to evolve
    int n_grid_configs = 10; //Number of Gridworld configurations to test each generation
    eap->pop_size = 54; //Population size (must be even when divided by 3)
    eap->mu = eap->pop_size;
    eap->lambda = eap->mu;
    eap->p_cross = 0.95; //Probability of Crossover
    eap->p_mut = 0.9; //Probability of mutation
    eap->a_size = 3*mcp->rollout_steps; //Array Size
    eap->n_steps = mcp->rollout_steps; //Size of rollout policy is number of rollout steps taken each rollout
    record_parameters(gp, mcp, eap, max_run, max_agents, max_gen, ea_s_runs);
    
    //Fitness Variables
    double mr = (double)max_run; //Double version of maximum number of MCTS runs
    double ml = (double)gp->max_lev;
    double variants = (double)n_grid_configs; //Double version of number of testing configurations
    double n_successful, actual_runs, speed, reliability, optimality;
    clear_txt_files(); //Clear all txt files which get appended
    
    //Create Txt Files -------------------------------------------------------------------------------------------------
    ofstream max_fit, avg_fit, min_fit;
    if(gp->cred_eval == 1){
        max_fit.open("GE_max_fitness.txt"); avg_fit.open("GE_average_fitness.txt"); min_fit.open("GE_min_fitness.txt");
    }
    if(gp->cred_eval == 2){
        max_fit.open("DE_max_fitness.txt"); avg_fit.open("DE_average_fitness.txt"); min_fit.open("DE_min_fitness.txt");
    }
    
    for(int i = 0; i < n_groups; i++){
        gp->episodes.push_back(0); //For measuring speed and reliability
    }
    for(int i = 0; i < mcp->rollout_steps; i++){
        mcp->rollout_policy.push_back(0); 
    }
    map->create_config_list(max_agents, gp->x_dim, gp->y_dim, n_grid_configs);
    
    //Run GA ----------------------------------------------------------------------------------------------------------
    for(int srn = 0; srn < ea_s_runs; srn++){ //Stat Runs for GA
        eap->create_pop();
        for(int i = 0; i < max_gen; i++){
            if(i == 0){ //For the first gen only, the initial parent pop must be evaluated
                for(int j = 0; j < eap->mu; j++){ //Evaluate Population
                    for(int rp = 0; rp < mcp->rollout_steps; rp++){ //Assign rollout policy
                        eap->rollout_decode(3*rp, eap->pop.at(j).pol);
                        mcp->rollout_policy.at(rp) = (int)round(eap->num*(4/(pow(2,3)-1)));
                    }
                    eap->pfit_vec.at(0) = 0;
                    n_successful = variants; actual_runs = 0; optimality = 0;
                    for(int nc = 0; nc < n_grid_configs; nc++){
                        for(int na = 0; na < max_agents; na++){ //Assign Agent/Goal Positions from List
                            map->agent_start_pos.at(na).agent_x = map->agent_list.at(max_agents*nc + na).agent_x;
                            map->agent_start_pos.at(na).agent_y = map->agent_list.at(max_agents*nc + na).agent_y;
                            map->goal_start_pos.at(na).goal_x = map->goal_list.at(max_agents*nc + na).goal_x;
                            map->goal_start_pos.at(na).goal_y = map->goal_list.at(max_agents*nc + na).goal_y;
                        }
                        gp->run_mcts(map, mcp, tp, stat_runs, starting_agents, max_agents, max_run, agent_increment);
                        if(gp->episodes.at(0) >= mr){ //If number of MCTS episodes is equal to max runs, the run failed
                            n_successful--;
                        }
                        optimality += ml - gp->final_lev;
                        actual_runs += gp->episodes.at(0); //Total number of MCTS episodes run for each configuration
                    }
                    actual_runs /= variants; //Average number of runs
                    speed = (mr-actual_runs)/mr; //Percentage Speed
                    reliability = n_successful/variants; //Percentage Reliability
                    optimality /= variants; optimality /= ml;
                    eap->pfit_vec.at(j) = speed + reliability + optimality;
                }
            }
            
            //Create Offspring Pop
            eap->re_order();
            eap->calc_fit_prob();
            eap->crossover();
            eap->mutation();
        
            //Evaluate Offspring --------------------------------------------------------------------------------------------
            for(int j = 0; j < eap->lambda; j++){
                for(int rp = 0; rp < mcp->rollout_steps; rp++){ //Assign rollout policy
                    eap->rollout_decode(3*rp, eap->new_pop.at(j).pol);
                    mcp->rollout_policy.at(rp) = (int)round(eap->num*(4/(pow(2,3)-1)));
                }
                eap->ofit_vec.at(j) = 0;
                n_successful = variants; actual_runs = 0; optimality = 0;
                for(int nc = 0; nc < n_grid_configs; nc++){
                    for(int na = 0; na < max_agents; na++){
                        map->agent_start_pos.at(na).agent_x = map->agent_list.at(max_agents*nc + na).agent_x;
                        map->agent_start_pos.at(na).agent_y = map->agent_list.at(max_agents*nc + na).agent_y;
                        map->goal_start_pos.at(na).goal_x = map->goal_list.at(max_agents*nc + na).goal_x;
                        map->goal_start_pos.at(na).goal_y = map->goal_list.at(max_agents*nc + na).goal_y;
                    }
                    gp->run_mcts(map, mcp, tp, stat_runs, starting_agents, max_agents, max_run, agent_increment);
                    if(gp->episodes.at(0) >= mr){ //If the number of MCTS runs is greater or equal than the max runs
                        n_successful--; //The run is not successful
                    }
                    optimality += ml - gp->final_lev;
                    actual_runs += gp->episodes.at(0);
                }
                actual_runs /= variants; //Average number of runs
                speed = (mr-actual_runs)/mr; //Percentage Speed
                reliability = n_successful/variants; //Percentage Reliability
                optimality /= variants; optimality /= ml;
                eap->ofit_vec.at(j) = reliability + speed + optimality;
            }
            
            eap->create_new_pop();
            eap->calc_fit_prob();
        
            //Record Max, Avg, Min
            max_fit << eap->pfit_vec.at(0) << "\t";
            avg_fit << eap->fit_sum/eap->pop_size << "\t";
            min_fit << eap->pfit_vec.at(eap->pop_size-1) << "\t";
            if(eap->pfit_vec.at(0) > eap->best_fit){
                eap->best_fit = eap->pfit_vec.at(0);
                eap->best_policy = eap->pop.at(0).pol;
            }
        }
        record_best_policy(eap, gp);
        max_fit << endl; avg_fit << endl; min_fit << endl;
        eap->clear_vecs();
    }
    
    max_fit.close(); avg_fit.close(); min_fit.close();
    
    return 0;
}

