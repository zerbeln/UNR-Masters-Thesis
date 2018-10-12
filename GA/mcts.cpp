//
//  mcts.cpp
//  MAMCTS
//
//  Created by Nick Zerbel on 5/25/17.
//  Copyright © 2017 Nick Zerbel. All rights reserved.
//

#include "mcts.hpp"

void monte_carlo::set_mc_parameters(multi_tree *tp, int a){
    a_num = a;
    lev = 0;
    node_number = n_num_vec.at(a_num);
}

void monte_carlo::create_root_nodes(multi_tree *tp, multi_agent *map){
    for(int ag = 0; ag < map->n_agents; ag++){ //Create Root Nodes
        tp->create_tree(); //Create a tree for each agent
        tp->create_level(ag); //Create Root Level
        n_nodes = tp->ag_tree.at(ag).tree_vec.at(0).level_vec.size();
        tp->create_node(0, n_nodes, map->agent_vec.at(ag).agent_x, map->agent_vec.at(ag).agent_y, ag, node_number, 0, -1); //Root Node
    }
}

void monte_carlo::mc_search(multi_tree *tp, multi_agent *map){
    for(int exp = 0; exp < mc_iterations; exp++){ //exp = number of mcts sexpansions per learning episode
        select(tp);
        expand(tp);
        for(int en = 0; en < tp->ag_tree.at(a_num).tree_vec.at(lev).level_vec.size(); en++){ //Rollout newly expanded nodes
            if(parent_number == tp->ag_tree.at(a_num).tree_vec.at(lev).level_vec.at(en).p_number){
                rollout(tp, map, en);
            }
        }
        back_propagate(tp);
    }
}

int monte_carlo::select_move(multi_tree *tp, int agn, int l){ //(agent number, level)
    int count; count = 0;
    double best; //Tracks best Q-value for action selection
    action_check = false;
    assert(tp->ag_tree.at(agn).tree_vec.size() > l);
    
    reward_vec.clear();
    for(int i = 0; i < tp->ag_tree.at(agn).tree_vec.at(l).level_vec.size(); i++){
        if(parent_number == tp->ag_tree.at(agn).tree_vec.at(l).level_vec.at(i).p_number){
            reward_vec.push_back(tp->ag_tree.at(agn).tree_vec.at(l).level_vec.at(i).UCB1);
            action_check = true;
        }
    }
    
    if(action_check == false){
        current_node = parent_number;
        action = 4; //Stay in position (just in case)
        goto skip;
    }
    
    best = *max_element(reward_vec.begin(), reward_vec.end()); //Best Q-value from among child nodes in level
    reward_vec.clear();
    
    for(int i = 0; i < tp->ag_tree.at(agn).tree_vec.at(l).level_vec.size(); i++){
        if(parent_number == tp->ag_tree.at(agn).tree_vec.at(l).level_vec.at(i).p_number){
            if(best == tp->ag_tree.at(agn).tree_vec.at(l).level_vec.at(i).UCB1){
                action = tp->ag_tree.at(agn).tree_vec.at(l).level_vec.at(i).action;
                current_node = tp->ag_tree.at(agn).tree_vec.at(l).level_vec.at(i).n_number; //Selected node
                count++;
                break;
            }
        }
    }
    assert(count == 1);
skip:
    assert(action >= 0);
    assert(action <= 4);
    return action;
}

//SELECTION------------------------------------------------------------------------------------------------------------
void monte_carlo::select(multi_tree *tp){
    double best, q_val; bool no_nodes;
    previous_x = tp->ag_tree.at(a_num).tree_vec.at(0).level_vec.at(0).x; //Set Coordinates to Root Position
    previous_y = tp->ag_tree.at(a_num).tree_vec.at(0).level_vec.at(0).y;
    parent_number = tp->ag_tree.at(a_num).tree_vec.at(0).level_vec.at(0).n_number;
    parent_visit = tp->ag_tree.at(a_num).tree_vec.at(0).level_vec.at(0).visit_count;
    
    if(tp->ag_tree.at(a_num).tree_vec.size() == 1){ //If there is only the root level, go immediately to expansion
        p_lev = 0;
        lev = 1;
        goto expansion_phase;
    }
    
    reward_vec.clear();
    for(int i = 1; i < tp->ag_tree.at(a_num).tree_vec.size(); i++){ //Starting at level 1 record all q values of child nodes
        no_nodes = true; //Boolean used to find when we reach an unexpanded node
        for(int j = 0; j < tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.size(); j++){ //For all nodes in the current level
            if(parent_number == tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(j).p_number){ //If it is a child node
                node_visit = tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(j).visit_count;
                q_val = tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(j).q_node; //Current Q-value of node
                tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(j).UCB1 = q_val + (epsilon*sqrt(log(parent_visit)/node_visit));
                reward_vec.push_back(tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(j).UCB1);
                no_nodes = false; //If a child node was found, the current parent node is expanded
            }
        }
        
        if(no_nodes == true){
            lev = i; //If there are no children of the current parent in this level, then the parent needs to be expanded
            goto expansion_phase;
        }
        
        best = *max_element(reward_vec.begin(), reward_vec.end()); //Pick the highest Q-value from among child nodes found
        reward_vec.clear();
        for(int k = 0; k < tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.size(); k++){
            if(best == tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(k).UCB1 && parent_number == tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(k).p_number){
                parent_visit = tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(k).visit_count;
                parent_number = tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(k).n_number; //New parent node
                previous_x = tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(k).x;
                previous_y = tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(k).y;
                lev = i + 1; //Needed in case selection reaches the bottom most level of the tree
                p_lev = i;
                break;
            }
        }
    }
    
expansion_phase:
    assert(p_lev == lev-1);
    assert(lev <= tp->ag_tree.at(a_num).tree_vec.size());
}

//EXPANSION---------------------------------------------------------------------------------------------------------------
void monte_carlo::expand(multi_tree *tp){
    if(lev == tp->ag_tree.at(a_num).tree_vec.size()){
        tp->create_level(a_num);
    }
    move_left(tp); //Explores Left Action (x--)
    move_up(tp); //Explores Up Action (y++)
    move_down(tp); //Explores Down Action (y--)
    move_right(tp); //Explores Right Action (x++)
    no_move(tp);
}

void monte_carlo::move_left(multi_tree *tp){
    reset_coordinates();
    ax--;
    pruning(tp);
    if(action_check == true){
        update_node_numbers(tp);
        tp->create_node(lev, n_nodes, ax, ay, a_num, node_number, parent_number, 0);
    }
}

void monte_carlo::move_up(multi_tree *tp){
    reset_coordinates();
    ay++;
    pruning(tp);
    if(action_check == true){
        update_node_numbers(tp);
        tp->create_node(lev, n_nodes, ax, ay, a_num, node_number, parent_number, 1);
    }
}

void monte_carlo::move_down(multi_tree *tp){
    reset_coordinates();
    ay--;
    pruning(tp);
    if(action_check == true){
        update_node_numbers(tp);
        tp->create_node(lev, n_nodes, ax, ay, a_num, node_number, parent_number, 2);
    }
}

void monte_carlo::move_right(multi_tree *tp){
    reset_coordinates();
    ax++;
    pruning(tp);
    if(action_check == true){
        update_node_numbers(tp);
        tp->create_node(lev, n_nodes, ax, ay, a_num, node_number, parent_number, 3);
    }
}

void monte_carlo::no_move(multi_tree *tp){
    reset_coordinates();
    check_boundaries(ax, ay);
    if(lev > max_lev){
        action_check = false;
    }
    prune(tp, lev-5);
    if(action_check == true){
        update_node_numbers(tp);
        tp->create_node(lev, n_nodes, ax, ay, a_num, node_number, parent_number, 4);
    }
}

void monte_carlo::pruning(multi_tree *tp){
    check_boundaries(ax, ay); //Agent cannot go out of bounds
    if(lev > max_lev){
        action_check = false;
    }
    if(action_check == true){
        prune(tp, lev); //Agent cannot revisit a previous state during a MCTS simulation
    }
}

void monte_carlo::check_boundaries(double xx, double yy){
    action_check = true;
    if(xx >= x_lim){
        action_check = false;
    }
    if(xx < 0){
        action_check = false;
    }
    if(yy >= y_lim){
        action_check = false;
    }
    if(yy < 0){
        action_check = false;
    }
}

void monte_carlo::prune(multi_tree *tp, int l){
    double prune_x, prune_y;
    for(int i = l-1; i >= 0; i--){ //Starting from the parent level, make sure no duplicate states are already in the tree
        for(int j = 0; j < tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.size(); j++){
            prune_x = tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(j).x;
            prune_y = tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(j).y;
            if(ax == prune_x && ay == prune_y){ //If a state has been visited before, prune the tree
                action_check = false;
                break;
            }
        }
        if(action_check == false){
            break;
        }
    }
}

void monte_carlo::update_node_numbers(multi_tree *tp){
    node_number++;
    n_nodes = tp->ag_tree.at(a_num).tree_vec.at(lev).level_vec.size();
}

void monte_carlo::reset_coordinates(){ //Reset agent coordinates to parent node state
    ax = previous_x;
    ay = previous_y;
}

//SIMULATION-----------------------------------------------------------------------------------------------------------------
void monte_carlo::rollout(multi_tree *tp, multi_agent *map, int n){
    double q_val; q_val = 0; int act;
    double dist, x, y, xi, yi, x_root, y_root;
    x = tp->ag_tree.at(a_num).tree_vec.at(lev).level_vec.at(n).x;
    y = tp->ag_tree.at(a_num).tree_vec.at(lev).level_vec.at(n).y;
    xi = x; yi = y;
    x_root = tp->ag_tree.at(a_num).tree_vec.at(lev).level_vec.at(0).x; //Starting Position
    y_root = tp->ag_tree.at(a_num).tree_vec.at(lev).level_vec.at(0).y; //Starting Position
    
    for(int i = 0; i < rollout_steps; i++){
        act = rollout_policy.at(i);
        if(act == 0){
            x--;
            check_boundaries(x, y);
            dist = abs(x-xi) + abs(y-yi);
            if(action_check == false || dist > obs_dist){ 
                x++;
            }
        }
        if(act == 1){
            y++;
            check_boundaries(x, y);
            dist = abs(x-xi) + abs(y-yi);
            if(action_check == false || dist > obs_dist){
                y--;
            }
        }
        if(act == 2){
            y--;
            check_boundaries(x, y);
            dist = abs(x-xi) + abs(y-yi);
            if(action_check == false || dist > obs_dist){
                y++;
            }
        }
        if(act == 3){
            x++;
            check_boundaries(x, y);
            dist = abs(x-xi) + abs(y-yi);
            if(action_check == false || dist > obs_dist){
                x--;
            }
        }
        
        for(int j = 0; j < map->n_agents; j++){
            if(x == map->goal_vec.at(j).goal_x && y == map->goal_vec.at(j).goal_y){
                q_val += rollout_reward;
            }
        }
    }
    tp->ag_tree.at(a_num).tree_vec.at(lev).level_vec.at(n).q_node = q_val;
}

//BACK-PROPAGATION-----------------------------------------------------------------------------------------------------------
void monte_carlo::back_propagate(multi_tree *tp){
    double n; n = 0;
    double q_val, q_prev;
    
    reward_vec.clear();
    
    for(int i = 0; i < tp->ag_tree.at(a_num).tree_vec.at(lev).level_vec.size(); i++){
        if(parent_number == tp->ag_tree.at(a_num).tree_vec.at(lev).level_vec.at(i).p_number){
            reward_vec.push_back(tp->ag_tree.at(a_num).tree_vec.at(lev).level_vec.at(i).q_node);
            n++;
        }
    }
    
    if(n > 0){
        q_val = *max_element(reward_vec.begin(), reward_vec.end()); //Average across all newly expanded nodes
    }
    if(n == 0){
        q_val = 0;
    }
    
    //Back-Propagate
    for(int i = lev-1; i >= 0; i--){ //i = level
        for(int j = 0; j < tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.size(); j++){ //j = node
            if(parent_number == tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(j).n_number){
                q_prev = tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(j).q_node;
                tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(j).visit_count += 1;
                node_visit = tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(j).visit_count;
                tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(j).q_node = q_prev + ((q_val-q_prev)/node_visit);
                parent_number = tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(j).p_number;
                break;
            }
        }
        
        n = 0;
        reward_vec.clear();
        for(int tn = 0; tn < tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.size(); tn++){
            if(parent_number == tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(tn).p_number){
                reward_vec.push_back(tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(tn).q_node);
                n++;
            }
        }
        assert(n > 0);
        q_val = *max_element(reward_vec.begin(), reward_vec.end()); //A parent's value is the average of its children's action values
        reward_vec.clear();
    }
}

void monte_carlo::back_propagate_evals(multi_agent *map, multi_tree *tp, double reward, int agn, int l, int nn){
    double q_val, q_prev; int count;
    //count = count for number of nodes, nv = node visit count
    
    count = 0;
    for(int i = 0; i < tp->ag_tree.at(agn).tree_vec.at(l).level_vec.size(); i++){ //Update current node value
        if(nn == tp->ag_tree.at(agn).tree_vec.at(l).level_vec.at(i).n_number){
            q_val = reward;
            q_prev = tp->ag_tree.at(agn).tree_vec.at(l).level_vec.at(i).q_node;
            tp->ag_tree.at(agn).tree_vec.at(l).level_vec.at(i).visit_count += 1;
            node_visit = tp->ag_tree.at(agn).tree_vec.at(l).level_vec.at(i).visit_count;
            tp->ag_tree.at(agn).tree_vec.at(l).level_vec.at(i).q_node = q_prev + ((q_val-q_prev)/node_visit);
            parent_number = tp->ag_tree.at(agn).tree_vec.at(l).level_vec.at(i).p_number;
            count++;
            break;
        }
    }
    assert(count > 0);
    
    count = 0;
    reward_vec.clear();
    for(int tn = 0; tn < tp->ag_tree.at(agn).tree_vec.at(l).level_vec.size(); tn++){
        if(parent_number == tp->ag_tree.at(agn).tree_vec.at(l).level_vec.at(tn).p_number){
            reward_vec.push_back(tp->ag_tree.at(agn).tree_vec.at(l).level_vec.at(tn).q_node);
            count++;
        }
    }
    assert(count > 0);
    q_val = *max_element(reward_vec.begin(), reward_vec.end());
    
    for(int i = l-1; i >= 0; i--){ //i = level
        for(int j = 0; j < tp->ag_tree.at(agn).tree_vec.at(i).level_vec.size(); j++){ //j = node
            if(parent_number == tp->ag_tree.at(agn).tree_vec.at(i).level_vec.at(j).n_number){
                q_prev = tp->ag_tree.at(agn).tree_vec.at(i).level_vec.at(j).q_node;
                tp->ag_tree.at(agn).tree_vec.at(i).level_vec.at(j).visit_count += 1;
                node_visit = tp->ag_tree.at(agn).tree_vec.at(i).level_vec.at(j).visit_count;
                tp->ag_tree.at(agn).tree_vec.at(i).level_vec.at(j).q_node = q_prev + ((q_val-q_prev)/node_visit);
                parent_number = tp->ag_tree.at(agn).tree_vec.at(i).level_vec.at(j).p_number;
                break;
            }
        }
        
        count = 0;
        reward_vec.clear();
        for(int tn = 0; tn < tp->ag_tree.at(agn).tree_vec.at(i).level_vec.size(); tn++){
            if(parent_number == tp->ag_tree.at(agn).tree_vec.at(i).level_vec.at(tn).p_number){
                reward_vec.push_back(tp->ag_tree.at(agn).tree_vec.at(i).level_vec.at(tn).q_node);
                count++;
            }
        }
        assert(count > 0);
        q_val = *max_element(reward_vec.begin(), reward_vec.end());
    }
}


