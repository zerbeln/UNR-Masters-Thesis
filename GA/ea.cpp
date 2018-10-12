//
//  ea.cpp
//  CS_776_Project
//
//  Created by Nick Zerbel on 11/16/17.
//  Copyright © 2017 Nicholas Zerbel. All rights reserved.
//

#include "ea.hpp"

void ea::create_pop(){
    policy p; int r;
    
    for(int i = 0; i < pop_size; i++){ //Parent population and fitness vectors
        pop.push_back(p);
        pfit_vec.push_back(0);
        ofit_vec.push_back(0);
        fit_prob.push_back(0);
    }
    for(int i = 0; i < (mu+lambda); i++){ //Combined parent and offspring population
        combined_pop.push_back(p);
        combined_fit.push_back(0);
        for(int j = 0; j < a_size; j++){
            combined_pop.at(i).pol.push_back(0);
        }
    }
    for(int i = 0; i < pop_size; i++){ //Randomly initialize parent population
        for(int j = 0; j < a_size; j++){
            r = rand() % 2;
            pop.at(i).pol.push_back(r);
        }
    }
    new_pop = pop;
    best_fit = 0;
    best_policy = pop.at(0).pol;
}

void ea::rollout_decode(int p, vector<int> invec){ //Decoder for rollout actions
    num = 0;
    for(int i = 0; i < 3; i++){
        num += invec.at(p+i)*pow(2,i);
    }
}

void ea::re_order(){
    for(int i = 0; i < mu; i++){
        for(int j = 0; j < mu; j++){
            if(i < j){
                if(pfit_vec.at(j) > pfit_vec.at(i)){
                    iter_swap(pfit_vec.begin() + i, pfit_vec.begin() + j);
                    iter_swap(pop.begin() + i, pop.begin() + j);
                }
            }
        }
    }
}

void ea::calc_fit_prob(){
    fit_sum = 0; double p = (double)pop_size;
    for(int i = 0; i < mu; i++){
        fit_sum += pfit_vec.at(i);
        fit_prob.at(i) = 0;
    }
    if(fit_sum > 0){
        for(int i = 0; i < mu; i++){
            if(i == 0){
                fit_prob.at(i) = pfit_vec.at(i)/fit_sum;
            }
            if(i > 0){
                fit_prob.at(i) = fit_prob.at(i-1) + (pfit_vec.at(i)/fit_sum);
            }
        }
    }
    if(fit_sum == 0){ //If fitnesses are all 0, then probabilities are equal
        for(int i = 0; i < mu; i++){
            if(i == 0){
                fit_prob.at(i) = 1/p;
            }
            if(i > 0){
                fit_prob.at(i) = fit_prob.at(i-1) + (1/p);
            }
        }
    }
}

int ea::select_parent(){
    double r; int p;
    r = (double)(rand())/RAND_MAX;
    for(int i = 0; i < pop_size; i++){
        if(i == 0){
            if(0 <= r && r < fit_prob.at(i)){
                p = i;
                break;
            }
        }
        if(i > 0){
            if(fit_prob.at(i-1) <= r && r < fit_prob.at(i)){
                p = i;
                break;
            }
        }
    }
    assert(0 <= p && p < pop_size);
    return p;
}

void ea::crossover(){
    double prob; int p1, p2, p3, cp1, cp2;
    for(int i = 0; i < lambda;){ //Replace bottom half of population with offspring from top half
        prob = (double)(rand())/RAND_MAX;
        p1 = select_parent(); //Parent 1
        p2 = select_parent(); //Parent 2
        p3 = select_parent(); //Parent 3
        if(prob <= p_cross){ //If the probability lands between 0 and p_cross, do crossover
            cp1 = (rand() % (a_size-10)) + 1; //Crossover Point 1
            cp2 = (rand() % (a_size-2)) + 1; //Crossover Point 2
            while(cp1 >= cp2){
                cp2 = (rand() % (a_size-2)) + 1; //Crossover Point 2
            }
            for(int j = 0; j < cp1; j++){
                new_pop.at(i).pol.at(j) = pop.at(p1).pol.at(j);
                new_pop.at(i+1).pol.at(j) = pop.at(p2).pol.at(j);
                new_pop.at(i+2).pol.at(j) = pop.at(p3).pol.at(j);
            }
            for(int j = cp1; j < cp2; j++){
                new_pop.at(i).pol.at(j) = pop.at(p2).pol.at(j);
                new_pop.at(i+1).pol.at(j) = pop.at(p3).pol.at(j);
                new_pop.at(i+2).pol.at(j) = pop.at(p1).pol.at(j);
            }
            for(int j = cp2; j < a_size; j++){
                new_pop.at(i).pol.at(j) = pop.at(p3).pol.at(j);
                new_pop.at(i+1).pol.at(j) = pop.at(p1).pol.at(j);
                new_pop.at(i+2).pol.at(j) = pop.at(p2).pol.at(j);
            }
        }
        if(prob > p_cross){
            new_pop.at(i) = pop.at(p1);
            new_pop.at(i+1) = pop.at(p2);
            new_pop.at(i+2) = pop.at(p3);
        }
        i += 3;
    }
    assert(new_pop.size() == pop_size);
}

void ea::mutation(){ //Mutate offspring population
    double prob; int b;
    for(int i = 0; i < lambda; i++){
        prob = (double)(rand())/RAND_MAX;
        if(prob <= p_mut){
            b = rand() % a_size; //Which element gets mutated
            if(new_pop.at(i).pol.at(b) == 0){
                new_pop.at(i).pol.at(b) = 1;
            }
            if(new_pop.at(i).pol.at(b) == 1){
                new_pop.at(i).pol.at(b) = 0;
            }
        }
    }
}

void ea::create_new_pop(){
    for(int i = 0; i < mu; i++){ //Parent Population
        combined_pop.at(i) = pop.at(i);
        combined_fit.at(i) = pfit_vec.at(i);
    }
    for(int i = 0; i < lambda; i++){ //Offspring Population
        combined_pop.at(mu + i) = new_pop.at(i);
        combined_fit.at(mu + i) = ofit_vec.at(i);
    }
    assert(combined_pop.size() == (mu+lambda));
    assert(combined_fit.size() == (mu+lambda));
    
    for(int i = 0; i < (mu+lambda); i++){ //Re-order combined population
        for(int j = 0; j < (mu+lambda); j++){
            if(i < j){
                if(combined_fit.at(j) > combined_fit.at(i)){
                    iter_swap(combined_fit.begin() + i, combined_fit.begin() + j);
                    iter_swap(combined_pop.begin() + i, combined_pop.begin() + j);
                }
            }
        }
    }
    for(int i = 0; i < mu; i++){ //Top half becomes new population
        pop.at(i) = combined_pop.at(i);
        pfit_vec.at(i) = combined_fit.at(i);
    }
    assert(pop.size() == mu);
}

void ea::clear_vecs(){
    pop.clear();
    new_pop.clear();
    pfit_vec.clear();
    ofit_vec.clear();
    fit_prob.clear();
    combined_pop.clear();
    combined_fit.clear();
}
