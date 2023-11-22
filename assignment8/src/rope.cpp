#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.

//        Comment-in this part when you implement the constructor
//        for (auto &i : pinned_nodes) {
//            masses[i]->pinned = true;
//        }
        Vector2D interval = (end - start)/num_nodes;
        for(int i=0;i<num_nodes;i++){
            masses.push_back(new Mass(start + interval*i,node_mass,false));
        }
        for(int i = 0;i<num_nodes-1;i++){
            springs.push_back(new Spring(masses[i],masses[i+1],k));
        }
        for(auto i:pinned_nodes){
            masses[i]->pinned = true;
        }

    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {

        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            //m2 指向m1
            Vector2D m1_to_m2_vector = s->m2->position - s->m1->position;
            // 计算其变形程度
            double length = m1_to_m2_vector.norm();
            double distort = length-s->rest_length;
            //计算弹簧力 这里取反了 是表示形变程度为负是向外 形变成都为正是向内
            //若distort为正 力是向内
            Vector2D b2a_forces = -s->k*(m1_to_m2_vector.unit())*distort;
            //受力=重力+弹簧力
            s->m1->forces -= b2a_forces;
            s->m2->forces += b2a_forces;
        }
        for (auto &m : masses)
        {
            float k_d=0.0001;
            if (!m->pinned)//如果m不是固定点
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                //这里的m->forces一直是0，-1 只有重力;
                m->forces +=gravity*m->mass;
                //增加全局阻尼
                m->forces -=k_d*m->velocity;
                // m->forces -=k_d*m->velocity; //阻尼计算
                Vector2D a =  m->forces/m->mass;
                m->velocity = m->velocity+a*delta_t;
                //半隐式欧拉法 用的式v_t+1 
                m->last_position = m->position;
                m->position = m->last_position+(m->velocity*delta_t);

                // TODO (Part 2): Add global damping
                // 增加全局阻尼
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            // 计算质点受力（无重力）
            Vector2D m1_to_m2_vector = s->m2->position - s->m1->position;
            // 计算其变形程度
            double length = m1_to_m2_vector.norm();
            double distort = length-s->rest_length;
            //计算弹簧力 这里取反了 是表示形变程度为负是向外 形变成都为正是向内
            //若distort为正 力是向内
            Vector2D b2a_forces = -s->k*(m1_to_m2_vector.unit())*distort;
            //受力=重力+弹簧力
            s->m1->forces -= b2a_forces;
            s->m2->forces += b2a_forces;

        }

        float damping_factor = 0.00005;
        for (auto &m : masses)
        {
            if (!m->pinned)
            {   //x(t)
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                m->forces+=gravity*m->mass;

                // m->position = 2*m->position-m->last_position+m->forces/(m->mass)*delta_t*delta_t;
                
                // TODO (Part 4): Add global Verlet damping
                m->position = m->position +(1-damping_factor)*(m->position-m->last_position)+m->forces/m->mass*delta_t*delta_t;
                m->last_position = temp_position;
            }   
            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }
}
