#include <iostream>
#include <dqrobotics/DQ.h>
#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimZmqInterface.h>
#include <memory>

using namespace DQ_robotics;
using namespace Eigen;

int main()
{
    auto vi = std::make_shared<DQ_CoppeliaSimZmqInterface>();
    vi->connect();
    auto vi_exp = std::make_shared<DQ_CoppeliaSimZmqInterface::experimental>(vi);
    vi_exp->close_scene();


    vi_exp->set_gravity(DQ(0));
    vi->set_stepping_mode(true);

    std::string robotname = "/Sphere";
    vi_exp->add_primitive(DQ_CoppeliaSimZmqInterface::PRIMITIVE::SPHEROID, robotname, {0.2, 0.2, 0.2});
    vi->set_object_pose(robotname, 1 + 0.5*E_*0.3*k_);
    vi_exp->set_object_color(robotname, {1, 0,0,0.5});
    vi_exp->set_object_as_static(robotname, false);
    //vi_exp->se


    vi->start_simulation();

    for (int i=0; i<200; i++)
    {
        DQ x = vi->get_object_pose(robotname);

        DQ w = 0.1*k_;
        DQ p_dot = 0.1*j_;

        //vi->set_angular_and_linear_velocities(robotname, w, p_dot);

        DQ twist_b = w + E_*(p_dot);
        DQ twist_a = x*twist_b*x.conj();

        vi_exp->set_twist(robotname, twist_b,
                          DQ_CoppeliaSimZmqInterface::REFERENCE::BODY_FRAME);

        std::cout<<"twist_a :    "<<twist_a<<std::endl;
        std::cout<<"twist_a_:    "<<vi_exp->get_twist(robotname)<<std::endl;
        std::cout<<" "<<std::endl;
        std::cout<<"twist_b :    "<<twist_b<<std::endl;
        std::cout<<"twist_b_:    "<<vi_exp->get_twist(robotname, DQ_CoppeliaSimZmqInterface::REFERENCE::BODY_FRAME)<<std::endl;


        vi->trigger_next_simulation_step();
    }

    vi->stop_simulation();
}
