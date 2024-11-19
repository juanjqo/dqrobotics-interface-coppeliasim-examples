/**
(C) Copyright 2024 DQ Robotics Developers

This file is based on DQ Robotics.

    DQ Robotics is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    DQ Robotics is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.

Contributors:
- Juan Jose Quiroz Omana


##### INSTRUCTIONS #######
1) Open CoppeliaSim. (You do not need to load a specific scene).
2) Run and enjoy!

*/

#include <dqrobotics/DQ.h>
#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterfaceZMQ.h>
#include <dqrobotics/interfaces/coppeliasim/robots/URXCoppeliaSimZMQRobot.h>

using namespace DQ_robotics;
using namespace Eigen;

VectorXd compute_control_signal(const MatrixXd J,
                                const VectorXd& q,
                                const double& damping,
                                const double& gain,
                                const VectorXd task_error);

int main()
{
    auto vi = std::make_shared<DQ_CoppeliaSimInterfaceZMQ>();
    vi->connect();


    auto vi_exp = std::make_shared<DQ_CoppeliaSimInterfaceZMQ::experimental>(vi);
    vi_exp->close_scene();

    // Load the models only if they are not already on the scene.
    vi_exp->load_from_model_browser("/robots/non-mobile/UR5.ttm", "/UR5");
    vi_exp->load_from_model_browser("/other/reference frame.ttm", "/Current_pose");
    //vi_exp->load_from_model_browser("/other/reference frame.ttm", "/Desired_pose");
    vi_exp->plot_reference_frame("/Desired_pose", DQ(1), 1.5, {0.02, 0.1});


    auto robot = URXCoppeliaSimZMQRobot("/UR5", vi, URXCoppeliaSimZMQRobot::MODEL::UR5);
    auto robot_model = robot.kinematics();

    vi_exp->enable_dynamics(false);
    //vi_exp->set_joint_control_modes(robot.get_joint_names(), DQ_CoppeliaSimZmqInterface::JOINT_CONTROL_MODE::POSITION);
    vi_exp->set_joint_modes(robot.get_joint_names(), DQ_CoppeliaSimInterfaceZMQ::JOINT_MODE::KINEMATIC);


    VectorXd q = robot.get_configuration_space();
    double gain = 10;
    double T = 0.001;
    double damping = 0.01;

    auto xd = robot_model.fkm(((VectorXd(6) <<  0.5, 0, 1.5, 0, 0, 0).finished()));
    vi->set_object_pose("/Desired_pose", xd);

    vi->start_simulation();

    for (int i=0; i<300; i++)
    {
        auto x = robot_model.fkm(q);
        vi->set_object_pose("/Current_pose", x);
        auto J =  robot_model.pose_jacobian(q);
        auto Jt = robot_model.translation_jacobian(J, x);
        auto task_error = (x.translation()-xd.translation()).vec4();
        auto u = compute_control_signal(Jt, q, damping, gain, task_error);
        q = q + T*u;
        robot.set_configuration_space_positions(q);
        std::cout<<"error: "<<task_error.norm()<<std::endl;
    }
    vi->stop_simulation();
}

VectorXd compute_control_signal(const MatrixXd J,
                                const VectorXd& q,
                                const double& damping,
                                const double& gain,
                                const VectorXd task_error)
{
    VectorXd u = (J.transpose()*J + damping*damping*MatrixXd::Identity(q.size(), q.size())).inverse()*
        J.transpose()*(-gain*task_error);
    return u;
}
