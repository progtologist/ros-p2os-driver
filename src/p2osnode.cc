/*
 *  P2OS for ROS
 *  Copyright (C) 2009
 *     David Feil-Seifer, Brian Gerkey, Kasper Stoy,
 *      Richard Vaughan, & Andrew Howard
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <iostream>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <p2os.h>
#include <p2os_driver/MotorState.h>

int main( int argc, char** argv )
{
    ros::init(argc,argv, "p2os");
    ros::NodeHandle n;

    P2OSNode *p = new P2OSNode(n);
    controller_manager::ControllerManager cm(p);

    if (!(p->get_psos_use_tcp()))
    {
        if( p->Setup() != 0 )
        {
            ROS_ERROR( "Setup of p2os over serial failed." );
            return -1;
        }
    }
    else
    {
        if( p->SetupTCP() != 0)
        {
            ROS_ERROR( "Setup of p2os over tcp failed." );
            return -1;
        }
    }

    p->ResetRawPositions();

    ros::Rate rate(p->get_frequency());
    int pulseCounter = 0;
    ros::Time time;

    while( ros::ok() )
    {
        p->check_and_set_vel();
        p->check_and_set_motor_state();
        p->check_and_set_gripper_state();
        p->check_and_set_arm_state();

        pulseCounter++;
        if ( pulseCounter > p->get_pulse())
        {
            p->SendPulse();
            pulseCounter = 0;
        }
        // Listen at a constant rate
        p->sendReceive(NULL,true);
        p->updateDiagnostics();
        ros::spinOnce();
        rate.sleep();
    }

    if( p->Shutdown() != 0 )
    {
        ROS_WARN( "p2os shutdown failed... your robot might be heading for the wall?" );
    }

    delete p;

    ROS_INFO( "Quitting... " );
    return 0;
}
