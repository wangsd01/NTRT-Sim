/*
 * Copyright © 2012, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * All rights reserved.
 * 
 * The NASA Tensegrity Robotics Toolkit (NTRT) v1 platform is licensed
 * under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0.
 * 
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific language
 * governing permissions and limitations under the License.
*/

/**
 * @file T6TPIDController.cpp
 * @author Ken Caluwaerts
 * @version 1.0.0
 * $Id$
 */

// This module
#include "T6PIDController.h"
// This application
#include "../T6Model.h"
// This library
#include "core/tgBasicActuator.h"
// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <iostream>

T6PIDController::T6PIDController(const ControlMode control_mode,double p,double i,double d) :
    control_mode(control_mode),c_p(p),c_i(i),c_d(d)
{
}

T6PIDController::~T6PIDController()
{
    std::size_t n = m_controllers.size();
    for(std::size_t i = 0; i < n; i++)
    {
        delete m_controllers[i];
    }
    m_controllers.clear();
    m_target.clear();
}	

void T6PIDController::onSetup(T6Model& subject)
{
    // std::cout << "onSetup called" << std::endl;
    const std::vector<tgBasicActuator*> actuators = subject.getAllActuators();
    for (size_t i = 0; i < actuators.size(); ++i)
    {
        tgBasicActuator * const pActuator = actuators[i];
        assert(pActuator != NULL);
        tgPIDController::Config config(c_p,c_i,c_d);
        tgPIDController* m_PIDController = new tgPIDController(pActuator, config);
        m_controllers.push_back(m_PIDController);
        m_target.push_back(0.);
        prev_rest_length_values.push_back(pActuator->getRestLength());
    }

}

void T6PIDController::onTeardown(T6Model& subject)
{
    // std::cout << "onTeardown called" << std::endl;
    std::size_t n = m_controllers.size();
    for(std::size_t i = 0; i < n; i++)
    {
        delete m_controllers[i];
    }
    m_controllers.clear();
    m_target.clear();

}

void T6PIDController::onAttach(T6Model& subject)
{
    std::cout << "onAttach called" << std::endl;
}


void T6PIDController::setTarget(const double target[])
{
    std::size_t n = m_controllers.size();
    for(std::size_t i = 0; i < n; i++)
    {
        m_target[i] = target[i];
    }
}


void T6PIDController::onStep(T6Model& subject, double dt)
{
    assert(dt>0.0);
    std::size_t n = m_controllers.size();
    const std::vector<tgBasicActuator*> actuators = subject.getAllActuators();

    for(std::size_t i = 0; i < n; i++)
    {
        tgBasicActuator * const pActuator = actuators[i];


        double error = m_target[i]-pActuator->getRestLength();
        // double m_intError += (error + prev_rest_length_values[i]) / 2.0 * dt;
        // double dError = (error - prev_rest_length_values[i]) / dt;
        double result = c_p * error;


//        pActuator->setControlInput(pActuator->getRestLength()+result*dt);
        pActuator->setControlInput(m_target[i], dt);
        
        prev_rest_length_values[i] = error;  

    }
    //std::cout << std::endl;
}
