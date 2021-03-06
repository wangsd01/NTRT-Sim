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

#ifndef JSON_MIXED_LEARNING_CONTROL_H
#define JSON_MIXED_LEARNING_CONTROL_H

/**
 * @file JSONMixedLearningControl.h
 * @brief A controller for the template class BaseSpineModelLearning
 * @author Brian Mirletz, Dawn Hustig-Schultz
 * @version 1.0.0
 * $Id$
 */

#include "dev/btietz/JSONTests/JSONCPGControl.h"

#include <json/value.h>

// Forward Declarations
class neuralNetwork;
class tgSpringCableActuator;

/**
 * JSONFeedbackControl learns the parameters for a CPG system on a
 * spine like tensegrity structure specified as a BaseSpineModelLearning. Parameters are generated by
 * AnnealEvolution and used in the CPGEquations family of classes.
 * tgImpedanceController controllers are used for the detailed muscle control.
 * Due to the number of parameters, the learned parameters are split
 * into one config file for the nodes and another 3 for the CPG's "edges".
 */
class JSONMixedLearningControl : public JSONCPGControl
{
public:

struct Config : public JSONCPGControl::Config
    {
    public:
        /**
         * The only constructor. 
         */
        Config( int ss,
        int tm,
        int om,
        int param,
        int segnum = 6,
        double ct = 0.1,
        double la = 0,
        double ha = 30,
        double lp = -1 * M_PI,
        double hp = M_PI,
        double kt = 0.0,
        double kp = 1000.0,
        double kv = 100.0,
        bool def = true,
        double cl = 10.0,
        double lf = 0.0,
        double hf = 30.0,
        double ffMin = 0.0,
        double ffMax = 0.0,
        double afMin = 0.0,
        double afMax = 0.0,
        double pfMin = 0.0,
        double pfMax = 0.0
        );
        
        const double freqFeedbackMin;
        const double freqFeedbackMax;
        const double ampFeedbackMin;
        const double ampFeedbackMax;
        const double phaseFeedbackMin;
        const double phaseFeedbackMax;
        
        // Values to be filled in by JSON file during onSetup
        int numStates;
        int numActions;
	int numHidden;
        
    };

    JSONMixedLearningControl(JSONMixedLearningControl::Config config,	
							std::string args,
							std::string resourcePath = "");
    
    virtual ~JSONMixedLearningControl();
    
    virtual void onSetup(BaseSpineModelLearning& subject);
    
    virtual void onStep(BaseSpineModelLearning& subject, double dt);
    
    virtual void onTeardown(BaseSpineModelLearning& subject);
	
protected:

    virtual void setupCPGs(BaseSpineModelLearning& subject, array_2D nodeActions, array_4D startingEdgeActions, array_4D middleEdgeActions, array_4D endingEdgeActions);
    
    virtual array_2D scaleNodeActions (Json::Value actions);
    
    std::vector<double> getFeedback(BaseSpineModelLearning& subject);
    
    std::vector<double> getCableState(const tgSpringCableActuator& cable);
    
    std::vector<double> transformFeedbackActions(std::vector< std::vector<double> >& actions);
    
    JSONMixedLearningControl::Config m_config;
    
    std::vector<tgCPGActuatorControl*> m_startingControllers;
    std::vector<tgCPGActuatorControl*> m_middleControllers;
    std::vector<tgCPGActuatorControl*> m_endingControllers;

    /// @todo generalize this if we need more than one
    neuralNetwork* nn;
    
};

#endif // JSON_MIXED_LEARNING_CONTROL_H
