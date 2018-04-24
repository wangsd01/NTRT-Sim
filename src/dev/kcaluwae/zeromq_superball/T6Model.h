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

#ifndef T6_MODEL_H
#define T6_MODEL_H

/**
 * @file T6Model.h
 * @brief Contains the definition of class T6Model.
 * $Id$
 */

// This library
#include "core/tgModel.h"
#include "core/tgSubject.h"
// The C++ Standard Library
#include <vector>

#include <map>
#include <set> 


// Forward declarations
class tgBasicActuator;
class tgModelVisitor;
class tgStructure;
class tgWorld;

/**
 * Class that creates the six strut "superball" model using tgcreator
 */
class T6Model : public tgSubject<T6Model>, public tgModel
{
public:
    struct Config
    { 
        double density;
        double radius;
        double density_mp;
        double radius_mp;
        double stiffnessPassive;
        double stiffnessActive;
        double damping;
        double rod_length;
        double rod_space;
        double rod_length_mp;
        double friction;
        double rollFriction;
        double restitution;
        double pretensionPassive;
        double pretensionActive;
        bool   hist;
        double maxTens;
        double targetVelocity;
        double motor_radius;
        double motor_friction;
        double motor_inertia;
        bool   backDrivable;
    };
    // Config config =
    // {
    //      0.38618,    // density (kg / length^3) weight of both endcaps: 3.3kg (1.65 kg each)
    //      0.35,     // radius (length) radius of an endcap
    //      0.208,      // density_mp (kg / length^3) weight of connecting rod: 200g 
    //      0.175,      //radius_mp (length) radius of the connecting rod
    //      998.25,   // stiffnessPassive (kg / sec^2)
    //      3152.36,  // stiffnessActive (kg / sec^2)
    //      200.0,    // damping (kg / sec)
    //      17.4,     // rod_length (length)
    //      4.5,      // rod_space (length)
    //      10.0,        // rod_length_mp (length)
    //      0.99,      // friction (unitless)
    //      0.01,     // rollFriction (unitless)
    //      0.0,      // restitution (?)
    //      10000.0,    // pretension -> set to 
    //      10000.0,   // pretension -> set to 
    //      0,         // History logging (boolean)
    //      200000000,   // maxTens
    //      200000000,    // targetVelocity
    //      0.09, // motor_radius // Spindle radius (length)
    //      2*4.24e-5, // motor_friction (kg*(length)^2/sec)
    //      4*2.749e-4, // motor_inertia (kg*(length)^2) // Inertia of motor, gearbox, and spindle all together
    //      0, // Not backDrivable

    //      // Use the below values for earlier versions of simulation.
    //      // 1.006,
    //      // 0.31,
    //      // 300000.0,
    //      // 3000.0,
    //      // 15.0,
    //      // 7.5,
    // };
    
    Config config =
    {
         0.4020756457,    // density (kg / length^3) weight of both endcaps: 3.3kg (1.65 kg each)
         0.5,     // radius (length) radius of an endcap
         0.4020756457,      // density_mp (kg / length^3) weight of connecting rod: 200g 
         0.5,      //radius_mp (length) radius of the connecting rod
         800,   // stiffnessPassive (kg / sec^2)
         800,  // stiffnessActive (kg / sec^2)
         200,    // damping (kg / sec)
         19,     // rod_length (length) Does nothing in V2
         4.5,      // rod_space (length) //Does nothing in V2
         10.0,        // rod_length_mp (length) does nothing in V2
         0.99,      // friction (unitless)
         0.01,     // rollFriction (unitless)
         0.0,      // restitution (?)
         1500.0,    // pretension -> set to 
         1500.0,   // pretension -> set to 
         0,         // History logging (boolean)
         3000,   // maxTens
         15,    // targetVelocity
         0.09, // motor_radius // Spindle radius (length) not used
         2*4.24e-5, // motor_friction (kg*(length)^2/sec) not used
         4*2.749e-4, // motor_inertia (kg*(length)^2) // Inertia of motor, gearbox, and spindle all together not used
         0, // Not backDrivable


         // 0.38618,    // density (kg / length^3) weight of both endcaps: 3.3kg (1.65 kg each)
         // 0.35,     // radius (length) radius of an endcap
         // 0.208,      // density_mp (kg / length^3) weight of connecting rod: 200g 
         // 0.175,      //radius_mp (length) radius of the connecting rod
         // 998.25,   // stiffnessPassive (kg / sec^2)
         // 3152.36,  // stiffnessActive (kg / sec^2)
         // 200.0,    // damping (kg / sec)
         // 17.4,     // rod_length (length)
         // 4.5,      // rod_space (length)
         // 10.0,        // rod_length_mp (length)
         // 0.99,      // friction (unitless)
         // 0.01,     // rollFriction (unitless)
         // 0.0,      // restitution (?)
         // 100.0,    // pretension -> set to 
         // 100.0,   // pretension -> set to 
         // 0,         // History logging (boolean)
         // 2000,   // maxTens
         // 2,    // targetVelocity
         // 0.09, // motor_radius // Spindle radius (length)
         // 2*4.24e-5, // motor_friction (kg*(length)^2/sec)
         // 4*2.749e-4, // motor_inertia (kg*(length)^2) // Inertia of motor, gearbox, and spindle all together
         // 0, // Not backDrivable

         // Use the below values for earlier versions of simulation.
         // 1.006,
         // 0.31,
         // 300000.0,
         // 3000.0,
         // 15.0,
         // 7.5,
    };

    std::map<std::string, double Config::*> Config_members =
    { 
        { "density", &Config::density },
        { "radius", &Config::radius },
        { "density_mp", &Config::density_mp },
        { "radius_mp", &Config::radius_mp },
        { "stiffnessPassive", &Config::stiffnessPassive },
        { "stiffnessActive", &Config::stiffnessActive },
        { "damping", &Config::damping },
        { "rod_length", &Config::rod_length },
        { "rod_space", &Config::rod_space },
        { "rod_length_mp", &Config::rod_length_mp },
        { "friction", &Config::friction },
        { "rollFriction", &Config::rollFriction },
        { "restitution", &Config::restitution },
        { "pretensionPassive", &Config::pretensionPassive },
        { "pretensionActive", &Config::pretensionActive },
        { "maxTens", &Config::maxTens },
        { "targetVelocity", &Config::targetVelocity },
        { "motor_radius", &Config::motor_radius },
        { "motor_friction", &Config::motor_friction },
        { "motor_inertia", &Config::motor_inertia }
    };

    void set_named_member( Config& target, std::string which_member, double value )
    {
        target.*(Config_members[which_member]) = value;
    }

	
	/**
     * The only constructor. Utilizes default constructor of tgModel
     * Configuration parameters are within the .cpp file in this case,
     * not passed in. 
     */
    T6Model();
	
    /**
     * Destructor. Deletes controllers, if any were added during setup.
     * Teardown handles everything else.
     */
    virtual ~T6Model();
    
    /**
     * Create the model. Place the rods and strings into the world
     * that is passed into the simulation. This is triggered
     * automatically when the model is added to the simulation, when
     * tgModel::setup(world) is called (if this model is a child),
     * and when reset is called. Also notifies controllers of setup.
     * @param[in] world - the world we're building into
     */
    virtual void setup(tgWorld& world, Config c=Config());
    
    /**
     * Undoes setup. Deletes child models. Called automatically on
     * reset and end of simulation. Notifies controllers of teardown
     */
    void teardown();
    
    /**
     * Step the model, its children. Notifies controllers of step.
     * @param[in] dt, the timestep. Must be positive.
     */
    virtual void step(double dt);
	
	/**
     * Receives a tgModelVisitor and dispatches itself into the
     * visitor's "render" function. This model will go to the default
     * tgModel function, which does nothing.
     * @param[in] r - a tgModelVisitor which will pass this model back
     * to itself 
     */
    virtual void onVisit(tgModelVisitor& r);
    
    /**
     * Return a vector of all muscles for the controllers to work with.
     * @return A vector of all of the muscles
     */
    const std::vector<tgBasicActuator*>& getAllActuators() const;
    
private:
	
	/**
     * A function called during setup that determines the positions of
     * the nodes based on construction parameters. Rewrite this function
     * for your own models
     * @param[in] tetra: A tgStructure that we're building into
     */
    static void addNodes(tgStructure& s, Config c=Config());
	
	/**
     * A function called during setup that creates rods from the
     * relevant nodes. Rewrite this function for your own models.
     * @param[in] s A tgStructure that we're building into
     */
    static void addRods(tgStructure& s);
	
	/**
     * A function called during setup that creates muscles (Strings) from
     * the relevant nodes. Rewrite this function for your own models.
     * @param[in] s A tgStructure that we're building into
     */
    static void addActuators(tgStructure& s);

private:
	
	/**
     * A list of all of the muscles. Will be empty until most of the way
     * through setup
     */
    std::vector<tgBasicActuator*> allActuators;
};

#endif  // T6_MODEL_H
