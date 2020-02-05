

/**
 * @file T1Model.cpp
 * @brief Contains the implementation of class T1Model.
 * $Id$
 */

// This module
#include "T1Model.h"
// This library
#include "core/tgBasicActuator.h"
#include "core/tgRod.h"
#include "core/tgSphere.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgSphereInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
// The Bullet Physics library
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <stdexcept>

namespace
{
    // see tgBasicActuator and tgRod for a descripton of these rod parameters
    // (specifically, those related to the motor moving the strings.)
    // NOTE that any parameter that depends on units of length will scale
    // with the current gravity scaling. E.g., with gravity as 98.1,
    // the length units below are in decimeters.

    // Note: This current model of the SUPERball rod is 1.5m long by 3 cm radius,
    // which is 0.00424 m^3.
    // For SUPERball v1.5, mass = 3.5kg per strut, which comes out to 
    // 0.825 kg / (decimeter^3).

    // similarly, frictional parameters are for the tgRod objects.
    const struct Config
    {
        double density;
        double radius;
        double stiffness;
        double damping;
        double rod_length;
        double rod_space;    
        double friction;
        double rollFriction;
        double restitution;
        double pretension;
        bool   hist;
        double maxTens;
        double targetVelocity;
    } c =
   {
     0.688,    // density (kg / length^3)
     0.5,     // radius (length)
     613.0,   // stiffness (kg / sec^2) was 1500
     200.0,    // damping (kg / sec)
     16.84,     // rod_length (length)
     7.5,      // rod_space (length)
     0.99, // 0.99,      // friction (unitless)
     0.01, // 0.01,     // rollFriction (unitless)
     0.0,      // restitution (?)
     2452.0,        // pretension -> set to 4 * 613, the previous value of the rest length controller
     0,			// History logging (boolean)
     100000,   // maxTens
     10000,    // targetVelocity

     // Use the below values for earlier versions of simulation.
     // 1.006,    
     // 0.31,     
     // 300000.0, 
     // 3000.0,   
     // 15.0,     
     // 7.5,      
  };
} // namespace

T1Model::T1Model() : tgModel() 
{
}

T1Model::~T1Model()
{
}

void T1Model::addNodes(tgStructure& s)
{
    const double half_length = c.rod_length / 2;

    s.addNode(0,  -1, 0);            // 0
    s.addNode(0,   1, 0);            // 1
    s.addNode(0,  0,  1, "anchor"); // 2
    s.addNode(0,  1,  0.5, "anchor"); // 3
}

void T1Model::addRods(tgStructure& s)
{
    s.addPair( 0,  1, "rod");
}

void T1Model::addNails(tgStructure& s)
{
    // s.addNail(2, "nail");
}

void T1Model::addActuators(tgStructure& s)
{
    s.addPair(3, 2,  "muscle");
}

void T1Model::setup(tgWorld& world)
{

    const tgRod::Config rodConfig(c.radius, c.density, c.friction, 
				c.rollFriction, c.restitution);
    
    /// @todo acceleration constraint was removed on 12/10/14 Replace with tgKinematicActuator as appropreate
    tgBasicActuator::Config muscleConfig(c.stiffness, c.damping, c.pretension, c.hist, 
					    c.maxTens, c.targetVelocity);
            
    // Start creating the structure
    tgStructure s;
    addNodes(s);
    addNails(s);
    addRods(s);
    addActuators(s);
    s.move(btVector3(0, 40, 0));

    // Add a rotation. This is needed if the ground slopes too much,
    // otherwise  glitches put a rod below the ground.
    // btVector3 rotationPoint = btVector3(0, 0, 0); // origin
    // btVector3 rotationAxis = btVector3(0, 1, 0);  // y-axis
    // double rotationAngle = M_PI/2;
    // s.addRotation(rotationPoint, rotationAxis, rotationAngle);

    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("rod", new tgRodInfo(rodConfig));
    spec.addBuilder("muscle", new tgBasicActuatorInfo(muscleConfig));

    const tgSphere::Config sphereConfig(0.1, 0.); // density is 0, means static
    spec.addBuilder("sphere", new tgSphereInfo(sphereConfig));
    
    // Create your structureInfo
    tgStructureInfo structureInfo(s, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // We could now use tgCast::filter or similar to pull out the
    // models (e.g. muscles) that we want to control. 
    allActuators = tgCast::filter<tgModel, tgBasicActuator> (getDescendants());

    // call the onSetup methods of all observed things e.g. controllers
    notifySetup();

    // Actually setup the children
    tgModel::setup(world);
}

void T1Model::step(double dt)
{
    // Precondition
    if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive");
    }
    else
    {
        // Notify observers (controllers) of the step so that they can take action
        notifyStep(dt);
        tgModel::step(dt);  // Step any children
    }
}

void T1Model::onVisit(tgModelVisitor& r)
{
    tgModel::onVisit(r);
}

const std::vector<tgBasicActuator*>& T1Model::getAllActuators() const
{
    return allActuators;
}
    
void T1Model::teardown()
{
    notifyTeardown();
    tgModel::teardown();
}
