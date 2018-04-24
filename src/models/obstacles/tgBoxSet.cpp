
// This module
#include "tgBoxSet.h"
// This library
#include "core/tgBox.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBoxInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
#include "tgcreator/tgNode.h"
#include "tgcreator/tgUtil.h"
// The Bullet Physics library
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <stdexcept>
#include <vector>
#include <numeric> // RAND_MAX

tgBoxSet::Config::Config(    std::vector<std::vector<double> > box_dimensions,
                             std::vector<std::vector<double> > box_positions,
                             btVector3 origin,
                             btScalar friction, 
                             btScalar restitution) :
m_origin(origin),
m_friction(friction),
m_restitution(restitution),
box_dims(box_dimensions),
box_pos(box_positions)
{
    assert(m_friction >= 0.0);
    assert(m_restitution >= 0.0);
    assert(box_pos.size()==box_dims.size());
}

tgBoxSet::tgBoxSet() : 
tgModel(),
m_config({},{})
{
    // Seed the random number generator
    /// @todo assess if doing this multiple times in a trial (here and evolution) causes problems
    // tgUtil::seedRandom(1);
}

tgBoxSet::tgBoxSet(tgBoxSet::Config& config) :
tgModel(),
m_config(config)
{
    // Seed the random number generator
    /// @todo assess if doing this multiple times in a trial (here and evolution) causes problems
    // tgUtil::seedRandom(1);
}

tgBoxSet::~tgBoxSet() {}
                     
void tgBoxSet::setup(tgWorld& world) 
{
    // const tgBox::Config boxConfig(m_config.m_width / 2.0, m_config.m_height / 2.0, 0.0, m_config.m_friction, 0.0, m_config.m_restitution);

    // Start creating the structure
    tgStructure s;
    tgBuildSpec spec;
    
    for(size_t i = 0; i < 2 * m_config.box_dims.size(); i += 2) 
    {
        const tgBox::Config boxConfig(m_config.box_dims[i/2][0]/2.0, m_config.box_dims[i/2][1]/2.0, 0.0, m_config.m_friction, 0.0, m_config.m_restitution);
        double x,y,z;
        // do
        // {
        x = m_config.box_pos[i/2][0];
        y = m_config.box_pos[i/2][1];
        z = m_config.box_pos[i/2][2]-m_config.box_dims[i/2][2]/2.0;
        // }
        // while(fabs(x)<10 && fabs(y)<10 && fabs(z)<10);

        // double xOffset = fieldSize.getX() * rand() / RAND_MAX;
        // double yOffset = fieldSize.getY() * rand() / RAND_MAX;
        // double zOffset = fieldSize.getZ() * rand() / RAND_MAX;
        
        btVector3 offset(x, y, z);
        
        tgNode position = offset;
        s.addNode(position);
        position += btVector3(0.0, 0.0, m_config.box_dims[i/2][2]);
        s.addNode(position);
        
        s.addPair(i, i+1, "box"+std::to_string(i/2));

        // Create the build spec that uses tags to turn the structure into a real model
        spec.addBuilder("box"+std::to_string(i/2), new tgBoxInfo(boxConfig));
    }

    s.move(btVector3(0,0,0)); // Set center of field to desired origin position

    // Create your structureInfo
    tgStructureInfo structureInfo(s, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // Actually setup the children
    tgModel::setup(world);
}

void tgBoxSet::step(double dt) {
    // Precondition
    if (dt <= 0.0) {
        throw std::invalid_argument("dt is not positive");
    }
    else {
        tgModel::step(dt); // Step any children
    }
}

void tgBoxSet::onVisit(tgModelVisitor& r) {
    tgModel::onVisit(r);
}

void tgBoxSet::teardown() {
    tgModel::teardown();
} 

