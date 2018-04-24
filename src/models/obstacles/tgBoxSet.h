
#ifndef TG_BOX_SET
#define TG_BOX_SET

// This library
#include "core/tgModel.h"
// The Bullet Physics Library
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <vector>

// Forward declarations
class tgModelVisitor;
class tgStructure;
class tgWorld;

class tgBoxSet : public tgModel
{
public: 
    
    struct Config
    {
        public:
            Config( std::vector<std::vector<double> > box_dimensions,
                    std::vector<std::vector<double> > box_positions,
                    btVector3 origin = btVector3(0.0, 0.0, 0.0),
                    btScalar friction = 0.5,
                    btScalar restitution = 0.0);

            /** Origin position of the block field */
            btVector3 m_origin;
            
            /** Friction value of the blocks, must be between 0 to 1 */
            btScalar  m_friction;

            /** Restitution coefficient of the blocks, must be between 0 to 1 */
            btScalar  m_restitution;

            std::vector<std::vector<double> > box_dims;
            std::vector<std::vector<double> > box_pos;
    };
    
   /**
    * Default constructor. Sets the default config values
    */
    tgBoxSet();

    /**
        * Origin constructor. Copies the config to m_config
        * @param[in] config - specifies the parameters of the block field
        */
    tgBoxSet(tgBoxSet::Config& config);

    /**
        * Destructor. Deletes controllers, if any were added during setup.
        * Teardown handles everything else.
        */
    virtual ~tgBoxSet();

    /**
        * Create the model.
        * @param[in] world - the world we're building into
        */
    virtual void setup(tgWorld& world);

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
        * Undoes setup. Deletes child models. Called automatically on
        * reset and end of simulation. Notifies controllers of teardown
        */
    void teardown();  

private:

    tgBoxSet::Config m_config;

};

#endif // TETRA_COLLISIONS_WALL
