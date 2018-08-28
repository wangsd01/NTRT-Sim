/**
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

#ifndef CORE_TERRAIN_TG_HEIGHTMAP_GROUND_H
#define CORE_TERRAIN_TG_HEIGHTMAP_GROUND_H

/**
 * @file tgHeightmapGround.h
 * @brief Contains the definition of class tgHeightmapGround.
 * @author Steven Lessard
 * $Id$
 */

#include "tgBulletGround.h"

#include "LinearMath/btScalar.h"
#include "LinearMath/btVector3.h"

// std::size_t
#include <cstddef>
#include <vector>

// Forward declarations
class btRigidBody;
class btTriangleIndexVertexArray;

class tgHeightmapGround : public tgBulletGround
{
    public:

        struct Config
        {
            public:
                Config(int random_seed = 123,
                       double start_value = 10,
                       double variance = 30,
                       double cell_size = .5,
                       int smoothing_iters = 2,
                       btVector3 size = btVector3(500.0, 1.5, 500.0),
                       btVector3 origin = btVector3(0,0,0),
                       double friction = .9,
                       double restitution = 0.0,
                       double incline_angle = 0.0
                       );

                int m_random_seed;

                /** Friction value of the ground, must be between 0 to 1 */
                btScalar  m_friction;

                /** Restitution coefficient of the ground, must be between 0 to 1 */
                btScalar  m_restitution;

                /** Size of the ground, must be between non-negitive */
                btVector3 m_size;

                btVector3 m_origin;

                /** Number of nodes in the x-direction */
                std::size_t m_nx;

                /** Number of nodes in the z-direction */
                std::size_t m_nz;

                double m_cellsize;

                double m_start_value;

                double m_variance;

                int m_smoothing_iters;

                double m_incline_angle;
        };

        /**
         * Default construction that uses the default values of config
         * Sets up a collision object that is stored in the bulletGround
         * object
         */
        tgHeightmapGround();

        /**
         * Allows a user to specify their own config
         */
        tgHeightmapGround(const tgHeightmapGround::Config& config);

        /** Clean up the implementation. Deletes m_pMesh */
        virtual ~tgHeightmapGround();

        /**
         * Setup and return a return a rigid body based on the collision 
         * object
         */
        virtual btRigidBody* getGroundRigidBody() const;

        btCollisionShape* inclineShape();
        /**
         * Returns the collision shape that forms a hilly ground
         */
        btCollisionShape* heightmapShape();
        
        btVector3* getVertices()
        {
            return m_vertices;
        }

        int* getIndices()
        {
            return m_pIndices;
        }

    private:  

        std::vector<std::vector<double> > cost_map;

        void midpoint_displace(std::vector<std::vector<double> >& heightmap,int x_min,int x_max,int y_min, int y_max);

        void create_new_cost_map();

        /** Store the configuration data for use later */
        Config m_config;

        /** Pre-condition: Quantity of triangles and vertices must each be greater than zero 
         *  Post-condition: Returns a mesh, as configured by the input parameters, 
         *                  to be used as a template for a btBvhTriangleMeshShape
         */
        btTriangleIndexVertexArray* createMesh(std::size_t triangleCount, int indices[], std::size_t vertexCount, btVector3 vertices[]);

        /** Pre-condition: Given mesh is a valig btTriangleIndexVertexArray with all values initialized
         *  Post-condition: Returns a btBvhTriangleMeshShape in the shape of the hills as configured 
         */
        btCollisionShape *createShape(btTriangleIndexVertexArray * pMesh);

        /**
         * @param[out] A flattened array of vertices in the mesh
         */
        void setVertices(btVector3 vertices[]);

        /**
         * @param[out] A flattened array of indices in the mesh
         */
        void setIndices(int indices[]);


        inline double uniform_random(double min, double max)
        {
            return (((double)rand() / (double)RAND_MAX) * (max - min)) + min;
        }
        
        // Store this so we can delete it later
        btTriangleIndexVertexArray* m_pMesh;
        btVector3 * m_vertices;
        int * m_pIndices;

};

#endif  // TG_HILLY_GROUND_H
