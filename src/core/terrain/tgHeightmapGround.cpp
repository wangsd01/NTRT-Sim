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

/**
 * @file tgHeightmapGround.cpp
 * @brief Contains the implementation of class tgHeightmapGround
 * @author Steven Lessard
 * $Id$
 */

//This Module
#include "tgHeightmapGround.h"

//Bullet Physics
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btTriangleIndexVertexArray.h"
#include "BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "LinearMath/btDefaultMotionState.h"
#include "LinearMath/btTransform.h"

// The C++ Standard Library
#include <cassert>
#include <iostream>

tgHeightmapGround::Config::Config( int random_seed,
                                   double start_value,
                                   double variance,
                                   double cell_size,
                                   int smoothing_iters,
                                   btVector3 size,
                                   btVector3 origin,
                                   double friction,
                                   double restitution,
                                   double incline_angle
                                   ) :
    m_random_seed(random_seed),
    m_friction(friction),
    m_restitution(restitution),
    m_size(size),
    m_origin(origin),
    m_cellsize(cell_size),
    m_start_value(start_value),
    m_variance(variance),
    m_smoothing_iters(smoothing_iters),
    m_incline_angle(incline_angle)
{
    if (incline_angle != 0.0) 
    {
        m_nx = 3;
        m_nz = 2;
    }
    else 
    {
        m_nx = size[0]/m_cellsize;
        m_nz = size[2]/m_cellsize;
    }
}

tgHeightmapGround::tgHeightmapGround() :
    m_config(Config())
{
    // @todo make constructor aux to avoid repeated code
    if (m_config.m_incline_angle != 0.0) 
    {
        pGroundShape = inclineShape();
    }
    else 
    {
        pGroundShape = heightmapShape();
    }
}

tgHeightmapGround::tgHeightmapGround(const tgHeightmapGround::Config& config) :
    m_config(config)
{
    if (m_config.m_incline_angle != 0.0) 
    {
        pGroundShape = inclineShape();
    }
    else 
    {
        pGroundShape = heightmapShape();
    }
}

tgHeightmapGround::~tgHeightmapGround()
{
    delete m_pMesh;
    delete[] m_pIndices;
    delete[] m_vertices;
}

btRigidBody* tgHeightmapGround::getGroundRigidBody() const
{
        // std::cout << "Hilly ground " << std::endl;
    const btScalar mass = 0.0;

    btTransform groundTransform;
    groundTransform.setIdentity();
    groundTransform.setOrigin(m_config.m_origin);

    // Using motionstate is recommended
    // It provides interpolation capabilities, and only synchronizes 'active' objects
    btDefaultMotionState* const pMotionState =
        new btDefaultMotionState(groundTransform);

    const btVector3 localInertia(0, 0, 0);

    btRigidBody::btRigidBodyConstructionInfo const rbInfo(mass, pMotionState, pGroundShape, localInertia);

    btRigidBody* const pGroundBody = new btRigidBody(rbInfo);

    assert(pGroundBody);
    return pGroundBody;
}  

btCollisionShape* tgHeightmapGround::inclineShape()
{
    int vertexCount = 6;
    int triangleCount = 4;
    btCollisionShape * pShape = 0;

    m_vertices = new btVector3[vertexCount];

    const btScalar L0 = 10.0;
    const btScalar L = 500.0;
    const btScalar h = tan(m_config.m_incline_angle)*L;

    m_vertices[0].setValue( -L, 0.0, -L);
    m_vertices[1].setValue( L0, 0.0, -L);
    m_vertices[2].setValue(  L,   h, -L);
    m_vertices[3].setValue( -L, 0.0,  L);
    m_vertices[4].setValue( L0, 0.0,  L);
    m_vertices[5].setValue(  L,   h,  L);


    m_pIndices = new int[triangleCount*3];
    setIndices(m_pIndices); 

    m_pMesh = createMesh(triangleCount, m_pIndices, vertexCount, m_vertices);

    // Create the shape object
    pShape = createShape(m_pMesh);

    // Set the margin
    pShape->setMargin(.02);

    assert(pShape);
    return pShape; 
}

btCollisionShape* tgHeightmapGround::heightmapShape() 
{

    int seed = rand();
    srand(m_config.m_random_seed);
    btCollisionShape * pShape = 0;
    // The number of vertices in the mesh
    // Hill Paramenters: Subject to Change
    const std::size_t vertexCount = m_config.m_nx * m_config.m_nz;
    cost_map.clear();
    for(unsigned i=0;i<m_config.m_nx;i++)
    {
        cost_map.push_back({});
        for(unsigned j=0;j<m_config.m_nz;j++)
        {
            cost_map.back().push_back(1);
        }
    }
    create_new_cost_map();

    if (vertexCount > 0) 
    {
        // The number of triangles in the mesh
        const std::size_t triangleCount = 2 * (m_config.m_nx - 1) * (m_config.m_nz - 1);

        // A flattened array of all vertices in the mesh
        m_vertices = new btVector3[vertexCount];

        // Supplied by the derived class
        setVertices(m_vertices);
        // A flattened array of indices for each corner of each triangle
        m_pIndices = new int[triangleCount * 3];

        // Supplied by the derived class
        setIndices(m_pIndices);

        // Create the mesh object
        m_pMesh = createMesh(triangleCount, m_pIndices, vertexCount, m_vertices);

        // Create the shape object
        pShape = createShape(m_pMesh);

        // Set the margin
        pShape->setMargin(.02);
        // DO NOT deallocate vertices, indices or pMesh until simulation is over!
        // The shape owns them, but will not delete them
    }

    assert(pShape);
    srand(seed);
    return pShape; 
}

btTriangleIndexVertexArray *tgHeightmapGround::createMesh(std::size_t triangleCount, int indices[], std::size_t vertexCount, btVector3 vertices[]) 
{
    const int vertexStride = sizeof(btVector3);
    const int indexStride = 3 * sizeof(int);

    btTriangleIndexVertexArray* const pMesh = 
        new btTriangleIndexVertexArray(triangleCount,
                indices,
                indexStride,
                vertexCount,
                (btScalar*) &vertices[0].x(),
                vertexStride);
    return pMesh;
}

btCollisionShape *tgHeightmapGround::createShape(btTriangleIndexVertexArray *pMesh) 
{
    const bool useQuantizedAabbCompression = true;
    btCollisionShape *const pShape = 
        new btBvhTriangleMeshShape(pMesh, useQuantizedAabbCompression);
    return pShape;
}

void tgHeightmapGround::setVertices(btVector3 vertices[]) 
{
    for (std::size_t i = 0; i < m_config.m_nx; i++)
    {
        for (std::size_t j = 0; j < m_config.m_nz; j++)
        {
            const btScalar x = (i - (m_config.m_nx * 0.5)) * m_config.m_cellsize;
            const btScalar y = cost_map[i][j];
            const btScalar z = (j - (m_config.m_nz * 0.5)) * m_config.m_cellsize;
            vertices[i + (j * m_config.m_nx)].setValue(x, y, z);
        }
    }
}

void tgHeightmapGround::setIndices(int indices[]) 
{
    int index = 0;
    for (std::size_t i = 0; i < m_config.m_nx - 1; i++)
    {
        for (std::size_t j = 0; j < m_config.m_nz - 1; j++)
        {
            indices[index++] = (j       * m_config.m_nx) + i;
            indices[index++] = (j       * m_config.m_nx) + i + 1;
            indices[index++] = ((j + 1) * m_config.m_nx) + i + 1;

            indices[index++] = (j       * m_config.m_nx) + i;
            indices[index++] = ((j + 1) * m_config.m_nx) + i + 1;
            indices[index++] = ((j + 1) * m_config.m_nx) + i;
        }
    }
}


void tgHeightmapGround::midpoint_displace(std::vector<std::vector<double> >& heightmap,int x_min,int x_max,int y_min, int y_max)
{
    if(x_max-x_min == 1 || y_max-y_min == 1)
        return;

    double bottom_left = heightmap[ x_min ][y_min];
    double bottom_right = heightmap[ x_max][y_min];
    double top_left = heightmap[ x_min][y_max];
    double top_right = heightmap[ x_max ][y_max];

    int mid_x = (x_min+x_max)/2;
    int mid_y = (y_min+y_max)/2;

    heightmap[mid_x ][mid_y] = (bottom_right + bottom_left + top_right + top_left)/4.0 + uniform_random(-m_config.m_variance,m_config.m_variance);
    heightmap[x_min ][mid_y] = (bottom_left  + top_left    )/2.0 + uniform_random(-m_config.m_variance,m_config.m_variance);
    heightmap[x_max ][mid_y] = (bottom_right + top_right   )/2.0 + uniform_random(-m_config.m_variance,m_config.m_variance);
    heightmap[mid_x ][y_min] = (bottom_right + bottom_left )/2.0 + uniform_random(-m_config.m_variance,m_config.m_variance);
    heightmap[mid_x ][y_max] = (top_right    + top_left    )/2.0 + uniform_random(-m_config.m_variance,m_config.m_variance);

    midpoint_displace(heightmap,x_min,mid_x,y_min,mid_y);
    midpoint_displace(heightmap,x_min,mid_x,mid_y,y_max);
    midpoint_displace(heightmap,mid_x,x_max,mid_y,y_max);
    midpoint_displace(heightmap,mid_x,x_max,y_min,mid_y);
}

void tgHeightmapGround::create_new_cost_map()
{
    
    cost_map[0][0]=m_config.m_start_value;
    cost_map.back()[0]=m_config.m_start_value;
    cost_map[0].back()=m_config.m_start_value;
    cost_map.back().back()=m_config.m_start_value;
    midpoint_displace(cost_map,0,m_config.m_nx-1,0,m_config.m_nz-1);
    for(int smooth_iter=0;smooth_iter<m_config.m_smoothing_iters;smooth_iter++)
    {
        auto copy_of_map = cost_map;
        for(unsigned i=1;i<m_config.m_nx-1;i++)
        {
            for(unsigned j=1;j<m_config.m_nz-1;j++)
            {
                //avg smoothing
                double avg = 
                             cost_map[i-1][j-1] +
                             cost_map[i-1][j] +
                             cost_map[i-1][j+1] +
                             cost_map[i][j-1] +
                             cost_map[i][j] +
                             cost_map[i][j+1] +
                             cost_map[i+1][j-1] +
                             cost_map[i+1][j] +
                             cost_map[i+1][j+1];
                avg/=9.0;
                copy_of_map[i][j] = avg;
            }
        }
        cost_map = copy_of_map;
    }
    double max_cost = 0;
    double min_cost = 0;
    for(unsigned i=0;i<m_config.m_nx;i++)
    {
        for(unsigned j=0;j<m_config.m_nz;j++)
        {
            if(cost_map[i][j]>max_cost)
            {
                max_cost = cost_map[i][j];
            }
            if(cost_map[i][j]<min_cost)
            {
                min_cost = cost_map[i][j];
            }
        }
    }
    for(unsigned i=0;i<m_config.m_nx;i++)
    {
        for(unsigned j=0;j<m_config.m_nz;j++)
        {
            cost_map[i][j]-=min_cost;
            cost_map[i][j]/=(max_cost-min_cost);
            cost_map[i][j]*=m_config.m_variance;
        }
    }
}

