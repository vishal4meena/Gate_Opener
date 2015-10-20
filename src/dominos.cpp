
/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

/* 
 * Base code for CS 251 Software Systems Lab 
 * Department of Computer Science and Engineering, IIT Bombay
 * 
 */


#include "cs251_base.hpp"
#include "render.hpp"

#ifdef __APPLE__
  #include <GLUT/glut.h>
#else
  #include "GL/freeglut.h"
#endif

#include <cstring>
using namespace std;

#include "dominos.hpp"

namespace cs251
{
  b2Body* b1;
  b2Body* sbody;
  
  dominos_t::dominos_t()
  {
    //Ground  
    {      
      b2EdgeShape shape; 
      shape.Set(b2Vec2(-900.0f, 0.0f), b2Vec2(900.0f, 0.0f));
      b2BodyDef bd; 
      b1 = m_world->CreateBody(&bd); 
      b1->CreateFixture(&shape, 0.0f);
    }
    
    /////////////////////////////////////////////

    //first ball
    {
      b2CircleShape circle;
      circle.m_radius = 0.6;
  
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 60.0f;
      ballfd.friction = 50000.0f;
      ballfd.restitution = 0.0f;
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      ballbd.position.Set(-39.5f, 45.0f);
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);
    }

    /////////////////////////////////////////////

    //slant planks
    {
      b2Body* sbody;
      b2PolygonShape poly;
      b2Vec2 vertices[4];
      vertices[0].Set(0,0);
      vertices[1].Set(7.5,-2.5);
      vertices[2].Set(7.4,-2.8);
      vertices[3].Set(-0.1,-0.3);
      poly.Set(vertices, 4);
      b2FixtureDef wedgefd;
      wedgefd.shape = &poly;
      wedgefd.density = 10.0f;
      wedgefd.friction = 0.0f;
      wedgefd.restitution = 0.0f;
      b2BodyDef wedgebd;
      wedgebd.position.Set(-42.2f, 42.5f);
      sbody = m_world->CreateBody(&wedgebd);
      sbody->CreateFixture(&wedgefd);
    }

    {
      b2Body* sbody;
      b2PolygonShape poly;
      b2Vec2 vertices[4];
      vertices[0].Set(0,0);
      vertices[1].Set(7.5,-2.5);
      vertices[2].Set(7.4,-2.8);
      vertices[3].Set(-0.1,-0.3);
      poly.Set(vertices, 4);
      b2FixtureDef wedgefd;
      wedgefd.shape = &poly;
      wedgefd.density = 10.0f;
      wedgefd.friction = 0.0f;
      wedgefd.restitution = 0.0f;
      b2BodyDef wedgebd;
      wedgebd.position.Set(-42.5f, 37.5f);
      sbody = m_world->CreateBody(&wedgebd);
      sbody->CreateFixture(&wedgefd);
    }

    {
      b2Body* sbody;
      b2PolygonShape poly;
      b2Vec2 vertices[4];
      vertices[0].Set(0,0);
      vertices[1].Set(7.5,2.5);
      vertices[2].Set(7.4,2.8);
      vertices[3].Set(-0.1,0.3);
      poly.Set(vertices, 4);
      b2FixtureDef wedgefd;
      wedgefd.shape = &poly;
      wedgefd.density = 10.0f;
      wedgefd.friction = 0.0f;
      wedgefd.restitution = 0.0f;
      b2BodyDef wedgebd;
      wedgebd.position.Set(-35.5f,37.5f);
      sbody = m_world->CreateBody(&wedgebd);
      sbody->CreateFixture(&wedgefd);
    }

    //////////////////////////////////////////////

    //plank-spring mass system
    {
      b2Body* sbody;
      b2PolygonShape poly;
      b2Vec2 vertices[4];
      vertices[0].Set(0,0);
      vertices[1].Set(24,0);
      vertices[2].Set(24,0.25);
      vertices[3].Set(0,0.25);
      poly.Set(vertices, 4);
      b2FixtureDef wedgefd;
      wedgefd.shape = &poly;
      wedgefd.density = 10.0f;
      wedgefd.friction =0.05f;
      wedgefd.restitution = 0.0f;
      b2BodyDef wedgebd;
      wedgebd.position.Set(-33.0f, 32.5f);
      sbody = m_world->CreateBody(&wedgebd);
      sbody->CreateFixture(&wedgefd);
    }

    ////////////////////////////////////////////////     

    {
      b2Body* sbody;
      b2PolygonShape poly;
      b2Vec2 vertices[4];
      vertices[0].Set(0,0);
      vertices[1].Set(0.75,0);
      vertices[2].Set(0.75,0.75);
      vertices[3].Set(0,0.75);
      poly.Set(vertices, 4);
      b2FixtureDef wedgefd;
      wedgefd.shape = &poly;
      wedgefd.density = 10.0f;
      wedgefd.friction = 0.0f;
      wedgefd.restitution = 0.0f;
      b2BodyDef wedgebd;
      wedgebd.position.Set(-33.0f, 32.7f);
      sbody = m_world->CreateBody(&wedgebd);
      sbody->CreateFixture(&wedgefd);
    }

    //////////////////////////////////////////////////////////

    {
      b2Body* sbody;
      b2PolygonShape poly;
      b2Vec2 vertices[4];
      vertices[0].Set(0,0);
      vertices[1].Set(0.75,0);
      vertices[2].Set(0.75,0.75);
      vertices[3].Set(0,0.75);
      poly.Set(vertices, 4);
      b2FixtureDef wedgefd;
      wedgefd.shape = &poly;
      wedgefd.density = 10.0f;
      wedgefd.friction = 0.0f;
      wedgefd.restitution = 0.0f;
      b2BodyDef wedgebd;
      wedgebd.position.Set(-31.0f, 32.7f);
      sbody = m_world->CreateBody(&wedgebd);
      sbody->CreateFixture(&wedgefd);
    }


    //////////////////////////////////////////////////////////

    //spring mass system 
    {
      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(-28.5,33.5);

      b2Body* body10;

      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 10.0;
      fd1->friction = 0.0f;
      fd1->restitution = -1000.0f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(0.75,0.62, b2Vec2(-2.25,0.38), 0);
      fd1->shape = &bs1;
      
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 10.0;
      fd2->friction = 0.051f;
      fd2->restitution = -1000.0f;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(1.5,1, b2Vec2(0.0f,0.0f), 0);
      fd2->shape = &bs2;
      
      body10 = m_world->CreateBody(bd);
      body10->CreateFixture(fd1);
      body10->CreateFixture(fd2);

      //**********************************************//

      b2BodyDef *bd2 = new b2BodyDef;
      bd2->type = b2_dynamicBody;
      bd2->position.Set(-23.25,33.5);

      b2Body* body11;
      
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 10.0;
      fd3->friction = 0.051f;
      fd3->restitution = -1000.0f;
      fd3->shape = new b2PolygonShape;
      b2PolygonShape bs3;
      bs3.SetAsBox(1.5,1, b2Vec2(0.0f,0.0f), 0);
      fd3->shape = &bs3;

      b2FixtureDef *fd4 = new b2FixtureDef;
      fd4->density = 10.0;
      fd4->friction = 0.0f;
      fd4->restitution = 0.0f;
      fd4->shape = new b2PolygonShape;
      b2PolygonShape bs4;
      bs4.SetAsBox(0.75,0.62, b2Vec2(2.25f,0.38f), 0);
      fd4->shape = &bs4;

      body11 = m_world->CreateBody(bd2);
      body11->CreateFixture(fd3);
      body11->CreateFixture(fd4);

      //********************************************//

      b2DistanceJointDef* distance_joint = new b2DistanceJointDef();

      b2Vec2 anchor1;
      anchor1.Set(-28.5, 33.5);
      b2Vec2 anchor2;
      anchor2.Set(-23.25, 33.5);
      
      distance_joint->Initialize(body10,body11,anchor1,anchor2);

      distance_joint->length =5.25f;
      distance_joint->collideConnected = true;
      distance_joint->frequencyHz =1.0f;
      distance_joint->dampingRatio = -0.097f;

      m_world->CreateJoint(distance_joint);

    }

    /////////////////////////////////////////////

    //small stand-second ball
    {
      b2Body* sbody;
      b2PolygonShape poly;
      b2Vec2 vertices[4];
      vertices[0].Set(0,0);
      vertices[1].Set(0.75,0);
      vertices[2].Set(0.75,0.75);
      vertices[3].Set(0,0.75);
      poly.Set(vertices, 4);
      b2FixtureDef wedgefd;
      wedgefd.shape = &poly;
      wedgefd.density = 10.0f;
      wedgefd.friction = 0.0f;
      wedgefd.restitution = -1000.0f;
      b2BodyDef wedgebd;
      wedgebd.position.Set(-12.25f, 32.7f);
      sbody = m_world->CreateBody(&wedgebd);
      sbody->CreateFixture(&wedgefd);
    }

    /////////////////////////////////////

    //second ball

    {
      b2CircleShape circle;
      circle.m_radius = 0.6f;
  
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 500.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = -2.0f;
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      ballbd.position.Set(-11.875f,33.675f);
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);
    }

    ///////////////////////////////////////////////////////////////////
  }
  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}