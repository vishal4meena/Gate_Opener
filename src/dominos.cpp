
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
    
  dominos_t::dominos_t()
  {
    //Ground  
    { 
      b2Body* b1;
      b2EdgeShape shape; 
      shape.Set(b2Vec2(-900.0f, 0.0f), b2Vec2(900.0f, 0.0f));
      b2BodyDef bd; 
      b1 = m_world->CreateBody(&bd); 
      b1->CreateFixture(&shape, 0.0f);
    }
    
    /////////////////////////////////////////////

    //first ball
    {
      b2Body* sbody;
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

    //http://www.emanueleferonato.com/2009/01/05/box2d-joints-distance-joint/
    //http://www.learn-cocos2d.com/api-ref/1.0/Box2D/html/classb2_distance_joint.html

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
      b2Body* sbody;
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

    // revolving plank
    {
      b2PolygonShape shape;
      shape.SetAsBox(3.75f, 0.1f);
  
      b2BodyDef bd;
      bd.position.Set(-6.75f, 30.5f);
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 500.0f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);

      b2PolygonShape shape2;
      b2BodyDef bd2;
      bd2.position.Set(-6.25f, 30.5f);
      b2Body* body2 = m_world->CreateBody(&bd2);

      b2RevoluteJointDef jointDef;
      jointDef.bodyA = body;
      jointDef.bodyB = body2;
      jointDef.localAnchorA.Set(0,0);
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = true;
      m_world->CreateJoint(&jointDef);
    }   

    ///////////////////////////////////////////////////////////////////////

    // plank-dominos
    {
      b2PolygonShape shape12;
      shape12.SetAsBox(12.0f, 0.2f);
      b2BodyDef bd11;
      bd11.position.Set(-21.0f, 28.8f);
      b2Body* body12 = m_world->CreateBody(&bd11);
      b2FixtureDef *fd11 = new b2FixtureDef;
      fd11->friction=100.0f;
      fd11->density = 10.0f;
      fd11->shape = new b2PolygonShape;
      fd11->shape = &shape12;
      body12->CreateFixture(fd11);
    }

    /////////////////////////////////////////////////////////////////////////

    //dominos
    {
      b2PolygonShape shape;
      shape.SetAsBox(0.1f, 1.0f);
  
      b2FixtureDef fd;
      fd.shape = &shape;
      fd.density = 700.0f;
      fd.friction = 0.5f;
      fd.restitution=0.0f;
    
      for (int i = 0; i < 13; ++i)
      {
        b2BodyDef bd;
        bd.type = b2_dynamicBody;
        bd.position.Set(-29.625f + 1.25f * i, 29.5f);
        b2Body* body = m_world->CreateBody(&bd);
        body->CreateFixture(&fd);
      }

      b2FixtureDef fd1;
      fd1.shape = &shape;
      fd1.density = 700.0f;
      fd1.friction = 0.01f;
      fd1.restitution=0.0f;
      b2BodyDef bd1;
      bd1.type = b2_dynamicBody;
      bd1.position.Set(-31.5f , 30.5f);
      b2Body* body = m_world->CreateBody(&bd1);
      body->CreateFixture(&fd1);
    }

    ////////////////////////////////////////////////////////
    
    //plank-brain-dots
    {
      b2PolygonShape shape12;
      shape12.SetAsBox(7.0f, 0.2f);
      b2BodyDef bd11;
      bd11.position.Set(-34.5f, 23.8f);
      //bd11.type = b2_dynamicBody;
      b2Body* body12 = m_world->CreateBody(&bd11);
      b2FixtureDef *fd11 = new b2FixtureDef;
      fd11->friction=100.0f;
      //fd11->restitution=100.0f;
      fd11->density = 10.0f;
      fd11->shape = new b2PolygonShape;
      fd11->shape = &shape12;
      body12->CreateFixture(fd11);
    }

    /////////////////////////////////////////////////////////////

    //v shape bucket
    {
      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(-34.5,24);

      b2Body* body10;

      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 10.0;
      fd1->friction = 0.1f;
      fd1->restitution = 0.f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape poly;
      b2Vec2 vertices[4];
      vertices[0].Set(0,0);
      vertices[1].Set(0,0.3);
      vertices[2].Set(-3,2.3);
      vertices[3].Set(-3,2);
      poly.Set(vertices, 4);
      fd1->shape = &poly;

      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 10.0;
      fd2->friction = 0.4;
      fd2->restitution = 0.0f;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape poly2;
      b2Vec2 vertices2[4];
      vertices2[0].Set(0,0);
      vertices2[1].Set(3,2);
      vertices2[2].Set(3,2.3);
      vertices2[3].Set(0,0.3);
      poly2.Set(vertices2, 4);
      fd2->shape = &poly2;

      body10 = m_world->CreateBody(bd);
      body10->CreateFixture(fd1);
      body10->CreateFixture(fd2);
    }


    ////////////////////////////////////
        
    //third ball
    {
      b2Body* sbody;
      b2CircleShape circle;
      circle.m_radius = 0.6;
      
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 75.0f;
      ballfd.friction = 0.001f;
      ballfd.restitution = -20.0f;
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      ballbd.position.Set(-32.5f, 24.6f);
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);
    }

    ///////////////////////////////////////////////

    //plank-bucket-full of balls
    {
      b2PolygonShape shape12;
      shape12.SetAsBox(8.0f, 0.2f);
      b2BodyDef bd11;
      bd11.position.Set(-19.0f, 19.8f);
      //bd11.type = b2_dynamicBody;
      b2Body* body12 = m_world->CreateBody(&bd11);
      b2FixtureDef *fd11 = new b2FixtureDef;
      fd11->friction=100.0f;
      fd11->density = 10.0f;
      fd11->shape = new b2PolygonShape;
      fd11->shape = &shape12;
      body12->CreateFixture(fd11);
    }

    
    ///////////////////////////////////////////////

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
      wedgefd.restitution = -1.0f;
      b2BodyDef wedgebd;
      wedgebd.position.Set(-27.0f, 20.0f);
      sbody = m_world->CreateBody(&wedgebd);
      sbody->CreateFixture(&wedgefd);
    }

    //v shape bucket full of balls
    {
      b2Body* sbody;
      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(-19,20);

      b2Body* body10;

      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 60.0f;
      fd1->friction = 0.1f;
      fd1->restitution = 0.0f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape poly;
      b2Vec2 vertices[4];
      vertices[0].Set(0,0);
      vertices[1].Set(0,0.4);
      vertices[2].Set(-2,3.4);
      vertices[3].Set(-2,3);
      poly.Set(vertices, 4);
      fd1->shape = &poly;

      //*********************************************//

      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 60.0;
      fd3->friction = 0.4;
      fd3->restitution = 0.0f;
      fd3->shape = new b2PolygonShape;
      b2PolygonShape poly3;
      b2Vec2 vertices3[4];
      vertices3[0].Set(0,0);
      vertices3[1].Set(1.5,0);
      vertices3[2].Set(1.5,0.4);
      vertices3[3].Set(0,0.4);
      poly3.Set(vertices3, 4);
      fd3->shape = &poly3;

      //*******************************************//

      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 60.0;
      fd2->friction = 0.4;
      fd2->restitution = 0.0f;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape poly2;
      b2Vec2 vertices2[4];
      vertices2[0].Set(1.5,0);
      vertices2[1].Set(3.5,3);
      vertices2[2].Set(3.5,3.4);
      vertices2[3].Set(1.5,0.4);
      poly2.Set(vertices2, 4);
      fd2->shape = &poly2;

      body10 = m_world->CreateBody(bd);
      body10->CreateFixture(fd1);
      body10->CreateFixture(fd3);
      body10->CreateFixture(fd2);

      //********************************************//

      b2CircleShape circle;
      circle.m_radius = 0.3;
      
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 4.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;
    
      for (int i = 0; i < 5; ++i)
      {
        b2BodyDef ballbd;
        ballbd.type = b2_dynamicBody;
        ballbd.position.Set(-17.95,20.3+ i*1);
        sbody = m_world->CreateBody(&ballbd);
        sbody->CreateFixture(&ballfd);
        ballbd.position.Set(-18.55,20.3+ i*1);
        sbody = m_world->CreateBody(&ballbd);
        sbody->CreateFixture(&ballfd);
      }
    }

    //////////////////////////////////////////////////

    //pulley system
    {
      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(-8.7,13);
      bd->fixedRotation = true;
      
      //The open box

      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 10.0;
      fd1->friction = 0.5;
      fd1->restitution = 0.0f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(1.8,0.1, b2Vec2(0.0f,0.0f), 0);
      fd1->shape = &bs1;

      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 10.0;
      fd2->friction = 0.5;
      fd2->restitution = 0.0f;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(0.1,1.5, b2Vec2(-1.90f,1.4f), 0);
      fd2->shape = &bs2;

      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 10.0;
      fd3->friction = 0.5;
      fd3->restitution = 0.0f;
      fd3->shape = new b2PolygonShape;
      b2PolygonShape bs3;
      bs3.SetAsBox(0.1,1.5, b2Vec2(1.90f,1.4f), 0);
      fd3->shape = &bs3;
       
      b2Body* box1 = m_world->CreateBody(bd);
      box1->CreateFixture(fd1);
      box1->CreateFixture(fd2);
      box1->CreateFixture(fd3);

      //The bar
      bd->position.Set(-3.5,13);  
      fd1->density =40.0f;    
      b2Body* box2 = m_world->CreateBody(bd);
      box2->CreateFixture(fd1);

      // The pulley joint
      b2PulleyJointDef* myjoint = new b2PulleyJointDef();
      b2Vec2 worldAnchorOnBody1(-8.5, 13); 
      b2Vec2 worldAnchorOnBody2(-3.5, 13); 
      b2Vec2 worldAnchorGround1(-8.5, 17); 
      b2Vec2 worldAnchorGround2(-3.5, 17);
      float32 ratio = 1.0f; 
      myjoint->Initialize(box1, box2, worldAnchorGround1, worldAnchorGround2, box1->GetWorldCenter(), box2->GetWorldCenter(), ratio);
      m_world->CreateJoint(myjoint);
    }

    ///////////////////////////////////////////////////////////////////

    // revolving plank
    {
      b2PolygonShape shape;
      shape.SetAsBox(1.0f, 0.2f);
  
      b2BodyDef bd;
      bd.position.Set(-0.9f, 14.7f);
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 100.0f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);

      b2PolygonShape shape2;
      b2BodyDef bd2;
      bd2.position.Set(-0.9f, 14.7f);
      b2Body* body2 = m_world->CreateBody(&bd2);

      b2RevoluteJointDef jointDef;
      jointDef.bodyA = body;
      jointDef.bodyB = body2;
      jointDef.localAnchorA.Set(0,0);
      jointDef.localAnchorB.Set(0,0);
      m_world->CreateJoint(&jointDef);

      //*******************************************//

      b2PolygonShape shape3;
      shape3.SetAsBox(1.0f, 0.2f);
  
      b2BodyDef bd3;
      bd3.position.Set(0.8f, 14.2f);
      bd3.type = b2_dynamicBody;
      b2Body* body3 = m_world->CreateBody(&bd3);
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 100.0f;
      fd3->shape = new b2PolygonShape;
      fd3->shape = &shape3;
      body3->CreateFixture(fd3);

      b2PolygonShape shape4;
      b2BodyDef bd4;
      bd4.position.Set(0.8f, 14.2f);
      b2Body* body4 = m_world->CreateBody(&bd4);

      b2RevoluteJointDef jointDef2;
      jointDef2.bodyA = body3;
      jointDef2.bodyB = body4;
      jointDef2.localAnchorA.Set(0,0);
      jointDef2.localAnchorB.Set(0,0);
      m_world->CreateJoint(&jointDef2);
    }
    
    //////////////////////////////////////////////////

    {
      b2PolygonShape shape;
      shape.SetAsBox(1.0f, 0.2f);
  
      b2BodyDef bd;
      bd.position.Set(2.5f, 13.8f);
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 0.001f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);

      b2PolygonShape shape2;
      b2BodyDef bd2;
      bd2.position.Set(2.5f, 13.8f);
      b2Body* body2 = m_world->CreateBody(&bd2);

      b2RevoluteJointDef jointDef;
      jointDef.bodyA = body;
      jointDef.bodyB = body2;
      jointDef.localAnchorA.Set(0,0);
      jointDef.localAnchorB.Set(0,0);
      m_world->CreateJoint(&jointDef);
    }

    ///////////////////////////////////////////

    //pendulums
    {
      b2CircleShape circle;
      circle.m_radius = 0.6f;
  
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 0.01f;
      ballfd.friction = 1.0f;
      ballfd.restitution = 1.0f;

      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      ballbd.position.Set(3.3f,14.6f);
      b2Body* b2 = m_world->CreateBody(&ballbd);
      b2->CreateFixture(&ballfd);
  
      b2Body* b4;
      {
        b2PolygonShape shape;
        shape.SetAsBox(4.2f, 0.2f);
          
        b2BodyDef bd;
        //bd.type = b2_dynamicBody;
        bd.position.Set(14.6f, 14.6f);
        b4 = m_world->CreateBody(&bd);
        b4->CreateFixture(&shape, 2.0f);
      }
  
      b2RevoluteJointDef jd;

      b2Vec2 anchor;
      anchor.Set(10.8f, 14.6f);


      jd.Initialize(b2, b4, anchor);
      m_world->CreateJoint(&jd);

      for(int i=0;i<6;i++)
      {
        b2BodyDef ballbd;
        ballbd.type = b2_dynamicBody;
        ballbd.position.Set(12+i*1.2001,7.1f);
        b2Body* b3 = m_world->CreateBody(&ballbd);
        b3->CreateFixture(&ballfd);

        b2RevoluteJointDef jd1;

        anchor.Set(12.0+ i*1.2001, 14.6f);

        jd1.Initialize(b3, b4, anchor);
        m_world->CreateJoint(&jd1);
      }
    }  

    /////////////////////////////////////////////////

    //another revolving plank
    {
      b2PolygonShape shape;
      shape.SetAsBox(2.0f, 0.2f);
    
      b2BodyDef bd;
      bd.position.Set(27.9f, 13.7f);
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 0.0001f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);

      b2PolygonShape shape2;
      b2BodyDef bd2;
      bd2.position.Set(27.9f, 13.7f);
      b2Body* body2 = m_world->CreateBody(&bd2);

      b2RevoluteJointDef jointDef;
      jointDef.bodyA = body;
      jointDef.bodyB = body2;
      jointDef.localAnchorA.Set(0,0);
      jointDef.localAnchorB.Set(0,0);
      //jointDef.collideConnected = true;
      m_world->CreateJoint(&jointDef);
    }
    
    //////////////////////////////////////

    //bar
    {
      b2PolygonShape shape;
      shape.SetAsBox(1.0f, 0.2f);
    
      b2BodyDef bd;
      bd.position.Set(27.9f, 14.1f);
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 0.01f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);
    }

    //////////////////////////////////////

    //another pulley system
    {
      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(28.9,10);
      bd->fixedRotation = true;
      
      //The Bar
      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density =0.1;
      fd1->friction = 0.5;
      fd1->restitution = 0.0f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(2,0.2);
      fd1->shape = &bs1;
      b2Body* box1 = m_world->CreateBody(bd);
      box1->CreateFixture(fd1);

      //The gate
      bd->position.Set(38.9,5);  
      fd1->density =40.0f;    
      b2Body* box2 = m_world->CreateBody(bd);
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 0.016f;
      fd2->friction = 0.5;
      fd2->restitution = 0.0f;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(0.5,5);
      fd2->shape = &bs2;
      box2->CreateFixture(fd2);

      // The pulley joint
      b2PulleyJointDef* myjoint = new b2PulleyJointDef();
      b2Vec2 worldAnchorOnBody1(28.9,10 ); 
      b2Vec2 worldAnchorOnBody2(38.9, 10);
      b2Vec2 worldAnchorGround1(28.9,12); 
      b2Vec2 worldAnchorGround2(38.9, 12);
      float32 ratio = 1.0f;
      myjoint->Initialize(box1, box2, worldAnchorGround1, worldAnchorGround2, box1->GetWorldCenter(), box2->GetWorldCenter(), ratio);
      m_world->CreateJoint(myjoint);
    }

    ///////////////////////////////////////

    //www/iforce2d.net/b2dtut/bodies
    //http://stackoverflow.com/questions/8007170/cocos2d-box2d-multiple-b2circle-shape-fixture-for-one-body-with-positioning

    //the car
    {
      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(-45,3);
      //bd->fixedRotation = true;
      
      //The open box

      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 10.0;
      fd1->friction = 0.0;
      fd1->restitution = 0.0f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(3,1.5, b2Vec2(0.0f,0.0f), 0);
      fd1->shape = &bs1;

      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 10.0;
      fd2->friction = 0.0;
      fd2->restitution = 0.0f;
      fd2->shape = new b2CircleShape;
      b2CircleShape circle;
      circle.m_radius = 0.75f;
      circle.m_p.Set(-2,-2.25);
      fd2->shape = &circle;

      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 10.0;
      fd3->friction = 0.0;
      fd3->restitution = 0.0f;
      fd3->shape = new b2CircleShape;
      b2CircleShape circle2;
      circle2.m_radius = 0.75f;
      circle2.m_p.Set(2,-2.25);
      fd3->shape = &circle2;

       
      b2Body* box1 = m_world->CreateBody(bd);
      box1->CreateFixture(fd1);
      box1->CreateFixture(fd2);
      box1->CreateFixture(fd3);

      box1->SetLinearVelocity(b2Vec2(1.477,0.0));
    }
  }
  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}