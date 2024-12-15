#include "stdafx.h"

// standard

#include <math.h>
#include<stdlib.h>
// glut
#include <GL/glut.h>

// Include standard headers
#include <assert.h>





// Include standard headers
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <sstream>

// Include GLUT and OpenGL
#define GLUT_DISABLE_ATEXIT_HACK


// Define constants
#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

#include "Quaternion.h"
#include "Interpolation.h"


// Represents a single boid with position, velocity, angular velocity, orientation, and other properties
struct Boid {
    Vec3 pos;
    Vec3 vel;
    Vec3 angVel;
    Quaternion ori;
    double radius;
    double mass;
    double momentOfInertia;
    Vec3 color;
};

double NUM_BOIDS = 30;
double dt = 0.016;


int g_windowWidth = 800;
int g_windowHeight = 600;

std::vector<Boid> boids;

double maxSpeed = 6.0;
double minSpeed = 3.0;
double neighborhoodRadius = 8.0; // Radius within which boids interact
double personalSpaceRadius = 3.0; // If two boids come within this distance, we respawn them

double separationWeight = 5.0;
double alignmentWeight = 0.1;
double cohesionWeight = 0.1;

// Predator
Vec3 predatorPos = { 0.0,0.0,0.0 };
double predatorRadius = 5;
double predatorAttractionWeight = 3.0;

double boundSize = 50.0;

// Timing Variables
double lastFrameTime = 0.0;
double totalTime = 0.0;

// Spawn a new boid at random position & velocity
void spawnBoid(Boid& b) {
    b.radius = 1.0;
    b.mass = 1.0;
    b.momentOfInertia = (2.0 / 5.0) * b.mass * (b.radius * b.radius);

    b.pos.x = ((std::rand() % 100) / 100.0 - 0.5) * boundSize;
    b.pos.y = ((std::rand() % 100) / 100.0 - 0.5) * boundSize;
    b.pos.z = ((std::rand() % 100) / 100.0 - 0.5) * boundSize;

    b.vel.x = ((std::rand() % 100) / 100.0 - 0.5) * maxSpeed;
    b.vel.y = ((std::rand() % 100) / 100.0 - 0.5) * maxSpeed;
    b.vel.z = ((std::rand() % 100) / 100.0 - 0.5) * maxSpeed;

    double roll = ((std::rand() % 360) / 180.0) * M_PI;
    double pitch = ((std::rand() % 360) / 180.0) * M_PI;
    double yaw = ((std::rand() % 360) / 180.0) * M_PI;
    b.ori = eulerToQuaternion(roll, pitch, yaw);

    b.angVel.x = ((std::rand() % 100) / 100.0 - 0.5) * 2.0;
    b.angVel.y = ((std::rand() % 100) / 100.0 - 0.5) * 2.0;
    b.angVel.z = ((std::rand() % 100) / 100.0 - 0.5) * 2.0;

    b.color.x = (std::rand() % 100) / 100.0;
    b.color.y = (std::rand() % 100) / 100.0;
    b.color.z = (std::rand() % 100) / 100.0;
}

// Initializes all boids and places the predator at a random position
void initBoids() {
    boids.resize(NUM_BOIDS);
    std::srand((unsigned)std::time(0));
    for (int i = 0; i < NUM_BOIDS; i++) {
        spawnBoid(boids[i]);
    }
    predatorPos.x = ((std::rand() % 100) / 100.0 - 0.5) * boundSize;
    predatorPos.y = ((std::rand() % 100) / 100.0 - 0.5) * boundSize;
    predatorPos.z = ((std::rand() % 100) / 100.0 - 0.5) * boundSize;
}


//Ensures each boid maintains speed between minSpeed and maxSpeed
void limitSpeed(Boid& b) {
    double speed = b.vel.Magnitude();
    if (speed > maxSpeed) {
        b.vel = b.vel.Normalized() * maxSpeed;
    }
    else if (speed < minSpeed && speed > 0.0001) {
        b.vel = b.vel.Normalized() * minSpeed;
    }
    else if (speed < 0.01) {
        // random kick if too slow
        b.vel.x = ((std::rand() % 100) / 100.0 - 0.5) * maxSpeed;
        b.vel.y = ((std::rand() % 100) / 100.0 - 0.5) * maxSpeed;
        b.vel.z = ((std::rand() % 100) / 100.0 - 0.5) * maxSpeed;
    }
}




//----------------------------------------------------------
// Flocking Behavior Functions
//----------------------------------------------------------


Vec3 computeSeparation(int i) {
    Vec3 steer = { 0.0,0.0,0.0 };
    int count = 0;
    for (int j = 0; j < NUM_BOIDS; j++) {
        if (j == i) continue;
        Vec3 diff = boids[j].pos - boids[i].pos;
        double dist = diff.Magnitude();
        // If within neighborhood, apply inverse cube law to strongly repel at close range
        if (dist < neighborhoodRadius && dist > 0.0001) {
            double scale = 1.0 / (dist * dist * dist); // inverse cube for strong push
            Vec3 away = diff * scale;
            steer = steer - away;
            count++;
        }
    }
    if (count > 0) {
        steer = steer * (1.0 / count);
    }
    return steer * separationWeight;
}

//Match velocity with neighbors
Vec3 computeAlignment(int i) {
    Vec3 avgVel = { 0.0,0.0,0.0 };
    int count = 0;
    for (int j = 0; j < NUM_BOIDS; j++) {
        if (j == i) continue;
        Vec3 diff = boids[j].pos - boids[i].pos;
        double dist = diff.Magnitude();
        if (dist < neighborhoodRadius) {
            avgVel = avgVel + boids[j].vel;
            count++;
        }
    }
    if (count > 0) {
        avgVel = avgVel * (1.0 / count);
        avgVel = avgVel.Normalized() * maxSpeed;
        Vec3 steer = avgVel - boids[i].vel;
        return steer * alignmentWeight;
    }
    return Vec3{ 0.0,0.0,0.0 };
}

// Move towards the average position of neighbors
Vec3 computeCohesion(int i) {
    Vec3 center = { 0.0,0.0,0.0 };
    int count = 0;
    for (int j = 0; j < NUM_BOIDS; j++) {
        if (j == i) continue;
        Vec3 diff = boids[j].pos - boids[i].pos;
        double dist = diff.Magnitude();
        if (dist < neighborhoodRadius) {
            center = center + boids[j].pos;
            count++;
        }
    }
    if (count > 0) {
        center = center * (1.0 / count);
        Vec3 diff = center - boids[i].pos;
        Vec3 desired = diff.Normalized() * maxSpeed;
        Vec3 steer = desired - boids[i].vel;
        return steer * cohesionWeight;
    }
    return Vec3{ 0.0,0.0,0.0 };
}

Vec3 confineToBounds(const Boid& b) {
    Vec3 steer = { 0.0,0.0,0.0 };
    double boundary = boundSize;
    double factor = 2.0;

    if (b.pos.x > boundary) steer.x = -factor;
    else if (b.pos.x < -boundary) steer.x = factor;

    if (b.pos.y > boundary) steer.y = -factor;
    else if (b.pos.y < -boundary) steer.y = factor;

    if (b.pos.z > boundary) steer.z = -factor;
    else if (b.pos.z < -boundary) steer.z = factor;

    return steer;
}


//Steer towards the predator's position
Vec3 seekPredator(int i) {
    Vec3 diff = predatorPos - boids[i].pos;
    double dist = diff.Magnitude();
    if (dist > 0.001) {
        Vec3 desired = diff.Normalized() * maxSpeed;
        Vec3 steer = desired - boids[i].vel;
        return steer * predatorAttractionWeight;
    }
    return Vec3{ 0.0,0.0,0.0 };
}

void checkPredatorInteraction() {
    for (int i = 0; i < NUM_BOIDS; i++) {
        Vec3 diff = boids[i].pos - predatorPos;
        double dist = diff.Magnitude();
        if (dist < boids[i].radius + predatorRadius) {
            predatorPos.x = ((std::rand() % 100) / 100.0 - 0.5) * boundSize;
            predatorPos.y = ((std::rand() % 100) / 100.0 - 0.5) * boundSize;
            predatorPos.z = ((std::rand() % 100) / 100.0 - 0.5) * boundSize;
        }
    }
}

// After updating physics, check if any two boids are inside personalSpaceRadius
// If so, eliminate them and spawn new boids at random positions
//------------------------starts here-----------------------------------
void enforceNoCollision() {
    

    for (int i = 0; i < NUM_BOIDS; i++) {
        for (int j = i + 1; j < NUM_BOIDS; j++) {
            Vec3 diff = boids[j].pos - boids[i].pos;
            double dist = diff.Magnitude();
            if (dist < personalSpaceRadius) {
                // Respawn both boids i and j with new positions
                //spawnBoid(boids[i]);
                spawnBoid(boids[j]);
            }
        }
    }
}

void updatePhysics() {
    std::vector<Vec3> steeringForces(NUM_BOIDS, Vec3{ 0.0,0.0,0.0 });
    for (int i = 0; i < NUM_BOIDS; i++) {
        Vec3 sep = computeSeparation(i);
        Vec3 ali = computeAlignment(i);
        Vec3 coh = computeCohesion(i);
        Vec3 conf = confineToBounds(boids[i]);
        Vec3 pred = seekPredator(i);

        Vec3 steer = sep + ali + coh + conf + pred;
        steeringForces[i] = steer;
    }

    for (int i = 0; i < NUM_BOIDS; i++) {
        boids[i].vel = boids[i].vel + steeringForces[i] * dt;
        limitSpeed(boids[i]);

        boids[i].pos.x += boids[i].vel.x * dt;
        boids[i].pos.y += boids[i].vel.y * dt;
        boids[i].pos.z += boids[i].vel.z * dt;

        boids[i].ori = quatIntegrate(boids[i].ori, boids[i].angVel, dt);
    }

    checkPredatorInteraction();

    // Enforce no collision by respawning boids that got too close
    enforceNoCollision();
}
//-------------------------end here for the existing code ---------------------

//-------use this code if don't want to reswapn or eliminate one of the object while colliding starts here ---------------------- 
//void resolveCollisions() {
//    for (int i = 0; i < NUM_BOIDS; i++) {
//        for (int j = i + 1; j < NUM_BOIDS; j++) {
//            Vec3 diff = boids[j].pos - boids[i].pos;
//            double dist = diff.Magnitude();
//            double combinedRadius = boids[i].radius + boids[j].radius;
//
//            if (dist < combinedRadius && dist > 0.0001) {
//                // They are overlapping, resolve the collision
//                double penetrationDepth = (combinedRadius - dist);
//
//                // Normalize the difference vector
//                Vec3 direction = diff * (1.0 / dist);
//
//                // Apply half penetration to each boid
//                Vec3 correction = direction * (penetrationDepth * 0.5);
//
//                // Adjust positions
//                boids[i].pos = boids[i].pos - correction;
//                boids[j].pos = boids[j].pos + correction;
//
//                // Optionally, adjust velocities slightly if you want to simulate a "bounce".
//                // For a simple correction, you might leave velocities as is.
//            }
//        }
//    }
//}
//
//void updatePhysics() {
//    std::vector<Vec3> steeringForces(NUM_BOIDS, Vec3{ 0.0,0.0,0.0 });
//    for (int i = 0; i < NUM_BOIDS; i++) {
//        Vec3 sep = computeSeparation(i);
//        Vec3 ali = computeAlignment(i);
//        Vec3 coh = computeCohesion(i);
//        Vec3 conf = confineToBounds(boids[i]);
//        Vec3 pred = seekPredator(i);
//
//        Vec3 steer = sep + ali + coh + conf + pred;
//        steeringForces[i] = steer;
//    }
//
//    for (int i = 0; i < NUM_BOIDS; i++) {
//        boids[i].vel = boids[i].vel + steeringForces[i] * dt;
//        limitSpeed(boids[i]);
//
//        boids[i].pos.x += boids[i].vel.x * dt;
//        boids[i].pos.y += boids[i].vel.y * dt;
//        boids[i].pos.z += boids[i].vel.z * dt;
//
//        boids[i].ori = quatIntegrate(boids[i].ori, boids[i].angVel, dt);
//    }
//
//    checkPredatorInteraction();
//
//    // Now resolve collisions without respawning
//    resolveCollisions();
//}

//------------ends here----------------------------------------


//----------------------------------------------------------
// Render
//----------------------------------------------------------


void drawBoid(const Boid& b) {
    glPushMatrix();
    float M[16];
    buildTransformMatrix(b.ori, b.pos, M);
    glMultMatrixf(M);

    
    glColor3f((GLfloat)b.color.x, (GLfloat)b.color.y, (GLfloat)b.color.z);
    
   
    glutSolidSphere((GLfloat)b.radius, 32, 32);
    glPopMatrix();
}



void drawPredator() {
    glPushMatrix();
    glTranslatef((GLfloat)predatorPos.x, (GLfloat)predatorPos.y, (GLfloat)predatorPos.z);
    glColor3f(1.0f, 1.0f, 1.0f);
    glutSolidSphere((GLfloat)predatorRadius, 100, 100);
    glPopMatrix();
}

void renderFloor() {
    glDisable(GL_LIGHTING);
    glColor3f(0.3f, 0.3f, 0.3f);//
    double size = 50.0;
    glBegin(GL_QUADS);
    glVertex3f(-size, 0.0f, -size);
    glVertex3f(size, 0.0f, -size);
    glVertex3f(size, 0.0f, size);
    glVertex3f(-size, 0.0f, size);
    glEnd();
    glEnable(GL_LIGHTING);
}

void display() {

    // Clear buffer
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f); //background
    glClearDepth(1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Render state
    glEnable(GL_DEPTH_TEST);
    glShadeModel(GL_SMOOTH);

    // Enable lighting
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

    GLfloat LightAmbient[] = { 0.4f,0.4f,0.4f,1.0f };
    GLfloat LightDiffuse[] = { 0.6f,0.6f,0.6f,1.0f };
    GLfloat LightSpecular[] = { 0.9f,0.9f,0.9f,1.0f };
    GLfloat LightPosition[] = { 5.0f,20.0f,20.0f,1.0f };

    glLightfv(GL_LIGHT0, GL_AMBIENT, LightAmbient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, LightDiffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, LightSpecular);
    glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);

    GLfloat material_Ka[] = { 0.8f,0.8f,0.8f,1.0f };
    GLfloat material_Kd[] = { 0.8f,0.8f,0.8f,1.0f };
    GLfloat material_Ks[] = { 0.9f,0.9f,0.9f,1.0f };
    //GLfloat material_Ke[] = { 0.1f, 0.0f, 0.1f, 1.0f };
    GLfloat material_Se = 10.0f;

    glMaterialfv(GL_FRONT, GL_AMBIENT, material_Ka);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, material_Kd);
    glMaterialfv(GL_FRONT, GL_SPECULAR, material_Ks);
    glMaterialf(GL_FRONT, GL_SHININESS, material_Se);

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_NORMALIZE);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0.0, 50.0, 70.0,
        0.0, 0.0, 0.0,
        0.0, 1.0, 0.0);
  
    //renderFloor();

    for (int i = 0; i < NUM_BOIDS; i++) {
        drawBoid(boids[i]);
    }

    drawPredator();

    

    // Swap back and front buffers
    glutSwapBuffers();
}

void keyboard(unsigned char key, int x, int y) {
    // Handle keyboard input if needed
    if (key == 27) { // ESC key
        exit(0);
    }
}



void reshape(int w, int h) {
    //screen size
    g_windowWidth = w;
    g_windowHeight = h;

    //Viewport
    glViewport(0, 0, w, h);

    //Projection Matrix
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, (double)w / (double)h, 0.1, 1000.0);
 
}

void timer(int value) {
    //Get curret time
    double currentTime = glutGet(GLUT_ELAPSED_TIME)/1000; //time in seconds
    double frameTime = (currentTime - (lastFrameTime));
    lastFrameTime = currentTime;
    totalTime += dt;

    
    updatePhysics();
 

    //Render
    glutPostRedisplay();

    //Reset timer
    glutTimerFunc(16, timer, 0);
}



int main(int argc, char** argv) {
   
    
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(g_windowWidth, g_windowHeight);
    glutInitWindowPosition(100, 100);
    glutCreateWindow("Flocking with Predator");

    // Init
    initBoids();

    // Set callback functions
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutTimerFunc(16, timer, 0);

    //Main Loop
    glutMainLoop();
    return 0;
}

