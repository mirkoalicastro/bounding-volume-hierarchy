#define _USE_MATH_DEFINES

#include <GL/gl.h>
#include <GL/glut.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <vector>
#include <iostream>
#include <ctime>
#include "bvhtree.h"

using namespace bvh;

/* WINDOW SETTINGS */
int w_width=600, w_height=600;

/* CAMERA SETTINGS */
float lx=0, lz=-1;
float eyex=0, eyey=0.5, eyez=20;
float centery=-0.525;
float angle=0;
float stepad=0.1, stepws=0.1, stepz=0.5;

/* MATERIAL DEFAULT SETTINGS */
float ambient[4] = {0.2,0.2,0.2,1};
float diffuse[4] = {0.8,0.8,0.8,1};
float emission[4] = {0,0,0,1};
float specular[4] = {0,0,0,1};
float zero[4] = {0,0,0,1};

/* SCENE OBJECTS POSITION */
std::vector<float> xs, ys, zs, rs; //asteroids
float rocketx, rockety, rocketz; //rocket

/* TIMING VARIABLES FOR FPS AND COLLISIONS COUNTER */
long timeframe = -1;
int tmpframe = 0, curframe = 0, frame = 0, timebase = 0;

/* ROCKET COLOR FOR EACH PART */
float rocketcolor[7][3] = {
    {0./255., 200./255., 175./255.}, //MAIN
    {0./255., 200./255., 175./255.}, //LEFT
    {0./255., 200./255., 175./255.}, //RIGHT
    {0./255., 200./255., 175./255.}, //FORWARD
    {0./255., 200./255., 175./255.}, //BACK
    {0./255., 200./255., 175./255.}, //TOP
    {0./255., 200./255., 175./255.}  //BOTTOM
};

/* ROCKET COLOR FOR EACH PART ON COLLISION */
float rocketactivecolor[7][3] = {
    {255./255., 55./255., 125./255.}, //MAIN
    {255./255., 55./255., 125./255.}, //LEFT
    {255./255., 55./255., 125./255.}, //RIGHT
    {255./255., 55./255., 125./255.}, //FORWARD
    {255./255., 55./255., 125./255.}, //BACK
    {255./255., 55./255., 125./255.}, //TOP
    {255./255., 55./255., 125./255.} //BOTTOM
};

float raycolor[3] = {50./255., 225./255., 255./255.};

float asteroidactivecolor[3] = {255./255., 50./255., 35./255.}; //COLOR FOR ASTEROIDS ON COLLISION

float rocketmainradius = 0.3f; //RADIUS OF ROCKET MAIN PART
float rocketsecondaryradius = 0.1f; //RADIUS OF ROCKET SECONDARY PARTS
float asteroidradius = 0.06f; //RADIUS OF ASTEROIDS
float rocketstep = 0.05; //ROCKET VELOCITY

char space[42] = "*****************************************"; //SEPARATOR CHARACTERS FOR CONSOLE DEBUGGING

float threshold = 0.5; //FOR BVH BOTTOM-UP GENERATION

/* GRID PARAMS FOR ASTEROIDS RANDOM GENERATION */
int from = -4; //-6;
int to = 4; //6;
float dense = 0.75; //0.75f;

/* SUPPORT VARIABLES */
float gox, goy, goz;
bool minusx, minusy, minusz;
int cm = 0, cl = 0, cr = 0, cf = 0, cba = 0, ct = 0, cbo = 0;
int seconds = 0;
BVHTree asteroidsTree;
BVHTree rocketTree;
Ray bulletRay;

/* QUALITY OF SPHERE RENDERING */
int slices = 4, stacks = 4;

int leveltree = 0; //LEVEL OF THE BVH TREE FOR DISPLAYING asteroid AABBs
int levelrtree = 0; //LEVEL OF THE BVH TREE FOR DISPLAYING rocket AABB

float bulletDirx = 0, bulletDiry = 0, bulletDirz = 0; // DIRECTION VECTOR FOR BULLET
bool showbullet = false; //SET TO TRUE TO SHOW AND TEST RAY COLLISION

bool stoprocket = false;

float lastx, lasty, lastz;

bool bvhDetection = false; //SET TO TRUE TO USE THE BVH TREE FOR COLLISION DETECTION, FALSE OTHERWISE
bool detailedCollision = true;

void createRocketTree() {
    if(lastx == rocketx && lasty == rockety && lastz == rocketz)
        return;
    rocketTree = BVHTree();
    NodeC* n[7];
    n[0] = new NodeC();
    n[0]->setType(Node::Type::LEAF);
    n[0]->setID(-1);
    n[0]->setObject(new NodeC::Circle(rocketx, rockety, rocketz, rocketmainradius));

    n[1] = new NodeC();
    n[1]->setType(Node::Type::LEAF);
    n[1]->setID(-2);
    n[1]->setObject(new NodeC::Circle(rocketx-rocketmainradius, rockety, rocketz, rocketsecondaryradius));

    n[2] = new NodeC();
    n[2]->setType(Node::Type::LEAF);
    n[2]->setID(-3);
    n[2]->setObject(new NodeC::Circle(rocketx+rocketmainradius, rockety, rocketz, rocketsecondaryradius));

    n[3] = new NodeC();
    n[3]->setType(Node::Type::LEAF);
    n[3]->setID(-4);
    n[3]->setObject(new NodeC::Circle(rocketx, rockety+rocketmainradius, rocketz, rocketsecondaryradius));

    n[4] = new NodeC();
    n[4]->setType(Node::Type::LEAF);
    n[4]->setID(-5);
    n[4]->setObject(new NodeC::Circle(rocketx, rockety-rocketmainradius, rocketz, rocketsecondaryradius));

    n[5] = new NodeC();
    n[5]->setType(Node::Type::LEAF);
    n[5]->setID(-6);
    n[5]->setObject(new NodeC::Circle(rocketx, rockety, rocketz+rocketmainradius, rocketsecondaryradius));

    n[6] = new NodeC();
    n[6]->setType(Node::Type::LEAF);
    n[6]->setID(-7);
    n[6]->setObject(new NodeC::Circle(rocketx, rockety, rocketz-rocketmainradius, rocketsecondaryradius));
    
    rocketTree.bottom_up(n, 7);

    lastx = rocketx;
    lasty = rockety;
    lastz = rocketz;
}

void createTree() {
    asteroidsTree = BVHTree(threshold);
    int numObjects = xs.size();
    long timeinseconds = (long) std::time(nullptr);
    NodeC* nodes[numObjects];
    for (int i = 0; i < numObjects; i++) {
        nodes[i] = new NodeC();
        nodes[i]->setType(Node::Type::LEAF);
        NodeC::Circle* circle = new NodeC::Circle(xs.at(i), ys.at(i), zs.at(i), rs.at(i));
        nodes[i]->setID(i);
        nodes[i]->setObject(circle);
    }
    asteroidsTree.bottom_up(nodes, numObjects, true);
    timeinseconds = (long) std::time(nullptr) - timeinseconds;
    std::cout << "Seconds to built BVH Tree with " << numObjects << " nodes: " << timeinseconds << "s" << std::endl;
    std::cout << "Press 'y' to continue" << std::endl;
    char w[1];
    std::cin >> w;
}

float limit(float min, float x, float max) {
    if(x < min)
        return min;
    if(x > max)
        return max;
    return x;
}

void restoreMaterial() {
    glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,ambient);
    glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,diffuse);
    glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,emission);
    glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,specular);
    glMaterialf(GL_FRONT_AND_BACK,GL_SHININESS,0);
}

void checkError(char *label) {
    GLenum error;
    while((error=glGetError()) != GL_NO_ERROR)
        printf("%s: %s\n", label, gluErrorString(error));
}

void changeSize(int w, int h) {
    if (h == 0)
        h = 1;

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glViewport(0, 0, w, h);
    gluPerspective(45.0f, (float)w/(float)h, 0.1f, 100.0f);
    glMatrixMode(GL_MODELVIEW);
}

void rocket(bool main, bool left, bool right, bool forward, bool back, bool top, bool bottom) {
    restoreMaterial();

    glPushMatrix();
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, main ? rocketactivecolor[0] : rocketcolor[0]);
        glutSolidSphere(rocketmainradius, slices, stacks); //main
    glPopMatrix();

    glPushMatrix();
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, left  ? rocketactivecolor[1] : rocketcolor[1]);
        glTranslatef(-rocketmainradius, 0, 0);
        glutSolidSphere(rocketsecondaryradius, slices, stacks); //left
    glPopMatrix();

    glPushMatrix();
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, right ? rocketactivecolor[2] : rocketcolor[2]);
        glTranslatef(rocketmainradius, 0, 0);
        glutSolidSphere(rocketsecondaryradius, slices, stacks); //right
    glPopMatrix();

    glPushMatrix();
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, forward ? rocketactivecolor[3] : rocketcolor[3]);
        glTranslatef(0, rocketmainradius, 0);
        glutSolidSphere(rocketsecondaryradius, slices, stacks); //forward
    glPopMatrix();
    
    glPushMatrix();
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, back ? rocketactivecolor[4] : rocketcolor[4]);
        glTranslatef(0, -rocketmainradius, 0);
        glutSolidSphere(rocketsecondaryradius, slices, stacks); //back
    glPopMatrix();

    glPushMatrix();
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, top ? rocketactivecolor[5] : rocketcolor[5]);
        glTranslatef(0, 0, rocketmainradius);
        glutSolidSphere(rocketsecondaryradius, slices, stacks); //top
    glPopMatrix();
    
    glPushMatrix();
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, bottom ? rocketactivecolor[6] : rocketcolor[6]);
        glTranslatef(0, 0, -rocketmainradius);
        glutSolidSphere(rocketsecondaryradius, slices, stacks); //bottom
    glPopMatrix();

    restoreMaterial();
}

bool checkCollision(float x1, float y1, float z1, float r1, float x2, float y2, float z2, float r2) {
    float distance = sqrt((x1 - x2) * (x1 - x2) +
                          (y1 - y2) * (y1 - y2) +
                          (z1 - z2) * (z1 - z2));
    for(int i=0; i<100; i++)
        distance += i*0;
    return distance < r1+r2;
}

void newGoal() {
    if(stoprocket)
        return;
    int length = to-from;
    gox = rand()%length - length/2.0;
    goy = rand()%length - length/2.0;
    goz = rand()%length - length/2.0;
    minusx = rocketx < gox;
    minusy = rockety < goy;
    minusz = rocketz < goz;
}

void idle() {
    if(minusx) {
        rocketx += rocketstep;
        if(rocketx > gox)
            rocketx = gox;
    } else {
        rocketx -= rocketstep;
        if(rocketx < gox)
            rocketx = gox;
    }
    if(minusy) {
        rockety += rocketstep;
        if(rockety > goy)
            rockety = goy;
    } else {
        rockety -= rocketstep;
        if(rockety < goy)
            rockety = goy;
    }
    if(minusz) {
        rocketz += rocketstep;
        if(rocketz > goz)
            rocketz = goz;
    } else {
        rocketz -= rocketstep;
        if(rocketz < goz)
            rocketz = goz;
    }
    if(rocketx == gox && rockety == goy && rocketz == goz)
        newGoal();

    glutPostRedisplay();
}

void redraw() {
    long timenow = (long) std::time(nullptr);

    bool print = false;

    if(timenow == timeframe)
        tmpframe++;
    else {
        curframe = tmpframe;
        tmpframe = 1;
        timeframe = timenow;
        print = true;
        seconds++;
    }

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glLoadIdentity();

    gluLookAt(eyex, eyey, eyez, eyex+lx, centery+1, eyez+lz, 0, 1, 0);
    glRotatef(-90, 1, 0, 0);

    restoreMaterial();

    bool main, left, right, forward, back, top, bottom;

    main=left=right=forward=back=top=bottom=false;
    int rcount = 0;
    int mcount = 0;

    glPushMatrix();

        std::vector<int> collisionResult;
        if(showbullet) {
            if(bvhDetection) {
                createRocketTree();
                rcount = asteroidsTree.rayCollision(&collisionResult, &bulletRay);
            } else {
                for(int i=0; i<xs.size(); i++) {
                    float x=xs.at(i);
                    float y=ys.at(i);
                    float z=zs.at(i);
                    float r=rs.at(i);
                    bool res = bulletRay.overlapSphere(x,y,z,r);
                    if(res)
                        collisionResult.push_back(i);
                    mcount += 1;
                }
            }
        } else {
            if(bvhDetection) {
                createRocketTree();
                if(detailedCollision) {
                    rcount = asteroidsTree.overlaps(&collisionResult, &rocketTree);

                    main = std::find(collisionResult.begin(), collisionResult.end(), -1) != collisionResult.end();
                    left = std::find(collisionResult.begin(), collisionResult.end(), -2) != collisionResult.end();
                    right = std::find(collisionResult.begin(), collisionResult.end(), -3) != collisionResult.end();
                    forward = std::find(collisionResult.begin(), collisionResult.end(), -4) != collisionResult.end();
                    back = std::find(collisionResult.begin(), collisionResult.end(), -5) != collisionResult.end();
                    top = std::find(collisionResult.begin(), collisionResult.end(), -6) != collisionResult.end();
                    bottom = std::find(collisionResult.begin(), collisionResult.end(), -7) != collisionResult.end();
                } else if(asteroidsTree.overlaps(&rocketTree))
                    main = left = right = forward = back = top = bottom = true;    
            }
        }

        bool firstbullet = false;

        for(int i=0; i<xs.size(); i++) {
            float x=xs.at(i);
            float y=ys.at(i);
            float z=zs.at(i);
            float r=rs.at(i);
            bool collision = false;
            glPushMatrix();
                glTranslatef(x, y, z);
                if(showbullet) {
                    collision = std::find(collisionResult.begin(), collisionResult.end(), i) != collisionResult.end();
                    if(collision) {
                        firstbullet = true;
                        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, asteroidactivecolor);
                    }
                    glutSolidSphere(r, slices, stacks);
                    if(collision)
                        restoreMaterial();
                } else {
                    if(!detailedCollision) {
                        if(!bvhDetection) {
                            mcount += 7;
                            if(!main) {
                                if(checkCollision(x, y, z, r, rocketx, rockety, rocketz, rocketmainradius))
                                    main = left = right = forward = back = top = bottom = true;
                                if(checkCollision(x, y, z, r, rocketx-rocketmainradius, rockety, rocketz, rocketsecondaryradius))
                                    main = left = right = forward = back = top = bottom = true;
                                if(checkCollision(x, y, z, r, rocketx+rocketmainradius, rockety, rocketz, rocketsecondaryradius))
                                    main = left = right = forward = back = top = bottom = true;
                                if(checkCollision(x, y, z, r, rocketx, rockety+rocketmainradius, rocketz, rocketsecondaryradius))
                                    main = left = right = forward = back = top = bottom = true;
                                if(checkCollision(x, y, z, r, rocketx, rockety-rocketmainradius, rocketz, rocketsecondaryradius))
                                    main = left = right = forward = back = top = bottom = true;
                                if(checkCollision(x, y, z, r, rocketx, rockety, rocketz+rocketmainradius, rocketsecondaryradius))
                                    main = left = right = forward = back = top = bottom = true;
                                if(checkCollision(x, y, z, r, rocketx, rockety, rocketz-rocketmainradius, rocketsecondaryradius))
                                    main = left = right = forward = back = top = bottom = true;
                            }
                        }
                        glutSolidSphere(r, slices, stacks);
                    } else {

                        if(!bvhDetection) {
                            mcount += 7;
                            if(checkCollision(x, y, z, r, rocketx, rockety, rocketz, rocketmainradius)) {
                                main = true;
                                collision = true;
                                cm++;
                            }
                            if(checkCollision(x, y, z, r, rocketx-rocketmainradius, rockety, rocketz, rocketsecondaryradius)) {
                                left = true;
                                collision = true;
                                cl++;
                            }
                            if(checkCollision(x, y, z, r, rocketx+rocketmainradius, rockety, rocketz, rocketsecondaryradius)) {
                                right = true;
                                collision = true;
                                cr++;
                            }
                            if(checkCollision(x, y, z, r, rocketx, rockety+rocketmainradius, rocketz, rocketsecondaryradius)) {
                                forward = true;
                                collision = true;
                                cf++;
                            }
                            if(checkCollision(x, y, z, r, rocketx, rockety-rocketmainradius, rocketz, rocketsecondaryradius)) {
                                back = true;
                                collision = true;
                                cba++;
                            }
                            if(checkCollision(x, y, z, r, rocketx, rockety, rocketz+rocketmainradius, rocketsecondaryradius)) {
                                top = true;
                                collision = true;
                                ct++;
                            }
                            if(checkCollision(x, y, z, r, rocketx, rockety, rocketz-rocketmainradius, rocketsecondaryradius)) {
                                bottom = true;
                                collision = true;
                                cbo++;
                            }
                        } else {
                            collision = std::find(collisionResult.begin(), collisionResult.end(), i) != collisionResult.end();
                        }
                        if(collision)
                            glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, asteroidactivecolor);
                        glutSolidSphere(r, slices, stacks);
                        if(collision)
                            restoreMaterial();
                    }
                }
            glPopMatrix();
        }

        if(showbullet) {
            glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION, firstbullet ? asteroidactivecolor : raycolor);
            glBegin(GL_LINES);
                glVertex3f(rocketx, rockety, rocketz);
                glVertex3f(bulletDirx*50.0, bulletDiry*50.0, bulletDirz*50.0);
            glEnd();
            restoreMaterial();
        }

        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, asteroidactivecolor);
        glPushMatrix();
            std::vector<Node*> nodes = asteroidsTree.getAtLevel(leveltree);
            for(int i=0; i<nodes.size(); i++) {
                Node* el = nodes.at(i);
                AABB* box = el->getAABB();
                glPushMatrix();
                    Point3d center = box->center();
                    glTranslatef(center[0], center[1], center[2]);
                    glScalef(box->width(),box->height(),box->depth());
                    glutWireCube(1);
                glPopMatrix();
            }
        glPopMatrix();
        if(bvhDetection && !showbullet) {
            glPushMatrix();
                std::vector<Node*> rnodes = rocketTree.getAtLevel(levelrtree);
                for(int i=0; i<rnodes.size(); i++) {
                    Node* el = rnodes.at(i);
                    AABB* box = el->getAABB();
                    glPushMatrix();
                        Point3d center = box->center();
                        glTranslatef(center[0], center[1], center[2]);
                        glScalef(box->width(),box->height(),box->depth());
                        glutWireCube(1);
                    glPopMatrix();
                }
            glPopMatrix();
        }
        restoreMaterial();
        glPushMatrix();
            glTranslatef(rocketx, rockety, rocketz);
            rocket(main, left, right, forward, back, top, bottom);
        glPopMatrix();
    glPopMatrix();
    glutSwapBuffers();

    if(print) {
        char s[8];
        sprintf(s, "FPS: %2d",curframe);

        std::cout << space << std::endl;
        std::cout << s << std::endl;
        std::cout << "Number of asteroids: " << xs.size() << std::endl;
        if(bvhDetection) {
            std::cout << "Height of BVH asteroids tree: " << asteroidsTree.height() << std::endl;
            std::cout << "Number of nodes at level " << (leveltree+1) << ": " << nodes.size() << std::endl;
        }
        std::cout << "Number of pairwise comparisons: " << (xs.size()*7) << std::endl;
        std::cout << "Method of collision detection: " << (bvhDetection ? "BVH" : "Manual") << std::endl;
        if(detailedCollision || !bvhDetection)
            std::cout << "Effective comparisons: " << (bvhDetection ? rcount : mcount) << std::endl;
        if(detailedCollision) {
            std::cout << "Collisions on main body: " << cm << std::endl;
            std::cout << "Collisions on left and right body: " << cl << "," << cr << std::endl;
            std::cout << "Collisions on forward and back body: " << cf << "," << cba << std::endl;
            std::cout << "Collisions on top and bottom body: " << ct << "," << cbo << std::endl;
            std::cout << "Total collisions in " << seconds << " seconds: " << (cm+cl+cr+cf+cba+ct+cbo) << std::endl;
        }
        std::cout << space << std::endl;
        std::cout << std::endl << std::endl << std::endl << std::endl << std::endl << std::endl << std::endl << std::endl << std::endl
                  << std::endl << std::endl << std::endl << std::endl << std::endl << std::endl;
        if(!detailedCollision && bvhDetection)
            std::cout << std::endl;
        if(!detailedCollision)
            std::cout << std::endl << std::endl << std::endl << std::endl << std::endl;
        if(!bvhDetection)
            std::cout << std::endl << std::endl;
    }

}

void processNormalKeys(unsigned char key, int x, int y) {
    bool update = false;
    float sqrtlength;
    switch(key) {
        case 'd':
            detailedCollision = !detailedCollision;
            update = true;
            break;
        case ' ':
            bvhDetection = !bvhDetection;
            update = true;
            break;
        case 'r':
            stoprocket = !stoprocket;
            if(!stoprocket)
                showbullet = false;
            update = true;
            break;
        case '+':
            leveltree = limit(0, leveltree+1, asteroidsTree.height()-1);
            update = true;
            break;
        case '-':
            leveltree = limit(0, leveltree-1, asteroidsTree.height()-1);
            update = true;
            break;
        case 'p':
            levelrtree = limit(0, levelrtree+1, rocketTree.height()-1);
            update = true;
            break;
        case 'l':
            levelrtree = limit(0, levelrtree-1, rocketTree.height()-1);
            update = true;
            break;            
        case 'w':
            centery = limit(-1, centery+stepws, 1);
            update = true;
            break;
        case 's':
            centery = limit(-1, centery-stepws, 1);
            update = true;
            break;
        case 'x':
            bulletDirx = ((rand() % 201) - 100)/100.0;
            bulletDiry = ((rand() % 201) - 100)/100.0;
            bulletDirz = ((rand() % 201) - 100)/100.0;
            sqrtlength = sqrt(bulletDirx*bulletDirx + bulletDiry*bulletDiry + bulletDirz*bulletDirz);
            bulletDirx /= sqrtlength;
            bulletDiry /= sqrtlength;
            bulletDirz /= sqrtlength;
            bulletRay.setOrigin(rocketx, rockety, rocketz);
            bulletRay.setDirection(bulletDirx, bulletDiry, bulletDirz);
            showbullet = true;
            stoprocket = true;
            update = true;
            break;
        case 27:
            exit(0);
            break;
    }
    if(update)
        glutPostRedisplay();
}

void processSpecialKeys(int key, int xx, int yy) {
    bool update = false;
    switch (key) {
        case GLUT_KEY_LEFT :
            angle -= stepad;
            lx = sin(angle);
            lz = -cos(angle);
            update = true;
            break;
        case GLUT_KEY_RIGHT :
            angle += stepad;
            lx = sin(angle);
            lz = -cos(angle);
            update = true;
            break;
        case GLUT_KEY_UP :
            eyex += lx * stepz;
            eyez += lz * stepz;
            update = true;
            break;
        case GLUT_KEY_DOWN :
            eyex -= lx * stepz;
            eyez -= lz * stepz;
            update = true;
            break;
    }
    if(update)
        glutPostRedisplay();
}

void newScene() {
    float randOffset;
    int length = to-from;
    rocketx = (float)(rand() % length) - length/2.0;
    rockety = (float)(rand() % length) - length/2.0;
    rocketz = (float)(rand() % length) - length/2.0;
    xs.clear();
    ys.clear();
    zs.clear();
    rs.clear();
    for(float x=from; x<=to; x+=dense) {
        for(float y=from; y<=to; y+=dense) {
            for(float z=from; z<=to; z+=dense) {
                randOffset = (float)rand()/(float)RAND_MAX - 0.5f;
                xs.push_back(randOffset+x-asteroidradius/2.0);
                randOffset = (float)rand()/(float)RAND_MAX - 0.5f;
                ys.push_back(randOffset+y-asteroidradius/2.0);
                randOffset = (float)rand()/(float)RAND_MAX - 0.5f;
                zs.push_back(randOffset+z-asteroidradius/2.0);
                rs.push_back(asteroidradius);
            }
        }
    }
    createTree();
    newGoal();
}

int main(int argc, char** argv) {
    srand(time(nullptr));
    std::cout << "Ready to go!" << std::endl;
    std::cout << "Press 'y' to continue" << std::endl;
    char wait[1];
    std::cin >> wait;
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowPosition(315,30);
    glutInitWindowSize(w_width,w_height);
    int windowId = glutCreateWindow(argv[0]);
    printf("window id %d\n", windowId);

    glutReshapeFunc(changeSize);
    glutDisplayFunc(redraw);
    glutIdleFunc(idle);
    glutKeyboardFunc(processNormalKeys);
    glutSpecialFunc(processSpecialKeys);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_NORMALIZE);

    newScene();

    glutMainLoop();
    return 0;
}