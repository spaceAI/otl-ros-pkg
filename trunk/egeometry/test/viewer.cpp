#include <stdlib.h>
#include <math.h>
//#include <GL/glut.h>
#include <GL/freeglut.h>
#include <egeometry/cascaded_coordinates.h>

namespace egeometry
{
namespace viewer
{

// light vector. LIGHTZ is implicitly 1
#define LIGHTX (1.0f)
#define LIGHTY (0.4f)

#define DEG2RAD(x) (3.14159265358979 * (x) / 180.0)

namespace
{
float view_xyz[3];	// position x,y,z
float view_hpr[3];	// heading, pitch, roll (degrees)
int mx=0,my=0; 	// mouse position
int mode = 0;		// mouse button bits
typedef boost::shared_ptr<Coordinates> CoordsPtr;
std::vector<CoordsPtr> objects;
}

void DrawObjects(std::vector<CoordsPtr> &objects);
void DrawBox(const Coordinates &c, const float* const sides, const float* const color);

void wrapCameraAngles()
{
    for (int i=0; i<3; i++) {
        while (view_hpr[i] > 180) view_hpr[i] -= 360;
        while (view_hpr[i] < -180) view_hpr[i] += 360;
    }
}

void initMotionModel()
{

//    float view_xyz[3] = {-10, 0, 1};
    //float view_hpr[3] = {0, 1, 0};

    view_xyz[0] = 10;
    view_xyz[1] = 0;
    view_xyz[2] = 1;
    view_hpr[0] = 180;
    view_hpr[1] = 0;
    view_hpr[2] = 0;
}

extern "C" void dsSetViewpoint (float xyz[3], float hpr[3])
{
    if (xyz) {
        view_xyz[0] = xyz[0];
        view_xyz[1] = xyz[1];
        view_xyz[2] = xyz[2];
    }
    if (hpr) {
        view_hpr[0] = hpr[0];
        view_hpr[1] = hpr[1];
        view_hpr[2] = hpr[2];
        wrapCameraAngles();
    }
}

void setCamera (float x, float y, float z, float h, float p, float r)
{
    glMatrixMode (GL_MODELVIEW);
    glLoadIdentity();
    glRotatef (90, 0,0,1);
    glRotatef (90, 0,1,0);
    glRotatef (r, 1,0,0);
    glRotatef (p, 0,1,0);
    glRotatef (-h, 0,0,1);
    glTranslatef (-x,-y,-z);
}

void setColor (float r, float g, float b, float alpha)
{
    GLfloat light_ambient[4],light_diffuse[4],light_specular[4];
    light_ambient[0] = r*0.3f;
    light_ambient[1] = g*0.3f;
    light_ambient[2] = b*0.3f;
    light_ambient[3] = alpha;
    light_diffuse[0] = r*0.7f;
    light_diffuse[1] = g*0.7f;
    light_diffuse[2] = b*0.7f;
    light_diffuse[3] = alpha;
    light_specular[0] = r*0.2f;
    light_specular[1] = g*0.2f;
    light_specular[2] = b*0.2f;
    light_specular[3] = alpha;
    glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, light_ambient);
    glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, light_diffuse);
    glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, light_specular);
    glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 5.0f);
}

void setTransform (const Coordinates &c)
{
    const FloatMatrix &R = c.GetWorldRotation();
    const FloatVector &pos = c.GetWorldPosition();
    GLfloat matrix[16];

    matrix[0]=R[0][0];
    matrix[1]=R[1][0];
    matrix[2]=R[2][0];
    matrix[3]=0;
    
    matrix[4]=R[0][1];
    matrix[5]=R[1][1];
    matrix[6]=R[2][1];
    matrix[7]=0;
    
    matrix[8]=R[0][2];
    matrix[9]=R[1][2];
    matrix[10]=R[2][2];
    matrix[11]=0;
    
    matrix[12]=pos[0];
    matrix[13]=pos[1];
    matrix[14]=pos[2];
    matrix[15]=1;
    glPushMatrix();
    glMultMatrixf (matrix);
}

// set shadow projection transform

void setShadowTransform()
{
    GLfloat matrix[16];
    for (int i=0; i<16; i++) matrix[i] = 0;
    matrix[0]=1;
    matrix[5]=1;
    matrix[8]=-LIGHTX;
    matrix[9]=-LIGHTY;
    matrix[15]=1;
    glPushMatrix();
    glMultMatrixf (matrix);
}

void drawBoxHere (const float sides[3])
{
    float lx = sides[0]*0.5f;
    float ly = sides[1]*0.5f;
    float lz = sides[2]*0.5f;

    // sides
    glBegin (GL_TRIANGLE_STRIP);
    glNormal3f (-1,0,0);
    glVertex3f (-lx,-ly,-lz);
    glVertex3f (-lx,-ly,lz);
    glVertex3f (-lx,ly,-lz);
    glVertex3f (-lx,ly,lz);
    glNormal3f (0,1,0);
    glVertex3f (lx,ly,-lz);
    glVertex3f (lx,ly,lz);
    glNormal3f (1,0,0);
    glVertex3f (lx,-ly,-lz);
    glVertex3f (lx,-ly,lz);
    glNormal3f (0,-1,0);
    glVertex3f (-lx,-ly,-lz);
    glVertex3f (-lx,-ly,lz);
    glEnd();

    // top face
    glBegin (GL_TRIANGLE_FAN);
    glNormal3f (0,0,1);
    glVertex3f (-lx,-ly,lz);
    glVertex3f (lx,-ly,lz);
    glVertex3f (lx,ly,lz);
    glVertex3f (-lx,ly,lz);
    glEnd();

    // bottom face
    glBegin (GL_TRIANGLE_FAN);
    glNormal3f (0,0,-1);
    glVertex3f (-lx,-ly,-lz);
    glVertex3f (-lx,ly,-lz);
    glVertex3f (lx,ly,-lz);
    glVertex3f (lx,-ly,-lz);
    glEnd();
}



void DrawCube(const Coordinates& c, double x, double y, double z,
              double r, double g, double b)
{
    GLdouble vertex[][3] = {
        { -x, -y, -z },
        {  x, -y, -z },
        {  x,  y, -z },
        { -x,  y, -z },
        { -x, -y,  z },
        {  x, -y,  z },
        {  x,  y,  z },
        { -x,  y,  z }
    };
      
    const static int face[][4] = {
        { 0, 1, 2, 3 },
        { 1, 5, 6, 2 },
        { 5, 4, 7, 6 },
        { 4, 0, 3, 7 },
        { 4, 5, 1, 0 },
        { 3, 2, 6, 7 }
    };
      
    const static GLdouble normal[][3] = {
        { 0.0, 0.0,-1.0 },
        { 1.0, 0.0, 0.0 },
        { 0.0, 0.0, 1.0 },
        {-1.0, 0.0, 0.0 },
        { 0.0,-1.0, 0.0 },
        { 0.0, 1.0, 0.0 }
    };

    GLfloat red[] = { r, g, b, 1.0 };
      
    int i, j;
      
    /* 材質を設定する */
    glMaterialfv(GL_FRONT, GL_DIFFUSE, red);
      
    glBegin(GL_QUADS);
    for (j = 0; j < 6; ++j) {
        glNormal3dv(normal[j]);
        for (i = 4; --i >= 0;) {
            glVertex3dv(vertex[face[j][i]]);
        }
    }
    glEnd();
}
    


/*
 * 地面を描く
 */
void DrawGround(double height)
{
    float color[4];
    color[3] = 1.0;
    for (int i = -10; i<= 10; i++)
    {
        for (int j = -10; j<= 10; j++)
        {
            if ( (i + j) % 2 == 0)
            {
                color[0] = 0.5;
                color[1] = 0.5;
                color[2] = 0.9;
            }
            else
            {
                color[0] = 0.5;
                color[1] = 0.9;
                color[2] = 0.5;
            }
            FloatVector pos(i, j, height - (0.01 * 0.5));
            float sides[3] = {1, 1, 0.01};
            Coordinates c(pos);
            DrawBox(c, sides, color);
        }
    }
}

void DrawBox(const Coordinates &c, const float* const sides, const float* const color)
{
    glShadeModel (GL_FLAT);
    setTransform(c);
    setColor (color[0], color[1], color[2], color[3]);
    drawBoxHere(sides);
    glPopMatrix();
}

/*
 * 画面表示
 */
void DisplayHandler(void)
{

//    const static GLfloat blue[] = { 0.6, 0.6, 0.8, 1.0 };     /* 球の色 */
    const static GLfloat lightpos[] = { 5.0, 5.0, 5.0, 1.0 }; /* 光源の位置 */

    /* 画面クリア */
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    /* モデルビュー変換行列の初期化 */
    glLoadIdentity();

    /* 光源の位置を設定 */
    glLightfv(GL_LIGHT0, GL_POSITION, lightpos);

    /* 視点の移動（シーンの方を奥に移す）*/
    //glTranslated(0.0, 0.0, -10.0);
    setCamera(view_xyz[0], view_xyz[1], view_xyz[2], 
              view_hpr[0], view_hpr[1], view_hpr[2]);

    DrawObjects(objects);
    DrawGround(0.0);
    glutSwapBuffers();
}

void ResizeHandler(int w, int h)
{
    /* ウィンドウ全体をビューポートにする */
    glViewport(0, 0, w, h);

    /* 透視変換行列の指定 */
    glMatrixMode(GL_PROJECTION);

    /* 透視変換行列の初期化 */
    glLoadIdentity();
    gluPerspective(30.0, (double)w / (double)h, 1.0, 100.0);

    /* モデルビュー変換行列の指定 */
    glMatrixMode(GL_MODELVIEW);
}

void KeyboardHandler(unsigned char key, int x, int y)
{
    FloatVector pos;
    FloatVector axis;
    /* ESC か q をタイプしたら終了 */
    switch (key)
    {
    case '\033':
    case 'q':
        exit(0);
        break;
    case 'h':
        view_xyz[1]+=0.1;
        break;
    case 'l':
        view_xyz[1]-=0.1;
        break;
    case 'j':
        view_xyz[2]+=0.1;
        break;
    case 'k':
        view_xyz[2]-=0.1;
        break;
    case 'u':
        view_xyz[0]+=0.1;
        break;
    case 'm':
        view_xyz[0]-=0.1;
        break;
    case 'r':
        axis[0] = 1;
        axis[1] = 0;
        axis[2] = 0;
        (*objects.begin())->RotateWithAxis(DEG2RAD(10), axis);
        //std::cout << **objects.begin() << std::endl;
        break;
    case 't':
        axis[0] = 1;
        axis[1] = 0;
        axis[2] = 0;
        (*objects.begin())->RotateWithAxis(DEG2RAD(-10), axis);
        //std::cout << **objects.begin() << std::endl;
        break;
    case 'f':
        pos.SetValue(0, 0.1, 0);
        //(*objects.begin())->Translate(pos);
        (*objects.begin())->Locate(pos);
        //std::cout << **objects.begin() << std::endl;
        break;
    case 'g':
        pos.SetValue(0, -0.1, 0);
        (*objects.begin())->Locate(pos);
        //(*objects.begin())->Translate(pos);
        //std::cout << **objects.begin() << std::endl;
        break;
    }
    
}


static void MouseHandler (int button, int state, int x, int y)
{
    switch (state)
    {
    case GLUT_DOWN:
        switch(button)
        {
        case GLUT_LEFT_BUTTON:
            mode |= 1;
            break;
        case GLUT_MIDDLE_BUTTON:
            mode |= 2;
            break;
        case GLUT_RIGHT_BUTTON:
            mode |= 4;
            break;
        default:
            break;
        }
        break;
    case GLUT_UP:
        switch(button)
        {
        case GLUT_LEFT_BUTTON:
            mode &= (~1);
            break;
        case GLUT_MIDDLE_BUTTON:
            mode &= (~2);
            break;
        case GLUT_RIGHT_BUTTON:
            mode &= (~4);
            break;
        default:
            break;
        }
    default:
        break;
    }
    mx = x;
    my = y;
}

void dsMotion (int mode, int deltax, int deltay);

void MotionHandler(int x, int y)
{
    dsMotion (mode, x - mx, y - my);
    mx = x;
    my = y;
}


void dsMotion (int mode, int deltax, int deltay)
{
    float side = 0.01f * float(deltax);
    float fwd = (mode==4) ? (0.01f * float(deltay)) : 0.0f;
    float s = (float) sin (DEG2RAD(view_hpr[0]));
    float c = (float) cos (DEG2RAD(view_hpr[0]));
  
//  std::cout << mode << std::endl;
    if (mode==1) {
        view_hpr[0] += float (deltax) * 0.5f;
        view_hpr[1] += float (deltay) * 0.5f;
    }
    else {
        view_xyz[0] += -s*side + c*fwd;
        view_xyz[1] += c*side + s*fwd;
        if (mode==2 || mode==5) view_xyz[2] += 0.01f * float(deltay);
    }
    wrapCameraAngles();
}


void Init(void)
{
    /* 初期設定 */
    glClearColor(1.0, 1.0, 1.0, 1.0);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
  
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
}

void IdleHandler()
{
    glutPostRedisplay();
}

void DrawObjects(std::vector<CoordsPtr> &objects)
{
    float sides[3] = {1,1,1.5};
    float color[4] = {1.0f, 0.2f, 0.2f, 2.0f};
    std::vector<CoordsPtr>::iterator it;
    for (it = objects.begin(); it!=objects.end(); ++it)
    {
        DrawBox(*(*it), sides, color);
    }
}

void SetObjects()
{
    FloatVector pos(0,0,1.0);
    CascadedCoordinates *c1 = new CascadedCoordinates(pos);
    //Coordinates *c1 = new Coordinates(pos);
    CoordsPtr p_c1(c1);
    objects.push_back(p_c1);
    
    FloatVector pos2(0, 0, 2.5);
    CascadedCoordinates *c2 = new CascadedCoordinates(pos2);
    //Coordinates *c2 = new Coordinates(pos2);
    CoordsPtr p_c2(c2);
    
    FloatVector axis(1, 0, 0);
    
    c2->RotateWithAxis(DEG2RAD(30), axis);
    c1->Assoc(*c2);
    objects.push_back(p_c2);
}

}
}

int main(int argc, char *argv[])
{
    using namespace egeometry::viewer;

    glutInit(&argc, argv);
    //  glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
    glutCreateWindow(argv[0]);
    glutDisplayFunc(DisplayHandler);
    glutReshapeFunc(ResizeHandler);
    glutKeyboardFunc(KeyboardHandler);
    glutMouseFunc(MouseHandler);
    glutMotionFunc(MotionHandler);
    glutIdleFunc(IdleHandler);
    Init();
    initMotionModel();
    SetObjects();
    glutMainLoop();
    return 0;
}
