#include <iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pangolin/pangolin.h>
using namespace cv;
using namespace std;
using namespace Eigen;
#define UsePangolin 0
struct RotationMatrix
{
    Eigen::Matrix3d matrix = Eigen::Matrix3d::Identity();
};

ostream& operator << ( ostream& out, const RotationMatrix& r ) 
{
    out.setf(ios::fixed);
    Eigen::Matrix3d  matrix = r.matrix;
    out<<'=';
    out<<"["<<setprecision(2)<<matrix(0,0)<<","<<matrix(0,1)<<","<<matrix(0,2)<<"],"
    << "["<<matrix(1,0)<<","<<matrix(1,1)<<","<<matrix(1,2)<<"],"
    << "["<<matrix(2,0)<<","<<matrix(2,1)<<","<<matrix(2,2)<<"]";
    return out;
}

istream& operator >> (istream& in, RotationMatrix& r )
{
    return in;
}

struct TranslationVector
{
    Vector3d trans = Vector3d(0,0,0);
};

struct PoIVector
{
    Vector3d trans = Vector3d(0,0,0);
};

ostream& operator << (ostream& out, const PoIVector& t)
{
    out<<"=["<<t.trans(0)<<','<<t.trans(1)<<','<<t.trans(2)<<"]";
    return out;
}

istream& operator >> ( istream& in, PoIVector& t)
{
    return in;
}

ostream& operator << (ostream& out, const TranslationVector& t)
{
    out<<"=["<<t.trans(0)<<','<<t.trans(1)<<','<<t.trans(2)<<"]";
    return out;
}

istream& operator >> ( istream& in, TranslationVector& t)
{
    return in;
}

struct QuaternionDraw
{
    Quaterniond q;
};

ostream& operator << (ostream& out, const QuaternionDraw quat )
{
    auto c = quat.q.coeffs();
    out<<"=["<<c[0]<<","<<c[1]<<","<<c[2]<<","<<c[3]<<"]";
    return out;
}

istream& operator >> (istream& in, const QuaternionDraw quat)
{
    return in;
}
int main ( int argc, char ** argv )
{
#if UsePangolin
Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
Eigen::AngleAxisd roataion_vector (M_PI/4, Eigen::Vector3d (0, 0, 1)); // Z axis 45 rotation
cout .precision(3); 
rotation_matrix = roataion_vector.matrix();
Eigen::Vector3d v(1,0,0);
Eigen::Vector3d v_rotated = roataion_vector*v;
 v_rotated = rotation_matrix*v;
 
 //eulerAngles
 Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2,1,0); // ZYX, Yaw pitch roll
cout << "eulerAngles =" << euler_angles <<endl;

//EularTransform
Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
T.rotate(roataion_vector);
cout << "T=" << T.matrix() <<endl;
T.pretranslate(Eigen::Vector3d(1,3,4));
cout << "T=" << T.matrix() <<endl;

//Quaterniond
Eigen::Vector3d v_transformed = T*v;
cout << v_transformed.transpose() << endl;
Eigen::Quaterniond q ;
 q = Eigen::Quaterniond(roataion_vector);
cout << "Quaterniond=" <<q.coeffs() <<endl;
cout << "Quaterniond" << q*v <<endl;
#else
//visualize pangolin
    pangolin::CreateWindowAndBind ( "visualize geometry", 1000, 600 );
    glEnable ( GL_DEPTH_TEST );
    pangolin::OpenGlRenderState s_cam (
        pangolin::ProjectionMatrix ( 1000, 600, 420, 420, 500, 300, 0.1, 1000 ),
        pangolin::ModelViewLookAt ( 3,3,3,0,0,0,pangolin::AxisY )
    );
    const int UI_WIDTH = 500;
    
    pangolin::View& d_cam = pangolin::CreateDisplay().SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0, -1000.0f/600.0f).SetHandler(new pangolin::Handler3D(s_cam));
    
    pangolin::Var<RotationMatrix> rotation_matrix("ui.R", RotationMatrix());
    pangolin::Var<TranslationVector> translation_vector("ui.t", TranslationVector());
    pangolin::Var<TranslationVector> euler_angles("ui.rpy", TranslationVector());
    pangolin::Var<QuaternionDraw> quaternion("ui.q", QuaternionDraw());
    pangolin::Var<PoIVector> quaternion2("ui.G", PoIVector());
    pangolin::CreatePanel("ui")
        .SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));
	
	
 while ( !pangolin::ShouldQuit() )
    {
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
        
        d_cam.Activate( s_cam );
        
        pangolin::OpenGlMatrix matrix = s_cam.GetModelViewMatrix();
       Matrix<double,4,4> m;
  for (int i=0;i<4;i++)
    for (int j=0;j<4;j++)
      m(i,j) = matrix(i,j);
        // m = m.inverse();
        RotationMatrix R; 
        for (int i=0; i<3; i++)
            for (int j=0; j<3; j++)
                R.matrix(i,j) = m(j,i);
        rotation_matrix = R;
        
        TranslationVector t;
        t.trans = Vector3d(m(0,3), m(1,3), m(2,3));
        t.trans = -R.matrix*t.trans;
        translation_vector = t;
        
        TranslationVector euler;
        euler.trans = R.matrix.transpose().eulerAngles(2,1,0);
        euler_angles = euler;
        
        QuaternionDraw quat;
        quat.q = Quaterniond(R.matrix);
        quaternion = quat;
        
        glColor3f(1.0,1.0,1.0);
        
        pangolin::glDrawColouredCube();
        // draw the original axis 
        glLineWidth(3);
        glColor3f ( 0.8f,0.f,0.f );
        glBegin ( GL_LINES );
        glVertex3f( 0,0,0 );
        glVertex3f( 10,0,0 );
        glColor3f( 0.f,0.8f,0.f);
        glVertex3f( 0,0,0 );
        glVertex3f( 0,10,0 );
        glColor3f( 0.2f,0.2f,1.f);
        glVertex3f( 0,0,0 );
        glVertex3f( 0,0,10 );
        glEnd();
        
        pangolin::FinishFrame();
    }


 
#endif
}
