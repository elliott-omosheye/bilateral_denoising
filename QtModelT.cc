#ifndef MODEL_CC
#define MODEL_CC


#include "QtModelT.hh"
#include <math.h> 
#include <QFile>
#include <QTextStream>
#include <QVarLengthArray>
#include <QtOpenGL>
#include <random>
#include <cmath>


template <typename M>
QtModelT<M>::QtModelT(M& m)
  : modelColor(100, 100, 100)
  , vertical(0.0f)
  , horizontal(0.0f)
  , depth(0.0f)
  , deg2Rad(0.0174532925)
{
  mesh = m;
  updateColour();
  calcNormals();
}


template <typename M>
QtModelT<M>::~QtModelT()
{

}

template <typename M>
void
QtModelT<M>::render()
{
    typename M::ConstFaceIter    fIt(mesh.faces_begin()),
                                 fEnd(mesh.faces_end());

    typename M::ConstFaceVertexIter fvIt;

    //std::cout << "Render" << "\n";
    glPushMatrix();
    glTranslatef(horizontal, vertical, 0);
    glRotatef(modelRotation.x(), 1, 0, 0);
    glRotatef(modelRotation.y(), 0, 1, 0);
    glRotatef(modelRotation.z(), 0, 0, 1);

    glEnable(GL_LIGHTING);
    glShadeModel(GL_FLAT);

    glEnable(GL_DEPTH_TEST);
    glEnableClientState(GL_VERTEX_ARRAY);

    glBegin(GL_TRIANGLES);
    for (; fIt!=fEnd; ++fIt)
    {

        glNormal3fv( &mesh.normal(*fIt)[0] );

        fvIt = mesh.cfv_iter(*fIt);
        glVertex3fv( &mesh.point(*fvIt)[0] );
        ++fvIt;
        glVertex3fv( &mesh.point(*fvIt)[0] );
        ++fvIt;
        glVertex3fv( &mesh.point(*fvIt)[0] );
     }
     glEnd();

    glPopMatrix();



}

template <typename M>
void
QtModelT<M>::applyTransformations()
{
  typedef typename M::Point Point;
  modelRotation = modelRotation * deg2Rad;
  Eigen::AngleAxis<float> aax(modelRotation.x(), Eigen::Vector3f(1, 0, 0));
  Eigen::AngleAxis<float> aay(modelRotation.y(), Eigen::Vector3f(0, 1, 0));
  Eigen::AngleAxis<float> aaz(modelRotation.z(), Eigen::Vector3f(0, 0, 1));
  Eigen::Quaternion<float> rotation = aax * aay * aaz;

  for (typename M::VertexIter v_it=mesh.vertices_begin(); v_it!=mesh.vertices_end(); ++v_it) 
  {
    Eigen::Vector3f p = Eigen::Vector3f(mesh.point(*v_it)[0], mesh.point(*v_it)[1], mesh.point(*v_it)[2]);
    p = rotation * p;
    mesh.set_point( *v_it, Point(p[0], p[1], p[2]) );
    mesh.set_point( *v_it, mesh.point(*v_it) + Point(horizontal, vertical, depth) );
  }
  horizontal = 0.0f;
  vertical = 0.0f;
  depth = 0.0f;
  modelRotation.setX(0.0f);
  modelRotation.setY(0.0f);
  modelRotation.setZ(0.0f);
}

template <typename M>
PointMatrix//flann::Matrix<float>
QtModelT<M>::buildMatrix()
{
  PointMatrix m(mesh.n_vertices(), 3);
  int count = 0;

  for (typename M::VertexIter v_it=mesh.vertices_begin(); v_it!=mesh.vertices_end(); ++v_it) 
  {
    m(count, 0) = mesh.point(*v_it)[0];
    m(count, 1) = mesh.point(*v_it)[1];
    m(count, 2) = mesh.point(*v_it)[2];
    count += 1;
  }
  return m;
}

template <typename M>
void
QtModelT<M>::updateTransformations(Matrix<double, 3, 3>& R, double x, double y, double z)
{
  typedef typename M::Point Point;
  for (typename M::VertexIter v_it=mesh.vertices_begin(); v_it!=mesh.vertices_end(); ++v_it) 
  {
    Eigen::Vector3d p = Eigen::Vector3d(mesh.point(*v_it)[0], mesh.point(*v_it)[1], mesh.point(*v_it)[2]);
    p = R * p;
    mesh.set_point( *v_it, Point(p[0], p[1], p[2]) );
    mesh.set_point( *v_it, mesh.point(*v_it) - Point(x, y, z) );
  }
  render();
}



template <typename M>
void
QtModelT<M>::addNoise(double sigma)
{
   std::random_device rd;
   std::mt19937 gen(rd());
   std::normal_distribution<> d(0, sigma);

  typedef typename M::Point Point;
  for (typename M::VertexIter v_it=mesh.vertices_begin(); v_it!=mesh.vertices_end(); ++v_it) 
  {
    Eigen::Vector3f p = Eigen::Vector3f(mesh.point(*v_it)[0], mesh.point(*v_it)[1], mesh.point(*v_it)[2]);
    p = p;
    mesh.set_point( *v_it, Point(p[0]+d(gen), p[1]+d(gen), p[2]+d(gen)) );
  }
  calcNormals();
}


template <typename M>
PointMatrix
QtModelT<M>::buildSampledMatrix()
{
  int noSamples = 5000;
  PointMatrix allMat = buildMatrix();
  PointMatrix randMat(noSamples, 3);
  for ( unsigned i = 0U; i < noSamples; ++i ) 
  { 
    float ind = float(rand()) / RAND_MAX;
    //std::cout << ind << "\n";
    //std::cout << floor(ind * mesh.n_vertices() ) << "\n";
    randMat.row(i) = allMat.row(floor(ind * mesh.n_vertices() ) );
  }

  return randMat;

}

template <typename M>
int
QtModelT<M>::getNoVerticies()
{
  return mesh.n_vertices();
}

template <typename M>
void
QtModelT<M>::updateRotation(QVector3D& rotationVec)
{
  modelRotation += rotationVec;
}

template <typename M>
void
QtModelT<M>::updateHorizontal(float x)
{
  horizontal += x;
}

template <typename M>
void
QtModelT<M>::updateVertical(float x)
{
  vertical += x;
}

template <typename M>
void
QtModelT<M>::updateColour()
{
  for (typename M::VertexIter v_it=mesh.vertices_begin(); v_it!=mesh.vertices_end(); ++v_it) 
  {
    mesh.set_color(*v_it, OpenMesh::Vec3f(modelColor.redF(), modelColor.blueF(), modelColor.greenF()));
  }
}

template <typename M>
void
QtModelT<M>::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
  std::cout << "paint" << "\n";
  painter->drawRect(boundingRect());
  painter->beginNativePainting();
  glPushMatrix();
  glTranslatef(horizontal, vertical, 0);
  glRotatef(modelRotation.x(), 1, 0, 0);
  glRotatef(modelRotation.y(), 0, 1, 0);
  glRotatef(modelRotation.z(), 0, 0, 1);

  glEnable(GL_DEPTH_TEST);
  glEnableClientState(GL_VERTEX_ARRAY);

  if ( mesh.has_vertex_colors() )
  {
    glEnableClientState( GL_COLOR_ARRAY );
    glColorPointer(3, GL_FLOAT, 0, mesh.vertex_colors());
  }
  //glColor4f(modelColor.redF(), modelColor.greenF(), modelColor.blueF(), 1.0f);
  glVertexPointer(3, GL_FLOAT, 0, mesh.points());
  glDrawArrays( GL_POINTS, 0, static_cast<GLsizei>(mesh.n_vertices()) );

  glDisableClientState(GL_VERTEX_ARRAY);
  glDisable(GL_DEPTH_TEST);

  glDisableClientState(GL_COLOR_ARRAY);  

  glPopMatrix();

  painter->endNativePainting();

}


template <typename M>
QRectF 
QtModelT<M>::boundingRect() const
{
  return QRectF(0,0, 1024, 768);
}

template <typename M>
void
QtModelT<M>::calcNormals()
{

  //std::cout << "calcNormals()" << "\n";
  OpenMesh::IO::Options opt;
  mesh.request_vertex_normals();
  // Add face normals as default property
  mesh.request_face_normals();
  // If the file did not provide vertex normals, then calculate them
  if ( !opt.check( OpenMesh::IO::Options::VertexNormal ) &&
       mesh.has_face_normals() && mesh.has_vertex_normals() )
  {
    // let the mesh update the normals
    mesh.update_face_normals();
    mesh.update_vertex_normals();

    //std::cout << "update_normals()" << "\n";
  }

}

#endif
