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
}


template <typename M>
QtModelT<M>::~QtModelT()
{

}

template <typename M>
void
QtModelT<M>::render()
{

  //std::cout << "Render" << "\n";
  glPushMatrix();
  glTranslatef(horizontal, vertical, 0);
  glRotatef(modelRotation.x(), 1, 0, 0);
  glRotatef(modelRotation.y(), 0, 1, 0);
  glRotatef(modelRotation.z(), 0, 0, 1);

  glEnable(GL_DEPTH_TEST);
  glEnableClientState(GL_VERTEX_ARRAY);
  
  //glPointSize(4.0);
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
void
QtModelT<M>::mergeColours(QtModelT<M>* m2)
{
  //std::cout << "merge colours" << "\n";
   typedef typename M::Point Point;

  PointMatrix m2Mat = m2->buildMatrix();
  typedef nanoflann::KDTreeEigenMatrixAdaptor< PointMatrix >  my_kd_tree_t;
  my_kd_tree_t   mat_index(3, m2Mat, 10);
  mat_index.index->buildIndex();
  const size_t num_results = 1;

  double meanDist = 0;
  for (typename M::VertexIter v_it=mesh.vertices_begin(); v_it!=mesh.vertices_end(); ++v_it) 
  {
     std::vector<double> query_pt(3);
     query_pt[0] = mesh.point(*v_it)[0];
     query_pt[1] = mesh.point(*v_it)[1];
     query_pt[2] = mesh.point(*v_it)[2];

     std::vector<size_t>   ret_index(num_results);
     std::vector<double>    out_dist_sqr(num_results);

     nanoflann::KNNResultSet<double> resultSet(num_results);

     resultSet.init(&ret_index[0], &out_dist_sqr[0] );
     mat_index.index->findNeighbors(resultSet, &query_pt[0], nanoflann::SearchParams(10));

     meanDist = meanDist + out_dist_sqr[0];

     if (out_dist_sqr[0] < 0.000005)
        mesh.set_color(*v_it, OpenMesh::Vec3f(255, 255, 255));
     else
        mesh.set_color(*v_it, OpenMesh::Vec3f(modelColor.blueF(), modelColor.greenF(), modelColor.redF()));

  }

  //std::cout << "Mean Dist: " << meanDist / mesh.n_vertices() << "\n";



}


template <typename M>
QRectF 
QtModelT<M>::boundingRect() const
{
  return QRectF(0,0, 1024, 768);
}

#endif
