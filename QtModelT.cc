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
#include <math.h> 

template <typename M>
QtModelT<M>::QtModelT(M& m)
  : modelColor(100, 100, 100)
  , vertical(0.0f)
  , horizontal(0.0f)
  , depth(0.0f)
  , deg2Rad(0.0174532925)
{
  mesh = m;

  calcNormals();

  gt_distance = 0.0;
  double min_x, max_x, min_y, max_y, min_z, max_z;
  bool first = true;
  for (typename M::VertexIter v_it=mesh.vertices_begin(); v_it!=mesh.vertices_end(); ++v_it) 
  {
    if(first){
      min_x = mesh.point(*v_it)[0];
      max_x = mesh.point(*v_it)[0];
      min_y = mesh.point(*v_it)[1];
      max_y = mesh.point(*v_it)[1];
      min_z = mesh.point(*v_it)[2];
      max_z = mesh.point(*v_it)[2];
      first = false;
    }

    if(mesh.point(*v_it)[0] < min_x )
      min_x = mesh.point(*v_it)[0];
    else if(mesh.point(*v_it)[0] > max_x )
      max_x = mesh.point(*v_it)[0];

    if(mesh.point(*v_it)[1] < min_y )
      min_y = mesh.point(*v_it)[1];
    else if(mesh.point(*v_it)[1] > max_y )
      max_y = mesh.point(*v_it)[1];

    if(mesh.point(*v_it)[2] < min_z )
      min_z = mesh.point(*v_it)[2];
    else if(mesh.point(*v_it)[2] > max_z )
      max_z = mesh.point(*v_it)[2];

  }
  typedef typename M::Point Point;
  for (typename M::VertexIter v_it=mesh.vertices_begin(); v_it!=mesh.vertices_end(); ++v_it) 
  {
    mesh.set_point( *v_it, Point(
          2.0*(mesh.point(*v_it)[0]-min_x)/(max_x-min_x) - 1.0,
          2.0*(mesh.point(*v_it)[1]-min_y)/(max_y-min_y) - 1.0,
          2.0*(mesh.point(*v_it)[2]-min_z)/(max_z-min_z) - 1.0)
    );
  }

  
  if (!OpenMesh::IO::write_mesh(mesh, "out_box.stl")) 
  {
    std::cerr << "write error\n";
    exit(1);
  }

  groundTruth = mesh;
  updateColour();
  calcNormals();
  getDistFromGroundTruth();
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

    //glBegin(GL_LINES);
    //glLineWidth(2.0f);
    //for (typename M::VertexIter v_it=mesh.vertices_begin(); v_it!=mesh.vertices_end(); ++v_it) 
    //{
      //glColor3b (255, 255, 255);
      //glVertex3f(mesh.point(*v_it)[0], mesh.point(*v_it)[1], mesh.point(*v_it)[2]);
      //glVertex3f(mesh.point(*v_it)[0]+mesh.normal(*v_it)[0], mesh.point(*v_it)[1]+mesh.normal(*v_it)[1], mesh.point(*v_it)[2]+mesh.normal(*v_it)[2]);

    //}
    //glEnd();
    glPopMatrix();



}

template <typename M>
void
QtModelT<M>::renderBackBuffer()
{
  glPushMatrix();
  glTranslatef(horizontal, vertical, 0);
  glRotatef(modelRotation.x(), 1, 0, 0);
  glRotatef(modelRotation.y(), 0, 1, 0);
  glRotatef(modelRotation.z(), 0, 0, 1);

  //int N = mesh.n_vertices();
  //float inc = 1.0 / N;
  //float r = 0.0;
  glPointSize(2.0f);
  glBegin(GL_POINTS);

  int r = 0;
  int g = 0;
  int b = 0;
  for (typename M::VertexIter v_it=mesh.vertices_begin(); v_it!=mesh.vertices_end(); ++v_it) 
  {
    if (b==255){
      r = 0;
      g = 0;
      b = 0;
    }
    else if(r == b)
      r++;
    else if(g < r && g == b)
      g++;
    else
      b++;

    glColor3b (r, g, b);
    glVertex3f(mesh.point(*v_it)[0], mesh.point(*v_it)[1], mesh.point(*v_it)[2]); 
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
  mesh.update_normals();
  calcNormals();
  getDistFromGroundTruth();
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

template <typename M>
void
QtModelT<M>::bilateralFiltering(double sigc, double sigs)
{
  std::cout << "filter" << "\n";
  MapTable map;
  nearestNeighbours(sigc, &map);
  std::cout << "got neighbours\n";
  PointMatrix matrix = buildMatrix();

  int c = 0;
  for (typename M::VertexIter v_it=mesh.vertices_begin(); v_it!=mesh.vertices_end(); ++v_it) 
  {
    std::vector< std::pair< size_t, double > > neighbourhood = map[c];
    c++;
    
    //standard dev
    /*
    std::vector<double> v;
    for (size_t i = 0; i < neighbourhood.size(); i++)
    {
      v.reserve(neighbourhood.size());
      Vec3f q = Vec3f(matrix(neighbourhood[i].first, 0), matrix(neighbourhood[i].first, 1),matrix(neighbourhood[i].first, 2));
      OpenMesh::Vec3f pointA = mesh.point(*v_it)-q;
      double h = 0.0;
      h += mesh.normal(*v_it)[0]*pointA[0];
      h += mesh.normal(*v_it)[1]*pointA[1];
      h += mesh.normal(*v_it)[2]*pointA[2];
      v.push_back(h);
    }
    double sumv = std::accumulate(v.begin(), v.end(), 0.0);
    double mean = sumv / v.size();
    std::vector<double> diff(v.size());
    std::transform(v.begin(), v.end(), diff.begin(),
                   std::bind2nd(std::minus<double>(), mean));
    double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / v.size());
    std::cout << stdev << "\n";
    /////*/
    
    
    float sum = 0.0;
    float normalizer = 0.0;
    //float sigc = radius; //radius of neighbourhood
    //float sigs = 0.001; //standard deviation
    
    for (size_t i = 0; i < neighbourhood.size(); i++)
    {
      Vec3f q = Vec3f(matrix(neighbourhood[i].first, 0), matrix(neighbourhood[i].first, 1),matrix(neighbourhood[i].first, 2));
      // Calculate Sum and normalizer
      //std::cout << q[0] << " " << q[1] << " " << q[2] << "\n";
      OpenMesh::Vec3f pointA = mesh.point(*v_it) - q;
      float t = pointA.length();
      float h = 0.0;
      //pointA.normalize_cond();
      typename M::Normal normalVector = mesh.normal(*v_it);
      h += normalVector[0]*pointA[0];
      h += normalVector[1]*pointA[1];
      h += normalVector[2]*pointA[2];

      float wc = exp(-t*t / (2*pow(sigc,2)));
      float ws = exp(-h*h / (2*pow(sigs,2)));
      if (c == 1) std::cout << h << "\n";
      sum += ((wc * ws) * h);
      normalizer += (wc * ws);

    }
    typename M::Point newPoint = mesh.point(*v_it) - (mesh.normal(*v_it) * (sum / normalizer) );
    std::cout << "(" << mesh.point(*v_it) << ")->(" << newPoint << ") " << (sum / normalizer) << "\n";
    mesh.set_point( *v_it,  newPoint);
    
        //float sum = 0.0;
    //float normalizer = 0.0;

    //for (MyMesh::VertexVertexIter vv_it=mesh.vv_iter(v_it.handle()); vv_it; ++vv_it)
    //{
    //// do something with e.g. mesh.point(*vv_it)
    //}

  }
  mesh.update_normals();
  getDistFromGroundTruth();
}

template <typename M>
void
QtModelT<M>::nearestNeighbours(double radius, MapTable* resultTable)
{
  //build kd tree
  PointMatrix pAll = buildMatrix();

  std::cout << pAll << "\n";

  typedef nanoflann::KDTreeEigenMatrixAdaptor<PointMatrix>  kd_tree_t;
  kd_tree_t mat_index(3, pAll, 10);
  mat_index.index->buildIndex();

  //find neighbourhood for each point
  int i = 0;
  for (typename M::VertexIter v_it=mesh.vertices_begin(); v_it!=mesh.vertices_end(); ++v_it)
  {
    
    
    typename M::Point p = mesh.point(*v_it);
    std::vector<double> query_pt(3);
    query_pt[0] = p[0];
    query_pt[1] = p[1];
    query_pt[2] = p[2];
    
    std::cout << p << "\n";
    
    std::vector< std::pair< size_t, double > > resultPairs;
    resultPairs.reserve(mesh.n_vertices());
    
    size_t count = mat_index.index->radiusSearch(&query_pt[0], radius, resultPairs, nanoflann::SearchParams(true));
    std::cout << resultPairs.size() << "\n";
    //std::cout << resultPairs << "\n";
    /*
    std::vector<typename M::Point> closest;
    closest.reserve(count);
    for (size_t i = 0; i < resultPairs.size(); i++){
      typename M::VertexIter vv_it = mesh.vertices_begin();
      for (size_t c = 0; c < resultPairs[i].first; c++){ // maybe c<= ??
        ++vv_it;
      }
      closest.push_back(mesh.point(*vv_it));
    }
    resultTable[p] = closest;
    //std::cout << closest.size() << "\n";
    */
    resultTable->push_back(resultPairs);
    i++;
  }
  std::cout << resultTable->size() << "\n";
}

template <typename M>
float
QtModelT<M>::triangleArea(typename M::ConstFaceVertexIter fvIt)
{
  
}


template <typename M>
void
QtModelT<M>::getDistFromGroundTruth()
{
  typename M::ConstFaceIter    fIt(mesh.faces_begin()),
                               fEnd(mesh.faces_end());
  for (; fIt!=fEnd; ++fIt)
  {
     triangleArea( &mesh.normal(*fIt) );
  }

  gt_distance = 0.5f;
}


#endif
