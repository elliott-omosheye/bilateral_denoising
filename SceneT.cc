#ifndef SCENE_CC
#define SCENE_CC

#include <iostream>
#include "Scene.hh"
#include <QVector3D>
#include "GLUT/glut.h"
#include <QtGui>
#include <QtOpenGL>
#include <OpenMesh/Core/Utils/vector_cast.hh>
#include <OpenMesh/Tools/Utils/Timer.hh>

#ifndef GL_MULTISAMPLE
#define GL_MULTISAMPLE  0x809D
#endif



template <typename M>
QDialog
*SceneT<M>::createDialog(const QString &windowTitle) const
{
  QDialog *dialog = new QDialog(0, Qt::CustomizeWindowHint | Qt::WindowTitleHint);
  
  dialog->setWindowOpacity(0.8);
  dialog->setWindowTitle(windowTitle);
  dialog->setLayout(new QVBoxLayout);
  
  return dialog;
}

template <typename M>
SceneT<M>::SceneT()
: m_backgroundColor(0.0f, 0.0f, 0.0f)
, m_distance(0.3f)
, m_vertical(-0.1f)
, m_horizontal(0.0f)
, TANSLATE_SPEED(0.01f)
{
  
  
  QWidget *examples = createDialog(tr("Examples"));
  m_twoMeshButton = new QPushButton(tr("Bunny 0 and 45 Example"));
  examples->layout()->addWidget(m_twoMeshButton );
  m_sameMeshButton = new QPushButton(tr("Bunny 0 and 0 Example"));
  examples->layout()->addWidget(m_sameMeshButton );
  m_globalButton = new QPushButton(tr("Bunny 0, 45 and 315 Example"));
  examples->layout()->addWidget(m_globalButton );

  
  QWidget *controls = createDialog(tr("Controls"));
  m_modelButton = new QPushButton(tr("Load model"));
  controls->layout()->addWidget(m_modelButton);
  
  QWidget *widgets[] = { controls, examples };
  
  for (uint i = 0; i < sizeof(widgets) / sizeof(*widgets); ++i) {
    QGraphicsProxyWidget *proxy = new QGraphicsProxyWidget(0, Qt::Dialog);
    proxy->setWidget(widgets[i]);
    addItem(proxy);
  }
  
  QPointF pos(10, 10);
  foreach (QGraphicsItem *item, items()) {
    item->setFlag(QGraphicsItem::ItemIsMovable);
    item->setCacheMode(QGraphicsItem::DeviceCoordinateCache);
    
    const QRectF rect = item->boundingRect();
    item->setPos(pos.x() - rect.x(), pos.y() - rect.y());
    pos += QPointF(0, 10 + rect.height());
  }
  
  groupBox = new QGroupBox(tr("Select Mesh"));
  radio1 = new QRadioButton(tr("All"));
  radio2 = new QRadioButton(tr("Blue Mesh"));
  radio3 = new QRadioButton(tr("Red Mesh"));
  radio4 = new QRadioButton(tr("Yellow Mesh"));
  radio5 = new QRadioButton(tr("Turquoise  Mesh"));
  radio6 = new QRadioButton(tr("Pink Mesh"));
  groupBox->setHidden(true);
  radio3->setHidden(true);
  radio4->setHidden(true);
  radio5->setHidden(true);
  radio6->setHidden(true);
  
  radio1->setChecked(true);
  QVBoxLayout *vbox = new QVBoxLayout;
  vbox->addWidget(radio1);
  vbox->addWidget(radio2);
  vbox->addWidget(radio3);
  vbox->addWidget(radio4);
  vbox->addWidget(radio5);
  vbox->addWidget(radio6);
  
  vbox->addStretch(1);
  groupBox->setLayout(vbox);
  controls->layout()->addWidget(groupBox);
  
  m_applyButton = new QPushButton(tr("Apply Local Transformations"));
  controls->layout()->addWidget(m_applyButton);
  m_applyButton->setHidden(true);
  
  m_icpButton = new QPushButton(tr("Run ICP (1 Iteration)"));
  controls->layout()->addWidget(m_icpButton);
  m_icpButton->setHidden(true);
  
  m_icpButton5 = new QPushButton(tr("Run ICP (5 Iterations)"));
  controls->layout()->addWidget(m_icpButton5);
  m_icpButton5->setHidden(true);
  m_icpButton10 = new QPushButton(tr("Run ICP (10 Iterations)"));
  controls->layout()->addWidget(m_icpButton10);
  m_icpButton10->setHidden(true);
  m_icpButton50 = new QPushButton(tr("Run ICP (50 Iterations)"));
  controls->layout()->addWidget(m_icpButton50);
  m_icpButton50->setHidden(true);
  m_icpButton100 = new QPushButton(tr("Run ICP (100 Iterations)"));
  controls->layout()->addWidget(m_icpButton100);
  m_icpButton100->setHidden(true);
  
  m_icpButtonNoise = new QPushButton(tr("Apply Noise to M2"));
  controls->layout()->addWidget(m_icpButtonNoise);
  m_icpButtonNoise->setHidden(true);
  
  m_icpButtonSampling = new QPushButton(tr("Run ICP with subsamping (100 Iterations)"));
  controls->layout()->addWidget(m_icpButtonSampling);
  m_icpButtonSampling->setHidden(true);
  
  
}

template <typename M>
void
SceneT<M>::runTwoMesh(const QString m1, const QString m2)
{
  models.clear();
  groupBox->setHidden(true);
  radio3->setHidden(true);
  radio4->setHidden(true);
  radio5->setHidden(true);
  radio6->setHidden(true);
  loadMesh(m1);
  loadMesh(m2);
}

template <typename M>
void
SceneT<M>::loadGlobal(const QString m1, const QString m2, const QString m3)
{
  models.clear();
  groupBox->setHidden(true);
  radio3->setHidden(true);
  radio4->setHidden(true);
  radio5->setHidden(true);
  radio6->setHidden(true);
  loadMesh(m1);
  loadMesh(m2);
  loadMesh(m3);

}

template <typename M>
void
SceneT<M>::drawForeground(QPainter *painter, const QRectF &rect)
{
  //std::cout << "Draw Foreground" << "\n";
  painter->beginNativePainting();
  
  if(models.size() > 0){
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluPerspective(70, width() / height(), 0.01, 1000);
    
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    
    glTranslatef(m_horizontal, m_vertical, -m_distance);
    glRotatef(m_rotation.x(), 1, 0, 0);
    glRotatef(m_rotation.y(), 0, 1, 0);
    glRotatef(m_rotation.z(), 0, 0, 1);
    
    glEnable(GL_MULTISAMPLE);
    for(typename std::vector<QtModelT<M>*>::size_type i = 0; i != models.size(); i++) {
      models[i]->render();
    }
    
    glDisable(GL_MULTISAMPLE);
    
    glPopMatrix();
    
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
  }
  
  painter->endNativePainting();
  
}



template <typename M>
void
SceneT<M>::drawBackground(QPainter *painter, const QRectF &)
{
  if (painter->paintEngine()->type() != QPaintEngine::OpenGL
      && painter->paintEngine()->type() != QPaintEngine::OpenGL2)
  {
    qWarning("OpenGLScene: drawBackground needs a QGLWidget to be set as viewport on the graphics view");
    return;
  }
  
  painter->beginNativePainting();
  glClearColor(m_backgroundColor.redF(), m_backgroundColor.greenF(), m_backgroundColor.blueF(), 1.0f);
  //glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  
  painter->endNativePainting();
}




template <typename M>
void
SceneT<M>::loadMesh(const QString filePath)
{
  if (filePath.isEmpty())
    return;
  
  
  std::cout << filePath.toStdString() << "\n";
  m_modelButton->setEnabled(false);
  QApplication::setOverrideCursor(Qt::BusyCursor);
  if(OpenMesh::IO::read_mesh(m_mymesh, filePath.toStdString(), _options))
  {

    models.push_back(new QtModelT<M>(m_mymesh));
    models.back()->updateColour(models.size());
    groupBox->setHidden(false);
    m_applyButton->setHidden(false);
    
    switch(models.size())
    {
      case 2:
        radio3->setHidden(false);
        m_icpButton->setHidden(false);
        m_icpButton5->setHidden(false);
        m_icpButton10->setHidden(false);
        m_icpButton50->setHidden(false);
        m_icpButton100->setHidden(false);
        m_icpButtonSampling->setHidden(false);
        m_icpButtonNoise->setHidden(false);
        break;
      case 3:
        radio4->setHidden(false);
        m_icpButtonNoise->setHidden(true);
        
        break;
      case 4:
        radio5->setHidden(false);
        break;
      case 5:
        radio6->setHidden(false);
        break;
    }
    
    std::clog << m_mymesh.n_vertices() << " vertices, "
    << m_mymesh.n_edges()    << " edge, "
    << m_mymesh.n_faces()    << " faces\n";
  }
  else
  {
    std::cout << "Error Loading Mesh" << "\n";
  }
  m_modelButton->setEnabled(true);
  QApplication::restoreOverrideCursor();
  
}

template <typename M>
void
SceneT<M>::updateColour()
{
  for(int i=0; i<(int)models.size(); i++)
  {
    for(int j=0; j<(int)models.size(); j++)
    {
      if(i != j)
      {
        //std::cout << "i: " << i << " j: " << j << "\n";
        models[i]->mergeColours(models[j]);
      }
    }
  }
}


template <typename M>
void
SceneT<M>::applyTransformations()
{
  std::cout << "Apply Transformations" << "\n";
  for(typename std::vector<QtModelT<M>*>::size_type i = 0; i != models.size(); i++) {
    models[i]->applyTransformations();
  }
}

template <typename M>
void
SceneT<M>::runICPglobalMeshes(int iterations, bool sampling)
{
  /*
  for (unsigned long i = 0; i < models.size(); i++)
  {
    for (unsigned long j = 0; j < models.size(); j++)
    {
      if (j != i) {
          runICPTwoMeshes(models[j], models[i], iterations);
      }
    }
  }
   */

  for (unsigned long i = 0; i < models.size(); i++)
  {
    M mymesh;
    for (unsigned long j = 0; j < models.size(); j++)
    {
      if (j != i) {
        M* mesh = models[j]->getMesh();
        for (typename M::VertexIter v_it=mesh->vertices_sbegin(); v_it!=mesh->vertices_end(); ++v_it){
          mymesh.add_vertex(mesh->point(*v_it));
        }
      }
    }
    QtModelT<M>* result = new QtModelT<M>(mymesh);
    std::cout << "run i: " << result->getMesh()->n_vertices() << " verts\n";
    sampling ? samplingICP(result, models[i], iterations) : runICPTwoMeshes(result, models[i], iterations) ;
    
  }

/*
  typedef nanoflann::KDTreeEigenMatrixAdaptor<PointMatrix> my_kd_tree_t;
  size_t smallx;
  size_t smally;
  //repeat until cannot find smallest distances
  int tries = models.size()-1;
  double threshold = 0.0;
  bool findPair = true;
  while (findPair){
    findPair = false;
    //find means for each surface
    PointMatrix means(models.size(), 3);
    for (unsigned long c = 0; c < models.size(); c++)
    {
      PointMatrix qAll = models[c]->buildMatrix();
      means(c,0) = qAll.colwise().mean()[0];
      means(c,1) = qAll.colwise().mean()[1];
      means(c,2) = qAll.colwise().mean()[2];
      std::cout << qAll.mean() << "\n";
    }
    my_kd_tree_t mat_index(3, means, 10);
    
    Matrix<double, Dynamic, Dynamic> mappings(models.size(), models.size());
    Matrix<double, Dynamic, Dynamic> distances(models.size(), models.size());
    
    //find distance between surfaces
    for (size_t i = 0; i < models.size(); i++)
    {
      PointMatrix pAll = models[i]->buildMatrix();
      std::vector<double> query_pt(3);
      query_pt[0] = means(i, 0);
      query_pt[1] = means(i, 1);
      query_pt[2] = means(i, 2);
      nanoflann::KNNResultSet<double> resultSet(models.size());
      std::vector<size_t>   ret_index(models.size());
      std::vector<double>    out_dist_sqr(models.size());
      resultSet.init(&ret_index[0], &out_dist_sqr[0]);
      mat_index.index->findNeighbors(resultSet, &query_pt[0], nanoflann::SearchParams(10));
      //fill matrices with results
      for (size_t j = 0; j < models.size(); j++){
        mappings(i, j) = ret_index[j];
        distances(i, j) = out_dist_sqr[j];
      }
    }
    //find smallest distances
    //
    double small = 100;
    for (size_t i = 0, nRows = distances.rows(), nCols = distances.cols(); i < nCols; ++i){
      for (size_t j = 0; j < nRows; ++j)
      {
        if (i != j && distances(j,i) > threshold && distances(j,i) < small){
          smallx = j;
          smally = i;
          small = distances(j,i);
          findPair = true;
        }
      }
    }
    
    //ICP
    if (findPair) {
      if ((sampling && samplingICP(models[smallx], models[smally], iterations)) || runICPTwoMeshes(models[smallx], models[smally], iterations)){
        //runICPTwoMeshes(models[smallx], models[smally], iterations);
        std::cout << smallx << " merge " << smally << " dist " << small << "\n\n";
        threshold = distances(smallx,smally);
        //merge
        M* mesh = models[smally]->getMesh();
        M* resultMesh = models[smallx]->getMesh();
        std::cout << resultMesh->n_vertices();
        for (typename M::VertexIter v_it=mesh->vertices_sbegin(); v_it!=mesh->vertices_end(); ++v_it){
          resultMesh->add_vertex(mesh->point(*v_it));
        };
        models[smallx]->updateColour(smallx);
        models.erase(models.begin() + smally);
      }
    } else if (tries-- < 1) findPair = false;
    
  }
  */
}

template <typename M>
bool
SceneT<M>::runICPTwoMeshes(QtModelT<M>* m1, QtModelT<M>* m2, int iterations)
{
  //q = still = m1, p = mover = m2;
  bool notConverged = true;
  int iterCount = 0;
  while(notConverged)
  {
    const float max_range = 0.01;
    
    const size_t num_results = 1;
    PointMatrix qAll = m1->buildMatrix();
    PointMatrix pAll = m2->buildMatrix();
    
    typedef nanoflann::KDTreeEigenMatrixAdaptor< PointMatrix >  my_kd_tree_t;
    my_kd_tree_t mat_index(3, qAll, 10);
    mat_index.index->buildIndex();
    
    int rowCount = m2->getNoVerticies();
    std::cout << rowCount << "rows \n";
    Matrix<double, Dynamic, 1> mappings(rowCount, 1);
    Matrix<double, Dynamic, 1> distances(rowCount, 1);
    
    for(int i=0; i < rowCount; i++)
    {
      std::vector<double> query_pt(3);
      query_pt[0] = pAll(i, 0);
      query_pt[1] = pAll(i, 1);
      query_pt[2] = pAll(i, 2);
      
      std::vector<size_t>   ret_index(num_results);
      std::vector<double>    out_dist_sqr(num_results);
      
      nanoflann::KNNResultSet<double> resultSet(num_results);
      
      resultSet.init(&ret_index[0], &out_dist_sqr[0] );
      mat_index.index->findNeighbors(resultSet, &query_pt[0], nanoflann::SearchParams(10));
      
      mappings(i, 0) = ret_index[0];
      distances(i, 0) = out_dist_sqr[0];
    }
    
    double distThreshold = (max_range < 4 * getMedian(distances)) ? max_range : 4 * getMedian(distances);
    std::vector<int> goodPairs;
    goodPairs.reserve(pAll.rows());
    for(int i = 0; i < rowCount; i++)
    {
      if(distances(i, 0) <= distThreshold)
        goodPairs.push_back(i);
    }
    
    PointMatrix q(goodPairs.size(), 3);
    PointMatrix p(goodPairs.size(), 3);
    for(int c = 0; c < (int)goodPairs.size(); c++)
    {
      p.row(c) = pAll.row(goodPairs[c]);
      q.row(c) = qAll.row(mappings(goodPairs[c], 0));
      //std::cout << "Mapping" << p.row(c) << "\t" << q.row(c) << "\t" << (p.row(c)-q.row(c)).norm()  << "\n";
    }
    
    PointMatrix qHat(q.rows(), 3);
    PointMatrix pHat(p.rows(), 3);
    
    getBaryCenteredPoints(qHat, q);
    getBaryCenteredPoints(pHat, p);
    
    Matrix<double, 3, 3> A(3,3);
    generateA(A, pHat, qHat);
    
    JacobiSVD< MatrixXd > svd(A, ComputeThinU | ComputeThinV);
    
    Matrix<double, 3, 3> R = svd.matrixU() * svd.matrixV().transpose();
    Matrix<double, 1, 3> temp1 = (R * p.colwise().mean().transpose());
    Matrix<double, 1, 3> temp2 = q.colwise().mean();
    Matrix<double, 1, 3> t = temp1 - temp2;
    std::cout << "Iteration " << iterCount << "\n";
    
    //std::cout << R << "\n";
    //std::cout << t << "\n";
    
    if (isnan(R(0,0) + R(1,1) + R(2,2)) || isinf(t.norm()) ) {
      std::cout << "threw error " << "\n\n";
      std::cout << "Sum of diagonal of R " << (R(0,0) + R(1,1) + R(2,2)) << "\n";
      std::cout << "Norm of t " << t.norm() << "\n\n";
      return false;
    }
    m2->updateTransformations(R, t(0, 0), t(0,1), t(0, 2));
    invalidate();
    
    std::cout << "Sum of diagonal of R " << (R(0,0) + R(1,1) + R(2,2)) << "\n";
    std::cout << "Norm of t " << t.norm() << "\n\n";
    
    if(t.norm() < 0.00005 && (R(0,0) + R(1,1) + R(2,2)) > 2.999)
    {
      std::cout << "Converged in " << iterCount << " iterations"  << "\n";
      notConverged = false;
      return true;
    } else if(iterCount >= iterations-1)
    {
      notConverged = false;
      std::cout << "No convergence after " << iterations << " iterations"   << "\n";
      return false;
    }
    iterCount++;
    update();
  }
  return false;
}

template <typename M>
void
SceneT<M>::runICP(int iterations)
{
  applyTransformations();
  std::cout << "Run ICP" << "\n";
  if(models.size() == 2)
  {
    OpenMesh::Utils::Timer timer;
    timer.start();
    runICPTwoMeshes(models[0], models[1], iterations);
    timer.stop();
    std::cout << "Time: " << timer.seconds() << " seconds" << "\n";

  }
  else if(models.size() > 2)
  {
    OpenMesh::Utils::Timer timer;
    timer.start();

    runICPglobalMeshes(iterations, false);

    timer.stop();
    std::cout << "Time: " << timer.seconds() << " seconds" << "\n";
    /*
     radio3->setHidden(true);
     radio4->setHidden(true);
     radio5->setHidden(true);
     radio6->setHidden(true);
     */
  }
  updateColour();
}

template <typename M>
void
SceneT<M>::runNoiseICP()
{
  models[1]->addNoise();
}

template <typename M>
bool
SceneT<M>::samplingICP(QtModelT<M>* m1, QtModelT<M>*  m2, int iterations)
{
  //q = still = m1, p = mover = m2;
  bool notConverged = true;
  int iterCount = 0;
  while(notConverged)
  {
    const float max_range = 0.01;
    
    const size_t num_results = 1;
    PointMatrix qAll = m1->buildMatrix();
    PointMatrix pAll = m2->buildSampledMatrix();
    
    typedef nanoflann::KDTreeEigenMatrixAdaptor< PointMatrix >  my_kd_tree_t;
    my_kd_tree_t mat_index(3, qAll, 10);
    mat_index.index->buildIndex();
    
    int rowCount = 5000;
    std::cout << rowCount << "rows \n";
    Matrix<double, Dynamic, 1> mappings(rowCount, 1);
    Matrix<double, Dynamic, 1> distances(rowCount, 1);
    
    for(int i=0; i < rowCount; i++)
    {
      std::vector<double> query_pt(3);
      query_pt[0] = pAll(i, 0);
      query_pt[1] = pAll(i, 1);
      query_pt[2] = pAll(i, 2);
      
      std::vector<size_t>   ret_index(num_results);
      std::vector<double>    out_dist_sqr(num_results);
      
      nanoflann::KNNResultSet<double> resultSet(num_results);
      
      resultSet.init(&ret_index[0], &out_dist_sqr[0] );
      mat_index.index->findNeighbors(resultSet, &query_pt[0], nanoflann::SearchParams(10));
      
      mappings(i, 0) = ret_index[0];
      distances(i, 0) = out_dist_sqr[0];
    }
    
    double distThreshold = (max_range < 4 * getMedian(distances)) ? max_range : 4 * getMedian(distances);
    std::vector<int> goodPairs;
    goodPairs.reserve(pAll.rows());
    for(int i = 0; i < rowCount; i++)
    {
      if(distances(i, 0) <= distThreshold)
        goodPairs.push_back(i);
    }
    
    PointMatrix q(goodPairs.size(), 3);
    PointMatrix p(goodPairs.size(), 3);
    for(int c = 0; c < (int)goodPairs.size(); c++)
    {
      p.row(c) = pAll.row(goodPairs[c]);
      q.row(c) = qAll.row(mappings(goodPairs[c], 0));
      //std::cout << "Mapping" << p.row(c) << "\t" << q.row(c) << "\t" << (p.row(c)-q.row(c)).norm()  << "\n";
    }
    
    PointMatrix qHat(q.rows(), 3);
    PointMatrix pHat(p.rows(), 3);
    
    getBaryCenteredPoints(qHat, q);
    getBaryCenteredPoints(pHat, p);
    
    Matrix<double, 3, 3> A(3,3);
    generateA(A, pHat, qHat);
    
    JacobiSVD< MatrixXd > svd(A, ComputeThinU | ComputeThinV);
    
    Matrix<double, 3, 3> R = svd.matrixU() * svd.matrixV().transpose();
    Matrix<double, 1, 3> temp1 = (R * p.colwise().mean().transpose());
    Matrix<double, 1, 3> temp2 = q.colwise().mean();
    Matrix<double, 1, 3> t = temp1 - temp2;
    std::cout << "Iteration " << iterCount << "\n";
    
    //std::cout << R << "\n";
    //std::cout << t << "\n";
    
    if (isnan(R(0,0) + R(1,1) + R(2,2)) || isinf(t.norm()) ) {
      std::cout << "threw error " << "\n\n";
      std::cout << "Sum of diagonal of R " << (R(0,0) + R(1,1) + R(2,2)) << "\n";
      std::cout << "Norm of t " << t.norm() << "\n\n";
    } else {
      m2->updateTransformations(R, t(0, 0), t(0,1), t(0, 2));
      invalidate();
      
      std::cout << "Sum of diagonal of R " << (R(0,0) + R(1,1) + R(2,2)) << "\n";
      std::cout << "Norm of t " << t.norm() << "\n\n";
      
      if(t.norm() < 0.00005 && (R(0,0) + R(1,1) + R(2,2)) > 2.999)
      {
        std::cout << "Converged in " << iterCount << " iterations"  << "\n";
        notConverged = false;
        return true;
      } else if(iterCount >= iterations-1)
      {
        notConverged = false;
        std::cout << "No convergence after " << iterations << " iterations"   << "\n";
        return false;
      }
      iterCount++;
      glDrawBuffer(GL_FRONT);
      update();
    }
  }
  return false;
}

template <typename M>
void
SceneT<M>::runSamplingICP()
{
    applyTransformations();
    OpenMesh::Utils::Timer timer;
    timer.start();
  if (models.size() == 2){
    samplingICP(models[0], models[1], 100);
  } else if (models.size()>2){
    runICPglobalMeshes(100, true);
    /*
     radio3->setHidden(true);
     radio4->setHidden(true);
     radio5->setHidden(true);
     radio6->setHidden(true);
     */
  }
    timer.stop();
    std::cout << "Time: " << timer.seconds() << " seconds" << "\n";
    updateColour();

}



template <typename M>
void 
SceneT<M>::generateA(Matrix<double, 3, 3>  &A, const PointMatrix &qHat, const PointMatrix &pHat)
{
  A = (pHat.transpose()*qHat)*(qHat.transpose()*qHat).inverse();
}


template <typename M>
void
SceneT<M>::getBaryCenteredPoints(PointMatrix &matHat, const PointMatrix &mat)
{
  //std::cout << "getBaryCenteredPoints called" << "\n";
  Matrix<double, 1, 3> mean = mat.colwise().mean();
  matHat = mat.rowwise() - mean;
}

/*
 *NEED TO WORK OUT WHY THIS ISN"T WORKING!!!!!!!
 */
template <typename M>
double
SceneT<M>::getMedian(Matrix<double, Dynamic, 1> mat)
{
  return mat.mean();
}

template <typename M>
void
SceneT<M>::wheelEvent(QGraphicsSceneWheelEvent *event)
{
  
  QGraphicsScene::wheelEvent(event);
  if (event->isAccepted())
    return;
  
  m_distance *= qPow(1.2, -event->delta() / 120.);
  event->accept();
  update();
  
}

template <typename M>
void
SceneT<M>::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
  QGraphicsScene::mouseMoveEvent(event);
  if (event->isAccepted())
    return;
  if (event->buttons() & Qt::LeftButton) {
    const QPointF delta = event->scenePos() - event->lastScenePos();
    QVector3D angularImpulse = QVector3D(delta.y(), delta.x(), 0) * 0.1;
    
    const int radioId = whichRadioButton();
    if(radioId  == 1){
      m_rotation += angularImpulse;
    }
    else
    {
      models[radioId-2]->updateRotation(angularImpulse);
    }
    
    event->accept();
    update();
  }
}

template <typename M>
void
SceneT<M>::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
  QGraphicsScene::mousePressEvent(event);
  if (event->isAccepted())
    return;
  
  m_mouseEventTime = m_time.elapsed();
  event->accept();
}

template <typename M>
void
SceneT<M>::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
  QGraphicsScene::mouseReleaseEvent(event);
  if (event->isAccepted())
    return;
  
  const int delta = m_time.elapsed() - m_mouseEventTime;
  event->accept();
  update();
}

template <typename M>
void
SceneT<M>::keyPressEvent( QKeyEvent* event)
{
  const int radioId = whichRadioButton();
  if(radioId  == 1)
  {
    
    switch(event->key())
    {
      case Key_Up:
        m_vertical += TANSLATE_SPEED;
        break;
      case Key_Down:
        m_vertical -= TANSLATE_SPEED;
        break;
      case Key_Right:
        m_horizontal += TANSLATE_SPEED;
        break;
      case Key_Left:
        m_horizontal -= TANSLATE_SPEED;
        break;
    }
  }
  else
  {
    switch(event->key())
    {
      case Key_Up:
        models[radioId-2]->updateVertical(TANSLATE_SPEED);
        break;
      case Key_Down:
        models[radioId-2]->updateVertical(-TANSLATE_SPEED);
        break;
      case Key_Right:
        models[radioId-2]->updateHorizontal(TANSLATE_SPEED);
        break;
      case Key_Left:
        models[radioId-2]->updateHorizontal(-TANSLATE_SPEED);
        break;
    }
  }
  event->accept();
  update();
}

template <typename M>
int
SceneT<M>::whichRadioButton()
{
  if(radio1->isChecked())
    return 1;
  else if(radio2->isChecked())
    return 2;
  else if(radio3->isChecked())
    return 3;
  else if(radio4->isChecked())
    return 4;
  else if(radio5->isChecked())
    return 5;
  else if(radio6->isChecked())
    return 6;
  else
    return 0;
  
}


#endif
