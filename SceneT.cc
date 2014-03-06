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
  m_ex1Button = new QPushButton(tr("Example 1 (ToDo)"));
  examples->layout()->addWidget(m_ex1Button );
  m_ex2Button= new QPushButton(tr("Example 2 (ToDo)"));
  examples->layout()->addWidget(m_ex2Button);
  m_ex3Button = new QPushButton(tr("Example 3 (ToDo)"));
  examples->layout()->addWidget(m_ex3Button );

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
    //models.back()->updateColour(models.size());
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
    m_rotation += angularImpulse;
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

  event->accept();
  update();
}




#endif
