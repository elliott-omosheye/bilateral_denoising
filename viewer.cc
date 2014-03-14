#include "Scene.hh"
#include <QtGui>
#include <QGLWidget>

using namespace Qt;
using namespace OpenMesh;

class GraphicsView : public QGraphicsView
{
public:
    GraphicsView()
    {
        setWindowTitle(tr("Bilaterial Mesh Denoising"));
    }

protected:
    void resizeEvent(QResizeEvent *event) {
        if (scene())
            scene()->setSceneRect(QRect(QPoint(0, 0), event->size()));
        QGraphicsView::resizeEvent(event);
    }
};

int main(int argc, char **argv)
{
    QApplication app(argc, argv);

    GraphicsView view;
    view.setViewport(new QGLWidget(QGLFormat(QGL::SampleBuffers)));
    view.setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
    view.setScene(new Scene);
    view.show();

    view.resize(1000, 1000);

    return app.exec();
}
