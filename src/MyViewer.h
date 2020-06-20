//TODO: when pressing C when there is no Guizmo
#ifndef MYVIEWER_H
#define MYVIEWER_H

// Mesh stuff:
#include "Mesh.h"

// Parsing:
#include "BasicIO.h"

// opengl and basic gl utilities:
#define GL_GLEXT_PROTOTYPES
#include <gl/openglincludeQtComp.h>
#include <GL/glext.h>
#include <QOpenGLFunctions_3_0>
#include <QOpenGLFunctions>
#include <QGLViewer/qglviewer.h>

#include <gl/GLUtilityMethods.h>
#include <QGLViewer/vec.h>

// Qt stuff:
#include <QFormLayout>
#include <QToolBar>
#include <QColorDialog>
#include <QFileDialog>
#include <QKeyEvent>
#include <QInputDialog>
#include <QLineEdit>

#include <eigen3/Eigen/SparseCore>


#include "qt/QSmartAction.h"
#include <QGLViewer/manipulatedFrame.h>
#include "gizmo.h"

class MyViewer : public QGLViewer , public QOpenGLFunctions_3_0
{
    Q_OBJECT

        Mesh mesh;
    Gizmo gizmo;
    QWidget * controls;
    std::vector<Gizmo> gizmos;
    unsigned int selectedGizmo = 0;
    bool toTransform = false;
    bool computedQi = false;
    bool Mode       = true; //switch between basic transformation and the fancy one


    public :
    MyViewer(QGLWidget * parent = NULL) : QGLViewer(parent) , QOpenGLFunctions_3_0() {
    }

    void transformMesh(){
        std::vector<Eigen::MatrixXf> listMatrix;//TODO: make a struct with the gauscoeff
        for(unsigned int i = 0; i < gizmos.size(); i++){
            listMatrix.push_back(gizmos[i].getMatrix());
        }
        mesh.transform(listMatrix);
    }

    void transformBasicMesh() {
        //TODO: to modify for do iit one time
        std::vector<Eigen::MatrixXf> listMatrix;//TODO: make a struct with the gauscoeff
        for(unsigned int i = 0; i < gizmos.size(); i++){
            listMatrix.push_back(gizmos[i].getMatrix());
        }
        std::vector<GausCoeff>gCoeffs;
        for(unsigned int i = 0; i < gizmos.size(); i++){
            gCoeffs.push_back(GausCoeff({(float)gizmos[i].getOrigin()[0], (float)gizmos[i].getOrigin()[1], (float)gizmos[i].getOrigin()[2]}, 1));
        }

        mesh.transform_Basic(listMatrix, gCoeffs);
    }

    void add_actions_to_toolBar(QToolBar *toolBar)
    {
        // Specify the actions :
        DetailedAction * open_mesh = new DetailedAction( QIcon("./icons/open.png") , "Open Mesh" , "Open Mesh" , this , this , SLOT(open_mesh()) );
        DetailedAction * save_mesh = new DetailedAction( QIcon("./icons/save.png") , "Save model" , "Save model" , this , this , SLOT(save_mesh()) );
        DetailedAction * help = new DetailedAction( QIcon("./icons/help.png") , "HELP" , "HELP" , this , this , SLOT(help()) );
        DetailedAction * saveCamera = new DetailedAction( QIcon("./icons/camera.png") , "Save camera" , "Save camera" , this , this , SLOT(saveCamera()) );
        DetailedAction * openCamera = new DetailedAction( QIcon("./icons/open_camera.png") , "Open camera" , "Open camera" , this , this , SLOT(openCamera()) );
        DetailedAction * saveSnapShotPlusPlus = new DetailedAction( QIcon("./icons/save_snapshot.png") , "Save snapshot" , "Save snapshot" , this , this , SLOT(saveSnapShotPlusPlus()) );

        // Add them :
        toolBar->addAction( open_mesh );
        toolBar->addAction( save_mesh );
        toolBar->addAction( help );
        toolBar->addAction( saveCamera );
        toolBar->addAction( openCamera );
        toolBar->addAction( saveSnapShotPlusPlus );
    }


    void draw() {

        if(toTransform && computedQi){
            if(Mode) {
                transformMesh();
            } else
                transformBasicMesh();
        }

        glEnable(GL_DEPTH_TEST);
        glEnable( GL_LIGHTING );
        glColorMaterial ( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE ) ;
        glEnable ( GL_COLOR_MATERIAL ) ;
        glBegin(GL_TRIANGLES);
        glDisable( GL_TEXTURE_2D );
        for( unsigned int t = 0 ; t < mesh.triangles.size() ; ++t ) {
            point3d const & p0 = mesh.vertices[ mesh.triangles[t][0] ].p;
            point3d const & p1 = mesh.vertices[ mesh.triangles[t][1] ].p;
            point3d const & p2 = mesh.vertices[ mesh.triangles[t][2] ].p;
            point3d const & n = point3d::cross( p1-p0 , p2-p0 ).direction();
            float * colors1 = mesh.colors[mesh.triangles[t][0]];
            float * colors2 = mesh.colors[mesh.triangles[t][1]];
            float * colors3 = mesh.colors[mesh.triangles[t][2]];
            glColor3f((colors1[0] + colors2[0] + colors3[0])/3.0,
                    (colors1[1] + colors2[1] + colors3[1])/3.0,
                    (colors1[2] + colors2[2] + colors3[2])/3.0);
            glNormal3f(n[0],n[1],n[2]);
            glVertex3f(p0[0],p0[1],p0[2]);
            glVertex3f(p1[0],p1[1],p1[2]);
            glVertex3f(p2[0],p2[1],p2[2]);
        }
        drawAxis();
        for(unsigned int i = 0; i < gizmos.size(); i++){
            glPushMatrix();
            if(i == selectedGizmo){
                glMultMatrixd(manipulatedFrame()->matrix());

                qglviewer::Vec position = manipulatedFrame()->position();
                qglviewer::Quaternion orientation = manipulatedFrame()->orientation();
                // Depuis la classe quaterion on peut recuperer une rotation matrix avec getRotationMatrix(qreal m[3][3]) const
                qreal rotationMatrix[3][3];
                orientation.getRotationMatrix(rotationMatrix);
                //std::cout << "Orientation du frame: "<< orientation << std::endl;
                //std::cout << "Position du frame: " << position << std::endl;
                gizmos[selectedGizmo].setTransfoMatrix(position, rotationMatrix);
                glScalef(0.3f, 0.3f, 0.3f);

                drawAxis();
                glPopMatrix();
            }
            else{
                glMultMatrixd(gizmos[i].getFrame()->matrix());
                glScalef(0.3f, 0.3f, 0.3f);

                drawAxis();
                glPopMatrix();

            }
        }

        glEnd();
    }

    void pickBackgroundColor() {
        QColor _bc = QColorDialog::getColor( this->backgroundColor(), this);
        if( _bc.isValid() ) {
            this->setBackgroundColor( _bc );
            this->update();
        }
    }

    void adjustCamera( point3d const & bb , point3d const & BB ) {
        point3d const & center = ( bb + BB )/2.f;
        setSceneCenter( qglviewer::Vec( center[0] , center[1] , center[2] ) );
        setSceneRadius( 1.5f * ( BB - bb ).norm() );
        showEntireScene();
    }


    void init() {
        makeCurrent();
        initializeOpenGLFunctions();

        setMouseTracking(true);// Needed for MouseGrabber.

        setBackgroundColor(QColor(255,255,255));

        // Lights:
        GLTools::initLights();
        GLTools::setSunsetLight();
        GLTools::setDefaultMaterial();

        //
        glShadeModel(GL_SMOOTH);
        glFrontFace(GL_CCW); // CCW ou CW

        glEnable(GL_DEPTH);
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LEQUAL);

        glEnable(GL_CLIP_PLANE0);

        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        glEnable(GL_COLOR_MATERIAL);

        //

        setManipulatedFrame(new qglviewer::ManipulatedFrame());


        setSceneCenter( qglviewer::Vec( 0 , 0 , 0 ) );
        setSceneRadius( 10.f );
        showEntireScene();
    }

    QString helpString() const {
        QString text("<h2>Skinning Subdivision Surfaces</h2>");
        text += "<p>";
        text += "This is a research application, it can explode. "
            "This project aims to optimize in real time the deformations of control points "
            "in order to obtain a smooth and intuitively animated subdivision surface.";
        text += "<h3>Supervisor</h3>";
        text += "<ul>";
        text += "<li>Jean-Marc THIERY</li>";
        text += "</ul>";
        text += "<h3>Participants</h3>";
        text += "<ul>";
        text += "<li>Mickael CORROYER</li>";
        text += "<li>Guillaume DELEPOULLE</li>";
        text += "<li>Julien EUDINE</li>";
        text += "</ul>";
        text += "<h3>Basics</h3>";
        text += "<p>";
        text += "<ul>";
        text += "<li>H   :   make this help appear</li>";
        text += "<li>Ctrl + mouse right button double click   :   choose background color</li>";
        text += "<li>Ctrl + T   :   change window title</li>";
        text += "<li>S   :   Subdivide the mesh with the Loop subdivision</li>";
        text += "<li>R   :   Redisplay the shape using the control points weights and the control points</li>";
        text += "<li>B   :   Display the basic mesh</li>";
        text += "<li>U   : Change the selected gizmo</li>";
        text += "</ul>";
        return text;
    }

    void updateTitle( QString text ) {
        this->setWindowTitle( text );
        emit windowTitleUpdated(text);
    }

    void keyPressEvent( QKeyEvent * event ) {
        if( event->key() == Qt::Key_H ) {
            help();
        }
        else if( event->key() == Qt::Key_T ) {
            if( event->modifiers() & Qt::CTRL )
            {
                bool ok;
                QString text = QInputDialog::getText(this, tr(""), tr("title:"), QLineEdit::Normal,this->windowTitle(), &ok);
                if (ok && !text.isEmpty())
                {
                    updateTitle(text);
                }
            }
        }
        else if (event->key() == Qt::Key_S) {
            if(computedQi) {
                std::cout << "To change the level of subdivision, please reset the mesh, by pressing 'R'!" <<std::endl;
            } else {
                mesh.subdivide();
                this->update();
            }
        }

        else if (event->key() == Qt::Key_R) {
            computedQi = false;
            mesh.reset();
            this->update();
        }

        else if (event->key() == Qt::Key_C) {
            std::vector<GausCoeff>gCoeffs;
            for(unsigned int i = 0; i < gizmos.size(); i++){
                gCoeffs.push_back(GausCoeff({(float)gizmos[i].getOrigin()[0], (float)gizmos[i].getOrigin()[1], (float)gizmos[i].getOrigin()[2]}, 1));
            }
            mesh.computeQis(gCoeffs);
            computedQi = true;
            transformMesh();
            this->update();
        }

        else if( event->key() == Qt::Key_U){
            selectedGizmo ++;
            selectedGizmo = selectedGizmo % gizmos.size();
            setManipulatedFrame(gizmos[selectedGizmo].getFrame());
            std::cout << "selected gizmo: " << selectedGizmo << std::endl;
        }

        else if( event->key() == Qt::Key_M){
            if(!computedQi)
                std::cout << "Before changing Mode and to allow the drawing please press 'C' to compute the constant matrix" << std::endl;
            else {
                if((Mode = !Mode)) {
                    std::cout << "Switched to the optimized transformation preserving the loop subdivision" << std::endl;
                    transformMesh();

                }
                else {
                    std::cout << "Switched to the basic transformation" << std::endl;
                    transformBasicMesh();
                }
                this->update();
            }

        }
        else if (event->key() == Qt::Key_D) {
            if (!gizmos.empty()) {
                gizmos.erase(gizmos.begin() + selectedGizmo);
                if (selectedGizmo > 0) {
                    selectedGizmo--;
                }
                setManipulatedFrame(gizmos[selectedGizmo].getFrame());
                update();
            }

        }
    }

    void mouseDoubleClickEvent( QMouseEvent * e )
    {
        if( (e->modifiers() & Qt::ControlModifier)  &&  (e->button() == Qt::RightButton) )
        {
            pickBackgroundColor();
            return;
        }

        if( (e->modifiers() & Qt::ControlModifier)  &&  (e->button() == Qt::LeftButton) )
        {
            //showControls();
            return;
        }

        QGLViewer::mouseDoubleClickEvent( e );
    }

    void mousePressEvent(QMouseEvent* e ) {
        QGLViewer::mousePressEvent(e);
        bool found;
        if( (e->modifiers() & Qt::AltModifier)  &&  (e->button() == Qt::LeftButton) )
        {
            qglviewer::Vec point = camera()->pointUnderPixel(e->pos(), found);
            //Avant multi gizmo
            /*manipulatedFrame()->setPosition(point);
              gizmo.setOrigin(point);*/

            //Avec multi gizmo
            gizmos.push_back(Gizmo());
            selectedGizmo = gizmos.size() - 1;
            gizmos[selectedGizmo].setOrigin(point);
            setManipulatedFrame(gizmos[selectedGizmo].getFrame());
            manipulatedFrame()->setPosition(point);
            toTransform = true;
            return;
        }
        else if((e->modifiers() & Qt::ControlModifier)  &&  (e->button() == Qt::LeftButton or e->button()==Qt::RightButton)){
            toTransform = true;
            return;
        }
    }

    void mouseMoveEvent(QMouseEvent* e  ){
        QGLViewer::mouseMoveEvent(e);


    }

    void mouseReleaseEvent(QMouseEvent* e  ) {
        QGLViewer::mouseReleaseEvent(e);
        if((e->modifiers() & Qt::ControlModifier || e->modifiers() & Qt::AltModifier)  &&  (e->button() == Qt::LeftButton || e->button()==Qt::RightButton)){
            toTransform = false;
            return;
        }
    }

signals:
    void windowTitleUpdated( const QString & );

    public slots:
        void open_mesh() {
            bool success = false;
            QString fileName = QFileDialog::getOpenFileName(NULL,"","");
            if ( !fileName.isNull() ) { // got a file name
                if(fileName.endsWith(QString(".off"))) {
                    mesh.vertices.clear();
                    mesh.triangles.clear();
                    success = OFFIO::openTriMesh(fileName.toStdString() , mesh.vertices , mesh.triangles );
                }
                else if(fileName.endsWith(QString(".obj"))) {
                    mesh.vertices.clear();
                    mesh.triangles.clear();
                    success = OBJIO::openTriMesh(fileName.toStdString() , mesh.vertices , mesh.triangles );
                }
                if(success) {
                    mesh.basicVertices = mesh.vertices;
                    mesh.resetVertices = mesh.vertices;
                    mesh.basicTriangles = mesh.triangles;
                    mesh.coeffs = std::vector<std::map< unsigned int, float > >(mesh.vertices.size());
                    mesh.colors = std::vector<float[3]> (mesh.vertices.size());
                    for(unsigned int i = 0; i<mesh.vertices.size(); i++) {
                        mesh.coeffs[i][i] = 1;
                        for(unsigned int k = 0; k<3; k++)
                            mesh.colors[i][k] = 0.7;
                    }
                    std::cout << fileName.toStdString() << " was opened successfully" << std::endl;
                    point3d bb(FLT_MAX,FLT_MAX,FLT_MAX) , BB(-FLT_MAX,-FLT_MAX,-FLT_MAX);
                    for( unsigned int v = 0 ; v < mesh.vertices.size() ; ++v ) {
                        bb = point3d::min(bb , mesh.vertices[v]);
                        BB = point3d::max(BB , mesh.vertices[v]);
                    }
                    adjustCamera(bb,BB);
                    update();
                }
                else
                    std::cout << fileName.toStdString() << " could not be opened" << std::endl;
            }
        }

    void save_mesh() {
        bool success = false;
        QString fileName = QFileDialog::getOpenFileName(NULL,"","");
        if ( !fileName.isNull() ) { // got a file name
            if(fileName.endsWith(QString(".off")))
                success = OFFIO::save(fileName.toStdString() , mesh.vertices , mesh.triangles );
            else if(fileName.endsWith(QString(".obj")))
                success = OBJIO::save(fileName.toStdString() , mesh.vertices , mesh.triangles );
            if(success)
                std::cout << fileName.toStdString() << " was saved" << std::endl;
            else
                std::cout << fileName.toStdString() << " could not be saved" << std::endl;
        }
    }

    void showControls()
    {
        // Show controls :
        controls->close();
        controls->show();
    }

    void saveCameraInFile(const QString &filename){
        std::ofstream out (filename.toUtf8());
        if (!out)
            exit (EXIT_FAILURE);
        // << operator for point3 causes linking problem on windows
        out << camera()->position()[0] << " \t" << camera()->position()[1] << " \t" << camera()->position()[2] << " \t" " " <<
            camera()->viewDirection()[0] << " \t" << camera()->viewDirection()[1] << " \t" << camera()->viewDirection()[2] << " \t" << " " <<
            camera()->upVector()[0] << " \t" << camera()->upVector()[1] << " \t" <<camera()->upVector()[2] << " \t" <<" " <<
            camera()->fieldOfView();
        out << std::endl;

        out.close ();
    }

    void openCameraFromFile(const QString &filename){

        std::ifstream file;
        file.open(filename.toStdString().c_str());

        qglviewer::Vec pos;
        qglviewer::Vec view;
        qglviewer::Vec up;
        float fov;

        file >> (pos[0]) >> (pos[1]) >> (pos[2]) >>
            (view[0]) >> (view[1]) >> (view[2]) >>
            (up[0]) >> (up[1]) >> (up[2]) >>
            fov;

        camera()->setPosition(pos);
        camera()->setViewDirection(view);
        camera()->setUpVector(up);
        camera()->setFieldOfView(fov);

        camera()->computeModelViewMatrix();
        camera()->computeProjectionMatrix();

        update();
    }


    void openCamera(){
        QString fileName = QFileDialog::getOpenFileName(NULL,"","*.cam");
        if ( !fileName.isNull() ) {                 // got a file name
            openCameraFromFile(fileName);
        }
    }
    void saveCamera(){
        QString fileName = QFileDialog::getSaveFileName(NULL,"","*.cam");
        if ( !fileName.isNull() ) {                 // got a file name
            saveCameraInFile(fileName);
        }
    }

    void saveSnapShotPlusPlus(){
        QString fileName = QFileDialog::getSaveFileName(NULL,"*.png","");
        if ( !fileName.isNull() ) {                 // got a file name
            setSnapshotFormat("PNG");
            setSnapshotQuality(100);
            saveSnapshot( fileName );
            saveCameraInFile( fileName+QString(".cam") );
        }
    }
};




#endif // MYVIEWER_H
