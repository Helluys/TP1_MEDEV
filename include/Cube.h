#pragma once

#include <iostream>
#include <ctime>
#include <cstdlib>
#include <vector>
#include "Avion.h"
#include "Joueur.h"
#include "Ennemi.h"
//#include "afficher.h"

// Base
#include <osgViewer/Viewer>
#include <osg/ShapeDrawable>
#include <osg/PositionAttitudeTransform>
#include <osg/Material>
#include <osg/StateSet>
#include <osg/MatrixTransform>
#include <osg/Geometry>
#include <osgDB/ReadFile>
#include <osgGA/TrackballManipulator>

// Keyboard input
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/StateSetManipulator>

class Cube {
    
public:
    Cube(int taille);
    ~Cube();
    void boucle();
    int getTaille() { return _taille; }
    osgViewer::Viewer* getViewer() { return _viewer; }
        
private:
    void constructionAvions();
    osg::Vec3f getSubCubePosition(int i,int j,int k);
    osg::ref_ptr<osg::Node> createSubCube(int i,int j,int k);
    osg::ref_ptr<osg::Group> createCube();
    void afficherCube();
    void afficherAvions();
    void elimination(std::vector<int> ListeTouches, std::vector<Avion*> &ListeAvion);
    bool verificationFin(std::vector<Avion*> &ListeAvion);
    
    int _compteur, _taille;
    bool _fin;
    std::vector<Avion*> _listeAvion;
    
    Joueur* _J1;
    Joueur* _J2;
    Ennemi* _E1;
    Ennemi* _E2;

    osgViewer::Viewer* _viewer;
    osg::ref_ptr<osg::Group> _root;
    osg::ref_ptr<osg::PositionAttitudeTransform> _patAvionAmi1;
    osg::ref_ptr<osg::PositionAttitudeTransform> _patAvionAmi2;
    osg::ref_ptr<osg::PositionAttitudeTransform> _patAlignement;
    osg::ref_ptr<osg::PositionAttitudeTransform> _patAvionEnnemi1;
    osg::ref_ptr<osg::PositionAttitudeTransform> _patAvionEnnemi2;
    osg::ref_ptr<osg::PositionAttitudeTransform> _patScale;
};
