#include "Cube.h"
#include <iostream>
#include <ctime>
#include <cstdlib>
#include <vector>

#include "Avion.h"
#include "Joueur.h"
#include "Ennemi.h"
#include "Cube.h"
//#include "afficher.h"

using namespace std;

Cube::Cube(int taille) : _taille(taille), _compteur(0), _fin(false)
{
    _viewer = new osgViewer::Viewer;
    _viewer->addEventHandler(new osgViewer::StatsHandler);
    _viewer->addEventHandler(new osgViewer::WindowSizeHandler);
    _viewer->addEventHandler( new osgGA::StateSetManipulator(_viewer->getCamera()->getOrCreateStateSet()));
    _viewer->setCameraManipulator(new osgGA::TrackballManipulator);
    _viewer->realize();	
    _viewer->setSceneData(_root.get());
    
    _root = new osg::Group;
    
    _J1 = new Joueur();
    _J2 = new Joueur();
    _E1 = new Ennemi();
    _E2 = new Ennemi();
    
    _J1->setPosition(osg::Vec3f(0,5,0));
    _J1->setDirection(osg::Vec3f(1,0,0));
    _J1->setId(0);
    _J2->setPosition(osg::Vec3f(0,3,0));
    _J2->setDirection(osg::Vec3f(1,0,0));
    _E1->setPosition(osg::Vec3f(2,5,2));
    _E1->setDirection(osg::Vec3f(-1,0,0));
    _E2->setPosition(osg::Vec3f(2,8,2));
    _E2->setDirection(osg::Vec3f(-1,0,0));
    _listeAvion.push_back(_J1);
    _listeAvion.push_back(_J2);
    _listeAvion.push_back(_E1);
    _listeAvion.push_back(_E2);

    constructionAvions();
}

Cube::~Cube()
{
    delete _J1;
    delete _J2;
    delete _E1;
    delete _E1;
}

void Cube::boucle()
{
    while(!_fin)
    {
	for (unsigned int i = 0; i < _listeAvion.size(); i++)
	    _listeAvion[i]->strategie(_listeAvion);
	
	for (unsigned int i = 0; i < _listeAvion.size(); i++)
	{
	    _listeAvion[i]->tourner();
	    _listeAvion[i]->avancer(_taille);
	}
	
	_listeAvion[0]->detecteCollision(_taille,_listeAvion);

	vector<int> _listeAvionsTouches;
	for (unsigned int i = 0; i < _listeAvion.size(); i++) // Chaque avion Tire et on recupere les id des avions touches
	{
	    int idTouche = _listeAvion[i]->tirer(_taille, _listeAvion);
	    if (idTouche != -1)
		_listeAvionsTouches.push_back(idTouche);
	}

	elimination(_listeAvionsTouches,_listeAvion);
	
	_patAvionAmi1->setPosition(_listeAvion[0]->getPosition());
	_patAvionAmi2->setPosition(_listeAvion[1]->getPosition());
	_patAvionEnnemi1->setPosition(_listeAvion[2]->getPosition());
	_patAvionEnnemi2->setPosition(_listeAvion[3]->getPosition());

	afficherCube();
	afficherAvions();
	
	_fin = _viewer->done();
	_viewer->frame();
    }
}

void Cube::constructionAvions()
{	
    _patAvionAmi1 = new osg::PositionAttitudeTransform();
    _patAvionAmi2 = new osg::PositionAttitudeTransform();
    _patAlignement = new osg::PositionAttitudeTransform();
    _patAlignement->setAttitude(osg::Quat(osg::DegreesToRadians(-60.0),osg::Vec3f(0,0,1)));
    _patAvionEnnemi1 = new osg::PositionAttitudeTransform();
    _patAvionEnnemi2 = new osg::PositionAttitudeTransform();
    _patScale = new osg::PositionAttitudeTransform();

    /* PAT INITIALISATION */ //corriger echelle position
    _patScale->setScale(osg::Vec3f(0.0001,0.0001,0.0001));
    _patAvionEnnemi1->setScale(osg::Vec3f(3,3,3));
    _patAvionEnnemi2->setScale(osg::Vec3f(3,3,3));

    /* MATERIAL CREATION & INITIALISATION */
    osg::ref_ptr<osg::Material> matAmi (new osg::Material);
    osg::ref_ptr<osg::Material> matEnnemi (new osg::Material);
    matAmi->setAmbient(matAmi->FRONT_AND_BACK,osg::Vec4f(1,1,1,1));
    matEnnemi->setAmbient(matEnnemi->FRONT_AND_BACK,osg::Vec4f(1,1,1,1));

    /* AIRCRAFTS IMPORTATION */
    osg::ref_ptr<osg::Node> avionAmi(osgDB::readNodeFile("Avion.3ds"));
    osg::ref_ptr<osg::Node> avionEnnemi(osgDB::readNodeFile("TIE-fighter.3ds"));

    /* STATE SETS CREATION & INITIALISATION */
    osg::ref_ptr<osg::StateSet> stateSetAmi1(_patAvionAmi1->getOrCreateStateSet());
    osg::ref_ptr<osg::StateSet> stateSetAmi2(_patAvionAmi2->getOrCreateStateSet());
    osg::ref_ptr<osg::StateSet> stateSetEnnemi1(_patAvionEnnemi1->getOrCreateStateSet());
    osg::ref_ptr<osg::StateSet> stateSetEnnemi2(_patAvionEnnemi2->getOrCreateStateSet());

    stateSetAmi1->setAttribute(matAmi);
    stateSetAmi2->setAttribute(matAmi);
    stateSetEnnemi1->setAttribute(matEnnemi);
    stateSetEnnemi2->setAttribute(matEnnemi);

    /* SCENE GRAPH */
    _root->addChild(_patScale.get());
    _patScale->addChild(_patAvionEnnemi1.get());
    _patScale->addChild(_patAvionEnnemi2.get());

    _patScale->addChild(_patAlignement.get());
    _patAlignement->addChild(_patAvionAmi1.get());
    _patAlignement->addChild(_patAvionAmi2.get());

    _patAvionAmi1->addChild(avionAmi.get());
    _patAvionAmi2->addChild(avionAmi.get());
    _patAvionEnnemi1->addChild(avionEnnemi.get());
    _patAvionEnnemi2->addChild(avionEnnemi.get());
}

osg::Vec3f Cube::getSubCubePosition(int i, int j, int k) {
    float x = (float)(2.0f*i)/_taille - 1;
    float y = (float)(2.0f*j)/_taille - 1;
    float z = (float)(2.0f*k)/_taille - 1;
    osg::Vec3f* pos = new osg::Vec3f(x,y,z);
    return *pos;
}

osg::ref_ptr<osg::Node> Cube::createSubCube(int i, int j,int k) {
	osg::ref_ptr<osg::PositionAttitudeTransform> translationSubCube (new osg::PositionAttitudeTransform);
   	osg::ref_ptr<osg::Geode> geodeSubCube (new osg::Geode);
	osg::ref_ptr<osg::Box> mySubCube (new osg::Box(osg::Vec3f(),2.0/_taille));
	osg::ref_ptr<osg::ShapeDrawable> drawableSubCube (new osg::ShapeDrawable(mySubCube.get()));
	geodeSubCube->addDrawable(drawableSubCube.get());


	translationSubCube->setPosition(getSubCubePosition(i,j,k));
	translationSubCube->addChild(geodeSubCube.get());
	return translationSubCube.get();
}

osg::ref_ptr<osg::Group> Cube::createCube() {
	osg::ref_ptr<osg::Group> cube (new osg::Group);
	osg::PolygonMode * polygonMode = new osg::PolygonMode;
    polygonMode->setMode( osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::POINT);
    osg::ref_ptr<osg::StateSet> stateSet (cube->getOrCreateStateSet());
    stateSet->setAttributeAndModes(polygonMode,osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);
	for(int i=0; i<_taille; i++) {
		for(int j=0; j<_taille; j++) {
			for(int k=0; k<_taille; k++) {
				cube->addChild(createSubCube(i,j,k));
			}
		}
	}

	return cube;
}

void Cube::afficherCube()
{
    osg::ref_ptr<osg::StateSet> rootStateSet(_root->getOrCreateStateSet());

    /* TERRAIN */
    //Loading the terrain node
    osg::ref_ptr<osg::Node> terrainNode (osgDB::readNodeFile("terrain3.3ds"));
    osg::ref_ptr<osg::PositionAttitudeTransform> terrainTranslation (new osg::PositionAttitudeTransform);

    terrainTranslation->setPosition(osg::Vec3f(0,0,-1.8));
    osg::ref_ptr<osg::MatrixTransform> terrainScaleTransform (new osg::MatrixTransform);
    osg::Matrix terrainScaleMatrix;
    terrainScaleMatrix.makeScale(osg::Vec3f(0.005f,0.005f,0.005f));
    terrainScaleTransform->setMatrix(terrainScaleMatrix);

    osg::ref_ptr<osg::PositionAttitudeTransform> translationPlane1 (new osg::PositionAttitudeTransform);
    osg::ref_ptr<osg::Geode> geodePlane1 (new osg::Geode);
    osg::ref_ptr<osg::Box> myPlane1 (new osg::Box(osg::Vec3f(),0.01,8,8));
    osg::ref_ptr<osg::ShapeDrawable> drawablePlane1 (new osg::ShapeDrawable(myPlane1.get()));
    geodePlane1->addDrawable(drawablePlane1.get());
    translationPlane1->setPosition(osg::Vec3f(-2,0,0));
    translationPlane1->addChild(geodePlane1.get());

    osg::ref_ptr<osg::PositionAttitudeTransform> translationPlane2 (new osg::PositionAttitudeTransform);
    osg::ref_ptr<osg::Geode> geodePlane2 (new osg::Geode);
    osg::ref_ptr<osg::Box> myPlane2 (new osg::Box(osg::Vec3f(),0.01,8,8));
    osg::ref_ptr<osg::ShapeDrawable> drawablePlane2 (new osg::ShapeDrawable(myPlane2.get()));
    geodePlane2->addDrawable(drawablePlane2.get());
    translationPlane2->setPosition(osg::Vec3f(2,0,0));
    translationPlane2->addChild(geodePlane2.get());

    osg::ref_ptr<osg::PositionAttitudeTransform> translationPlane3 (new osg::PositionAttitudeTransform);
    osg::ref_ptr<osg::Geode> geodePlane3 (new osg::Geode);
    osg::ref_ptr<osg::Box> myPlane3 (new osg::Box(osg::Vec3f(),8,0.01,8));
    osg::ref_ptr<osg::ShapeDrawable> drawablePlane3 (new osg::ShapeDrawable(myPlane3.get()));
    geodePlane3->addDrawable(drawablePlane3.get());
    translationPlane3->setPosition(osg::Vec3f(0,2,0));
    translationPlane3->addChild(geodePlane3.get());

    osg::ref_ptr<osg::PositionAttitudeTransform> translationPlane4 (new osg::PositionAttitudeTransform);
    osg::ref_ptr<osg::Geode> geodePlane4 (new osg::Geode);
    osg::ref_ptr<osg::Box> myPlane4 (new osg::Box(osg::Vec3f(),8,8,0.01));
    osg::ref_ptr<osg::ShapeDrawable> drawablePlane4 (new osg::ShapeDrawable(myPlane4.get()));
    geodePlane4->addDrawable(drawablePlane4.get());
    translationPlane4->setPosition(osg::Vec3f(0,0,4));
    translationPlane4->addChild(geodePlane4.get());

    osg::ref_ptr<osg::Material> material (new osg::Material);

    //Setting material 1 - capsule
    material->setAmbient(osg::Material::FRONT_AND_BACK,osg::Vec4(0,0,0.8,1));
    material->setDiffuse(osg::Material::FRONT_AND_BACK,osg::Vec4(0,0,0.8,1));
    material->setSpecular(osg::Material::FRONT_AND_BACK,osg::Vec4(0,0,0.8,1));
    material->setEmission(osg::Material::FRONT_AND_BACK,osg::Vec4(0,0,0,0));
    material->setShininess(osg::Material::FRONT_AND_BACK, 0);

    osg::ref_ptr<osg::StateSet> nodeStateSet1 ( geodePlane1->getOrCreateStateSet() );
    osg::ref_ptr<osg::StateSet> nodeStateSet2 ( geodePlane2->getOrCreateStateSet() );
    osg::ref_ptr<osg::StateSet> nodeStateSet3 ( geodePlane3->getOrCreateStateSet() );
    osg::ref_ptr<osg::StateSet> nodeStateSet4 ( geodePlane4->getOrCreateStateSet() );

    nodeStateSet1->setAttribute(material);
    nodeStateSet2->setAttribute(material);
    nodeStateSet3->setAttribute(material);
    nodeStateSet4->setAttribute(material);

    /* MASTER CUBE */
    osg::ref_ptr<osg::Group> cube = createCube();

    /* LIGHTING */
    //Create nodes
    osg::ref_ptr<osg::Group> lightGroup0 (new osg::Group);
    osg::ref_ptr<osg::LightSource> lightSource0 = new osg::LightSource;

    //Create a local light
    osg::Vec4f lightPosition0(0,-2,3,0);

    osg::ref_ptr<osg::Light> myLight0 = new osg::Light;
    myLight0->setLightNum(0);
    myLight0->setPosition(lightPosition0);
    myLight0->setDiffuse(osg::Vec4f(1,1,1,1));
    myLight0->setAmbient(osg::Vec4f(0.3,0.3,0.3,1));
    myLight0->setSpecular(osg::Vec4f(1,0,0,0));

    //Set light source parameters
    lightSource0->setLight(myLight0.get());

    //Add to light source group
    lightGroup0->addChild(lightSource0.get());

    // Allumage des lumieres
    rootStateSet->setMode(GL_LIGHT0, osg::StateAttribute::ON);

    /* SCENE GRAPH*/
    terrainScaleTransform->addChild(terrainNode);
    terrainTranslation->addChild(terrainScaleTransform.get());
    _root->addChild(cube.get());
    _root->addChild(lightGroup0.get());
    _root->addChild(terrainTranslation.get());
    //_root->addChild(translationPlane1.get());
    //_root->addChild(translationPlane2.get());
    //_root->addChild(translationPlane3.get());
    //_root->addChild(translationPlane4.get());
}

void Cube::afficherAvions() {	
	osg::Quat attitude;

	_patAvionAmi1->setPosition(_listeAvion[0]->getPosition());
	_patAvionAmi2->setPosition(osg::Vec3d(_listeAvion[1]->getPosition()[1],_listeAvion[1]->getPosition()[1],_listeAvion[1]->getPosition()[2]));
	_patAvionEnnemi1->setPosition(getSubCubePosition(_listeAvion[2]->getPosition()[2],_listeAvion[2]->getPosition()[1],_listeAvion[2]->getPosition()[2]));
	_patAvionEnnemi2->setPosition(getSubCubePosition(_listeAvion[3]->getPosition()[3],_listeAvion[3]->getPosition()[1],_listeAvion[3]->getPosition()[2]));
	
	attitude.makeRotate(osg::Vec3d(1,0,0),osg::Vec3d(_listeAvion[0]->getDirection()));
	_patAvionAmi1->setAttitude(attitude);
	attitude.makeRotate(osg::Vec3d(1,0,0),osg::Vec3d(_listeAvion[1]->getDirection()));
	_patAvionAmi2->setAttitude(attitude);
	attitude.makeRotate(osg::Vec3d(1,0,0),osg::Vec3d(_listeAvion[2]->getDirection()));
	_patAvionEnnemi1->setAttitude(attitude);
	attitude.makeRotate(osg::Vec3d(1,0,0),osg::Vec3d(_listeAvion[3]->getDirection()));
	_patAvionEnnemi2->setAttitude(attitude);
}

void Cube::elimination(vector<int> ListeTouches, vector<Avion*> &_listeAvion) // Supprime les avions contenus dans le vecteur Listetouches du vecteur principal _listeAvion e partir de leurs Id
{
    for (unsigned int i=0;i<ListeTouches.size();i++) // on elimine les avions touches
            {
                for(unsigned int j= 0; j<_listeAvion.size();j++)
                {
                    if (_listeAvion[j]->getId() == ListeTouches[i])
                    {
                         _listeAvion.erase(_listeAvion.begin() + j);
                    }
                }

            }
}

bool Cube::verificationFin(std::vector<Avion*> &_listeAvion) // Si il reste deux avions ou moins, on regarde leurs camps pour savoir si la partie est finie
{
    if (_listeAvion.size() == 2)
    {
       return  (_listeAvion[0]->getCamp() == _listeAvion[1]->getCamp());
    }
    if (_listeAvion.size() == 1)
    {
        return true ;
    }
    else
    {
        return false ;
    }
}
