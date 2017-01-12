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
/* Affichage du monde */

Cube::Cube(int _n) {
	fin = false;
	root = new osg::Group;
	viewer = new osgViewer::Viewer;
	n = _n;
	compteur =0;
    Joueur* J1 = new Joueur();
    Joueur* J2 = new Joueur();
    Ennemi* E1 = new Ennemi();
    Ennemi* E2 = new Ennemi();
    J1->setPosition(osg::Vec3f(0,5,0));
    J1->setDirection(osg::Vec3f(1,0,0));
    J1->setId(0);
    J2->setPosition(osg::Vec3f(0,3,0));
    J2->setDirection(osg::Vec3f(1,0,0));
    E1->setPosition(osg::Vec3f(2,5,2));
    E1->setDirection(osg::Vec3f(-1,0,0));
    E2->setPosition(osg::Vec3f(2,8,2));
    E2->setDirection(osg::Vec3f(-1,0,0));
    ListeAvion.push_back(J1);
    ListeAvion.push_back(J2);
    ListeAvion.push_back(E1);
    ListeAvion.push_back(E2);


	ConstructionAvion();

	viewer->addEventHandler(new osgViewer::StatsHandler);

	//Windows size handler
	viewer->addEventHandler(new osgViewer::WindowSizeHandler);

	// add the state manipulator
    viewer->addEventHandler( new osgGA::StateSetManipulator(viewer->getCamera()->getOrCreateStateSet()));
	
	
	viewer->setCameraManipulator(new osgGA::TrackballManipulator);
    viewer->realize();
	
	viewer->setSceneData(root.get());
	afficherCube();
}
void Cube::ConstructionAvion() {
	
	patAvionAmi1 = new osg::PositionAttitudeTransform();
	patAvionAmi2 = new osg::PositionAttitudeTransform();
	patAlignement = new osg::PositionAttitudeTransform();
	patAlignement->setAttitude(osg::Quat(osg::DegreesToRadians(-60.0),osg::Vec3f(0,0,1)));
	patAvionEnnemi1 = new osg::PositionAttitudeTransform();
	patAvionEnnemi2 = new osg::PositionAttitudeTransform();
	patSCALE = new osg::PositionAttitudeTransform();
		/* PAT INITIALISATION */ //corriger echelle position

	patSCALE->setScale(osg::Vec3f(0.0001,0.0001,0.0001));
	patAvionEnnemi1->setScale(osg::Vec3f(3,3,3));
	patAvionEnnemi2->setScale(osg::Vec3f(3,3,3));

	/* MATERIAL CREATION & INITIALISATION */

	osg::ref_ptr<osg::Material> mat (new osg::Material);
	osg::ref_ptr<osg::Material> mat2 (new osg::Material);
	mat->setAmbient(mat->FRONT_AND_BACK,osg::Vec4f(1,1,1,1));
	mat2->setAmbient(mat2->FRONT_AND_BACK,osg::Vec4f(1,1,1,1));

	/* AIRCRAFTS IMPORTATION */

	osg::ref_ptr<osg::Node> avionAmi(osgDB::readNodeFile("Avion.3ds"));
	osg::ref_ptr<osg::Node> avionEnnemi(osgDB::readNodeFile("TIE-fighter.3ds"));

	/* STATE SETS CREATION & INITIALISATION */

	osg::ref_ptr<osg::StateSet> StateSetAmi1(patAvionAmi1->getOrCreateStateSet());
	osg::ref_ptr<osg::StateSet> StateSetAmi2(patAvionAmi2->getOrCreateStateSet());
	osg::ref_ptr<osg::StateSet> StateSetEnnemi1(patAvionEnnemi1->getOrCreateStateSet());
	osg::ref_ptr<osg::StateSet> StateSetEnnemi2(patAvionEnnemi2->getOrCreateStateSet());

	StateSetAmi1->setAttribute(mat);
	StateSetAmi2->setAttribute(mat);
	StateSetEnnemi1->setAttribute(mat2);
	StateSetEnnemi2->setAttribute(mat2);

	/* SCENE GRAPH */
	root->addChild(patSCALE.get());
	patSCALE->addChild(patAvionEnnemi1.get());
	patSCALE->addChild(patAvionEnnemi2.get());
	
	patSCALE->addChild(patAlignement.get());
	patAlignement->addChild(patAvionAmi1.get());
	patAlignement->addChild(patAvionAmi2.get());

	patAvionAmi1->addChild(avionAmi.get());
	patAvionAmi2->addChild(avionAmi.get());
	patAvionEnnemi2->addChild(avionEnnemi.get());
	patAvionEnnemi1->addChild(avionEnnemi.get());
}

void Cube::afficherCube()
{
	// StateSet de root
	osg::ref_ptr<osg::StateSet> rootStateSet (root->getOrCreateStateSet());

/* TERRAIN */

	osg::ref_ptr<osg::PositionAttitudeTransform> terrainTranslation (new osg::PositionAttitudeTransform);

	terrainTranslation->setPosition(osg::Vec3f(0,0,-1.8));
	// Create transformation node
	osg::ref_ptr<osg::MatrixTransform> terrainScaleMat (new osg::MatrixTransform);

	// Scale matrix
	osg::Matrix terrainScaleMatrix;
	terrainScaleMatrix.makeScale(osg::Vec3f(0.005f,0.005f,0.005f));

	//Loading the terrain node
	osg::ref_ptr<osg::Node> terrainNode (osgDB::readNodeFile("terrain3.3ds"));
	//Set transformation node parameters
	terrainScaleMat->setMatrix(terrainScaleMatrix);

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

	terrainScaleMat->addChild(terrainNode);
	terrainTranslation->addChild(terrainScaleMat.get());
	root->addChild(cube.get());
	root->addChild(lightGroup0.get());
	root->addChild(terrainTranslation.get());
	//root->addChild(translationPlane1.get());
	//root->addChild(translationPlane2.get());
	//root->addChild(translationPlane3.get());
	//root->addChild(translationPlane4.get());
}


osg::Vec3f Cube::getSubCubePosition(int i, int j, int k) {
	float x = (float)(2.0f*i)/n - 1;
	float y = (float)(2.0f*j)/n - 1;
	float z = (float)(2.0f*k)/n - 1;
	osg::Vec3f* pos = new osg::Vec3f(x,y,z);
	return *pos;
}

osg::ref_ptr<osg::Node> Cube::createSubCube(int i, int j,int k) {
	osg::ref_ptr<osg::PositionAttitudeTransform> translationSubCube (new osg::PositionAttitudeTransform);
   	osg::ref_ptr<osg::Geode> geodeSubCube (new osg::Geode);
	osg::ref_ptr<osg::Box> mySubCube (new osg::Box(osg::Vec3f(),2.0/n));
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
	for(int i=0; i<n; i++) {
		for(int j=0; j<n; j++) {
			for(int k=0; k<n; k++) {
				cube->addChild(createSubCube(i,j,k));
			}
		}
	}

	return cube;
}

void Cube::afficherAvion() {	
	osg::Quat attitude;

	patAvionAmi1->setPosition(ListeAvion[0]->getPosition());
	patAvionAmi2->setPosition(osg::Vec3d(ListeAvion[1]->getPosition()[1],ListeAvion[1]->getPosition()[1],ListeAvion[1]->getPosition()[2]));
	patAvionEnnemi1->setPosition(getSubCubePosition(ListeAvion[2]->getPosition()[2],ListeAvion[2]->getPosition()[1],ListeAvion[2]->getPosition()[2]));
	patAvionEnnemi2->setPosition(getSubCubePosition(ListeAvion[3]->getPosition()[3],ListeAvion[3]->getPosition()[1],ListeAvion[3]->getPosition()[2]));
	
	attitude.makeRotate(osg::Vec3d(1,0,0),osg::Vec3d(ListeAvion[0]->getDirection()));
	patAvionAmi1->setAttitude(attitude);
	attitude.makeRotate(osg::Vec3d(1,0,0),osg::Vec3d(ListeAvion[1]->getDirection()));
	patAvionAmi2->setAttitude(attitude);
	attitude.makeRotate(osg::Vec3d(1,0,0),osg::Vec3d(ListeAvion[2]->getDirection()));
	patAvionEnnemi1->setAttitude(attitude);
	attitude.makeRotate(osg::Vec3d(1,0,0),osg::Vec3d(ListeAvion[3]->getDirection()));
	patAvionEnnemi2->setAttitude(attitude);
}

void Cube::elimination(vector<int> ListeTouches, vector<Avion*> &ListeAvion) // Supprime les avions contenus dans le vecteur Listetouches du vecteur principal ListeAvion e partir de leurs Id
{
    for (unsigned int i=0;i<ListeTouches.size();i++) // on elimine les avions touches
            {
                for(unsigned int j= 0; j<ListeAvion.size();j++)
                {
                    if (ListeAvion[j]->getId() == ListeTouches[i])
                    {
                         ListeAvion.erase(ListeAvion.begin() + j);
                    }
                }

            }
}

bool Cube::VerificationFin(std::vector<Avion*> &ListeAvion) // Si il reste deux avions ou moins, on regarde leurs camps pour savoir si la partie est finie
{
    if (ListeAvion.size() == 2)
    {
       return  (ListeAvion[0]->getCamp() == ListeAvion[1]->getCamp());
    }
    if (ListeAvion.size() == 1)
    {
        return true ;
    }
    else
    {
        return false ;
    }
}

void Cube::mainLoop() {

		afficherAvion();
	
    for (unsigned int i=0;i<ListeAvion.size();i++) // Annonce les parametres de chaque avion
    {
        ListeAvion[i]->strategie(ListeAvion);
    }
    for (unsigned int i=0;i<ListeAvion.size();i++) // faire tourner et avancer tous les avions
    {
        ListeAvion[i]->tourner();
        ListeAvion[i]->avancer(n);
    }
	/*
    ListeAvion[0]->DetecteCollision(n,ListeAvion);

    vector<int> ListeAvionsTouches ;
    for (unsigned int i=0;i<ListeAvion.size();i++) // Chaque avion Tire et on recupere les id des avions touches
    {
        int idTouche = ListeAvion[i]->tirer(n, ListeAvion);
        if (idTouche != -1 )
        {
            ListeAvionsTouches.push_back(idTouche);
        }
    }
    elimination(ListeAvionsTouches,ListeAvion);
	*/

	patAvionAmi1->setPosition(ListeAvion[0]->getPosition());
	fin = viewer->done();
    viewer->frame();
}
