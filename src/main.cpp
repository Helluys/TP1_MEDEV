
#include <iostream>
#include <ctime>
#include <cstdlib>
#include "Avion.h"
#include "Joueur.h"
#include "Ennemi.h"
#include <vector>
#include "Cube.h"
//#include "afficher.h"

int main()
{
    srand(time(NULL));
    Cube jeu(13);
    jeu.boucle();

    return 0;
}

