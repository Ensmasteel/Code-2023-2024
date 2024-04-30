#include "GameElement.h"

#define xmax 2.0
#define ymax 3.0

void Base::setCoordonnees(VectorOriented coor){
    this->coordonnees=coor;
}

VectorOriented Base::getCoordonnees(){
    return coordonnees;
}

bool Base::empty(){
    return this->nbGateau==0;
}

bool Base::full(){
    return this->nbGateau==3;
}

void Base::setNbGateau(uint8_t nbg){
    nbGateau=nbg;
}

void GameElement::setup(CouleurBase coulB){

    couleurBase=coulB;

    palets[0].coordonnees=Vector(0.225,0.775);
    palets[1].coordonnees=Vector(0.225+0.04*1.41/2.0,ymax-0.775-0.04*1.41/2.0);
    palets[2].coordonnees=Vector(xmax-0.225,0.775);
    palets[3].coordonnees=Vector(xmax-0.225+0.04*1.41/2.0,ymax-0.775-0.04*1.41/2.0);

    palets[0].couleur=Jaune;
    palets[1].couleur=Jaune;
    palets[2].couleur=Jaune;
    palets[3].couleur=Jaune;

    palets[4].coordonnees=Vector(0.225,0.535);//????
    palets[5].coordonnees=Vector(0.225,ymax-0.575);
    palets[6].coordonnees=Vector(xmax-0.225,0.535);
    palets[7].coordonnees=Vector(xmax-0.225,ymax-0.575);

    palets[4].couleur=Rose;
    palets[5].couleur=Rose;
    palets[6].couleur=Rose;
    palets[7].couleur=Rose;

    palets[8].coordonnees = Vector(0.725,1.125);
    palets[9].coordonnees = Vector(0.725,ymax-1.125);
    palets[10].coordonnees = Vector(xmax-0.725,1.125);
    palets[11].coordonnees = Vector(xmax-0.725,ymax-1.125);

    palets[8].couleur=Marron;
    palets[9].couleur=Marron;
    palets[10].couleur=Marron;
    palets[11].couleur=Marron;
    




    if(couleurBase==Vert){

    
        bases[0].setCoordonnees(VectorOriented(0.225,0.225,-3*PI/4));
        bases[1].setCoordonnees(VectorOriented(xmax-0.2,1.125,0.0));
        bases[2].setCoordonnees(VectorOriented(0.225,ymax-1.125,PI));
        bases[3].setCoordonnees(VectorOriented(xmax-0.225,ymax-0.225,PI/4));
        bases[4].setCoordonnees(VectorOriented(0.675,ymax-0.225,PI/2));

        palets[2].onTable = false;
        
        palets[3].onTable = false;
        palets[6].onTable = false;
        palets[7].onTable = false;
        palets[10].onTable = false;
        palets[11].onTable = false;
        //palets[1].onTable = false;
        //palets[5].onTable = false;


    }
    else{
        bases[0].setCoordonnees(VectorOriented(xmax-0.225,0.225,-PI/4));
        bases[1].setCoordonnees(VectorOriented(0.225,1.125,PI));
        bases[2].setCoordonnees(VectorOriented(xmax-0.2,ymax-1.125,0.0));
        bases[3].setCoordonnees(VectorOriented(0.225,ymax-0.225,3*PI/4));
        bases[4].setCoordonnees(VectorOriented(xmax-0.675,ymax-0.225,PI/2));

        palets[0].onTable = false;
        palets[1].onTable = false;
        //palets[3].onTable = false;
        palets[4].onTable = false;
        palets[5].onTable = false;
        palets[8].onTable = false;
        palets[9].onTable = false;
        //palets[7].onTable = false;
        


    }

}

Palet* GameElement::getPaletInProgress(){
    return paletInProgress;
}

Base* GameElement::getBaseInProgress(){
    return baseInProgress;
}

Palet* GameElement::nearestPaletJaune(VectorOriented posRobot){
    float distanceMin=10.0;
    int imin=0;
    for(int i=0; i<12; i++){
        float dist = posRobot.distanceWith(palets[i].coordonnees);
        if((palets[i].onTable==true) && (palets[i].couleur==Jaune)&& (dist<distanceMin)){
            distanceMin=dist;
            imin=i;
        }
    }
    return &palets[imin];
}

Palet* GameElement::nearestPaletRose(VectorOriented posRobot){
    float distanceMin=10.0;
    int imin=0;
    for(int i=0; i<12; i++){
        float dist = posRobot.distanceWith(palets[i].coordonnees);
        if((palets[i].onTable==true)&& (palets[i].couleur==Rose)&& (dist<distanceMin)){
            //palets[i].coordonnees.printDebug("Rose :");
            distanceMin=dist;
            imin=i;
        }
    }
    return &palets[imin];
}

Palet* GameElement::nearestPaletMarron(VectorOriented posRobot){
    float distanceMin=10.0;
    int imin=0;
    for(int i=0; i<12; i++){
        float dist = posRobot.distanceWith(palets[i].coordonnees);
        if((palets[i].onTable==true) && (palets[i].couleur==Marron) && (dist<distanceMin)){
            distanceMin=dist;
            imin=i;
        }
    }
    return &palets[imin];
}

int GameElement::countPaletsInTable(){
    int inTab = 0;
    for (int k=0;k<12;k++){
        if (palets[k].onTable){
            inTab+=1;
        }
    }
    return inTab;
}

Vector GameElement::nearestPalet(VectorOriented posRobot, bool jaune, bool marron, bool rose){
    
    nearestJaune = nearestPaletJaune(posRobot);
    nearestRose = nearestPaletRose(posRobot);
    nearestMarron = nearestPaletMarron(posRobot);
    float distanceJaune = posRobot.distanceWith(nearestJaune->coordonnees);
    float distanceRose = posRobot.distanceWith(nearestRose->coordonnees);
    float distanceMarron = posRobot.distanceWith(nearestMarron->coordonnees);
    float distanceMin = 10.0;
    if(jaune){
        if (distanceJaune<distanceMin){
            couleurChosen = Jaune;
            distanceMin=distanceJaune;
        }
        
    }
    if(marron){
        
        if (distanceMarron<distanceMin){
            couleurChosen = Marron;
            distanceMin=distanceMarron;
        }
    }
    if(rose){
        if (distanceRose<distanceMin){
            couleurChosen = Rose;
            distanceMin=distanceRose;
        }
    }
    switch(couleurChosen){
        case Jaune:
            paletInProgress=nearestJaune;
            return nearestJaune->coordonnees;
            break;
        case Rose:
            paletInProgress=nearestRose;
            return nearestRose->coordonnees;
            break;
        case Marron:
            paletInProgress=nearestMarron;
            return nearestMarron->coordonnees;
            break;
        default:
            break;
    }
}

void GameElement::printDebug(const String& prefix)
{
    paletInProgress->coordonnees.printDebug(prefix);
}

CouleurPalet GameElement::getCouleurPalet(){
    return couleurChosen;
}

VectorOriented GameElement::nearestBase(VectorOriented posRobot){
    float distanceMin=10.0;
    int imin=0;

    for(int i=0; i<5; i++){
        float dist = bases[i].getCoordonnees().distanceWith(posRobot);
        if((!bases[i].full()) && (dist<distanceMin)){//ATTENTION
            distanceMin=dist;
            imin=i;
        }
    }
    baseInProgress=&bases[imin];
    float angle = bases[imin].getCoordonnees().getTheta();
    uint8_t nbG = bases[imin].getNbGateau();
    return bases[imin].getCoordonnees()-VectorOriented(nbG*0.10*cos(angle),nbG*0.10*sin(angle),0.0);
}

VectorOriented GameElement::nearestBaseEmpty(VectorOriented posRobot){
    float distanceMin=10.0;
    int imin=0;

    for(int i=0; i<5; i++){
        float dist = bases[i].getCoordonnees().distanceWith(posRobot);
        if((bases[i].empty()) && (dist<distanceMin)){//ATTENTION
            distanceMin=dist;
            imin=i;
        }
    }
    baseInProgress=&bases[imin];
    return bases[imin].getCoordonnees();
}