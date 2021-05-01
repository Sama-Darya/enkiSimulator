/*
    Enki - a fast 2D robot simulator
    Copyright (C) 2017 Bernd Porr <mail@berndporr.me.uk>
    Copyright (C) 1999-2016 Stephane Magnenat <stephane at magnenat dot net>
    Copyright (C) 2004-2005 Markus Waibel <markus dot waibel at epfl dot ch>
    Copyright (c) 2004-2005 Antoine Beyeler <abeyeler at ab-ware dot com>
    Copyright (C) 2005-2006 Laboratory of Intelligent Systems, EPFL, Lausanne
    Copyright (C) 2006-2008 Laboratory of Robotics Systems, EPFL, Lausanne
    See AUTHORS for details

    This program is free software; the authors of any publication 
    arising from research using this software are asked to add the 
    following reference:
    Enki - a fast 2D robot simulator
    http://home.gna.org/enki
    Stephane Magnenat <stephane at magnenat dot net>,
    Markus Waibel <markus dot waibel at epfl dot ch>
    Laboratory of Intelligent Systems, EPFL, Lausanne.

    You can redistribute this program and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

/*
Custom Robot example which has ground sensors and follows a line
It has also a camera which looks to the front and IR sensors
*/
#include <../../enki/Enki.h>
#include <QApplication>
#include <QtGui>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <sstream>
#include <iomanip>
#include "Racer.h"
#include "cldl/Neuron.h"
#include "cldl/Layer.h"
#include "cldl/Net.h"
#include "bandpass.h"
#include "parameters.h"
#include <chrono>


#define learning
//#define reflex
#define oldMethod

using namespace Enki;
using namespace std;
using namespace std::chrono;

double	maxx = 250;
double	maxy = 250;

int countSteps=0;
int firstStep = 1; //so that it does propInputs once and then back/forth in that order
int nInputs= ROW1N+ROW2N+ROW3N;
#ifdef learning
    int nPredictors=nInputs;
#endif


class EnkiPlayground : public EnkiWidget
{
protected:
	Racer* racer;
    double speed = SPEED, prevX, prevY, prevA;
    FILE* errorlog = nullptr;
    FILE* fcoord = nullptr;
    FILE* fspeed = nullptr;

#ifdef learning
    Net* net;
    double* pred = nullptr;
    Bandpass*** bandpass = 0;
    double minT = MINT, maxT = MAXT, dampingCoeff = DAMPINGCOEFF;
    FILE** filtlog = nullptr;
    FILE* predlog = nullptr;
    FILE* stats = nullptr;
    FILE* wlog = nullptr;
    FILE* gradientlog = nullptr;
    std::vector<int> injectionLayers_BackProp;
#endif


public:
    EnkiPlayground(World *world, QWidget *parent = 0):
		EnkiWidget(world, parent)
	{
        racer = new Racer(nInputs);
        racer->pos = Point(maxx/2 +35, maxy/2 +5); // x and y of the start point
        racer->leftSpeed = speed;
        racer->rightSpeed = speed;
        world->addObject(racer);

        fcoord = fopen("coordData.tsv","wt");
        errorlog = fopen("errorData.tsv","wt");
        fspeed = fopen ("speeds.tsv","wt");

#ifdef learning
        filtlog = new FILE*[NFILTERS];
        filtlog[0]= fopen("fp1.tsv","wt");
        filtlog[1]= fopen("fp2.tsv","wt");
        filtlog[2]= fopen("fp3.tsv","wt");
        filtlog[3]= fopen("fp4.tsv","wt");
        filtlog[4]= fopen("fp5.tsv","wt");
        predlog = fopen("predictors.tsv","wt");
        stats = fopen("stats.tsv", "wt");
        wlog = fopen("weight_distances.tsv", "wt");
        gradientlog = fopen("gradient.tsv","wt");
        racer->setPreds(ROW1P,ROW1N,ROW1S);
        racer->setPreds(ROW2P,ROW2N,ROW2S);
        racer->setPreds(ROW3P,ROW3N,ROW3S);
        bandpass = new Bandpass**[nPredictors];
        for(int i=0;i<nPredictors;i++) {
            if (bandpass != nullptr) {
                bandpass[i] = new Bandpass*[NFILTERS];
                 double fs = 1;
                 double fmin = fs/maxT;
                 double fmax = fs/minT;
                 double df = (fmax-fmin)/((double)(NFILTERS-1));
                 double f = fmin;

                for(int j=0;j<NFILTERS;j++) {
                    bandpass[i][j] = new Bandpass();
                    bandpass[i][j]->setParameters(f,dampingCoeff);
                    f = f + df;
                    for(int k=0;k<maxT;k++) {
                        double a = 0;
                        if (k==minT) {
                            a = 1;
                        }
                        double b = bandpass[i][j]->filter(a);
                        assert(b != NAN);
                        assert(b != INFINITY);
                    }
                    bandpass[i][j]->reset();
                }
            }
        }
        int NetnInputs = nPredictors * NFILTERS;
        int nLayers= NLAYERS;
        int nNeurons[NLAYERS]={N1,N2,N3,N4,N5,N6,N7,N8,N9,N10,N11};
        int* nNeuronsp=nNeurons;
#ifdef oldMethod
        net = new Net(NLAYERS, nNeuronsp, NetnInputs);
#else
        net = new Net(nLayers, nNeuronsp, NetnInputs, nPROPAGATIONS);
#endif
        net->initNetwork(Neuron::W_RANDOM, Neuron::B_NONE, Neuron::Act_Sigmoid);
        net->setLearningRate(LEARNINGRATE);
        injectionLayers_BackProp.reserve(1);
        injectionLayers_BackProp = {NLAYERS-1};
        pred = new double[nInputs];
        fprintf(stats, "%d\n", nPredictors);
        for (int i=0; i<nLayers; i++){
            fprintf(stats, "%d\n", nNeurons[i]);
        }
#endif
    }
    ~EnkiPlayground(){
        fclose(fcoord);
        fclose(errorlog);
        fclose(fspeed);
#ifdef learning
        fclose(stats);
        fclose(predlog);
        fclose(filtlog[0]);
        fclose(filtlog[1]);
        fclose(filtlog[2]);
        fclose(filtlog[3]);
        fclose(filtlog[4]);
        fclose(gradientlog);
        delete[] pred;
#endif
    }

virtual void sceneCompletedHook()
	{
		double leftGround = racer->groundSensorLeft.getValue();
		double rightGround = racer->groundSensorRight.getValue();
        double error = (leftGround - rightGround) * ERRORGAIN;
        fprintf(errorlog, "%e\t", error);


#ifdef reflex
        racer->leftSpeed  = speed + error;
        racer->rightSpeed = speed - error;
#endif

#ifdef learning
        for(int i=0; i<nInputs; i++) {
            // workaround of a bug in Enki
            pred[i] = - (racer->groundSensorArray[i]->getValue()) * PREDGAIN;
            if (pred[i]<0) pred[i] = 0;
        }       
        double pred_filtered[nPredictors][NFILTERS];
        for (int i=0; i<nPredictors; i++){
            for (int j=0; j<NFILTERS; j++){
                pred_filtered[i][j]=bandpass[i][j]->filter(pred[i]);
            }
        }
        double* pred_pointer= pred_filtered[0];

        if (firstStep == 1){
            net->setInputs(pred_pointer);
            net->propInputs();
            firstStep = 0;
        }
#ifdef oldMethod
        net->setErrorCoeff(0,1,0,0,0,0);
        net->setGlobalError(error);
        net->setBackwardError(error);
        net->propErrorBackward();
#else
        net->masterPropagate(injectionLayers_BackProp, 0,
                                     Net::BACKWARD, error,
                                     Neuron::Value);
#endif

        net->updateWeights();
        net->snapWeights();
        net->setInputs(pred_pointer);
        net->propInputs();

        double Output= net->getOutput(0) + 2 * net->getOutput(1);
        double error2 = error + Output * NETWORKGAIN;
        racer->leftSpeed  = speed + error2;
        racer->rightSpeed = speed - error2;

        // save to files:
        for(int i=0; i<nPredictors; i++) {
            fprintf(predlog,"%e\t",pred[i]);
        }
        fprintf(predlog,"\n");

        for (int f = 0; f<NFILTERS; f++){
            for (int p = 0; p<nPredictors; p++){
                fprintf(filtlog[f],"%e\t",pred_filtered[p][f]);
            }
            fprintf(filtlog[f],"\n");
        }
        fprintf(fspeed, "%e\t%e\n" , racer->leftSpeed , racer->rightSpeed);
        for (int i = 0; i <NLAYERS; i++){
          fprintf(wlog, "%e\t", net->getLayerWeightDistance(i));
        }
        fprintf(wlog, "%e\n", net->getWeightDistance());

#ifdef oldMethod
#else
        net->snapFistLayerWeights();
#endif
#endif
        fprintf(fcoord,"%e\t%e\n",racer->pos.x,racer->pos.y);
        countSteps ++;
        if (countSteps == STEPSCOUNT){
            cout<< "exiting program: total number of steps is achieved" <<endl;
            qApp->quit();
        }
//        milliseconds ms = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
//        cout << "time is: " << ms.count() << endl;
    }
};

int main(int argc, char *argv[])
{
    srand(5);
    QApplication app(argc, argv);
    QImage gt;
    gt = QGLWidget::convertToGLFormat(QImage("cc.png"));
    if (gt.isNull()) {
        fprintf(stderr,"Texture file not found\n");
        exit(1);
    }
    const uint32_t *bits = (const uint32_t*)gt.constBits();
    World world(maxx, maxy,
                Color(1000, 1000, 1000), World::GroundTexture(gt.width(), gt.height(), bits));
//    cout<<gt.width()<<" "<<gt.height()<<endl;
    EnkiPlayground viewer(&world);
    viewer.show();
    return app.exec();
}

