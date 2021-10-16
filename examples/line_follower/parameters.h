#ifndef PARAMETERS_H
#define PARAMETERS_H

#endif // PARAMETERS_H

#define learning
//#define impulseMeasure
//#define reflex
//#define oldMethod
//#define significanceLearning

#define STEPSCOUNT 1000
// Network Structure
#define nPROPAGATIONS 1
#define LEARNINGRATE 0.1


#define NLAYERS 11
    #define N1 10
    #define N2 10
    #define N3 10
    #define N4 10
    #define N5 10
    #define N6 10
    #define N7 10
    #define N8 10
    #define N9 10
    #define N10 10
    #define N11 2

// Predictor Array
#define ROW1P 11
#define ROW1N 48
#define ROW1S 0.25

#define ROW2P 12
#define ROW2N 48
#define ROW2S 0.25

#define ROW3P 13
#define ROW3N 48
#define ROW3S 0.25

// Error Arrays
#define SENSORLEFT 5
#define SENSORRIGHT -5

// Filter Specifications
#define NFILTERS 5
#define MINT 3 //3
#define MAXT 10 //10
#define DAMPINGCOEFF 0.51

// Robot speed
#define SPEED 30 // for reflex it is 60
#define ERRORGAIN 50 // for reflex it is 200

#define NETWORKGAIN 5
#define PREDGAIN 1


