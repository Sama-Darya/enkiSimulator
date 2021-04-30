#ifndef PARAMETERS_H
#define PARAMETERS_H

#endif // PARAMETERS_H

#define STEPSCOUNT 2000
// Network Structure
#define nPROPAGATIONS 1
#define LEARNINGRATE 0.01

#define NLAYERS 11
    #define N1 11
    #define N2 11
    #define N3 11
    #define N4 11
    #define N5 11
    #define N6 11
    #define N7 11
    #define N8 11
    #define N9 11
    #define N10 4
    #define N11 2

// Predictor Array
#define ROW1P 11
#define ROW1N 6
#define ROW1S 3

#define ROW2P 12
#define ROW2N 4
#define ROW2S 3

#define ROW3P 13
#define ROW3N 2
#define ROW3S 3

// Error Arrays
#define SENSORLEFT 8
#define SENSORRIGHT -8

// Filter Specifications
#define NFILTERS 5
#define MINT 3 //3
#define MAXT 10 //10
#define DAMPINGCOEFF 0.51

// Robot speed
#define SPEED 20 // for reflex it is 60
#define ERRORGAIN 100 // for reflex it is 200

#define NETWORKGAIN 1
#define PREDGAIN 10


