
#ifndef AutoQuadStates_H
#define AutoQuadStates_H

#include "AutoQuad.h"

class StateManager{
  private:
    QuadController *quadController;
    int state, stateLast;
    bool stateInit;
    
    void idleState( double dt );  //  0
    void hoverState( double dt ); //  1
    void navState( double dt );   //  2
    void landState( double dt );  //  3
    
  public:
    StateManager( QuadController *qc, int initialState );
    void setState( int s );
    int getState();
    void update( double dt );
    
};


#endif

