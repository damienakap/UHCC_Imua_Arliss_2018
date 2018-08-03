
#include "AutoQuadStates.h"





/***************
 * #############
 * State manager
 * #############
 ***************
 */
StateManager::StateManager( QuadController *qc, int initialState ){
  this->quadController = qc;
  this->stateLast = initialState;
  this->state = initialState;
  this->stateInit = false;
  
}
/***********
 * Set state
 ***********
 */
void StateManager::setState( int s ){
  this->stateLast = this->state;
  this->state = s;
  this->stateInit = false;
}
/***********
 * Get State
 ***********
 */
int StateManager::getState(){ return this->state; }
/**************
 * Update state
 **************
 */
void StateManager::update( double dt ){

  switch( this->state ){
    case 1:
      this->hoverState( dt );
      break;
    case 2:
      this->navState( dt );
      break;
    case 3:
      this->landState( dt );
      break;
    default:
      this->idleState( dt );
  }
  
}

/********
 * ######
 * States
 * ######
 ********
 */

/**************
 * Idle State 0
 **************
 */
void StateManager::idleState( double dt ){
  if( !(this->stateInit) ){
    (*this->quadController).thrust = 0;
    (*this->quadController).pidOn = false;
    this->stateInit = false;
  }

  (*this->quadController).calculate( dt );
  
}
/***************
 * Hover State 1
 ***************
 */
void StateManager::hoverState( double dt ){
  if( !(this->stateInit) ){
    (*this->quadController).thrust = (*this->quadController).hoverThrust;
    (*this->quadController).pidOn = true;
    this->stateInit = false;
  }

  (*this->quadController).calculate( dt );
  
}
/********************
 * Navigation State 2
 ********************
 */
void StateManager::navState( double dt ){
  if( !(this->stateInit) ){
    (*this->quadController).thrust = (*this->quadController).hoverThrust;
    (*this->quadController).pidOn = true;
    this->stateInit = false;
  }

  (*this->quadController).calculate( dt );
  
}
/**************
 * Land State 3
 **************
 */
void StateManager::landState( double dt ){

  if( !(this->stateInit) ){
    (*this->quadController).thrust = (*this->quadController).hoverThrust * 0.75;
    (*this->quadController).pidOn = true;
    this->stateInit = false;
  }

  (*this->quadController).calculate( dt );
  
}





