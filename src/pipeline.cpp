/*************************************************
 * Defines
 ************************************************/

#include "pipeline.h"

#define BRICKS_PER_LAYER (2)

/*************************************************
 * Local function declarations
 ************************************************/

void v_generateBrickStates(State         * const a_brickStates,
                           Position const * const p_baseCenter);

void v_loadBaseCenter(Position * const p_baseCenter);

/*************************************************
 * Global function definitions
 ************************************************/

void PPLN_v_init(PPLN_Parameters * const p_parameters)
{
  v_loadBaseCenter(&p_parameters->baseCenter);

  v_generateBrickStates(&p_parameters->a_brickStates[0],
    &p_parameters->baseCenter);
}

/*************************************************
 * Local function definitions
 ************************************************/
 
void v_generateBrickStates(State          * const a_brickStates,
                           Position const * const p_baseCenter)
{
  int i_brickIdx      = 0;
  int i_layerBrickIdx = 0;
  int i_layerIdx      = 0;

  for (i_layerIdx = 0; i_layerIdx < PPLN_LAYERS_MAX; i_layerIdx++)
  {
    for (i_layerBrickIdx = 0; i_layerBrickIdx < BRICKS_PER_LAYER; i_layerBrickIdx++)
    {
      a_brickStates[i_brickIdx].position.f_x = 0.0f;
      a_brickStates[i_brickIdx].position.f_y = 0.0f;
      a_brickStates[i_brickIdx].position.f_z = 0.0f;

      a_brickStates[i_brickIdx].quaternion.f_x = 0.0f;
      a_brickStates[i_brickIdx].quaternion.f_y = 0.0f;
      a_brickStates[i_brickIdx].quaternion.f_z = 0.0f;
      a_brickStates[i_brickIdx].quaternion.f_w = 0.0f;

      i_brickIdx++;
    }
  }
}

void v_loadBaseCenter(Position * const p_baseCenter)
{
  p_baseCenter->f_x = 0.0f;
  p_baseCenter->f_y = 0.0f;
  p_baseCenter->f_z = 0.0f;
}
