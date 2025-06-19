/*************************************************
 * Includes
 ************************************************/

#include "pipeline.h"

/*************************************************
 * Defines
 ************************************************/

#define BRICKS_PER_LAYER (2)

/*************************************************
 * Local function declarations
 ************************************************/

void v_generateBrickStates(State          * const a_brickStates,
                           Position const * const p_baseCenter);

void v_loadBaseCenter(Position * const p_baseCenter);

/*************************************************
 * Global function definitions
 ************************************************/

void PPLN_v_initialize(PPLN_Parameters * const p_parameters)
{
  v_loadBaseCenter(&p_parameters->baseCenter);

  v_generateBrickStates(&p_parameters->a_brickStates[0],
                        &p_parameters->baseCenter);
}

void PPLN_v_runCommand(PPLN_Parameters       * const p_parameters,
                       PPLN_Command    const         e_command)
{
  switch (e_command)
  {
    case (PPLN_Command::CloseGripper):
    {
      CMD_v_closeGripper(p_parameters);

      break;
    }

    case (PPLN_Command::DropBrick):
    {
      CMD_v_dropBrick(p_parameters);

      break;
    }

    case (PPLN_Command::GoOverDropPoint):
    {
      CMD_v_goOverDropPoint(p_parameters);

      break;
    }

    case (PPLN_Command::GoToReceive):
    {
      CMD_v_goToReceive(p_parameters);

      break;
    }

    case (PPLN_Command::OpenGripper):
    {
      CMD_v_openGripper(p_parameters);

      break;
    }

    default:
    {
      break;
    }
  }
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
