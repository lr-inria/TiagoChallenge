/*************************************************
 * Includes
 ************************************************/

#include "pipeline.h"

/*************************************************
 * Defines
 ************************************************/

#define BRICK_HEIGHT     (1.5f)
#define BRICK_LENGTH     (7.5f)
#define BRICK_WIDTH      (2.5f)
#define BRICKS_PER_LAYER (2)

/*************************************************
 * Local function declarations
 ************************************************/

void v_generateBrickStates(State          * const a_brickStates,
                           int            * const p_brickQty,
                           Position const * const p_baseCenter);

void v_loadBaseCenter(Position * const p_baseCenter);

/*************************************************
 * Global function definitions
 ************************************************/

void PPLN_v_initialize(PPLN_Parameters * const p_parameters)
{
  v_loadBaseCenter(&p_parameters->baseCenter);

  v_generateBrickStates(&p_parameters->a_brickStates[0],
                        &p_parameters->i_brickQty,
                        &p_parameters->baseCenter);

  p_parameters->i_towerProgress = 0;
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
                           int            * const p_brickQty,
                           Position const * const p_baseCenter)
{
  float const f_delta = 0.5 * (BRICK_LENGTH - BRICK_WIDTH);
  float const f_xBase = p_baseCenter->f_x;
  float const f_yBase = p_baseCenter->f_y;
  float const f_zBase = p_baseCenter->f_z;

  int i_brickIdx      = 0;
  int i_layerBrickIdx = 0;
  int i_layerIdx      = 0;

  while (i_layerIdx < PPLN_LAYERS_MAX)
  {
    // Left brick
    a_brickStates[i_brickIdx].position.f_x = f_xBase - f_delta;
    a_brickStates[i_brickIdx].position.f_y = f_yBase + 0.0f;
    a_brickStates[i_brickIdx].position.f_z = f_zBase + ((i_layer + 0.5f) * BRICK_HEIGHT);

    a_brickStates[i_brickIdx].quaternion.f_x = 0.0f;
    a_brickStates[i_brickIdx].quaternion.f_y = 0.0f;
    a_brickStates[i_brickIdx].quaternion.f_z = PI_4;
    a_brickStates[i_brickIdx].quaternion.f_w = 1.0f;

    i_brickIdx++;

    // Right brick
    a_brickStates[i_brickIdx].position.f_x = f_xBase + f_delta;
    a_brickStates[i_brickIdx].position.f_y = f_yBase + 0.0f;
    a_brickStates[i_brickIdx].position.f_z = f_zBase + ((i_layer + 0.5f) * BRICK_HEIGHT);

    a_brickStates[i_brickIdx].quaternion.f_x = 0.0f;
    a_brickStates[i_brickIdx].quaternion.f_y = 0.0f;
    a_brickStates[i_brickIdx].quaternion.f_z = PI_4;
    a_brickStates[i_brickIdx].quaternion.f_w = 1.0f;

    i_brickIdx++;
    i_layerIdx++;

    // Top brick
    a_brickStates[i_brickIdx].position.f_x = f_xBase + 0.0f;
    a_brickStates[i_brickIdx].position.f_y = f_yBase + f_delta;
    a_brickStates[i_brickIdx].position.f_z = f_zBase + ((i_layer + 0.5f) * BRICK_HEIGHT);

    a_brickStates[i_brickIdx].quaternion.f_x = 0.0f;
    a_brickStates[i_brickIdx].quaternion.f_y = 0.0f;
    a_brickStates[i_brickIdx].quaternion.f_z = 0.0f;
    a_brickStates[i_brickIdx].quaternion.f_w = 1.0f;

    i_brickIdx++;

    // Bottom brick
    a_brickStates[i_brickIdx].position.f_x = f_xBase + 0.0f;
    a_brickStates[i_brickIdx].position.f_y = f_yBase - f_delta;
    a_brickStates[i_brickIdx].position.f_z = f_zBase + ((i_layer + 0.5f) * BRICK_HEIGHT);

    a_brickStates[i_brickIdx].quaternion.f_x = 0.0f;
    a_brickStates[i_brickIdx].quaternion.f_y = 0.0f;
    a_brickStates[i_brickIdx].quaternion.f_z = 0.0f;
    a_brickStates[i_brickIdx].quaternion.f_w = 1.0f;

    i_brickIdx++;
    i_layerIdx++;
  }

  (*p_brickQty) = i_brickIdx;
}

void v_loadBaseCenter(Position * const p_baseCenter)
{
  p_baseCenter->f_x = 0.0f;
  p_baseCenter->f_y = 0.0f;
  p_baseCenter->f_z = 0.0f;
}
