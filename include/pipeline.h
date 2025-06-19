#ifndef PIPELINE_H_
#define PIPELINE_H_

/*************************************************
 * Includes
 ************************************************/

#include "commands.hpp"
#include "utils.hpp"

/*************************************************
 * Defines
 ************************************************/

#define PPLN_LAYERS_MAX (100)

/*************************************************
 * Enums
 ************************************************/

enum class PPLN_Command
{
  CloseGripper,
  DropBrick,
  GoOverDropPoint,
  GoToReceive,
  OpenGripper
};

/*************************************************
 * Structures
 ************************************************/

struct class PPLN_Parameters
{
  State                                     a_brickStates [PPLN_LAYERS_MAX] = {};
  Position                                  baseCenter                      = {};
  int                                       i_brickQty                      = 0;
  int                                       i_towerProgress                 = 0;
  std::make_shared<icr_Motionplanning_arms> p_commandNode                   = {};
};

/*************************************************
 * Global function declarations
 ************************************************/

void PPLN_v_initialize(PPLN_Parameters                           * const p_parameters,
                       std::make_shared<icr_Motionplanning_arms>         p_newCommandNode);

void PPLN_v_runCommand(PPLN_Parameters       * const p_parameters,
                       PPLN_Command    const         e_command);

#endif // PIPELINE_H_
