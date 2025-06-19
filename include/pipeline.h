/*************************************************
 * Includes
 ************************************************/

#include "utils.hpp"

/*************************************************
 * Defines
 ************************************************/

#define PPLN_LAYERS_MAX (100)

/*************************************************
 * Structures
 ************************************************/

struct class PPLN_Parameters
{
  State    a_brickStates [PPLN_LAYERS_MAX] = {};
  Position baseCenter                      = {};
};

/*************************************************
 * Global function declarations
 ************************************************/

void PPLN_v_init(PPLN_Parameters * const p_parameters);
