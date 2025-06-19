#ifndef COMMANDS_H_
#define COMMANDS_H_

/*************************************************
 * Includes
 ************************************************/

#include "utils.hpp"

/*************************************************
 * Global function declarations
 ************************************************/

void CMD_v_closeGripper(PPLN_Parameters const * const p_parameters);

void CMD_v_dropBrick(PPLN_Parameters const * const p_parameters);

void CMD_v_goOverDropPoint(PPLN_Parameters const * const p_parameters);

void CMD_v_goToReceive(PPLN_Parameters const * const p_parameters);

void CMD_v_openGripper(PPLN_Parameters const * const p_parameters);

#endif // COMMANDS_H_
