
#ifndef DRONECAN_H_
#define DRONECAN_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdio.h>
#include "cli.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/
#define DRONECAN_NODE_ID       98
 
/*******************************************************************************
 * Types
 ******************************************************************************/

/*******************************************************************************
 * public functions
 ******************************************************************************/
int dronecan_initialize(void);
int dronecan_sendBMSStatus(void);

/*******************************************************************************
 * EOF
 ******************************************************************************/

#endif /* DRONECAN_H_ */
