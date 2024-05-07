#include <canard.h>

#include <sched.h>

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include <errno.h>

#include <net/if.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <nuttx/random.h>
#include <pthread.h>


#include <poll.h>

#include <nuttx/can.h>
#include <netpacket/can.h>

#include <nuttx/semaphore.h>

#include "dronecan.h"

#include "cli.h"

#include "dronecan/drivers/socketcan.h"

#include "dronecan/generated/dronecan_msgs.h"

/****************************************************************************
 * Defines
 ****************************************************************************/
#ifndef SEM_OCCUPIED_ERROR
#define SEM_OCCUPIED_ERROR 255
#endif

#define ONE_SEC_IN_NS                           1000000000
#define MS_TO_NS_MULT                           1000000

#define APP_NODE_NAME                           "org.nxp.bms" //"org.uavcan.libcanardv1.nuttx.demo" //CONFIG_EXAMPLES_LIBCANARDV1_APP_NODE_NAME

#define UNIQUE_ID_LENGTH_BYTES                   16

#define LIBDRONECAN_DAEMON_PRIORITY             110
#define LIBDRONECAN_DAEMON_STACK_SIZE           2500
#define CAN_DEVICE                              "can0"

/****************************************************************************
 * private data
 ****************************************************************************/ 
pthread_t gCanTID;

static sem_t gUavcanSem;

static bool gUavcanInitialized = false; 

static bool g_canard_daemon_started;

struct pollfd gFd;


static CanardInstance canard;

/* Arena for memory allocation, used by the library */

static uint8_t canard_memory_pool[1024];

static uint8_t unique_id[16] =
{ 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x01
};

/* Node status variables */

static uint8_t node_health = 0;
static uint8_t node_mode = 0;


static int DronecanTask(int argc, char *argv[]);

uint64_t getMonotonicTimestampUSec(void)
{
  struct timespec ts;

  memset(&ts, 0, sizeof(ts));
  if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0)
    {
      abort();
    }

  return ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL;
}

int dronecan_initialize(void)
{
    int ret = EXIT_SUCCESS;

    if(!gUavcanInitialized)
    {
        // initialize semaphore
        ret = sem_init(&gUavcanSem, 0, 0);
        sem_setprotocol(&gUavcanSem, SEM_PRIO_NONE);

        // check for errors
        if(ret)
        {
            // output to user
            cli_printfError("uavcan_initialize ERROR: Couldn't initialize semaphore!\n");

            // return to the user 
            return ret;
        }

        if (g_canard_daemon_started) {
            cli_printf("canard_main: receive and send task already running\n");
            return EXIT_SUCCESS;
        }

        ret = task_create("UAVCAN", LIBDRONECAN_DAEMON_PRIORITY,
            LIBDRONECAN_DAEMON_STACK_SIZE, DronecanTask, NULL);

        gCanTID = ret;

        if (ret < 0) {
            int errcode = errno;
            cli_printfError("uavcan_initialize ERROR: Failed to start UAVCAN: %d\n",
                errcode);
            return EXIT_FAILURE;
        }
        else
        {
            ret = 0;
        }

        // remember it is initialized
        gUavcanInitialized = true;
    }

    // return 
    return ret;
}

int dronecan_sendBMSStatus(void)
{
    int ret = 0;

    return ret;
}

static void onTransferReceived(CanardInstance *ins,
                               CanardRxTransfer *transfer)
{

}

static bool shouldAcceptTransfer(const CanardInstance * ins,
                                 uint64_t * out_data_type_signature,
                                 uint16_t data_type_id,
                                 CanardTransferType transfer_type,
                                 uint8_t source_node_id)
{
  if (canardGetLocalNodeID(ins) == CANARD_BROADCAST_NODE_ID)
    {
      /* If we're in the process of allocation of dynamic node ID, accept
       * only relevant transfers.
       */
    }
  else
    {
      if ((transfer_type == CanardTransferTypeRequest) &&
          (data_type_id == UAVCAN_PROTOCOL_GETNODEINFO_ID))
        {
          *out_data_type_signature =
           UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE;
          return true;
        }
    }

  return false;
}

void processTxRxOnce(SocketCANInstance *socketcan, int timeout_msec)
{
  const CanardCANFrame *txf;

  /* Transmitting */

  for (txf = NULL; (txf = canardPeekTxQueue(&canard)) != NULL; )
    {
      const int tx_res = socketcanTransmit(socketcan, txf, 0);
      if (tx_res < 0 && tx_res != -EAGAIN)           /* Failure - drop the frame and report */
        {
          canardPopTxQueue(&canard);
          fprintf(stderr,
                  "Transmit error %d, frame dropped, errno '%s'\n",
                  tx_res, strerror(errno));
        }
      else if (tx_res > 0)      /* Success - just drop the frame */
        {
          canardPopTxQueue(&canard);
        }
      else                      /* Timeout - just exit and try again later */
        {
          break;
        }
    }

  /* Receiving */

  CanardCANFrame rx_frame;
  const uint64_t timestamp = getMonotonicTimestampUSec();

  const int rx_res = socketcanReceive(socketcan, &rx_frame, timeout_msec);

  if (rx_res < 0)               /* Failure - report */
    {
      fprintf(stderr, "Receive error %d, errno '%s'\n", rx_res,
              strerror(errno));
    }
  else if (rx_res > 0)          /* Success - process the frame */
    {
      canardHandleRxFrame(&canard, &rx_frame, timestamp);
    }
  else
    {
      ;                         /* Timeout - nothing to do */
    }
}

void process1HzTasks(uint64_t timestamp_usec)
{
}

static int DronecanTask(int argc, char *argv[])
{
    static SocketCANInstance socketcan;
    int errval = 0;

    const char * const can_iface_name = "can0";
    int16_t res = socketcanInit(&socketcan, can_iface_name);

    if (res < 0)
    {
        fprintf(stderr, "Failed to open CAN iface '%s'\n", can_iface_name);
        return 1;
    }

    canardInit(&canard, canard_memory_pool, sizeof(canard_memory_pool),
        onTransferReceived, shouldAcceptTransfer, (void *)(12345));
    canardSetLocalNodeID(&canard, DRONECAN_NODE_ID);
    printf("canard_daemon: canard initialized\n");
    printf("start node (ID: %d Name: %s)\n",
            DRONECAN_NODE_ID,
            APP_NODE_NAME);

    g_canard_daemon_started = true;

    uint64_t next_1hz_service_at = getMonotonicTimestampUSec();

    for (; ; )
    {
        processTxRxOnce(&socketcan, 10);
        
        const uint64_t ts = getMonotonicTimestampUSec();

        if (ts >= next_1hz_service_at)
        {
          next_1hz_service_at += 1000000;
          process1HzTasks(ts);
        }
    }

    // terminate the deamon
    g_canard_daemon_started = false;
    cli_printfWarning("canard_daemon: Terminating!\n");
    fflush(stdout);
    return errval;
}