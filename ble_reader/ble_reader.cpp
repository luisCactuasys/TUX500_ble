/**
 * @file ble_reader.c
 * @author Luís Conceição (luis.conceicao@actuasys.com)
 * @brief This file is the main top level file for the BLE reader application deamon 
 * @version 0.1
 * @date 2024-04-22
 * 
 * @copyright Copyright (c) 2023
 * 
 */
/*
 *   Copyright (c) 2022 Martijn van Welie
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:
 *
 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.
 *
 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 *
 */

/*
 * *********************** Includes *******************************************
 */
#include <glib.h>
#include <stdio.h>
#include <signal.h>

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <ctype.h>

#include <stdlib.h>
#include <stdarg.h>
#include <signal.h>
#include <stdlib.h>
#include <sys/msg.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "adapter.h"
#include "device.h"
#include "logger.h"
#include "agent.h"
#include "application.h"
#include "advertisement.h"
#include "utility.h"
#include "parser.h"
#include <syslog.h>

#include <sys/ioctl.h>
#include "tools.h"
#include "json.h"

/*
 * ********************** Defines **********************************************
 */
#define TAG "Main"
#define CUD_CHAR "00002901-0000-1000-8000-00805f9b34fb"

// "BLE Reader" Primary Service
#define M3_TUX_SERVICE_UUID "0000fff0-0000-1000-8000-00805f9b34fb"
// Characteristic 1. Used to receive thus as Write property 
#define M3_TUX_CHAR_1_UUID "0000fff1-0000-1000-8000-00805f9b34fb"
// Characteristic 2. Used to transmit thus as Notify property
#define M3_TUX_CHAR_2_UUID "0000fff2-0000-1000-8000-00805f9b34fb"

/*
 ************************* Sctructs & TypeDefs ********************************  
 */


struct blemsgbuf {
  long    mtype;       	/* Message type */
  char    mtext[4000];  /* Client information structure */
};

/*
 * *********************** Private Variables **********************************
 */
GMainLoop *loop = NULL;
Adapter *default_adapter = NULL;
GPtrArray *adv_service_uuids = NULL;
Advertisement *advertisement = NULL;
Application *app = NULL;

// ------- M3 --------
int     g_qid;
volatile sig_atomic_t g_exit = 0;
int daemonized = 0;

static void* g_MsgQHandle = NULL;


/*
 * ********************** Private Function Prototypes **************************
 */
// --------- M3 ----------
void term(int signum);
static void daemonize();
void printf_d(const char* fmt, ...);
int32_t m3_bleReaderInit();

void* MsgQRxThread(void* pContext);
int RpcNoHandler(JsonObject& request);
int RpcSendCardInfo(JsonObject& request);


// -------------- Original ------------
void on_powered_state_changed(Adapter *adapter, gboolean state);
void on_central_state_changed(Adapter *adapter, Device *device);
const char *on_local_char_read( const Application *application, const char *address,
                                const char *service_uuid, const char *char_uuid);
const char *on_local_char_write(const Application *application, const char *address, 
        const char *service_uuid, const char *char_uuid, GByteArray *byteArray);
void on_local_char_start_notify(const Application *application,
                                const char *service_uuid, const char *char_uuid);
void on_local_char_stop_notify(const Application *application, 
                                const char *service_uuid, const char *char_uuid);
gboolean callback(gpointer data);
static void cleanup_handler(int signo);



/*
 * ********************** Function definitions *********************************
 */

/**
 * @brief   Top level functio. Here the the low level DBus and BlueZ stuff is 
 *  handled and initialized.
 *          Services and Characteristics are added.
 *          Aplication is started.
 * @return int 
 */
int main(void) {
    
    GDBusConnection *dbusConnection;
    eResult tool_res = FRAMEWORK_SUCCESS;

    //daemonize();

    // Setup handler for CTRL+C
    if (signal(SIGINT, cleanup_handler) == SIG_ERR)
    {    
        //log_error(TAG, "can't catch SIGINT");     
        /**
         * TODO: redirect the logger using "logger" API. Instead of outputing
         *   to stdout one could use the syslog when in daemon
         * 
         */
    }

    tool_res = framework_CreateThread(&g_MsgQHandle, MsgQRxThread, NULL);
    if(FRAMEWORK_SUCCESS != tool_res)
    {
        printf_d("Failed to launch MsgQ thread");
        return -1;
    }

	  printf_d("##  GPIO/WDOG Daemon  ##\n");

		//framework_CreateMutex(&mutexReversal);



    // Get a DBus connection
    dbusConnection = g_bus_get_sync(G_BUS_TYPE_SYSTEM, NULL, NULL);
    // Setup mainloop
    loop = g_main_loop_new(NULL, FALSE);
    // Get the default default_adapter
    default_adapter = binc_adapter_get_default(dbusConnection);

    if (default_adapter != NULL) 
    {
        // log_debug(TAG, "using default_adapter '%s'", 
        //                 binc_adapter_get_path(default_adapter));

        // Make sure the adapter is on
        binc_adapter_set_powered_state_cb(   default_adapter, 
                                            &on_powered_state_changed);
        if (!binc_adapter_get_powered_state(default_adapter)) 
        {
            binc_adapter_power_on(default_adapter);
        }

        //Start Services and Characteristics
        m3_bleReaderInit();
    } 
    else 
    {
        //log_debug("MAIN", "No default_adapter found");
    }

    // Bail out after some time
    g_timeout_add_seconds(600, callback, loop);

    // Start the mainloop
    g_main_loop_run(loop);

    // Clean up mainloop
    g_main_loop_unref(loop);

    // Disconnect from DBus
    g_dbus_connection_close_sync(dbusConnection, NULL, NULL);
    g_object_unref(dbusConnection);
    return 0;
}


/*
 * --------------- M3 definitions ------------
*/

void printf_d(const char* fmt, ...)
{
	va_list args;
	char buf[1000];
	va_start(args, fmt);
	vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);

	if (daemonized)
		syslog(LOG_NOTICE, "%s", buf);
	else
	{
		printf("%s", buf);
		fflush(stdout);
	}
}

/**
 * @brief Flags the rest of the lines of execution to terminate
 * 
 * @param signum 
 */
void term(int signum)
{
	g_exit = 1;
    
    /**
     * TODO:
     *      - close and release any reasources
     * 
     */
}

/**
 * @brief This function puts the process running in the background.
 *      - It closes the IO streams. Any log should be made with log files  
 */
static void daemonize()
{
    pid_t pid;

    /* Fork off the parent process */
    pid = fork();
		
    /* An error occurred */
    if (pid < 0)
    {
        printf_d("ERROR %d\n", pid);
        exit(EXIT_FAILURE);
    }

    /* Success: Let the parent terminate */
    if (pid > 0)
        exit(EXIT_SUCCESS);

    /* On success: The child process becomes session leader */
    if (setsid() < 0)
    {
        printf_d("ERROR %d\n", setsid());
        exit(EXIT_FAILURE);
    }

    /* Catch, ignore and handle signals */
		struct sigaction action;
    memset(&action, 0, sizeof(struct sigaction));
    action.sa_handler = term;
		sigaction(SIGTERM, &action, NULL);
		
    /* Fork off for the second time*/
    pid = fork();
		
    /* An error occurred */
    if (pid < 0)
    {
        printf_d("ERROR %d\n", pid);
        exit(EXIT_FAILURE);
    }

    /* Success: Let the parent terminate */
    if (pid > 0)
        exit(EXIT_SUCCESS);

    /* Set new file permissions */
    umask(0);

    /* Change the working directory to the root directory */
    /* or another appropriated directory */
    chdir("/");

    /* Close all open file descriptors */
    int x;
    for (x = sysconf(_SC_OPEN_MAX); x>=0; x--)
    {
        close (x);	// TODO:
    }

    /* Open the log file */
    openlog ("m3bled", LOG_PID, LOG_DAEMON);
		daemonized = 1;
}



/**
 * @brief Initializes
 * 
 * @param app 
 * @return int32_t 
 */
int32_t m3_bleReaderInit()
{
    // Setup remote central connection state callback
    binc_adapter_set_remote_central_cb(default_adapter, &on_central_state_changed);

    // Setup advertisement
    adv_service_uuids = g_ptr_array_new();
    g_ptr_array_add(adv_service_uuids, (gpointer)M3_TUX_SERVICE_UUID);

    advertisement = binc_advertisement_create();
    binc_advertisement_set_local_name(advertisement, "TUX_BLE");
    binc_advertisement_set_services(advertisement, adv_service_uuids);
    g_ptr_array_free(adv_service_uuids, TRUE);
    binc_adapter_start_advertising(default_adapter, advertisement);

    // Start application
    app = binc_create_application(default_adapter);

    // Add Primary Service
    binc_application_add_service(app, M3_TUX_SERVICE_UUID);
    
    // Add characteristic 1 - RX with "Write". Also, add description to char 1 
    binc_application_add_characteristic(
            app,
            M3_TUX_SERVICE_UUID,
            M3_TUX_CHAR_1_UUID,
            GATT_CHR_PROP_WRITE);
    binc_application_add_descriptor(
            app,
            M3_TUX_SERVICE_UUID,
            M3_TUX_CHAR_1_UUID,
            CUD_CHAR,
            GATT_CHR_PROP_READ);

    binc_application_add_characteristic(
            app,
            M3_TUX_SERVICE_UUID,
            M3_TUX_CHAR_2_UUID,
            GATT_CHR_PROP_INDICATE);
    binc_application_add_descriptor(
            app,
            M3_TUX_SERVICE_UUID,
            M3_TUX_CHAR_2_UUID,
            CUD_CHAR,
            GATT_CHR_PROP_READ);

    const guint8 desc_char1[] = "Characteristic 1";
    const guint8 desc_char2[] = "Characteristic 2";
    GByteArray *desc_char1_array = g_byte_array_sized_new(sizeof(desc_char1));
    GByteArray *desc_char2_array = g_byte_array_sized_new(sizeof(desc_char2));
    g_byte_array_append(desc_char1_array, desc_char1, sizeof(desc_char1));
    g_byte_array_append(desc_char2_array, desc_char2, sizeof(desc_char2));
    binc_application_set_desc_value(app, M3_TUX_SERVICE_UUID, 
                                    M3_TUX_CHAR_1_UUID, CUD_CHAR, desc_char1_array);
    binc_application_set_desc_value(app, M3_TUX_SERVICE_UUID, 
                                    M3_TUX_CHAR_2_UUID, CUD_CHAR, desc_char2_array);

    // Set callbacks
    binc_application_set_char_read_cb(app, &on_local_char_read);
    binc_application_set_char_write_cb(app, &on_local_char_write);
    
    binc_application_set_char_start_notify_cb(app, &on_local_char_start_notify);
    binc_application_set_char_stop_notify_cb(app, &on_local_char_stop_notify);
    
    binc_adapter_register_application(default_adapter, app);

    return 0;
}


/**
 * @brief This is the 
 * 
 * @param pContext 
 * @return void* 
 */
void* MsgQRxThread(void* pContext)
{
	/// AMB
        
	if((g_qid = msgget( 31335, IPC_CREAT | 0660 )) == -1)
	{
		printf_d("MsgQ get failed\n");
		//res = 0x11;
	}
	else
		printf_d("MsgQ get OK\n");


  JsonRpcAddHandler((char*)"sendCardInfo", RpcSendCardInfo);
  JsonRpcAddHandler((char*)"*", RpcNoHandler);

	// Clean old messages
	struct blemsgbuf qbuf;
	int cleaned = 0;
	while (!g_exit && msgrcv(g_qid, &qbuf, sizeof(qbuf.mtext), 2, IPC_NOWAIT) >= 0)	cleaned++;
	printf_d("Cleaned %d old messages from queue", cleaned);

  memset(&qbuf, 0, sizeof(qbuf));
  while (!g_exit)
	{
		ssize_t rcv;
		if ((rcv = msgrcv(g_qid, &qbuf, sizeof(qbuf.mtext), 2, IPC_NOWAIT)) < 0)
		{
			// No message or error
			int err = errno;
			if (err != ENOMSG) {
				printf_d("error recv %ld  %d", rcv, err);
				usleep(5000e3);
			}
			usleep(50e3);
			continue;
		}

		// Process received message
		printf_d("Received message: %s", qbuf.mtext);
		JsonRpcProcess(qbuf.mtext);
		memset(&qbuf, 0, sizeof(qbuf));
	}

	/*if (g_qid)
		msgctl(g_qid, IPC_RMID, NULL);*/
	printf_d("MsgQRxThread exited");
  return NULL;
}


/**
 * @brief   Handler for non-existing method 
 * 
 * @param request 
 * @return   0 if all good 
 */
int RpcNoHandler(JsonObject& request)
{
  // No handler registered for invoked method
  std::string id = request["id"].as<char*>();

  if (id.length())
  {
    // Send response
    //MsgQSendError((char*)id.c_str(), -32601, (char*)"Method does not exist");
  }

  return 0;
}


/**
 * @brief   This function id used to send the card information to acess control server
 * 
 * @param request 
 * @return  0 if all good 
 */
int RpcSendCardInfo(JsonObject& request)
{
  // No handler registered for invoked method
  std::string id = request["id"].as<char*>();

  if (id.length())
  {
    // Send response
    //MsgQSendError((char*)id.c_str(), -32601, (char*)"Method does not exist");
  }

  return 0;
}


// -------------- Original ------------

/**
 * @brief 
 * 
 * @param adapter 
 * @param state 
 */
void on_powered_state_changed(Adapter *adapter, gboolean state) 
{
    //log_debug(TAG, "powered '%s' (%s)", state ? "on" : "off", 
    //                                binc_adapter_get_path(adapter));
}

/**
 * @brief 
 * 
 * @param adapter 
 * @param device 
 */
void on_central_state_changed(Adapter *adapter, Device *device)
{
    char *deviceToString = binc_device_to_string(device);
    //log_debug(TAG, deviceToString);
    g_free(deviceToString);

    //log_debug(TAG, "remote central %s is %s", binc_device_get_address(device),
    //                 binc_device_get_connection_state_name(device));
    ConnectionState state = binc_device_get_connection_state(device);
    if (state == BINC_CONNECTED) {
        binc_adapter_stop_advertising(adapter, advertisement);
    } else if (state == BINC_DISCONNECTED){
        binc_adapter_start_advertising(adapter, advertisement);
    }
}


/**
 * @brief 
 * 
 * @param application 
 * @param address 
 * @param service_uuid 
 * @param char_uuid 
 * @return const char* 
 */
const char *on_local_char_read( const Application *application, const char *address,
                                const char *service_uuid, const char *char_uuid) 
{
    //log_debug(TAG, "on_char_read");
    
    if(!g_str_equal(service_uuid, M3_TUX_SERVICE_UUID))
    {
        return BLUEZ_ERROR_REJECTED;
    }

    //does nothing (no charecteristic has read permition)
    
    return BLUEZ_ERROR_NOT_SUPPORTED;
}


/**
 * @brief 
 * 
 * @param application 
 * @param address 
 * @param service_uuid 
 * @param char_uuid 
 * @param byteArray 
 * @return const char* 
 */
const char *on_local_char_write(const Application *application, const char *address, 
        const char *service_uuid, const char *char_uuid, GByteArray *byteArray) 
{    
    if(!g_str_equal(service_uuid, M3_TUX_SERVICE_UUID))
    {
        return BLUEZ_ERROR_REJECTED;
    }

    // Check if it is the correct characteristic (rx with "Write")
    if(!g_str_equal(char_uuid, M3_TUX_CHAR_1_UUID))
    {
        return BLUEZ_ERROR_INVALID_VALUE_LENGTH;
    }
    
    // Store received 
    // TODO:

    return NULL;
}


/**
 * @brief 
 * 
 * @param application 
 * @param service_uuid 
 * @param char_uuid 
 */
void on_local_char_start_notify(const Application *application,
                                const char *service_uuid, const char *char_uuid)
{
    //log_debug(TAG, "on start notify");
    printf_d("%s", "on start notify");

    // Check if it is the correct characteristic (tx with "Notify")
    if(!g_str_equal(service_uuid, M3_TUX_SERVICE_UUID))
    {
        //log_warn(TAG, "start notify: invalid Service");
        return;
    }
    if(!g_str_equal(char_uuid, M3_TUX_CHAR_2_UUID))
    {
        //log_warn(TAG, "start notify: invalid Characteristic");
        return;
    }
    
    //log_info(TAG, "Notifications Enabled...");
    printf_d("%s", "Notifications Enabled...");
}


/**
 * @brief 
 * 
 * @param application 
 * @param service_uuid 
 * @param char_uuid 
 */
void on_local_char_stop_notify(const Application *application, 
                                const char *service_uuid, const char *char_uuid)
{
    //log_debug(TAG, "on stop notify");
    //log_info(TAG, "Notifications Disabled...");
}


/**
 * @brief 
 * 
 * @param data 
 * @return gboolean 
 */
gboolean callback(gpointer data) {
    if (app != NULL) {
        binc_adapter_unregister_application(default_adapter, app);
        binc_application_free(app);
        app = NULL;
    }

    if (advertisement != NULL) {
        binc_adapter_stop_advertising(default_adapter, advertisement);
        binc_advertisement_free(advertisement);
    }

    if (default_adapter != NULL) {
        binc_adapter_free(default_adapter);
        default_adapter = NULL;
    }

    g_main_loop_quit((GMainLoop *) data);
    return FALSE;
}


/**
 * @brief 
 * 
 * @param signo 
 */
static void cleanup_handler(int signo) {
    if (signo == SIGINT) {
        //log_error(TAG, "received SIGINT");
        callback(loop);
    }
}

