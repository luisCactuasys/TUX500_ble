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
#include <sys/msg.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "aes.h"
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
#include "M3Crc32.h"



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

#define MAX_PDU_SIZE 64

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
uint8_t m3_bleKey[16];

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

//thread
void* MsgQRxThread(void* pContext);
//message IO
int MsgQSendError(char* requestId, long errorCode, char* errorMessage);
int MsgQSendResult(char* requestId, JsonObject& result);
int MsgQInvoke(char* requestId, char* method, JsonObject& params);

// requested fuctionalities
int RpcNoHandler(JsonObject& request);
int RpcSetAutoRead(JsonObject& request);
int RpcGetVersion(JsonObject& request);
int RpcSetKey(JsonObject& request);
int RpcAcknowledge(JsonObject& request);

// notes and requests
int JSNotifyDetect();
int JSNotifyDeparture();
int JSNotifyCardInfo(uint32_t appId, const char* cardInfo);

// utility
int hexStr2Arr(const uint8_t* in, uint8_t* out, size_t len);
void btox(uint8_t *xp, const uint8_t *bb, int n);
int8_t m3_comFrameContruct(uint8_t* dest, uint16_t *totalLen, uint32_t appId, 
                        uint8_t type, const uint8_t* info, uint16_t infoLen);
static uint8_t m3_decryptFrame(uint8_t *src, uint16_t *len);
static uint8_t m3_encryptFrame(uint8_t *src, uint16_t *len);



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
    //default Key
    std::string bleKeyStr = "11111111111111111111111111111111";
    hexStr2Arr((const uint8_t*)bleKeyStr.c_str(), m3_bleKey, 16);

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
    
    callback(loop);
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
    openlog ("m3btd", LOG_PID, LOG_DAEMON);
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
    //binc_adapter_start_advertising(default_adapter, advertisement);

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
        
    if((g_qid = msgget( 31337, IPC_CREAT | 0660 )) == -1)
    {
        printf_d("MsgQ get failed\n");
        //res = 0x11;
    }
    else
        printf_d("MsgQ get OK\n");

    JsonRpcAddHandler((char*)"setAutoRead", RpcSetAutoRead);
    JsonRpcAddHandler((char*)"getVersion", RpcGetVersion);
    JsonRpcAddHandler((char*)"setKey", RpcSetKey);
    JsonRpcAddHandler((char*)"acknowledge", RpcAcknowledge);
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
 * @brief Send an error response
 * 
 * @param requestId 
 * @param errorCode 
 * @param errorMessage 
 * @return int 
 */
int MsgQSendError(char* requestId, long errorCode, char* errorMessage)
{
	int res = 0;

	// Make sure the request is valid
	if (requestId && requestId[0])
	{
		// Send response
		struct blemsgbuf qbuf;
		int length = 0;

		qbuf.mtype = 1;
		JsonRpcError(requestId, errorCode, errorMessage, qbuf.mtext, sizeof(qbuf.mtext));
		printf_d("[MsgQSendError] Responding error: %s", qbuf.mtext);
		
		length = sizeof(struct blemsgbuf) - sizeof(long);
		if((res = msgsnd( g_qid, &qbuf, length, IPC_NOWAIT )) == -1)
		{
			printf_d("Failed to msgsnd: %d\n", res);
		}
		else
		{
			printf_d("msgsnd OK %d\n", res);
		}
	}

	return res;
}

/**
 * @brief Send a result response
 * 
 * @param requestId 
 * @param result 
 * @return int 
 */
int MsgQSendResult(char* requestId, JsonObject& result)
{
  	int res = 0;
	struct blemsgbuf qbuf;
	int length = 0;

	// Make sure the request is valid
	if (requestId && requestId[0])
	{
		// Send response
		qbuf.mtype = 1;
		JsonRpcResult(requestId, result, qbuf.mtext, sizeof(qbuf.mtext));
		printf_d("[MsgQSendResult] Responding result: %s", qbuf.mtext);
		
		length = sizeof(struct blemsgbuf) - sizeof(long);
		if((res = msgsnd( g_qid, &qbuf, length, IPC_NOWAIT )) == -1)
		{
			printf_d("Failed to msgsnd: %d\n", res);
		}
		else
		{
			printf_d("msgsnd OK %d\n", res);
		}
	}

  return res;
}

/**
 * @brief Invoke a method
 * 
 * @param requestId 
 * @param method 
 * @param params 
 * @return	returns the result of "msgsnd" function.
 */
int MsgQInvoke(char* requestId, char* method, JsonObject& params)
{
	int res = 0;
	struct blemsgbuf qbuf;
	int length = 0;

	// Send invocation	
	qbuf.mtype = 1;
	JsonRpcInvoke(requestId, method, params, qbuf.mtext, sizeof(qbuf.mtext));
	printf_d("[MsgQInvoke] Invoking: %s", qbuf.mtext);
	
	length = sizeof(struct blemsgbuf) - sizeof(long);
	if((res = msgsnd( g_qid, &qbuf, length, IPC_NOWAIT )) == -1)
	{
		printf_d("Failed to msgsnd: %d\n", res);
	}
	else
	{
		printf_d("msgsnd OK %d\n", res);
	}

	return res;
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
    MsgQSendError((char*)id.c_str(), -32601, (char*)"Method does not exist");
  }

  return 0;
}

/**
 * @brief 	Returns a version string
 * 
 * @param request 
 * @return 	0 if all good
 */
int RpcGetVersion(JsonObject& request)
{
  	const char secioVersion[16] = "v0.1.2";
	std::string id = request["id"].as<char*>();
    JsonObject& params = request["params"];

    std::string boas = params["boas"].as<char*>(); 

    printf_d(" IN [RpcGetVersion], boas = %s", boas.c_str());
    
	if (id.length())
	{
		// Send response
		StaticJsonBuffer<200> jsonResultBuffer;
		JsonObject& jsonResult = jsonResultBuffer.createObject();
		jsonResult["firmwareVersion"] = secioVersion;

		MsgQSendResult((char*)id.c_str(), jsonResult);
	}

	return 0;
}


/**
 * @brief   Starts or 
 * 
 * @param request 
 * @return int 
 */
int RpcSetAutoRead(JsonObject& request)
{
	int status = 0;
    static int m3_bleEnabled = 0;
    static int m3_bleReaderEnabled = 0;

    std::string bleKeyStr;
    std::string id = request["id"].as<char*>();
    JsonObject& params = request["params"];

    if (!params.containsKey("enabled"))
    {
        // Send error response
        MsgQSendError((char*)id.c_str(), -32602, (char*)"Missing 'action' parameter");
        return 0;
    }
    if (!params.containsKey("readerEnabled"))
    {
        // Send error response
        MsgQSendError((char*)id.c_str(), -32602, (char*)"Missing 'action' parameter");
        return 0;
    }

    // else, get the Key and store it
    m3_bleEnabled = params["enabled"];
    m3_bleReaderEnabled = params["readerEnabled"];

    printf_d("\n[SetAutoRead] Enabled = %d", m3_bleEnabled);
    printf_d("\n[SetAutoRead] ReaderEnabled = %d", m3_bleReaderEnabled);

    if(m3_bleEnabled || m3_bleReaderEnabled)
    {
        binc_adapter_start_advertising(default_adapter, advertisement);
    }
    else
    {
        binc_adapter_stop_advertising(default_adapter, advertisement);
    }

    if (id.length())
    {
        // Send response
        StaticJsonBuffer<40> jsonResultBuffer;
        JsonObject& jsonResult = jsonResultBuffer.createObject();

        MsgQSendResult((char*)id.c_str(), jsonResult);
    }

	return 0;
}


/**
 * @brief   Gets frame encryption Key  
 * 
 * @param request 
 * @return  0 if all good  
 */
int RpcSetKey(JsonObject& request)
{
	int status = 0;
    std::string bleKeyStr;
    std::string id = request["id"].as<char*>();
    JsonObject& params = request["params"];

    if (!params.containsKey("key"))
    {
        // Send error response
        MsgQSendError((char*)id.c_str(), -32602, (char*)"Missing 'action' parameter");
        return 0;
    }
    
    // else, get the Key and store it
    bleKeyStr = params["key"].as<char*>();

    hexStr2Arr((const uint8_t*)bleKeyStr.c_str(), m3_bleKey, 16);

    printf_d("\n[SetKey] keyStr = %s", bleKeyStr.c_str());
    printf_d("\n[SetKey] keyArray = ");

    for(int i = 0; i < 16; i++)
    {
        printf_d("%.2X", m3_bleKey[i]);
    }

    if (id.length())
    {
        // Send response
        StaticJsonBuffer<40> jsonResultBuffer;
        JsonObject& jsonResult = jsonResultBuffer.createObject();

        MsgQSendResult((char*)id.c_str(), jsonResult);
    }

	return 0;
}


/**
 * @brief   Called when the high level respondes 
 * 
 * @param request 
 * @return  0 if 
 */
int RpcAcknowledge(JsonObject& request)
{
	int ret = 0;
    uint32_t appId = 0;
    std::string status;
    std::string id = request["id"].as<char*>();
    JsonObject& params = request["params"];

    if (!params.containsKey("appId"))
    {
        // Send error response
        MsgQSendError((char*)id.c_str(), -32602, (char*)"Missing 'action' parameter");
        return 0;
    }
    if (!params.containsKey("status"))
    {
        // Send error response
        MsgQSendError((char*)id.c_str(), -32602, (char*)"Missing 'action' parameter");
        return 0;
    }
    
    // else, get the Key and store it
    appId = params["appId"];
    status = params["status"].as<char*>();
    printf_d("\n[Acknowledge] appId = %X", appId);
    printf_d("\n[Acknowledge] status = %s", status.c_str());


    uint8_t dest[64];
    uint16_t totalLen;
    //uint32_t appId; 
    uint8_t type = 0x06;    //ACK 
    const uint8_t* info = NULL;
    uint16_t infoLen = 0;

    ret = m3_comFrameContruct(dest, &totalLen, appId, type, info, infoLen);
 
    ret = m3_encryptFrame(dest, &totalLen);


    GByteArray *byteArray = g_byte_array_sized_new(totalLen);
    g_byte_array_append(byteArray, dest, totalLen);

    ret = binc_application_notify(app, (const char*)M3_TUX_SERVICE_UUID, (const char *)M3_TUX_CHAR_2_UUID, byteArray);
    g_byte_array_free(byteArray, TRUE);

	return ret;
}


/**
 * @brief 
 * 
 * @return int 
 */
int JSNotifyDetect()
{
  StaticJsonBuffer<200> jsonNoteBuffer;
  JsonObject& jsonNote = jsonNoteBuffer.createObject();
  
  return MsgQInvoke(NULL, (char*)"detect", jsonNote);
}


/**
 * @brief 
 * 
 * @return int 
 */
int JSNotifyDeparture()
{
  StaticJsonBuffer<200> jsonNoteBuffer;
  JsonObject& jsonNote = jsonNoteBuffer.createObject();
  
  return MsgQInvoke(NULL, (char*)"Departure", jsonNote);
}


/**
 * @brief 
 * 
 * @param appId 
 * @param cardInfo 
 * @return int 
 */
int JSNotifyCardInfo(uint32_t appId, const char* cardInfo)
{
  StaticJsonBuffer<200> jsonNoteBuffer;
  JsonObject& jsonNote = jsonNoteBuffer.createObject();
  jsonNote["appId"] = appId;
  jsonNote["cardInfo"] = (char*)cardInfo;
  
  return MsgQInvoke(NULL, (char*)"read", jsonNote);
}



// -------------- M3 utulity ----------
/*
https://stackoverflow.com/questions/3408706/hexadecimal-string-to-byte-array-in-c
User: Ali80

/// in: valid chars are 0-9 + A-F + a-f
/// out_len_max==0: convert until the end of input string, out_len_max>0 only convert this many numbers
/// returns actual out size
*/
int hexStr2Arr(const uint8_t* in, uint8_t* out, size_t len)
{
    for (int i = 0; i < len; i++) {
        uint8_t ch0 = in[2 * i];
        uint8_t ch1 = in[2 * i + 1];
        uint8_t nib0 = (ch0 & 0xF) + (ch0 >> 6) | ((ch0 >> 3) & 0x8);
        uint8_t nib1 = (ch1 & 0xF) + (ch1 >> 6) | ((ch1 >> 3) & 0x8);
        out[i] = (nib0 << 4) | nib1;
    }
    return len;
}



void btox(uint8_t *xp, const uint8_t *bb, int n)
{
    const char xx[]= "0123456789ABCDEF";
    while (--n >= 0) xp[n] = xx[(bb[n>>1] >> ((1 - (n&1)) << 2)) & 0xF];
}


/**
 * @brief   This function construct a frame command acordin gto established 
 *      protocol
 * 
 * @param dest      - beffuer that the frame is to be written to. It must be of
 *              a suficient size
 * @param totalLen  - returned size of the total frame 
 * @param appId     - identification of the device that the frame is directed to
 * @param type      - type of frame command
 * @param info      - string to be placed on the "info" field
 * @param infoLen   - size of the info string
 * @return      SUCESS if all good 
 */
int8_t m3_comFrameContruct(uint8_t* dest, uint16_t *totalLen, uint32_t appId, 
                        uint8_t type, const uint8_t* info, uint16_t infoLen)
{
    uint32_t finalCRC = 0;

    finalCRC = M3Set_crc32_init_value(CRC_INITIALVALUE);

    if(!info)
    {
        infoLen = 0;
    }

            dest[0] = 0x01;
    memcpy(&dest[1], &appId, sizeof(uint32_t));
            dest[5] = type;
            dest[6] = LO_UINT16(infoLen);
            dest[7] = HI_UINT16(infoLen);
    if(info)
    {
        memcpy(&dest[8], info, infoLen);
    }
    dest[8 + infoLen] = 0x04;
    finalCRC = M3Fast_crc32(finalCRC, &dest[1], 7 + infoLen);

    memcpy(&dest[9 + infoLen], &finalCRC, sizeof(uint32_t));

    // frame header and footer size is 8 and 5 respectiblly 
    *totalLen = 8 + infoLen + 5;
    
    return 0;
}

/**
 * @brief This function verifies, prepares and requests encryption of a frame
 * 
 * @param src   - Frame. This is the source array and will be used to return
 *              the encrypted frame. This buffer must have the necessary space for
 *              a possible padding. It's up to the user to make sure the buffer is
 *              long enough.
 * @param len   - lenght of the total frame
 * @param iv    - Initialization Vector for AES CBC crypto mode
 * @return      - 0 if all good, 
 *                1 if the padded frame has exceeds
 */
static uint8_t m3_encryptFrame(uint8_t *src, uint16_t *len)
{
    uint8_t auxFrame[MAX_PDU_SIZE];
    uint16_t auxLen = 0;
    uint16_t padLen = 16 - (*len % 16);
    struct AES_ctx ctx;
    const uint8_t iv[16] =  {   0xc6, 0x6e, 0xbd, 0x70, 0x9e, 0xc9, 0xee, 0x11,
                                0x91, 0x1b, 0xa2, 0x71, 0x8c, 0x65, 0xa4, 0x16};

    //calculate and set padding
    auxLen = *len + padLen;
    if(auxLen >= MAX_PDU_SIZE)
    {
        return 1;
    }
    memset(&src[*len], padLen, padLen);

    //encrypt
    AES_init_ctx_iv(&ctx, (const uint8_t*)m3_bleKey, (const uint8_t*)iv);
    AES_CBC_encrypt_buffer(&ctx, src, auxLen);
    *len = auxLen;

    return 0;
}

/**
 * @brief This function requests decryption, verifies and prepares a frame
 * 
 * @param src   - Frame. This is the source array and will be used to return
 *                the decrypted frame 
 * @param len   - Lenght of the total frame
 * @param iv    - Initialization Vector for AES CBC crypto mode
 * @return      - 0 if all good
 *                1 if the decrypted frame does not has valid "frameLen"
 */
static uint8_t m3_decryptFrame(uint8_t *src, uint16_t *len)
{
    uint16_t auxLen = 0;
    struct AES_ctx ctx;
    const uint8_t iv[16] =  {   0xc6, 0x6e, 0xbd, 0x70, 0x9e, 0xc9, 0xee, 0x11,
                                0x91, 0x1b, 0xa2, 0x71, 0x8c, 0x65, 0xa4, 0x16};


    AES_init_ctx_iv(&ctx, (const uint8_t*)m3_bleKey, (const uint8_t*)iv);
    AES_CBC_decrypt_buffer(&ctx, src, (uint16_t)*len);

    //remove possible padding
    auxLen = 1 + 4 + 1 + 2 + *((uint16_t*)&src[6]) + 1 + 4;

    if(auxLen >= MAX_PDU_SIZE)
    {
        return 1;
    }
    *len = auxLen;

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
    // log_debug(TAG, "powered '%s' (%s)", state ? "on" : "off", 
    //                                 binc_adapter_get_path(adapter));

    printf_d("[powered_state]: powered '%s' (%s)", state ? "on" : "off", 
                                    binc_adapter_get_path(adapter));
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
        JSNotifyDetect();
        binc_adapter_stop_advertising(adapter, advertisement);
    } else if (state == BINC_DISCONNECTED){
        JSNotifyDeparture();
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

    //does nothing (no characteristic has read permition)
    
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
    uint8_t frame[MAX_PDU_SIZE];
    uint16_t frameTotalLen = 0;
    uint32_t *appId;
    uint8_t  *type;
    uint16_t *numBytes;
    uint32_t *frameCRC;
    uint32_t calcCRC;

    

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
    if((byteArray->len % 16) != 0)
        return BLUEZ_ERROR_INVALID_VALUE_LENGTH;

    memcpy(frame, byteArray->data, byteArray->len);
    frameTotalLen = byteArray->len;

    m3_decryptFrame(frame, &frameTotalLen);

    appId       = (uint32_t*)&frame[1];
    type        =  (uint8_t*)&frame[5];
    numBytes    = (uint16_t*)&frame[6];

    printf_d("\n[Char Write] decrypted frame = ");

    for(int i = 0; i < frameTotalLen; i++)
    {
        printf_d("%.2X", frame[i]);
    }

    /**
     * TODO: Verify if "frame Info" is a valid string 
     * 
     */
    JSNotifyCardInfo(*appId, (const char *)&frame[8]);

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

