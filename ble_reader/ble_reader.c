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

#include "adapter.h"
#include "device.h"
#include "logger.h"
#include "agent.h"
#include "application.h"
#include "advertisement.h"
#include "utility.h"
#include "parser.h"
#include <syslog.h>

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
 * *********************** Private Variables **********************************
 */
GMainLoop *loop = NULL;
Adapter *default_adapter = NULL;
GPtrArray *adv_service_uuids = NULL;
Advertisement *advertisement = NULL;
Application *app = NULL;


/*
 * ********************** Private Function Prototypes **************************
 */
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

// --------- M3 ----------
int32_t m3_bleReaderInit();

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
    
    // Get a DBus connection
    GDBusConnection *dbusConnection = g_bus_get_sync(G_BUS_TYPE_SYSTEM, NULL, NULL);

    // Setup handler for CTRL+C
    if (signal(SIGINT, cleanup_handler) == SIG_ERR)
    {    
        log_error(TAG, "can't catch SIGINT");
    }

    // Setup mainloop
    loop = g_main_loop_new(NULL, FALSE);
    // Get the default default_adapter
    default_adapter = binc_adapter_get_default(dbusConnection);

    if (default_adapter != NULL) 
    {
        log_debug(TAG, "using default_adapter '%s'", 
                        binc_adapter_get_path(default_adapter));

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
        log_debug("MAIN", "No default_adapter found");
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



/**
 * @brief 
 * 
 * @param adapter 
 * @param state 
 */
void on_powered_state_changed(Adapter *adapter, gboolean state) 
{
    log_debug(TAG, "powered '%s' (%s)", state ? "on" : "off", 
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
    log_debug(TAG, deviceToString);
    g_free(deviceToString);

    log_debug(TAG, "remote central %s is %s", binc_device_get_address(device),
                     binc_device_get_connection_state_name(device));
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
    log_debug(TAG, "on_char_read");
    
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
    log_debug(TAG, "on start notify");

    // Check if it is the correct characteristic (tx with "Notify")
    if(!g_str_equal(service_uuid, M3_TUX_SERVICE_UUID))
    {
        log_warn(TAG, "start notify: invalid Service");
        return;
    }
    if(!g_str_equal(char_uuid, M3_TUX_CHAR_2_UUID))
    {
        log_warn(TAG, "start notify: invalid Characteristic");
        return;
    }
    
    log_info(TAG, "Notifications Enabled...");
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
    log_debug(TAG, "on stop notify");
    log_info(TAG, "Notifications Disabled...");
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
        log_error(TAG, "received SIGINT");
        callback(loop);
    }
}


/*
 * --------------- M3 definitions ------------
*/

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
    g_ptr_array_add(adv_service_uuids, M3_TUX_SERVICE_UUID);

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
