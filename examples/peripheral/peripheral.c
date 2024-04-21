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

#include <glib.h>
#include <stdio.h>
#include <signal.h>
#include "adapter.h"
#include "device.h"
#include "logger.h"
#include "agent.h"
#include "application.h"
#include "advertisement.h"
#include "utility.h"
#include "parser.h"

#define TAG "Main"
#define CUD_CHAR "00002901-0000-1000-8000-00805f9b34fb"

// "BLE Reader" Primary Service
#define M3_TUX_SERVICE_UUID "0000fff0-0000-1000-8000-00805f9b34fb"
// Characteristic 1. Used to receive thus as Write property 
#define M3_TUX_CHAR_1_UUID "0000fff1-0000-1000-8000-00805f9b34fb"
// Characteristic 2. Used to transmit thus as Notify property
#define M3_TUX_CHAR_2_UUID "0000fff2-0000-1000-8000-00805f9b34fb"


GMainLoop *loop = NULL;
Adapter *default_adapter = NULL;
Advertisement *advertisement = NULL;
Application *app = NULL;


void on_powered_state_changed(Adapter *adapter, gboolean state) 
{
    log_debug(TAG, "powered '%s' (%s)", state ? "on" : "off", 
                                    binc_adapter_get_path(adapter));
}

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

void on_local_char_updated(const Application *application, 
        const char *service_uuid, const char *char_uuid, GByteArray *byteArray)
{
    log_debug(TAG, "on char updated");
}

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
    
    /**
     * TODO: Set necessary flags
     * 
     */
    

    
                // if(!g_str_equal(char_uuid, M3_TUX_CHAR_2_UUID)) 
                // {
                //     const guint8 bytes[] = {0x06, 0x6A, 0x01, 0x00, 0xff, 0xe6, 0x07, 
                //                             0x03, 0x03, 0x10, 0x04, 0x00, 0x01};
                //     GByteArray *byteArray = g_byte_array_sized_new(sizeof(bytes));

                //     g_byte_array_append(byteArray, bytes, sizeof(bytes));
                //     binc_application_notify(application, service_uuid, char_uuid, byteArray);
                //     g_byte_array_free(byteArray, TRUE);
                // }
}

void on_local_char_stop_notify(const Application *application, 
                                const char *service_uuid, const char *char_uuid)
{
    log_debug(TAG, "on stop notify");
}
 
/**
 * @brief This callback is called just before the descriptor's value is returned. 
 *          Use it to update the descriptor before it is read
 * 
 */
const char *on_local_desc_read(const Application *application, const char *address, 
            const char *service_uuid, const char *char_uuid, const char *desc_uuid)
{

    return NULL;
}

/**
 * @brief This callback is called just before the descriptor's value is set.
 * Use it to accept (return NULL), or reject (return BLUEZ_ERROR_*) the byte array
 * 
 */
const char *on_local_desc_write( const Application *application, const char *address,
                                const char *service_uuid, const char *char_uuid,
                                const char *desc_uuid, const GByteArray *byteArray)
{
    return NULL;
}


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

static void cleanup_handler(int signo) {
    if (signo == SIGINT) {
        log_error(TAG, "received SIGINT");
        callback(loop);
    }
}

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

        // Setup remote central connection state callback
        binc_adapter_set_remote_central_cb(default_adapter, &on_central_state_changed);

        // Setup advertisement
        GPtrArray *adv_service_uuids = g_ptr_array_new();
        g_ptr_array_add(adv_service_uuids, M3_TUX_SERVICE_UUID);

        advertisement = binc_advertisement_create();
        binc_advertisement_set_local_name(advertisement, "BINC");
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
        // binc_application_add_descriptor(
        //         app,
        //         M3_TUX_SERVICE_UUID,
        //         M3_TUX_CHAR_1_UUID,
        //         CUD_CHAR,
        //         GATT_CHR_PROP_READ | GATT_CHR_PROP_WRITE);

        binc_application_add_characteristic(
                app,
                M3_TUX_SERVICE_UUID,
                M3_TUX_CHAR_2_UUID,
                GATT_CHR_PROP_INDICATE);

        const guint8 cud[] = "hello there";
        GByteArray *cudArray = g_byte_array_sized_new(sizeof(cud));
        g_byte_array_append(cudArray, cud, sizeof(cud));
        // binc_application_set_desc_value(app, M3_TUX_SERVICE_UUID, 
        //                                 TEMPERATURE_CHAR_UUID, CUD_CHAR, cudArray);

        // Set callbacks
        binc_application_set_char_read_cb(app, &on_local_char_read);
        binc_application_set_char_write_cb(app, &on_local_char_write);

        //binc_application_set_char_update_cb(app, &on_local_char_updated); //does not exist
        
        binc_application_set_char_start_notify_cb(app, &on_local_char_start_notify);
        binc_application_set_char_stop_notify_cb(app, &on_local_char_stop_notify);
        
        binc_application_set_desc_read_cb(app, &on_local_desc_read);    
        binc_application_set_desc_write_cb(app, &on_local_desc_write);


        binc_adapter_register_application(default_adapter, app);
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