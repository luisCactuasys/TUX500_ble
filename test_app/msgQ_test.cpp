
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <errno.h>

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
#include <syslog.h>

#include <sys/ioctl.h>
#include "tools.h"
#include "json.h"


#define MSG_KEY 31337
const int MSG_BLE2JSAPP = 1;
const int MSG_JSAPP2BLE = 2;

// Define the structure for the message buffer
struct blemsgbuf {
    long mtype; // Message type
    char mtext[4000]; // Message data
};


int msgqid;

void printf_d(const char* fmt, ...)
{
	va_list args;
	char buf[1000];
	va_start(args, fmt);
	vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);
	
    printf("%s", buf);
    fflush(stdout);
	
}

int MsgQInvoke(char* requestId, char* method, JsonObject& params)
{
    int res = 0;
    struct blemsgbuf qbuf;
    
    // Create message in Json format
    
    JsonRpcInvoke(requestId, method, params, qbuf.mtext, sizeof(qbuf.mtext));
    qbuf.mtype = MSG_JSAPP2BLE;
    printf("[MsgQInvoke] Invoking: type = %ld, content = %s", qbuf.mtype, qbuf.mtext);
    
    if((res = msgsnd(msgqid, &qbuf, sizeof(qbuf.mtext), IPC_NOWAIT )) == -1)
    {
        printf("Failed to msgsnd: %d\n", res);
        int err = errno;
        if (err != ENOMSG) {
            printf_d("error recv %d  %d", res, err);
            usleep(5000e3);
        }
    }   
    else
    {
        printf("msgsnd OK %d\n", res);
    }

    return res;
}


int Notify(const char* event, int value)
{
  StaticJsonBuffer<200> jsonResultBuffer;
  JsonObject& jsonResult = jsonResultBuffer.createObject();
  jsonResult["getVersion"] = value;
  return MsgQInvoke(NULL, (char*)event, jsonResult);
}

int main() {
    
    struct blemsgbuf buf;
    const char* methodGetVer = "setKey";
    int requestId = 0;
    std::string requestIdstr = std::to_string(requestId);
    ssize_t rcv;
    int res = 0;
    
    // Create or open the message queue
    if ((msgqid = msgget(31337, IPC_CREAT | 0660)) == -1) {
        perror("msgget");
        exit(EXIT_FAILURE);
    }
    printf("Message Open: %d", msgqid);

    StaticJsonBuffer<200> jsonResultBuffer;
    JsonObject& jsonResult = jsonResultBuffer.createObject();
    //jsonResult["key"] = "AABBCC00112233445566778899DDEEFF";
    jsonResult["key"] = "11111111111111111111111111111111";
    res = MsgQInvoke((char*)std::to_string(requestId++).c_str(), (char*)methodGetVer, jsonResult);
    printf("Key sent to the queue.\n Waiting for ");
    
    StaticJsonBuffer<200> jsonResultBuffer2;
    JsonObject& jsonResult2 = jsonResultBuffer2.createObject();
    jsonResult2["enabled"] = "1";
    jsonResult2["readerEnabled"] = "1";
    res = MsgQInvoke((char*)std::to_string(requestId++).c_str(), (char*)"setAutoRead", jsonResult2);
    printf("AutoRead sent to the queue.\n Waiting for ");

    // if ((rcv = msgrcv(msgqid, &buf, sizeof(buf.mtext), MSG_BLE2JSAPP, IPC_NOWAIT)) < 0) {
    //         // No message or error
    //         int err = errno;
    //         if (err != ENOMSG) {
    //             perror("msgrcv");
    //             usleep(5000e3);
    //         }
    //         usleep(50e3);
    // }
    // printf("Received message: %s\n", buf.mtext);

    // Read messages from the queue
    printf("Reading messages from the queue:\n");
    while (1) {
       
        if ((rcv = msgrcv(msgqid, &buf, sizeof(buf.mtext), MSG_BLE2JSAPP, IPC_NOWAIT)) < 0) {
            // No message or error
            int err = errno;
            if (err != ENOMSG) {
                perror("msgrcv");
                usleep(5000e3);
            }
            usleep(50e3);
            continue;
        }

        printf("Received message: %s\n", buf.mtext);
        StaticJsonBuffer<200> jsonReceivedBuffer;
        JsonObject& jsonReceived = jsonReceivedBuffer.parseObject(buf.mtext);
        JsonObject& jsonReceivedParams = jsonReceived["params"];
        uint32_t recvAppId = 0;
        std::string recvCardInfo;
        if(!jsonReceivedParams.containsKey("appId"))
        {
            continue;
        }
        if(!jsonReceivedParams.containsKey("cardInfo"))
        {
            continue;
        }

        recvAppId = jsonReceivedParams["appId"];
        recvCardInfo = jsonReceivedParams["cardInfo"].as<char*>();

        StaticJsonBuffer<200> jsonToSendBuffer;
        JsonObject& jsonToSend = jsonToSendBuffer.createObject();
        jsonToSend["appId"] = recvAppId;
        jsonToSend["status"] = "read";
        res = MsgQInvoke((char*)std::to_string(requestId++).c_str(), (char*)"acknowledge", jsonToSend);


        memset(&buf, 0, sizeof(buf));
    }

    // Close the message queue
    if (msgctl(msgqid, IPC_RMID, NULL) == -1) {
        perror("msgctl");
        exit(EXIT_FAILURE);
    }

    return 0;
}