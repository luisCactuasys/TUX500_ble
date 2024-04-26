
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <errno.h>

#define MSG_KEY 31337

// Define the structure for the message buffer
struct blemsgbuf {
    long mtype; // Message type
    char mtext[256]; // Message data
};

int main() {
    int msgqid;
    struct blemsgbuf buf;

    // Create or open the message queue
    if ((msgqid = msgget(MSG_KEY, IPC_CREAT | 0660)) == -1) {
        perror("msgget");
        exit(EXIT_FAILURE);
    }

    // Send a message to the queue
    printf("Enter a message to send to the queue: ");
    fgets(buf.mtext, sizeof(buf.mtext), stdin);
    buf.mtype = 2; // Set the message type
    if (msgsnd(msgqid, &buf, strlen(buf.mtext) + 1, IPC_NOWAIT) == -1) {
        perror("msgsnd");
        exit(EXIT_FAILURE);
    }
    printf("Message sent to the queue.\n");

    // Read messages from the queue
    printf("Reading messages from the queue:\n");
    while (1) {
        ssize_t rcv;
        if ((rcv = msgrcv(msgqid, &buf, sizeof(buf.mtext), 2, IPC_NOWAIT)) < 0) {
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
        memset(&buf, 0, sizeof(buf));
        break; // Break after receiving one message for this example
    }

    // Close the message queue
    if (msgctl(msgqid, IPC_RMID, NULL) == -1) {
        perror("msgctl");
        exit(EXIT_FAILURE);
    }

    return 0;
}