#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#define PORT 2840

struct trolley2spreader {
    int mode0; 
    int mode1; 
    int mode2;    
    int mode3; 
    int trolley_heart_beat;
};

struct spreader2trolley {
    int State0; 
    int State1; 
    int State2;    
    int State3; 
    int mode0; 
    int mode1; 
    int mode2;    
    int mode3; 
    int spreader_heart_beat;
};

void handle_communication(int sock) {
    struct trolley2spreader trolley_data;
    struct spreader2trolley spreader_data;
    int counter = 0;

    // Communication loop
    while (1) {
        // Prepare data to send
        trolley_data.mode0 = 1;
        trolley_data.mode1 = 2;
        trolley_data.mode2 = 3;
        trolley_data.mode3 = 4;
        trolley_data.trolley_heart_beat = counter;
        counter = counter > 1000 ? 0 : (counter + 1);
        // Send data to server
        if (send(sock, &trolley_data, sizeof(trolley2spreader), 0) < 0) {
            printf("\nSend failed\n");
            close(sock);
            return;
        }
        printf("Data sent to server\n");

        // Receive data from server
        if (recv(sock, &spreader_data, sizeof(spreader2trolley), 0) <= 0) {
            printf("\nServer disconnected\n");
            close(sock);
            return;
        }
        printf("Trolley Side Received: State0=%d, State1=%d, State2=%d,  State3=%d, mode0=%d, mode1=%d, mode2=%d, mode3=%d, spreader_heart_beat=%d\n",
               spreader_data.State0, spreader_data.State1, spreader_data.State2, spreader_data.State3,
               spreader_data.mode0, spreader_data.mode1, spreader_data.mode2, spreader_data.mode3,
               spreader_data.spreader_heart_beat);

        // Optionally, you can add a sleep here to control the rate of communication
        usleep(50 * 1000); // 50 ms
    }
}

int main() {
    int sock = 0;
    struct sockaddr_in serv_addr;

    while (1) {
        // Create socket
        if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
            printf("\n Socket creation error \n");
            return -1;
        }

        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(PORT);

        // Convert IPv4 and IPv6 addresses from text to binary form
        if (inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr) <= 0) {
            printf("\nInvalid address/ Address not supported \n");
            return -1;
        }

        // Try to connect to server
        while (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
            printf("\nConnection Failed \n");
            close(sock);
            sleep(1); // Wait for 1 second before retrying
        }

        printf("Connected to server\n");
        handle_communication(sock);
    }

    return 0;
}
