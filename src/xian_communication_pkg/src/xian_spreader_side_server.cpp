#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
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

void handle_client(int client_socket) {
    struct trolley2spreader trolley_data;
    struct spreader2trolley spreader_data;
    int counter = 0;

    

    // Communication loop
    while (1) {
        usleep(50 * 1000); // 50 ms
        // Prepare data to send
        spreader_data.State0 = 0;
        spreader_data.State1 = 1;
        spreader_data.State2 = 2;
        spreader_data.State3 = 3;
        spreader_data.mode0 = 4;
        spreader_data.mode1 = 5;
        spreader_data.mode2 = 6;
        spreader_data.mode3 = 7;
        spreader_data.spreader_heart_beat = counter;
        counter = counter > 1000 ? 0 : (counter + 1);
        // Send data to client
        if (send(client_socket, &spreader_data, sizeof(spreader2trolley), 0) < 0) {
            printf("Send failed\n");
            close(client_socket);
            return;
        }
        printf("Data sent to client\n");

        // Receive data from client
        if (recv(client_socket, &trolley_data, sizeof(trolley2spreader), 0) <= 0) {
            printf("Client disconnected\n");
            close(client_socket);
            return;
        }
        printf("Spreader Side Received: mode0=%d, mode1=%d, mode2=%d, mode3=%d, trolley_heart_beat=%d\n",
               trolley_data.mode0, trolley_data.mode1, trolley_data.mode2, trolley_data.mode3, trolley_data.trolley_heart_beat);
    }
}

int main() {
    int server_fd, new_socket;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);

    // Create socket
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    // Set socket options
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    // Bind socket to port
    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

    // Start listening
    if (listen(server_fd, 3) < 0) {
        perror("listen");
        exit(EXIT_FAILURE);
    }

    printf("Server listening on port %d...\n", PORT);

    // Accept and handle incoming connections
    while (1) {
        if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen)) < 0) {
            perror("accept");
            continue;
        }
        printf("Client connected\n");
        handle_client(new_socket);
    }

    close(server_fd);
    return 0;
}
