#include <stdio.h>
#include <string.h>
#include <arpa/inet.h>
#include <unistd.h>

#define SERVER_IP "172.20.10.2" // Replace with your server's IP address
#define PORT 4242
#define BUF_SIZE 2048

int main() {
    int sock;
    struct sockaddr_in server_addr;
    char buffer[BUF_SIZE];  // Buffer for one byte of data

    // Create socket
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("Socket creation failed");
        return 1;
    }

    // Set up the server address
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(PORT);
    inet_pton(AF_INET, SERVER_IP, &server_addr.sin_addr);

    // Connect to the server
    if (connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        perror("Connection failed");
        return 1;
    }
    printf("Connected to server at %s:%d\n", SERVER_IP, PORT);

    
    // Send data to the server
    const char *message = "Hello from the client!";
    size_t message_len = strlen(message);
    send(sock, message, message_len, 0);
    printf("Sent %ld bytes to server\n", message_len);

    // Receive data from the server
    int bytes_received = recv(sock, buffer, BUF_SIZE, 0);
    printf("Received %d bytes from server\n", bytes_received);

    // Print received data
    buffer[bytes_received] = '\0'; // Null-terminate the received string
    printf("Received message: %s\n", buffer);

    
   

    // Clean up
    close(sock);
    return 0;
}
