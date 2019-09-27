void response(int conn_fd, const char *message);
char *calculate_accept_key(const char *buffer, char *server_key);
void websocket_shakehand(int conn_fd, const char *server_key);
char *deal_data(const char *buffer, int *len);
