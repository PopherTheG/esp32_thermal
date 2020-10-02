#ifndef COMPONENTS_CLOUD_CLOUD_TCP
#define COMPONENTS_CLOUD_CLOUD_TCP

#include <stdlib.h>
#include <stdint.h>

typedef struct {
    void* tcp_handle;
    void* task_handle;
} cloud_tcp_t;

typedef cloud_tcp_t* cloud_tcp_handle_t;

cloud_tcp_handle_t cloud_tcp_init(void);

int cloud_tcp_deinit(cloud_tcp_handle_t handle);

int cloud_tcp_connect(cloud_tcp_handle_t handle, const char* host, uint16_t port);

int cloud_tcp_disconnect(cloud_tcp_handle_t handle);

int cloud_tcp_send(cloud_tcp_handle_t handle, const uint8_t* data, size_t len);


#endif /* COMPONENTS_CLOUD_CLOUD_TCP */
