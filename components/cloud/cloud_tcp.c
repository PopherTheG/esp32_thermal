#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_transport.h"
#include "esp_transport_tcp.h"
#include "cloud_tcp.h"

cloud_tcp_handle_t cloud_tcp_init(void)
{
    cloud_tcp_t* handle = calloc(1, sizeof(cloud_tcp_t));
    handle->tcp_handle = esp_transport_init();

    return handle;
}

int cloud_tcp_deinit(cloud_tcp_handle_t handle) {
    if(handle) {
        esp_transport_destroy(handle->tcp_handle);
        return 0;
    }
    return -1;
}

int cloud_tcp_connect(cloud_tcp_handle_t handle, const char* host, uint16_t port)
{
    return esp_transport_connect(handle->tcp_handle,host, port,  1000);
}

int cloud_tcp_disconnect(cloud_tcp_handle_t handle)
{
    return esp_transport_close(handle->tcp_handle);
}

int cloud_tcp_send(cloud_tcp_handle_t handle, const uint8_t* data, size_t len)
{
    return esp_transport_write(handle->tcp_handle, (char*)data, len, 1000);
}