#pragma once
#include <stdint.h>
#include <stddef.h>
typedef int esp_err_t;
#define ESP_OK 0
typedef struct client_s *usb_host_client_handle_t;
typedef struct dev_s *usb_device_handle_t;
typedef enum { USB_TRANSFER_STATUS_COMPLETED, USB_TRANSFER_STATUS_ERROR, USB_TRANSFER_STATUS_TIMED_OUT, USB_TRANSFER_STATUS_CANCELED } usb_transfer_status_t;
typedef struct usb_transfer_s usb_transfer_t;
typedef void (*usb_transfer_cb_t)(usb_transfer_t *);
struct usb_transfer_s { uint8_t *const data_buffer; const size_t data_buffer_size; int num_bytes; int actual_num_bytes; uint32_t flags; usb_device_handle_t device_handle; uint8_t bEndpointAddress; usb_transfer_status_t status; uint32_t timeout_ms; usb_transfer_cb_t callback; void *context; };
typedef struct { uint8_t bmRequestType; uint8_t bRequest; uint16_t wValue; uint16_t wIndex; uint16_t wLength; } usb_setup_packet_t;
esp_err_t usb_host_transfer_alloc(size_t, int, usb_transfer_t **);
esp_err_t usb_host_transfer_free(usb_transfer_t *);
esp_err_t usb_host_transfer_submit(usb_transfer_t *);
esp_err_t usb_host_transfer_submit_control(usb_host_client_handle_t, usb_transfer_t *);
esp_err_t usb_host_client_handle_events(usb_host_client_handle_t, uint32_t);
esp_err_t usb_host_interface_claim(usb_host_client_handle_t, usb_device_handle_t, uint8_t, uint8_t);
esp_err_t usb_host_interface_release(usb_host_client_handle_t, usb_device_handle_t, uint8_t);
esp_err_t usb_host_endpoint_halt(usb_device_handle_t, uint8_t);
esp_err_t usb_host_endpoint_flush(usb_device_handle_t, uint8_t);
esp_err_t usb_host_endpoint_clear(usb_device_handle_t, uint8_t);
