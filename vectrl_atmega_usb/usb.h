/*
usb.h
Based on: https://github.com/kmani314/ATMega32u4-HID-Keyboard
*/
#ifndef USB_H
#define USB_H
#include <stdbool.h>
#include <stdint.h>

#define DESCRIPTOR_DEVICE     0x01
#define DESCRIPTOR_CONFIG     0x02
#define DESCRIPTOR_STRING     0x03
#define DESCRIPTOR_INTERFACE  0x04
#define DESCRIPTOR_ENDPOINT   0x05
#define DESCRIPTOR_HID        0x21
#define DESCRIPTOR_HID_REPORT 0x22


#define CFG_ATTR_BUS_POWERED   0x80
#define CFG_ATTR_SELF_POWERED  0x40
#define CFG_ATTR_REMOTE_WAKEUP 0x20

#define IF_CLASS_HID 0x03
#define IF_SUBCLASS_HID_NONE 0x00
#define IF_SUBCLASS_HID_BOOT 0x01
#define IF_PROTOCOL_HID_NONE 0x00
#define IF_PROTOCOL_HID_KEYBOARD 0x01
#define IF_PROTOCOL_HID_MOUSE 0x02

typedef struct _device_descriptor_t {
	uint8_t length;
	uint8_t descriptor_type;
	uint8_t usb_version_bcd[2];
	uint8_t device_class;
	uint8_t device_subclass;
	uint8_t device_protocol;
	uint8_t max_packet_size;
	uint8_t vendor_id[2];
	uint8_t product_id[2];
	uint8_t device_version[2];
	uint8_t manufacturer_name_index;
	uint8_t product_name_index;
	uint8_t serial_number_index;
	uint8_t num_configurations;
} __attribute__((packed)) device_descriptor_t;

typedef struct _configuration_descriptor_t {
	uint8_t length;
	uint8_t descriptor_type;
	uint8_t total_length[2];
	uint8_t num_interfaces;
	uint8_t configuration_value;
	uint8_t configuration_description_idx;
	uint8_t attributes;
	uint8_t max_power; // in units of 2mA
} __attribute__((packed)) configuration_descriptor_t;

typedef struct _interface_descriptor_t {
	uint8_t length;
	uint8_t descriptor_type;
	uint8_t interface_number;
	uint8_t alternate_setting;
	uint8_t num_endpoints;
	uint8_t interface_class;
	uint8_t interface_sub_class;
	uint8_t interface_protocol;
	uint8_t interface_description_idx;
} __attribute__((packed)) interface_descriptor_t;

typedef struct _report_descriptor_item_t {
	uint8_t descriptor_type;
	uint8_t length[2];
} __attribute__((packed)) report_descriptor_item_t;

typedef struct _hid_descriptor_t {
	uint8_t length;
	uint8_t descriptor_type;
	uint8_t hid_version_bcd[2];
	uint8_t country_code;
	uint8_t num_descriptors; // number of report_descriptor_item_t elements immediately after this field
	report_descriptor_item_t report_descriptor_items[];
} __attribute__((packed)) hid_descriptor_t;

typedef struct _endpoint_descriptor_t {
	uint8_t length;
	uint8_t descriptor_type;
	uint8_t endpoint_address;
	uint8_t attributes;
	uint8_t max_packet_size[2];
	uint8_t interval;
} __attribute__((packed)) endpoint_descriptor_t;

typedef struct _configuration_descriptor_tree_t {
	configuration_descriptor_t configuration_descriptor;
	interface_descriptor_t interface_descriptor;
	hid_descriptor_t hid_descriptor;
	report_descriptor_item_t report_descriptor_items[1];
	endpoint_descriptor_t endpoint_descriptor;
} __attribute__((packed)) configuration_descriptor_tree_t;

uint8_t usb_config_status;

int usb_init();
bool get_usb_config_status();
int usb_send(int8_t encoder0, uint8_t pressed_btn[], uint8_t count);

#define GET_STATUS 0x00
#define CLEAR_FEATURE 0x01
#define SET_FEATURE 0x03
#define SET_ADDRESS 0x05
#define GET_DESCRIPTOR 0x06
#define GET_CONFIGURATION 0x08
#define SET_CONFIGURATION 0x09
#define GET_INTERFACE 0x0A
#define SET_INTERFACE 0x0B

#define ID_VENDOR 0x05dc  // from https://github.com/obdev/v-usb/blob/master/usbdrv/USB-IDs-for-free.txt
#define ID_PRODUCT 0x16c0  // Generic HID device
// have legitimate IDs)
#define CONTROLLER_ENDPOINT_NUM 1

// HID Class-specific request codes - refer to HID Class Specification
// Chapter 7.2 - Remarks

#define GET_REPORT 0x01
#define GET_IDLE 0x02
#define GET_PROTOCOL 0x03
#define SET_REPORT 0x09
#define SET_IDLE 0x0A
#define SET_PROTOCOL 0x0B

#endif