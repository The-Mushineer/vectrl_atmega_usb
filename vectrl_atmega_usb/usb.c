/*
usb.c
USB Controller initialization, device setup, and HID interrupt routines
Based on: https://github.com/kmani314/ATMega32u4-HID-Keyboard
*/

#include "config.h"
#include "usb.h"
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "avr/io.h"


volatile uint8_t pressed_buttons[9] = {0};

static uint16_t keyboard_idle_value =
    125;  // HID Idle setting, how often the device resends unchanging reports,
          // we are using a scaling of 4 because of the register size
static uint8_t current_idle =
    0;  // Counter that updates based on how many SOFE interrupts have occurred
static uint8_t this_interrupt =
    0;  // This is not the best way to do it, but it
        // is much more readable than the alternative

/*  Report descriptor - generated using the HID Descriptor Tool
    https://usb.org/document-library/hid-descriptor-tool
*/
static const uint8_t ctrl_report_descriptor[] PROGMEM = {
	0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
	0x09, 0x05,                    // USAGE (Game Pad)
	0xa1, 0x01,                    // COLLECTION (Application)
	0x05, 0x01,                    //   USAGE_PAGE (Generic Desktop)
	0x09, 0x37,                    //   USAGE (Dial)
	0x15, 0x81,                    //   LOGICAL_MINIMUM (-127)
	0x25, 0x7f,                    //   LOGICAL_MAXIMUM (127)
	0x95, 0x02,                    //   REPORT_COUNT (2)
	0x75, 0x08,                    //   REPORT_SIZE (8)
	0x81, 0x06,                    //   INPUT (Data,Var,Rel)
	0x05, 0x09,                    //   USAGE_PAGE (Button)
	0x19, 0x00,                    //   USAGE_MINIMUM (No Buttons Pressed)
	0x29, 0x10,                    //   USAGE_MAXIMUM (Button 16)
	0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
	0x25, 0x10,                    //   LOGICAL_MAXIMUM (16)
	0x75, 0x08,                    //   REPORT_SIZE (8)
	0x95, 0x06,                    //   REPORT_COUNT (6)
	0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
	0xc0                           // END_COLLECTION
};


/*  Device Descriptor - The top level descriptor when enumerating a USB device`
        Specification: USB 2.0 (April 27, 2000) Chapter 9 Table 9-5
*/
static const device_descriptor_t device_descriptor PROGMEM = {
	.length = sizeof(device_descriptor_t),
	.descriptor_type = DESCRIPTOR_DEVICE, // The type of descriptor
	.usb_version_bcd = {0x00, 0x01}, // The USB protcol supported
	.device_class = 0, // The Device Class, 0 indicating that the HID interface will specify it
	.device_subclass = 0, // HID will specify
	.device_protocol = 0, // No class specific protocols on a device level
	.max_packet_size = 32, // 32 byte packet size; control endpoint was configured in UECFG1X to be 32 bytes
	.vendor_id = {(ID_VENDOR & 255), ((ID_VENDOR >> 8) & 255)},
	.product_id = {(ID_PRODUCT & 255), ((ID_PRODUCT >> 8) & 255)},
	.device_version = {0, 1},
	.manufacturer_name_index = 1,
	.product_name_index = 2,
	.serial_number_index = 3,
	.num_configurations = 1,	
};

static const configuration_descriptor_tree_t configuration_descriptor_tree PROGMEM = {
	.configuration_descriptor = {
		.length = sizeof(configuration_descriptor_t), // length 
		.descriptor_type = DESCRIPTOR_CONFIG, // 2 is device
		.total_length = {sizeof(configuration_descriptor_tree_t), 0},
		.num_interfaces = 1, // 1 interface
		.configuration_value = 1,
		.configuration_description_idx = 0, // no string descriptors
		.attributes = CFG_ATTR_BUS_POWERED | CFG_ATTR_REMOTE_WAKEUP,
		.max_power = 50, // 100mA
	},
	.interface_descriptor = {
		.length = sizeof(interface_descriptor_t),
		.descriptor_type = DESCRIPTOR_INTERFACE, // 4 is interface
		.interface_number = 0,
		.alternate_setting = 0, // no alternate settings
		.num_endpoints = 1,
		.interface_class = IF_CLASS_HID,
		.interface_sub_class = IF_SUBCLASS_HID_NONE,
		.interface_protocol = IF_PROTOCOL_HID_NONE,
		.interface_description_idx = 0,
	},
	.hid_descriptor = {
		.length = sizeof(hid_descriptor_t) + sizeof(report_descriptor_item_t) * 1,
		.descriptor_type = DESCRIPTOR_HID,
		.hid_version_bcd = {0x11, 0x01},
		.country_code = 0,
		.num_descriptors = 1,
	},
	.report_descriptor_items = {
		{
			.descriptor_type = DESCRIPTOR_HID_REPORT,
			.length = {sizeof(ctrl_report_descriptor), 0},
		},
	},
	.endpoint_descriptor = {
		.length = sizeof(endpoint_descriptor_t),
		.descriptor_type = DESCRIPTOR_ENDPOINT,
		.endpoint_address = CONTROLLER_ENDPOINT_NUM | 0x80,  // set endpoint to IN endpoint
		.attributes = 0x03, // set to interrupt
		.max_packet_size = {8, 0}, // size of the banks
		.interval = 0x0A, // poll for new data once every 10 ms
		},
};

/*   String descriptors
*/
const uint8_t string_descriptor_zero[] PROGMEM = {
	// Length
	4,
	// Type
	DESCRIPTOR_STRING,
	// Language IDs
	0x09, 0x04
};

const uint8_t string_descriptor_manufacturer_name[] PROGMEM = {
	// Length
	24,
	// Type
	DESCRIPTOR_STRING,
	// Text
	'L', 0, 'e', 0, 'o', 0, 'n', 0, ' ', 0, 'O', 0, 'l', 0, 'i', 0, 'v', 0, 'e', 0, 'r', 0
};

const uint8_t string_descriptor_product_name[] PROGMEM = {
	// Length
	50,
	// Type
	DESCRIPTOR_STRING,
	// Text
	'V', 0, 'i', 0, 'd', 0, 'e', 0, 'o', 0, ' ', 0, 'E', 0, 'd', 0, 'i', 0, 't', 0, 'i', 0, 'n', 0, 'g', 0, ' ', 0, 'C', 0, 'o', 0, 'n', 0, 't', 0, 'r', 0, 'o', 0, 'l', 0, 'l', 0, 'e', 0, 'r', 0,
};

const uint8_t string_descriptor_serial[] PROGMEM = {
	// Length
	10,
	// Type
	DESCRIPTOR_STRING,
	// Text
	'T', 0, 'e', 0, 's', 0, 't', 0
};

int usb_init() {
  cli();  // Global Interrupt Disable

  UHWCON |= (1 << UVREGE);  // Enable USB Pads Regulator

  PLLCSR |= 0x12;  // Configure to use 16mHz oscillator

  while (!(PLLCSR & (1 << PLOCK)))
    ;  // Wait for PLL Lock to be achieved

  USBCON |=
      (1 << USBE) | (1 << OTGPADE);  // Enable USB Controller and USB power pads
  USBCON &= ~(1 << FRZCLK);          // Unfreeze the clock

  UDCON &= ~(1<<LSM);  // FULL SPEED MODE
  UDCON &= ~(1<<DETACH);  // Attach USB Controller to the data bus

  UDIEN |= (1 << EORSTE) |
           (1 << SOFE);  // Re-enable the EORSTE (End Of Reset) Interrupt so we
                         // know when we can configure the control endpoint
  usb_config_status = 0;
  sei();  // Global Interrupt Enable
  return 0;
}

static void _send_data(int8_t encoder0) {
	UEDATX = encoder0;
	UEDATX = 0;
	UEDATX = pressed_buttons[0];
	UEDATX = pressed_buttons[1];
	UEDATX = pressed_buttons[2];
	UEDATX = pressed_buttons[3];
	UEDATX = pressed_buttons[4];
	UEDATX = pressed_buttons[5];
}

int usb_send(int8_t encoder0, uint8_t pressed_btn[], uint8_t count) {
	if (!usb_config_status)
		return -1;  // Why are you even trying
	cli();
	UENUM = CONTROLLER_ENDPOINT_NUM;
	
	for (uint8_t idx = 0; idx < count; idx++) {		
		pressed_buttons[idx] = pressed_btn[idx];
	}
	for (uint8_t idx = count; idx < 9; idx++) {
		pressed_buttons[idx] = 0;
	}

	while (!(UEINTX & (1 << RWAL)))
	;  // Wait for banks to be ready
	_send_data(encoder0);

	UEINTX = 0b00111010;
	current_idle = 0;
	sei();
	return 0;
}

bool get_usb_config_status() {
  return usb_config_status;
}

ISR(USB_GEN_vect) {
  uint8_t udint_temp = UDINT;
  UDINT = 0;

  if (udint_temp & (1 << EORSTI)) {  // If end of reset interrupt
    // Configure Control Endpoint
    UENUM = 0;             // Select Endpoint 0, the default control endpoint
    UECONX = (1 << EPEN);  // Enable the Endpoint
    UECFG0X = 0;      // Control Endpoint, OUT direction for control endpoint
    UECFG1X |= 0x22;  // 32 byte endpoint, 1 bank, allocate the memory
    usb_config_status = 0;

    if (!(UESTA0X &
          (1 << CFGOK))) {  // Check if endpoint configuration was successful
      return;
    }

    UERST = 1;  // Reset Endpoint
    UERST = 0;

    UEIENX =
        (1 << RXSTPE);  // Re-enable the RXSPTE (Receive Setup Packet) Interrupt
    return;
  }
  if ((udint_temp & (1 << SOFI)) &&
      usb_config_status) {  // Check for Start Of Frame Interrupt and correct
                            // usb configuration, send keypress if a keypress
                            // event has not been sent through usb_send
    this_interrupt++;
    if (keyboard_idle_value &&
        (this_interrupt & 3) == 0) {  // Scaling by four, trying to save memory
      UENUM = CONTROLLER_ENDPOINT_NUM;
      if (UEINTX & (1 << RWAL)) {  // Check if banks are writable
        current_idle++;
        if (current_idle ==
            keyboard_idle_value) {  // Have we reached the idle threshold?
          current_idle = 0;
          _send_data(0);
          UEINTX = 0b00111010;
        }
      }
    }
  }
}

ISR(USB_COM_vect) {
  UENUM = 0;
  if (UEINTX & (1 << RXSTPI)) {
    uint8_t bmRequestType = UEDATX;  // UEDATX is FIFO; see table in README
    uint8_t bRequest = UEDATX;
    uint16_t wValue = UEDATX;
    wValue |= UEDATX << 8;
    uint16_t wIndex = UEDATX;
    wIndex |= UEDATX << 8;
    uint16_t wLength = UEDATX;
    wLength |= UEDATX << 8;

    UEINTX &= ~(
        (1 << RXSTPI) | (1 << RXOUTI) |
        (1 << TXINI));  // Handshake the Interrupts, do this after recording
                        // the packet because it also clears the endpoint banks
    if (bRequest == GET_DESCRIPTOR) {
      // The Host is requesting a descriptor to enumerate the device
      const uint8_t* descriptor;
      uint16_t descriptor_length;

      switch (wValue) {
	      case 0x0100:   // Is the host requesting a device descriptor?
	      descriptor = (uint8_t*) &device_descriptor;
	      descriptor_length = pgm_read_byte(descriptor);
	      break;
		  
	      case 0x0200:   // Is it asking for a configuration descriptor?
	      descriptor = (uint8_t*) &configuration_descriptor_tree;
	      descriptor_length = configuration_descriptor_tree.configuration_descriptor.total_length[1];
	      descriptor_length <<= 8;
	      descriptor_length |= configuration_descriptor_tree.configuration_descriptor.total_length[0];
	      break;
		  
	      case 0x2100:  // Is it asking for a HID Descriptor?
	      descriptor = (uint8_t*) &configuration_descriptor_tree.hid_descriptor;
	      descriptor_length = pgm_read_byte(descriptor);
	      break;
		  
	      case 0x2200: // Is it asking for the  HID Report Descriptor?
	      descriptor = ctrl_report_descriptor;
	      descriptor_length = sizeof(ctrl_report_descriptor);
	      break;
		  
	      case 0x0300: // String descriptor 0
	      descriptor = string_descriptor_zero;
	      descriptor_length = sizeof(string_descriptor_zero);
	      break;
		  
	      case 0x0301: // String descriptor 1
	      descriptor = string_descriptor_manufacturer_name;
	      descriptor_length = sizeof(string_descriptor_manufacturer_name);
	      break;
		  
	      case 0x0302: // String descriptor 2
	      descriptor = string_descriptor_product_name;
	      descriptor_length = sizeof(string_descriptor_product_name);
	      break;
		  
	      case 0x0303: // String descriptor 3
	      descriptor = string_descriptor_serial;
	      descriptor_length = sizeof(string_descriptor_serial);
	      break;
		  
	      default:
	      // Enable the endpoint and stall, the descriptor does not exist
	      UECONX |= (1 << STALLRQ) | (1 << EPEN);
	      return;
      }

      uint8_t request_length =
          wLength > 255 ? 255
                        : wLength;  // Our endpoint is only so big; the USB Spec
                                    // says to truncate the response if the size
                                    // exceeds the size of the endpoint

      descriptor_length =
          request_length > descriptor_length
              ? descriptor_length
              : request_length;  // Truncate to descriptor length at most

      while (descriptor_length > 0) {
        while (!(UEINTX & (1 << TXINI)))
          ;  // Wait for banks to be ready for data transmission
        if (UEINTX & (1 << RXOUTI))
          return;  // If there is another packet, exit to handle it

        uint8_t thisPacket =
            descriptor_length > 32
                ? 32
                : descriptor_length;  // Make sure that the packet we are
                                      // getting is not too big to fit in the
                                      // endpoint

        for (int i = 0; i < thisPacket; i++) {
          UEDATX = pgm_read_byte(
              descriptor +
              i);  // Send the descriptor over UEDATX, use pgmspace functions
                   // because the descriptors are stored in flash
        }

        descriptor_length -= thisPacket;
        descriptor += thisPacket;
        UEINTX &= ~(1 << TXINI);
      }
      return;
    }

    if (bRequest == SET_CONFIGURATION &&
        bmRequestType ==
            0) {  // Refer to USB Spec 9.4.7 - This is the configuration request
                  // to place the device into address mode
      usb_config_status = wValue;
      UEINTX &= ~(1 << TXINI);
      UENUM = CONTROLLER_ENDPOINT_NUM;
      UECONX = 1;
      UECFG0X = 0b11000001;  // EPTYPE Interrupt IN
      UECFG1X = 0b00000110;  // Dual Bank Endpoint, 8 Bytes, allocate memory
      UERST = 0x1E;          // Reset all of the endpoints
      UERST = 0;
      return;
    }

    if (bRequest == SET_ADDRESS) {
      UEINTX &= ~(1 << TXINI);
      while (!(UEINTX & (1 << TXINI)))
        ;  // Wait until the banks are ready to be filled

      UDADDR = wValue | (1 << ADDEN);  // Set the device address
      return;
    }

    if (bRequest == GET_CONFIGURATION &&
        bmRequestType == 0x80) {  // GET_CONFIGURATION is the host trying to get
                                  // the current config status of the device
      while (!(UEINTX & (1 << TXINI)))
        ;  // Wait until the banks are ready to be filled
      UEDATX = usb_config_status;
      UEINTX &= ~(1 << TXINI);
      return;
    }

    if (bRequest == GET_STATUS) {
      while (!(UEINTX & (1 << TXINI)))
        ;
      UEDATX = 0;
      UEDATX = 0;
      UEINTX &= ~(1 << TXINI);
      return;
    }

    if (wIndex == 0) {  // Is this a request to the keyboard interface for HID
                        // class-specific requests?
      if (bmRequestType ==
          0xA1) {  // GET Requests - Refer to the table in HID Specification 7.2
                   // - This byte specifies the data direction of the packet.
                   // Unnecessary since bRequest is unique, but it makes the
                   // code clearer
        if (bRequest == GET_REPORT) {  // Get the current HID report
          while (!(UEINTX & (1 << TXINI)))
            ;  // Wait for the banks to be ready for transmission
          _send_data(0);
          UEINTX &= ~(1 << TXINI);
          return;
        }
        if (bRequest == GET_IDLE) {
          while (!(UEINTX & (1 << TXINI)))
            ;

          UEDATX = keyboard_idle_value;

          UEINTX &= ~(1 << TXINI);
          return;
        }
        if (bRequest == GET_PROTOCOL) {
          while (!(UEINTX & (1 << TXINI)))
            ;

          UEDATX = 0;

          UEINTX &= ~(1 << TXINI);
          return;
        }
      }

      if (bmRequestType ==
          0x21) {  // SET Requests - Host-to-device data direction
        if (bRequest == SET_REPORT) {
          while (!(UEINTX & (1 << RXOUTI)))
            ;  // This is the opposite of the TXINI one, we are waiting until
               // the banks are ready for reading instead of for writing
          UEINTX &= ~(1 << TXINI);  // Send ACK and clear TX bit
          UEINTX &= ~(1 << RXOUTI);
          return;
        }
        if (bRequest == SET_IDLE) {
          keyboard_idle_value = wValue;  //
          current_idle = 0;

          UEINTX &= ~(1 << TXINI);  // Send ACK and clear TX bit
          return;
        }
        if (bRequest ==
            SET_PROTOCOL) {  // This request is only mandatory for boot devices,
          UEINTX &= ~(1 << TXINI);  // Send ACK and clear TX bit
          return;
        }
      }
    }
  }
  UECONX |= (1 << STALLRQ) |
            (1 << EPEN);  // The host made an invalid request or there was an
                          // error with one of the request parameters
}
