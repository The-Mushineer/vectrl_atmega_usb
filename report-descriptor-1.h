// D:\Projetos\vectrl_atmega_usb\report-descriptor-1.h


char ReportDescriptor[37] = {
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

