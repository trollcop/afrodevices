/* *****************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 LeafLabs LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 * ****************************************************************************/

#ifndef __DESCRIPTORS_H
#define __DESCRIPTORS_H

#define __packed __attribute__((__packed__))

#define USB_DESCRIPTOR_TYPE_DEVICE        0x01
#define USB_DESCRIPTOR_TYPE_CONFIGURATION 0x02
#define USB_DESCRIPTOR_TYPE_STRING        0x03
#define USB_DESCRIPTOR_TYPE_INTERFACE     0x04
#define USB_DESCRIPTOR_TYPE_ENDPOINT      0x05

#define USB_DEVICE_CLASS_CDC              0x02
#define USB_DEVICE_SUBCLASS_CDC           0x00
#define USB_INTERFACE_CLASS_CDC           0x02
/* CDC Abstract Control Model */
#define USB_INTERFACE_SUBCLASS_CDC_ACM    0x02
#define USB_INTERFACE_CLASS_DIC           0x0A

#define USB_CONFIG_ATTR_BUSPOWERED        0b10000000
#define USB_CONFIG_ATTR_SELF_POWERED      0b11000000

#define EP_TYPE_INTERRUPT                 0x03
#define EP_TYPE_BULK                      0x02

#define USB_DESCRIPTOR_ENDPOINT_IN        0x80
#define USB_DESCRIPTOR_ENDPOINT_OUT       0x00

#define USB_DESCRIPTOR_STRING_LEN(x) (2 + (x << 1))

#if defined(__cplusplus)
extern "C" {
#endif

#define USB_DESCRIPTOR_STRING(len)		\
  struct {					\
    uint8_t bLength;				\
    uint8_t bDescriptorType;			\
    uint16_t bString[len];			\
  } __packed

#define CDC_FUNCTIONAL_DESCRIPTOR_SIZE(DataSize) (3 + DataSize)
#define CDC_FUNCTIONAL_DESCRIPTOR(DataSize)	\
  struct					\
  {						\
    uint8_t bLength;				\
    uint8_t bDescriptorType;			\
    uint8_t SubType;				\
    uint8_t Data[DataSize];			\
  } __packed

    typedef struct {
        uint8_t bLength;
        uint8_t bDescriptorType;
        uint16_t bcdUSB;
        uint8_t bDeviceClass;
        uint8_t bDeviceSubClass;
        uint8_t bDeviceProtocol;
        uint8_t bMaxPacketSize0;
        uint16_t idVendor;
        uint16_t idProduct;
        uint16_t bcdDevice;
        uint8_t iManufacturer;
        uint8_t iProduct;
        uint8_t iSerialNumber;
        uint8_t bNumConfigurations;
    } __packed USB_Descriptor_Device;

    typedef struct {
        uint8_t bLength;
        uint8_t bDescriptorType;
        uint16_t wTotalLength;
        uint8_t bNumInterfaces;
        uint8_t bConfigurationValue;
        uint8_t iConfiguration;
        uint8_t bmAttributes;
        uint8_t bMaxPower;
    } __packed USB_Descriptor_Config_Header;

    typedef struct {
        uint8_t bLength;
        uint8_t bDescriptorType;
        uint8_t bInterfaceNumber;
        uint8_t bAlternateSetting;
        uint8_t bNumEndpoints;
        uint8_t bInterfaceClass;
        uint8_t bInterfaceSubClass;
        uint8_t bInterfaceProtocol;
        uint8_t iInterface;
    } __packed USB_Descriptor_Interface;


    typedef struct {
        uint8_t bLength;
        uint8_t bDescriptorType;
        uint8_t bEndpointAddress;
        uint8_t bmAttributes;
        uint16_t wMaxPacketSize;
        uint8_t bInterval;
    } __packed USB_Descriptor_Endpoint;

    typedef struct {
        USB_Descriptor_Config_Header Config_Header;
        USB_Descriptor_Interface CCI_Interface;
         CDC_FUNCTIONAL_DESCRIPTOR(2) CDC_Functional_IntHeader;
         CDC_FUNCTIONAL_DESCRIPTOR(2) CDC_Functional_CallManagement;
         CDC_FUNCTIONAL_DESCRIPTOR(1) CDC_Functional_ACM;
         CDC_FUNCTIONAL_DESCRIPTOR(2) CDC_Functional_Union;
        USB_Descriptor_Endpoint ManagementEndpoint;
        USB_Descriptor_Interface DCI_Interface;
        USB_Descriptor_Endpoint DataOutEndpoint;
        USB_Descriptor_Endpoint DataInEndpoint;
    } __packed USB_Descriptor_Config;

    typedef struct {
        uint8_t bLength;
        uint8_t bDescriptorType;
        uint16_t bString[];
    } USB_Descriptor_String;

    extern const USB_Descriptor_Device usbVcomDescriptor_Device;
    extern const USB_Descriptor_Config usbVcomDescriptor_Config;

    extern const uint8_t usbVcomDescriptor_LangID[USB_DESCRIPTOR_STRING_LEN(1)];
    extern const uint8_t usbVcomDescriptor_iManufacturer[USB_DESCRIPTOR_STRING_LEN(8)];
    extern const uint8_t usbVcomDescriptor_iProduct[USB_DESCRIPTOR_STRING_LEN(10)];

#if defined(__cplusplus)
}
#endif
#endif                          // __DESCRIPTORS_H
