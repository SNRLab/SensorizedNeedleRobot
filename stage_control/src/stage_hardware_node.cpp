//libusb standard header file
#include "libusb-1.0/libusb.h"
#include <stdio.h>
#include <string.h>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "stage_control_interfaces/srv/controller_command.hpp"
#include "std_msgs/msg/float64.hpp"

// Motorized Stage API /////////////////////////////////////
#pragma region ARCUSPERFORMAX

//these are defined by Arcus
#define PERFORMAX_RETURN_SERIAL_NUMBER 0x0
#define PERFORMAX_RETURN_DESCRIPTION 0x1
#define PERFORMAX_MAX_DEVICE_STRLEN 256

//these defines are to conveniently turn these types transparent
//note: C does not have a bool type.
typedef int AR_BOOL;
typedef long AR_DWORD;
typedef void AR_VOID;
//typedef usb_dev_handle*	AR_HANDLE;
typedef libusb_device_handle *AR_HANDLE;

//the function definitions are the same as the windows API
AR_BOOL fnPerformaxComGetNumDevices(AR_DWORD *numDevices);
AR_BOOL fnPerformaxComGetProductString(AR_DWORD dwNumDevice, AR_VOID *lpDeviceString, AR_DWORD dwOptions);
AR_BOOL fnPerformaxComOpen(AR_DWORD dwDeviceNum, AR_HANDLE *pHandle);
AR_BOOL fnPerformaxComClose(AR_HANDLE pHandle);
AR_BOOL fnPerformaxComSetTimeouts(AR_DWORD dwReadTimeout, AR_DWORD dwWriteTimeout);
AR_BOOL fnPerformaxComSendRecv(AR_HANDLE Handle, AR_VOID *wBuffer, AR_DWORD dwNumBytesToWrite, AR_DWORD dwNumBytesToRead, AR_VOID *rBuffer);
AR_BOOL fnPerformaxComFlush(AR_HANDLE Handle);

//the following does _not_ need to be called before using the other functions. It is safe to ignore its existence
AR_BOOL InitializePerformaxLibrary(AR_VOID);

struct libusb_context *usb_context = 0;
AR_DWORD libusb_ReadTimeout = 0;
AR_DWORD libusb_WriteTimeout = 0;

#define AR_FALSE 0
#define AR_TRUE 1

// Internal function to allow us to keep a single list of
// vendor/product codes that are compatible with this driver
int _is_performax_device_by_vendor_product(int vendor, int product)
{
    if ((vendor == 0x1589) && (product == 0xa101))
    {
        return AR_TRUE;
    }
    return AR_FALSE;
}

// Internal function to figure out if the libusb_device_descriptor is a performax device
int _is_performax_device(struct libusb_device_descriptor *descriptor)
{
    return _is_performax_device_by_vendor_product(descriptor->idVendor, descriptor->idProduct);
}

// Iterate through the list of usb devices present and count the number of
// devices that we could use (based on returning true to _is_performax_device)

AR_BOOL fnPerformaxComGetNumDevices(AR_DWORD *numDevices)
{
    ssize_t device_count, i;
    libusb_device **list;
    struct libusb_device_descriptor descriptor;

    if (!InitializePerformaxLibrary())
    {
        return AR_FALSE;
    }

    *numDevices = 0;

    device_count = libusb_get_device_list(usb_context, &list);

    for (i = 0; i < device_count; i++)
    {

        if (0 == libusb_get_device_descriptor(list[i], &descriptor))
        {
            if (_is_performax_device(&descriptor))
            {
                (*numDevices)++;
            }
        }
    }

    libusb_free_device_list(list, 1); // Unreference and remove items from the list
    return AR_TRUE;
}

// Internal function
// Return a libusb_device_descriptor for the device given by the dwNumDevice offset.
// On success, the offset number in the libusb_device list is returned.
// On error, -1 is returned.
int _get_libusb_device_offset_from_arcos_offset(libusb_device **list, ssize_t list_count,
                                                struct libusb_device_descriptor *descriptor, AR_DWORD dwNumDevice)
{
    int i;

    for (i = 0; i < list_count; i++)
    {
        // iterate through each device and find a perfmax device:
        if (0 == libusb_get_device_descriptor(list[i], descriptor))
        {
            if (_is_performax_device(descriptor))
            {
                if (dwNumDevice)
                {
                    // skip to the perfmax device offset in dwNumDevice
                    dwNumDevice--;
                }
                else
                { // This is the one we're interested in...
                    return i;
                }
            }
        }
    }

    return -1; // Couldn't find the offset requested...
}

AR_BOOL fnPerformaxComGetProductString(AR_DWORD dwNumDevice, AR_VOID *lpDeviceString, AR_DWORD dwOptions)
{
    ssize_t device_count, i;
    libusb_device **list;
    struct libusb_device_descriptor descriptor;
    AR_BOOL result;
    libusb_device_handle *device_handle;

    if (!InitializePerformaxLibrary())
    {
        return AR_FALSE;
    }

    result = AR_TRUE; // no error yet...

    // Get the list of all devices:
    device_count = libusb_get_device_list(usb_context, &list);

    i = _get_libusb_device_offset_from_arcos_offset(list, device_count, &descriptor, dwNumDevice);
    if (i < 0)
    {
        result = AR_FALSE; // invlaid dwNumDevice id; we should have found the offset above...
    }
    else
    {
        if (libusb_open(list[i], &device_handle) != 0)
        {
            result = AR_FALSE; // libusb_open error
        }
        else
        {
            if (dwOptions == PERFORMAX_RETURN_SERIAL_NUMBER)
            {
                if (0 > libusb_get_string_descriptor_ascii(
                            device_handle, descriptor.iSerialNumber, (unsigned char *)lpDeviceString, PERFORMAX_MAX_DEVICE_STRLEN))
                {
                    result = AR_FALSE; // invalid descriptor
                }
            }
            else if (dwOptions == PERFORMAX_RETURN_DESCRIPTION)
            {
                if (0 > libusb_get_string_descriptor_ascii(
                            device_handle, descriptor.iProduct, (unsigned char *)lpDeviceString, PERFORMAX_MAX_DEVICE_STRLEN))
                {
                    result = AR_FALSE; // invalid descriptor
                }
            }
            else
            {
                result = AR_FALSE; // invlaid dwOption
            }
            libusb_close(device_handle);
        }
    }

    libusb_free_device_list(list, 1); // Unreference and remove items from the list
    return result;
}

int _send_urb_control(AR_HANDLE device_handle, int id)
{

    return libusb_control_transfer(device_handle,
                                   0x40, // bmRequestType
                                   0x02, // bRequest,
                                   id,   // wValue,
                                   0x00, // wIndex,
                                   NULL, // data,
                                   0,    // wLength,
                                   libusb_WriteTimeout);
}

AR_BOOL fnPerformaxComOpen(AR_DWORD dwDeviceNum, AR_HANDLE *device_handle)
{
    ssize_t device_count, i;
    libusb_device **list;
    struct libusb_device_descriptor descriptor;
    AR_BOOL result;

    if (!InitializePerformaxLibrary())
    {
        return AR_FALSE;
    }

    result = AR_TRUE; // no error yet...

    // Get the list of all devices:
    device_count = libusb_get_device_list(usb_context, &list);

    i = _get_libusb_device_offset_from_arcos_offset(list, device_count, &descriptor, dwDeviceNum);
    if (i < 0)
    {
        result = AR_FALSE; // invlaid dwNumDevice id; we should have found the offset above...
    }
    else
    {
        if (0 != libusb_open(list[i], device_handle))
        {
            result = AR_FALSE; // libusb_open error
        }
        if (0 != libusb_claim_interface(*device_handle, 0))
        {
            result = AR_FALSE; // libusb_open error
        }
    }

    libusb_free_device_list(list, 1); // Unreference and remove items from the list

    _send_urb_control(*device_handle, 0x02); // Should document this better; it's some open command
    return result;
}

AR_BOOL fnPerformaxComClose(AR_HANDLE device_handle)
{
    AR_BOOL result;

    if (!InitializePerformaxLibrary())
    {
        return AR_FALSE;
    }

    _send_urb_control(device_handle, 0x04); // Should document this better; it's some close command

    result = AR_TRUE; // no error yet...

    if (0 != libusb_release_interface(device_handle, 0))
    {
        result = AR_FALSE; // libusb_open error
    }
    libusb_close(device_handle);
    return result;
}

AR_BOOL fnPerformaxComSetTimeouts(AR_DWORD dwReadTimeout, AR_DWORD dwWriteTimeout)
{
    if (!InitializePerformaxLibrary())
    {
        return AR_FALSE;
    }

    libusb_ReadTimeout = dwReadTimeout;
    libusb_WriteTimeout = dwWriteTimeout;

    return AR_TRUE; // TODO: errors for wacky times?
}

AR_BOOL fnPerformaxComSendRecv(AR_HANDLE device_handle, AR_VOID *wBuffer, AR_DWORD dwNumBytesToWrite, AR_DWORD dwNumBytesToRead, AR_VOID *rBuffer)
{
    if (!InitializePerformaxLibrary())
    {
        return AR_FALSE;
    }

    int transferred;
    int result;
    unsigned char buffer[4096];

    // clear any outstanding reads:
    result = libusb_bulk_transfer(device_handle, 0x82, buffer, sizeof(buffer), &transferred, libusb_ReadTimeout);
    // if the above fails, it's probably ok.  We probably don't care.

    result = libusb_bulk_transfer(device_handle, 0x02, (unsigned char *)wBuffer, dwNumBytesToWrite, &transferred, libusb_WriteTimeout);

#ifdef DEBUGARCUS
    printf("sent: %d (%d) result %d '%s'\n", transferred, (int)dwNumBytesToWrite, result, (char *)wBuffer);
#endif // DEBUGARCUS

    if (0 != result)
    {
        return AR_FALSE;
    }

    result = libusb_bulk_transfer(device_handle, 0x82, (unsigned char *)rBuffer, dwNumBytesToRead, &transferred, libusb_ReadTimeout);

#ifdef DEBUGARCUS
    printf("received: %d (%d) result %d - %s\n", transferred, (int)dwNumBytesToRead, result, (char *)rBuffer);
#endif // DEBUGARCUS
    if (0 != result)
    {
        return AR_FALSE;
    }

    return AR_TRUE;
}

AR_BOOL fnPerformaxComFlush(AR_HANDLE device_handle)
{
    if (!InitializePerformaxLibrary())
    {
        return AR_FALSE;
    }

    if (_send_urb_control(device_handle, 0x01) == 0)
    { // Should document this better; it's some flush command
        return AR_TRUE;
    }
    return AR_FALSE;
}

//the following does _not_ need to be called before using the other functions. It is safe to ignore its existence

// Since the original interface contained the above line, all functions call the initialization function
// below.  Since we have no context being passed inthe fnPerformax functions, we don't get the opportunity
// anywhere to properly close the libusb library with libusb_exit...

AR_BOOL InitializePerformaxLibrary(AR_VOID)
{
    if (usb_context)
    {
        return AR_TRUE;
    }

    if (!libusb_init(&usb_context))
    {
        return AR_TRUE;
    }

    return AR_FALSE;
}
#pragma endregion
// Motorized Stage API ////////////////////////////////////

using ControllerCommand = stage_control_interfaces::srv::ControllerCommand;
using namespace std::chrono_literals;

class Stage : public rclcpp::Node
{
public:
    explicit Stage() : Node("stage_hardware_node")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing...");
        if (!intialize())
        {
            RCLCPP_ERROR(this->get_logger(), "Could not initialize, shutting down...");
            rclcpp::shutdown();
        }

        // Start service
        RCLCPP_INFO(this->get_logger(), "Starting command service...");
        service = this->create_service<ControllerCommand>("controller/command",
                                                          std::bind(&Stage::send_command, this, std::placeholders::_1, std::placeholders::_2));

        // Start stage pose publisher
        x_publisher = this->create_publisher<std_msgs::msg::Float64>("stage/joint_states/x", 10);
        z_publisher = this->create_publisher<std_msgs::msg::Float64>("stage/joint_states/z", 10);
        timer = this->create_wall_timer(
            50ms, std::bind(&Stage::publish_state, this));

        // Start stage position command subscribers
        x_subscriber = this->create_subscription<std_msgs::msg::Float64>(
            "stage/x_position_controller/command",
            10,
            std::bind(&Stage::x_command_callback, this, std::placeholders::_1));

        z_subscriber = this->create_subscription<std_msgs::msg::Float64>(
            "stage/z_position_controller/command",
            10,
            std::bind(&Stage::z_command_callback, this, std::placeholders::_1));

        //run();
        RCLCPP_INFO(this->get_logger(), "Ready.");
    }

    ~Stage()
    {
        fnPerformaxComClose(Handle);
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr x_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr z_publisher;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr x_subscriber;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr z_subscriber;
    rclcpp::Service<ControllerCommand>::SharedPtr service;
    rclcpp::TimerBase::SharedPtr timer;
    AR_HANDLE Handle; //usb handle
    char out[64];
    char in[64];
    bool homed = false;

    void x_command_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        std::string s = "X" + std::to_string(mm_to_pulses(msg->data));
        strcpy(out, s.c_str());
        if (!fnPerformaxComSendRecv(Handle, out, 64, 64, in))
        {
            RCLCPP_ERROR(this->get_logger(), "Could not send\n");
        }
    }

    void z_command_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        std::string s = "Y" + std::to_string(mm_to_pulses(-1*msg->data));
        strcpy(out, s.c_str());
        if (!fnPerformaxComSendRecv(Handle, out, 64, 64, in))
        {
            RCLCPP_ERROR(this->get_logger(), "Could not send\n");
        }
    }

    void publish_state()
    {
        auto x = std_msgs::msg::Float64();
        auto z = std_msgs::msg::Float64();

        // Query x position
        strcpy(out, "PX"); //move the motor
        if (!fnPerformaxComSendRecv(Handle, out, 64, 64, in))
        {
            RCLCPP_ERROR(this->get_logger(), "Could not send\n");
        }
        x.data = pulses_to_mm(atof(in));

        // Query z position
        strcpy(out, "PY"); //move the motor
        if (!fnPerformaxComSendRecv(Handle, out, 64, 64, in))
        {
            RCLCPP_ERROR(this->get_logger(), "Could not send\n");
        }
        z.data = -1*pulses_to_mm(atof(in));

        // Publish
        x_publisher->publish(x);
        z_publisher->publish(z);
    }

    void send_command(const std::shared_ptr<ControllerCommand::Request> request,
                      std::shared_ptr<ControllerCommand::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Received command: %s", request->command.c_str());

        if (request->command.compare("home") == 0)
        {
            home();
            return;
        }
        else if (request->command.compare("zero") == 0)
        {
            // Zero x position
            strcpy(out, "PX=0");
            if (!fnPerformaxComSendRecv(Handle, out, 64, 64, in))
            {
                RCLCPP_ERROR(this->get_logger(), "Could not send\n");
            }

            // Zero z position
            strcpy(out, "PY=0");
            if (!fnPerformaxComSendRecv(Handle, out, 64, 64, in))
            {
                RCLCPP_ERROR(this->get_logger(), "Could not send\n");
            }
        }
        else if (request->command.compare("flush") == 0)
        {
            fnPerformaxComFlush(Handle);
            return;
        }

        strcpy(out, request->command.c_str());
        if (!fnPerformaxComSendRecv(Handle, out, 64, 64, in))
        {
            RCLCPP_ERROR(this->get_logger(), "Could not send\n");
        }
        RCLCPP_INFO(this->get_logger(), "Received response: %s", in);
        response->response = std::string(in);
    }

    bool intialize()
    {
        char lpDeviceString[PERFORMAX_MAX_DEVICE_STRLEN];
        AR_DWORD num;

        memset(out, 0, 64);
        memset(in, 0, 64);

        //acquire information

        if (!fnPerformaxComGetNumDevices(&num))
        {
            RCLCPP_ERROR(this->get_logger(), "error in fnPerformaxComGetNumDevices\n");
            return false;
        }
        if (num < 1)
        {
            RCLCPP_ERROR(this->get_logger(), "No motor found\n");
            return false;
        }

        if (!fnPerformaxComGetProductString(0, lpDeviceString, PERFORMAX_RETURN_SERIAL_NUMBER) ||
            !fnPerformaxComGetProductString(0, lpDeviceString, PERFORMAX_RETURN_DESCRIPTION))
        {
            RCLCPP_ERROR(this->get_logger(), "error acquiring product string\n");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "device description: %s\n", lpDeviceString);

        //setup the connection

        if (!fnPerformaxComOpen(0, &Handle))
        {
            RCLCPP_ERROR(this->get_logger(), "Error opening device\n");
            return false;
        }

        if (!fnPerformaxComSetTimeouts(5000, 5000))
        {
            RCLCPP_ERROR(this->get_logger(), "Error setting timeouts\n");
            return false;
        }
        // if (!fnPerformaxComFlush(Handle))
        // {
        //     RCLCPP_ERROR(this->get_logger(), "Error flushing the coms\n");
        //     return false;
        // }

        // setup the device

        strcpy(out, "LSPD=100"); //set low speed
        if (!fnPerformaxComSendRecv(Handle, out, 64, 64, in))
        {
            RCLCPP_ERROR(this->get_logger(), "Could not send\n");
            return false;
        }
        strcpy(out, "HSPD=10000"); //set high speed
        if (!fnPerformaxComSendRecv(Handle, out, 64, 64, in))
        {
            RCLCPP_ERROR(this->get_logger(), "Could not send\n");
            return false;
        }

        strcpy(out, "POL=16"); //set polarity on the limit switch to be positive
        if (!fnPerformaxComSendRecv(Handle, out, 64, 64, in))
        {
            RCLCPP_ERROR(this->get_logger(), "Could not send\n");
            return false;
        }

        strcpy(out, "ACC=300"); //set acceleration
        if (!fnPerformaxComSendRecv(Handle, out, 64, 64, in))
        {
            RCLCPP_ERROR(this->get_logger(), "Could not send\n");
            return false;
        }

        strcpy(out, "EO1=1"); //enable x
        if (!fnPerformaxComSendRecv(Handle, out, 64, 64, in))
        {
            RCLCPP_ERROR(this->get_logger(), "Could not send\n");
            return false;
        }

        strcpy(out, "EO2=1"); //enable y
        if (!fnPerformaxComSendRecv(Handle, out, 64, 64, in))
        {
            RCLCPP_ERROR(this->get_logger(), "Could not send\n");
            return false;
        }

        strcpy(out, "ID"); //read ID
        if (!fnPerformaxComSendRecv(Handle, out, 64, 64, in))
        {
            RCLCPP_ERROR(this->get_logger(), "Could not send\n");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Arcus Product: %s\n", in);

        strcpy(out, "DN"); //read Device Number
        if (!fnPerformaxComSendRecv(Handle, out, 64, 64, in))
        {
            RCLCPP_ERROR(this->get_logger(), "Could not send\n");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Device Number: %s\n", in);

        return true;
    }

    bool home()
    {
        RCLCPP_INFO(this->get_logger(), "Initializing homing sequence...");

        strcpy(out, "HLX+");
        if (!fnPerformaxComSendRecv(Handle, out, 64, 64, in))
        {
            RCLCPP_ERROR(this->get_logger(), "Could not send\n");
            return false;
        }

        strcpy(out, "HLY-");
        if (!fnPerformaxComSendRecv(Handle, out, 64, 64, in))
        {
            RCLCPP_ERROR(this->get_logger(), "Could not send\n");
            return false;
        }

        rclcpp::Rate rate(10);
        while (is_moving())
        {
            rate.sleep();
        }

        strcpy(out, "ABORT");
        if (!fnPerformaxComSendRecv(Handle, out, 64, 64, in))
        {
            RCLCPP_ERROR(this->get_logger(), "Could not send\n");
            return false;
        }

        strcpy(out, "PX=0"); //zero X position
        if (!fnPerformaxComSendRecv(Handle, out, 64, 64, in))
        {
            RCLCPP_ERROR(this->get_logger(), "Could not send\n");
            return false;
        }

        strcpy(out, "PY=0"); //zero Y position
        if (!fnPerformaxComSendRecv(Handle, out, 64, 64, in))
        {
            RCLCPP_ERROR(this->get_logger(), "Could not send\n");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Homing complete.");

        this->homed = true;
        return true;
    }

    bool is_moving()
    {
        strcpy(out, "MSTX"); //read Device Number
        if (!fnPerformaxComSendRecv(Handle, out, 64, 64, in))
        {
            RCLCPP_ERROR(this->get_logger(), "Could not send\n");
            return false;
        }
        int x_status = atoi(in);
        bool x_moving = x_status == 1 || x_status == 2 || x_status == 4;

        strcpy(out, "MSTY"); //read Device Number
        if (!fnPerformaxComSendRecv(Handle, out, 64, 64, in))
        {
            RCLCPP_ERROR(this->get_logger(), "Could not send\n");
            return false;
        }
        int z_status = atoi(in);
        bool z_moving = z_status == 1 || z_status == 2 || z_status == 4;

        return x_moving || z_moving;
    }

    double pulses_to_mm(double pulses)
    {
        return pulses * (0.00125 / 1000.0);
    }

    double mm_to_pulses(double mm)
    {
        return mm / (0.00125 / 1000.0);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Stage>());
    rclcpp::shutdown();
    return 0;
}