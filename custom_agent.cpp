#include <uxr/agent/transport/custom/CustomAgent.hpp>
#include <uxr/agent/transport/endpoint/CustomEndPoint.hpp>


#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <signal.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <uxr/agent/transport/can/CanAgentLinux.hpp>
#include <uxr/agent/utils/Conversion.hpp>
#include <uxr/agent/logger/Logger.hpp>

#include <net/if.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

/**
 * This custom XRCE Agent example attempts to show how easy is for the user to define a custom
 * Micro XRCE-DDS Agent behaviour, in terms of transport initialization and closing, and also
 * regarding read and write operations.
 * For this simple case, an UDP socket is opened on port 8888. Additionally, some information
 * messages are being printed to demonstrate the custom behaviour.
 * As the endpoint is already defined, we are using the provided
 * `eprosima::uxr::IPv4EndPoint` by the library.
 * Other transport protocols might need to implement their own endpoint struct.
 */

int main(int argc, char** argv)
{
    eprosima::uxr::Middleware::Kind mw_kind(eprosima::uxr::Middleware::Kind::FASTDDS);
    
    struct pollfd poll_fd;
    struct sockaddr_can addr;
    struct ifreq ifr;

    /**
     * @brief Agent's initialization behaviour description.
     */
    eprosima::uxr::CustomAgent::InitFunction init_function = [&]() -> bool
    {
        //struct sockaddr_can addr;
        struct canfd_frame frame;
        struct can_frame rec_frame;

        bool rv = false;

        if ((poll_fd.fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) 
        {
		perror("socket creation: ");
		return false;
        }
        
        if (-1 != poll_fd.fd)
        {
            struct sockaddr_in address{};
            strcpy(ifr.ifr_name, "can0");
            ioctl(poll_fd.fd, SIOCGIFINDEX, &ifr);
            poll_fd.events = POLLIN;
            addr.can_family  = AF_CAN;
            addr.can_ifindex = ifr.ifr_ifindex;
            if (-1 != bind(poll_fd.fd,
                (struct sockaddr *)&addr,
                sizeof(addr)))
                {
                    rv = true;
                }
            else
            {
                perror("socket bind: ");
            }
        }
            {
                rv = true;
                UXR_AGENT_LOG_INFO(
                    UXR_DECORATE_GREEN(
                        "This is an example of a custom Micro XRCE-DDS Agent INIT function"),
                    "fd: {}",
                    poll_fd.fd);
            }
        return rv;
    };

    /**
     * @brief Agent's destruction actions.
     */
    eprosima::uxr::CustomAgent::FiniFunction fini_function = [&]() -> bool
    {
        if (-1 == poll_fd.fd)
        {
            return true;
        }

        if (0 == close(poll_fd.fd))
        {
            poll_fd.fd = -1;

            UXR_AGENT_LOG_INFO(
                UXR_DECORATE_GREEN(
                    "This is an example of a custom Micro XRCE-DDS Agent FINI function"),
                "fd: {}",
                poll_fd.fd);

            return true;
        }
        else
        {
            return false;
        }
    };

    /**
     * @brief Agent's incoming data functionality.
     */
    eprosima::uxr::CustomAgent::RecvMsgFunction recv_msg_function = [&](
            eprosima::uxr::CustomEndPoint* source_endpoint,
            uint8_t* buffer,
            size_t buffer_length,
            int timeout,
            eprosima::uxr::TransportRc& transport_rc) -> ssize_t
    {
        struct can_frame frame;
        uint16_t rv;
        rv = 8;

        read(poll_fd.fd,&frame,sizeof(struct can_frame));
        if (frame.can_id == 15)
        {
            memcpy(buffer,&(frame.data),frame.can_dlc);
            source_endpoint->set_member_value<uint32_t>("ID",frame.can_id);
            transport_rc = eprosima::uxr::TransportRc::ok;
            return frame.can_dlc;
        }
        else
        {
            perror("wrong ID");
            return -1;
        }
    };

    /**
     * @brief Agent's outcoming data flow definition.
     */
    eprosima::uxr::CustomAgent::SendMsgFunction send_msg_function = [&](
        const eprosima::uxr::CustomEndPoint* destination_endpoint,
        uint8_t* buffer,
        size_t message_length,
        eprosima::uxr::TransportRc& transport_rc) -> ssize_t
    {
        struct can_frame frame;
        uint8_t * ptr = buffer;
        int rv;

        ssize_t bytes_sent = 0;
        frame.can_id = 12;
        frame.can_dlc = 8;

        uint16_t cycle, rest;
        
        cycle = message_length / frame.can_dlc;
        rest = message_length % frame.can_dlc;

        for(cycle; cycle>0; cycle--)
        {
            //TODO add poll to get free fd
            memcpy(&(frame.data),ptr,8);
            rv = write(poll_fd.fd, &frame, 16);
            if (rv != -1)
            {
                bytes_sent += 8;
            }
            ptr += 8;
        }
        if (rest != 0)
        {
            frame.can_dlc = rest;
            memcpy(&(frame.data), ptr, rest);
            rv = write(poll_fd.fd, &frame,  16);
            if (rv != -1)
            {
                bytes_sent += rest;
            }    
        }
        transport_rc = eprosima::uxr::TransportRc::ok;
        return bytes_sent;
    };

    /**
     * Run the main application.
     */
    try
    {
        /**
         * EndPoint definition for this transport. We define an address and a port.
         */
        eprosima::uxr::CustomEndPoint custom_endpoint;
        custom_endpoint.add_member<uint32_t>("ID");
        /**
         * Create a custom agent instance.
         */
        eprosima::uxr::CustomAgent custom_agent(
            "CAN_BUS",
            &custom_endpoint,
            mw_kind,
            true,
            init_function,
            fini_function,
            send_msg_function,
            recv_msg_function);

        /**
         * Set verbosity level
         */
        custom_agent.set_verbose_level(0);

        /**
         * Run agent and wait until receiving an stop signal.
         */
        custom_agent.start();

        int n_signal = 0;
        sigset_t signals;
        sigwait(&signals, &n_signal);

        /**
         * Stop agent, and exit.
         */
        custom_agent.stop();
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cout << e.what() << std::endl;
        return 1;
    }
}