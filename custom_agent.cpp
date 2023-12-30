#include <uxr/agent/transport/custom/CustomAgent.hpp>
#include <uxr/agent/transport/endpoint/CustomEndPoint.hpp>

#include <sys/time.h>

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
//#include <linux/can/length.h>


#include <errno.h>

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
int64_t time_ms(void);
uint8_t len_to_dlc(size_t * len);


int main(int argc, char** argv)
{
    eprosima::uxr::Middleware::Kind mw_kind(eprosima::uxr::Middleware::Kind::FASTDDS);
    
    struct pollfd poll_fd;
    struct sockaddr_can addr;
    struct ifreq ifr; 

    int enable_canfd = 1;

    static const uint8_t len2dlc[65] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 9, 9 , 9, 10, 10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15};
    static const uint8_t dlc2len[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
    
    /**
     * @brief Agent's initialization behaviour description.
     */
    eprosima::uxr::CustomAgent::InitFunction init_function = [&]() -> bool
    {
        bool rv = false;

        if ((poll_fd.fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) 
        {
		perror("CAN socket creation fail: ");
		return false;
        }
        
        if (-1 != poll_fd.fd) //CAN start
        {
            struct sockaddr_in address{};
            strcpy(ifr.ifr_name, "can0");
            ioctl(poll_fd.fd, SIOCGIFINDEX, &ifr);
            poll_fd.events = POLLIN;
            addr.can_family  = AF_CAN;
            addr.can_ifindex = ifr.ifr_ifindex;
            if (-1 != bind(poll_fd.fd,      //Binding CAN soc
                (struct sockaddr *)&addr,
                sizeof(addr)))
                {
                    //enabling CANFD if it happens work with it
                    if (-1 != setsockopt(poll_fd.fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
                    &enable_canfd, sizeof(enable_canfd)))
                    {
                    rv = true;
                    }
                    else
                    {
                    enable_canfd = 0;
                    }
                }
            else
            {
                perror("CAN socket bind: ");
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
        struct canfd_frame fdframe;
        //struct can_frame raw_can_frame;

        uint16_t rv = 0;
	    
        if(enable_canfd == 1)
        {
                rv = poll(&poll_fd, 1, timeout);
                    if (rv > 0)
                    {
                        read(poll_fd.fd, &fdframe, sizeof(struct canfd_frame));
                        memcpy(buffer,&(fdframe.data),fdframe.len);
                        source_endpoint->set_member_value<uint16_t>("ID",fdframe.can_id);
                        transport_rc = (-1 != rv)
                        ? eprosima::uxr::TransportRc::ok
                        : eprosima::uxr::TransportRc::server_error;
                        return fdframe.len;
                    }
        }
        else
        {
            std::cout<<"CAN RAW TODO"<<std::endl;
        }

        transport_rc = eprosima::uxr::TransportRc::timeout_error;
        return -1;
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

        struct canfd_frame fdframe;

        uint8_t * ptr = buffer;
        ssize_t rv;
        uint8_t len_to_send=0;
        uint8_t rest_to_send=0;
        uint8_t cycle = 0;
        size_t rest = 0;
          
        if(enable_canfd == 1)
        {

            ssize_t bytes_sent = 0;
            fdframe.can_id = destination_endpoint->get_member<uint16_t>("ID") + 20;

            len_to_send = len_to_dlc(&message_length);

            cycle = message_length / len_to_send;
            rest = message_length % len_to_send;

            fdframe.len = len_to_send;

            for(cycle; cycle>0; cycle--)
            {
                memcpy(&(fdframe.data), ptr, len_to_send);
                rv = write(poll_fd.fd, &fdframe, sizeof(struct canfd_frame));
                bytes_sent += len_to_send;
                ptr += len_to_send;
            }

            while (rest != 0)
            {
                rest_to_send = len_to_dlc(&rest);
                fdframe.len = rest_to_send;
                rest -= rest_to_send;
                memcpy(&(fdframe.data), ptr, rest_to_send);
                rv = write(poll_fd.fd, &fdframe, sizeof(struct canfd_frame));
                bytes_sent += rest_to_send;
                ptr += rest_to_send;
            }
            if (rv == -1)  //TODO REMOVE CHECK FOR PERFORMANCE
            {
                perror("Error in sending messge in rest div");
            }
        
        transport_rc = (-1 != rv)
        ? eprosima::uxr::TransportRc::ok
        : eprosima::uxr::TransportRc::server_error;
        return bytes_sent;
        }
        else
        {
            return -1; //TODO CAN_RAW
        }
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
        custom_endpoint.add_member<uint16_t>("ID");
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

int64_t time_ms(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (int64_t)tv.tv_sec * 1000 + tv.tv_usec / 1000;
}


uint8_t len_to_dlc(size_t * len)
{
    uint8_t dlc = 0;
    if (* len<= 8)
    {
        dlc = (uint8_t) * len;
    }
    else if (* len < 12)
    {
        dlc = 8;
    }
    else if (* len < 16)
    {
        dlc = 12;
    }
    else if (* len < 20)
    {
        dlc = 16;
    }
    else if (* len < 24)
    {
        dlc = 20;
    }
    else if (* len < 32)
    {
        dlc = 24;
    }
    else if (* len < 48)
    {
        dlc = 32;
    }
    else if (* len < 64)
    {
        dlc = 48;
    }
    else
    {
        dlc = 64;
    }

    return dlc;
}
