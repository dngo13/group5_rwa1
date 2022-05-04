#ifndef AGV_H
#define AGV_H
#include "../util/util.h"

namespace motioncontrol {
    class Agv {

        public:
        explicit Agv(ros::NodeHandle&, std::string agv_name);

        /**
         * @brief Submit an AGV shipment
         *
         * @param shipment_type Shipment type which should match the order
         * @param station Assembly station
         * @return true AGV successfully submitted
         * @return false AGV not successfully submitted
         */
        bool shipAgv(std::string shipment_type, std::string station);
        bool getAGVStatus();


    private:
        /**
         * @brief State for AGV1
         *
         * @param msg
         */
        void agv_state_callback(const std_msgs::String& msg);
        std::string agv_name_;
        ros::ServiceClient agv_client_;
        ros::Subscriber agv_state_subscriber_;
        bool agv_ready_;
    };//class
}//namespace

#endif